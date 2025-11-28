"""
Teams + AI Integration Flask App
- Based on multi-hri-mr/management/scripts/app.py patterns
- Config-driven (no hardcoding)
- AI Bot integration
"""

import os
import sys
import yaml
import requests
from pathlib import Path
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit, join_room, leave_room

# Add ai_bot to path
sys.path.insert(0, str(Path(__file__).parent))
from ai_bot import AIBot
from riley_api import RILEYAPIHandler
from history_api import history_api

# Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'teams-ai-integration-2025'

# Socket.IO
socketio = SocketIO(app, cors_allowed_origins="*")

# Register API Blueprints
# RILEY API will be registered after instantiation in __main__
app.register_blueprint(history_api, url_prefix='/api')

# Config paths
SHARED_DATA_DIR = Path('/shared_data/config')  # Docker mount point
AI_AGENTS_CONFIG = Path('/app/config/ai_agents_config.yaml')
RILEY_PERSONA_CONFIG = Path('/app/config/personas/riley_persona.yaml')
TEAMS_ROOM_CONFIG = Path('/app/config/teams_room_config.yaml')

# Global state
map_data = {}
ai_agents_config = {}
teams_room_config = {}
active_rooms = {}
active_rileys = {}


def load_configs():
    """Load all configs"""
    global map_data, ai_agents_config, teams_room_config

    # Load map_data (shared_data, read-only)
    map_data_path = SHARED_DATA_DIR / 'map_data.yaml'
    if map_data_path.exists():
        with open(map_data_path) as f:
            map_data = yaml.safe_load(f)
            print(f"✓ Loaded map_data: {len(map_data.get('registered_users', {}))} users")
    else:
        print(f"Warning: map_data not found at {map_data_path}")
        map_data = {}

    # Load AI agents config
    if AI_AGENTS_CONFIG.exists():
        with open(AI_AGENTS_CONFIG) as f:
            ai_agents_config = yaml.safe_load(f)
            print(f"✓ Loaded ai_agents_config")
    else:
        print(f"Warning: ai_agents_config not found at {AI_AGENTS_CONFIG}")
        ai_agents_config = {}

    # Load teams room config
    if TEAMS_ROOM_CONFIG.exists():
        with open(TEAMS_ROOM_CONFIG) as f:
            teams_room_config = yaml.safe_load(f)
            print(f"✓ Loaded teams_room_config")
    else:
        print(f"Warning: teams_room_config not found at {TEAMS_ROOM_CONFIG}")
        teams_room_config = {}


def build_riley_system_prompt(persona_file='riley_persona.yaml'):
    """
    Load riley_persona.yaml (or specified file) and convert to formatted system prompt

    Args:
        persona_file: Name of persona file in /app/config/personas/ (default: 'riley_persona.yaml')

    Returns:
        str: Formatted system prompt for RILEY
    """
    persona_path = Path(f'/app/config/personas/{persona_file}')

    if not persona_path.exists():
        print(f"Warning: {persona_file} not found, using default prompt")
        return "You are RILEY, a helpful AI assistant."

    try:
        with open(persona_path) as f:
            persona = yaml.safe_load(f)

        # Build system prompt from YAML structure
        prompt_parts = []

        # Header
        p = persona.get('persona', {})
        prompt_parts.append(f"You are {p.get('name', 'RILEY')}. {p.get('description', 'A helpful AI assistant')}.")
        prompt_parts.append("")

        # Core principles
        core = persona.get('core_principles', {})
        prompt_parts.append("=== Core Principles ===")
        prompt_parts.append("")

        if 'honesty' in core:
            prompt_parts.append(f"1. {core['honesty']['description']}")
            for rule in core['honesty'].get('rules', []):
                prompt_parts.append(f"   - {rule}")
            prompt_parts.append("")

        if 'diligence' in core:
            prompt_parts.append(f"2. {core['diligence']['description']}")
            for rule in core['diligence'].get('rules', []):
                prompt_parts.append(f"   - {rule}")
            prompt_parts.append("")

        if 'data_driven' in core:
            prompt_parts.append(f"3. {core['data_driven']['description']}")
            for rule in core['data_driven'].get('rules', []):
                prompt_parts.append(f"   - {rule}")
            prompt_parts.append("")

        # Personality & Expertise
        personality = persona.get('personality', {})
        if personality:
            prompt_parts.append("=== Personality ===")
            for trait in personality.get('traits', []):
                prompt_parts.append(f"- {trait}")
            prompt_parts.append("")

            if 'expertise' in personality:
                prompt_parts.append("Expertise:")
                for exp in personality['expertise']:
                    prompt_parts.append(f"- {exp}")
                prompt_parts.append("")

        # Responsibilities & Role
        resp = persona.get('responsibilities', {})
        if resp:
            prompt_parts.append("=== Responsibilities ===")
            for r in resp.get('primary', []):
                prompt_parts.append(f"- {r}")
            prompt_parts.append("")

        # Language rules
        comm = persona.get('communication', {})
        if comm:
            lang = comm.get('language_consistency', {})
            if lang:
                prompt_parts.append("=== Language Rules (CRITICAL) ===")
                prompt_parts.append(f"★ {lang.get('rule', '')}")
                prompt_parts.append("")
                # Include instruction if present
                instruction = lang.get('instruction', '')
                if instruction:
                    prompt_parts.append(instruction.strip())
                    prompt_parts.append("")
                prompt_parts.append("Allowed:")
                for allowed in lang.get('allowed', []):
                    prompt_parts.append(f"  ✓ {allowed}")
                prompt_parts.append("")
                prompt_parts.append("Forbidden (NEVER use):")
                for forbidden in lang.get('forbidden', []):
                    prompt_parts.append(f"  ✗ {forbidden}")
                prompt_parts.append("")

        # Response style
        style = persona.get('response_style', {})
        if style:
            prompt_parts.append("=== Response Style ===")
            prompt_parts.append("Preferred:")
            for pref in style.get('preferred', []):
                prompt_parts.append(f"- {pref}")
            prompt_parts.append("")
            prompt_parts.append("Avoid:")
            for avoid in style.get('avoid', []):
                prompt_parts.append(f"- {avoid}")
            prompt_parts.append("")

        # Response pattern (simplified for speed)
        pattern = persona.get('response_pattern', {})
        if pattern:
            prompt_parts.append("=== CRITICAL: Response Pattern ===")
            prompt_parts.append(f"★ {pattern.get('rule', '')}")

            # Length decision
            length_decision = pattern.get('length_decision', '')
            if length_decision:
                prompt_parts.append("")
                prompt_parts.append(length_decision.strip())

            ex = pattern.get('example', {})
            if ex:
                prompt_parts.append("")
                prompt_parts.append(f"User: {ex.get('user', '')}")
                prompt_parts.append(f"✓ Good: {ex.get('good', '')}")
                prompt_parts.append(f"✗ Bad: {ex.get('bad', '')}")

        system_prompt = "\n".join(prompt_parts)
        print(f"✓ Built RILEY system prompt from {persona_file} ({len(system_prompt)} chars)")
        return system_prompt

    except Exception as e:
        print(f"Error loading {persona_file}: {e}")
        return "You are RILEY, a helpful AI assistant."


def get_camera_url(user_id):
    """Get camera stream URL for user"""
    # Use map_data registered_users
    users = map_data.get('registered_users', {})
    if user_id in users:
        return f"/mrc/live/{user_id}"
    return None


@app.route('/')
def index():
    """Redirect to login"""
    return render_template('login.html')


@app.route('/login')
def login():
    """Login page"""
    return render_template('login.html')


@app.route('/teams-room')
def teams_room():
    """Teams + AI Room (v1 - Socket.IO based)"""
    return render_template('teams_room.html')


@app.route('/room-v2')
def teams_room_v2():
    """RILEY Expert Room (v2 - REST API based)"""
    return render_template('teams_room_v2.html')


@app.route('/api/users')
def get_users():
    """Get registered users from map_data.yaml"""
    users = map_data.get('registered_users', {})
    return jsonify({'users': users})


@app.route('/api/config')
def get_config():
    """Get client config"""
    riley = ai_agents_config.get('riley', {})
    return jsonify({
        'riley': riley,
        'teams_room': teams_room_config
    })


@app.route('/api/user/<user_id>/camera')
def get_user_camera(user_id):
    """Get camera stream URL for user"""
    stream_url = get_camera_url(user_id)

    if stream_url:
        devices = device_config.get('devices', {})
        device = devices.get(user_id, {})

        return jsonify({
            'status': 'success',
            'user_id': user_id,
            'stream_url': stream_url,
            'device_ip': device.get('device_ip'),
            'owner': device.get('owner')
        })

    return jsonify({'status': 'error', 'message': 'User not found'}), 404


@app.route('/mrc/live/<user_id>')
def mrc_proxy(user_id):
    """
    MRC stream proxy
    (Simplified version - full implementation would stream from Device Portal)
    """
    devices = device_config.get('devices', {})

    if user_id not in devices:
        return jsonify({'error': 'User not found'}), 404

    device = devices[user_id]
    device_ip = device.get('device_ip')
    portal_config = device_config.get('device_portal', {})

    # Build Device Portal URL
    protocol = portal_config.get('protocol', 'https')
    port = portal_config.get('port', 443)
    endpoint = portal_config.get('endpoints', {}).get('live_stream', '/api/holographic/stream/live.mp4')

    device_url = f"{protocol}://{device_ip}:{port}{endpoint}"

    # For now, return URL (full proxy would stream bytes)
    return jsonify({
        'device_url': device_url,
        'note': 'Full proxy implementation would stream bytes from Device Portal'
    })


# Socket.IO events

@socketio.on('connect')
def handle_connect():
    """Client connected"""
    emit('connected', {'status': 'connected'})


@socketio.on('join_room')
def handle_join_room(data):
    """User joins room"""
    room = data.get('room')
    user_id = data.get('user_id')
    user_name = data.get('user_name')  # Get user_name from client
    user_type = data.get('type', 'human')

    join_room(room)

    # Initialize room
    if room not in active_rooms:
        active_rooms[room] = {'humans': [], 'ai_bots': []}

    # Add to room
    if user_type == 'human':
        active_rooms[room]['humans'].append(user_id)

    # Get camera URL if available
    camera_url = get_camera_url(user_id)

    # Notify room
    emit('participant_joined', {
        'user_id': user_id,
        'user_name': user_name,  # Include user_name
        'type': user_type,
        'video_url': camera_url
    }, room=room)


@socketio.on('user_message')
def handle_user_message(data):
    """User sends message to AI Bot"""
    room = data.get('room')
    user_id = data.get('user_id')
    text = data.get('text')

    print(f"[DEBUG] user_message received: room={room}, user={user_id}, text={text[:50]}...")

    # Get AI Bot for this room
    bot_key = f"{room}_{user_id}"
    ai_bot = active_rileys.get(bot_key)

    print(f"[DEBUG] bot_key={bot_key}, ai_bot found={ai_bot is not None}")

    if ai_bot:
        # Process with AI Bot
        try:
            print(f"[DEBUG] Calling on_human_voice...")
            response_data = ai_bot.on_human_voice(text)
            print(f"[DEBUG] Got response: {str(response_data)[:100]}...")
            emit('ai_message', response_data, room=room)
            print(f"[DEBUG] Emitted ai_message to room {room}")
        except Exception as e:
            print(f"[ERROR] on_human_voice failed: {e}")
            emit('ai_message', {
                'bot_id': 'system',
                'text': f'Error: {str(e)}',
                'tts_engine': 'browser'
            }, room=room)
    else:
        print(f"[DEBUG] No AI bot found for {bot_key}")
        emit('ai_message', {
            'bot_id': 'system',
            'text': 'AI Bot not available. Please spawn one first.',
            'tts_engine': 'browser'
        }, room=room)


@socketio.on('spawn_ai_bot')
def handle_spawn_ai_bot(data):
    """Spawn AI Bot for user"""
    room = data.get('room')
    user_id = data.get('user_id')
    persona_language = data.get('persona_language', 'ko')  # 'ko' or 'en'

    bot_key = f"{room}_{user_id}"

    # Check if already exists
    if bot_key in active_rileys:
        emit('ai_bot_joined', {
            'bot_id': bot_key,
            'status': 'already_exists'
        }, room=room)
        return

    # Create AI Bot
    try:
        riley_config = ai_agents_config.get('riley', {})

        # Select persona file based on language
        persona_file = 'riley_persona_en.yaml' if persona_language == 'en' else 'riley_persona.yaml'

        # Build system prompt from selected persona file
        system_prompt = build_riley_system_prompt(persona_file)

        # Load provider-specific config
        provider = riley_config.get('provider', 'ollama')
        provider_config_path = f'/app/config/providers/{provider}.yaml'
        provider_config = {}
        try:
            with open(provider_config_path, 'r') as f:
                provider_config = yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load provider config: {e}")

        ai_bot = AIBot(
            user_id=bot_key,
            room=room,
            llm_config={
                'provider': provider,
                'model': provider_config.get('model', 'llama3.2'),
                'timeout': provider_config.get('timeout', 180),
                'url': provider_config.get('url', 'http://host.docker.internal:11434'),
                'api_key': provider_config.get('api_key', ''),
                'system_prompt': system_prompt
            },
            zmq_config=riley_config.get('zmq', {}),
            tts_config=riley_config.get('voice', {}),
            actual_user_id=user_id
        )

        ai_bot.start()
        active_rileys[bot_key] = ai_bot

        # Add to room
        if room in active_rooms:
            active_rooms[room]['ai_bots'].append(bot_key)

        emit('ai_bot_joined', {
            'bot_id': bot_key,
            'status': 'created'
        }, room=room)

    except Exception as e:
        emit('error', {'message': f'Failed to spawn AI Bot: {str(e)}'}, room=room)


if __name__ == '__main__':
    load_configs()

    # Initialize RILEY API Handler (Dependency Injection)
    riley_api_handler = RILEYAPIHandler(
        active_rileys=active_rileys,
        ai_agents_config=ai_agents_config,
        build_riley_system_prompt=build_riley_system_prompt
    )
    app.register_blueprint(riley_api_handler.blueprint, url_prefix='/api/riley')

    print("Teams + AI Integration Server")
    print(f"Shared Data: {SHARED_DATA_DIR}")

    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
