"""
RILEY API Blueprint v2.0
Production-grade API for multi-user RILEY management

Changes in v2.0:
- Config paths updated to new ai/ structure
- Expert profiles: ai/knowledge/experts/{id}.yaml (individual files)
- Profile template: ai/knowledge/_defaults.yaml
- Providers: ai/providers/{provider}.yaml
- User preferences: ai/users/_defaults.yaml + overrides/
- Uses shared ConfigLoader from lib/config_utils.py

- Spawn/manage RILEY instances
- User-to-RILEY communication
- RILEY-to-RILEY communication
- Expert profile integration (RAG-ready)
"""

from flask import Blueprint, request, jsonify
from .ai_bot import AIBot
import yaml
import os
from pathlib import Path
from typing import Optional, List

# Import shared ConfigLoader
import sys
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'lib'))
from config_utils import ConfigLoader, deep_merge


class AIConfigLoader(ConfigLoader):
    """
    AI-specific config loader extending base ConfigLoader
    Handles expert profiles, providers, and user preferences
    """

    def load_provider(self, provider_name: str) -> dict:
        """Load LLM provider config from ai/providers/{provider}.yaml"""
        return self.load_yaml(f'ai/providers/{provider_name}.yaml') or {}

    def load_ai_defaults(self) -> dict:
        """Load AI defaults from ai/_defaults.yaml"""
        return self.load_yaml('ai/_defaults.yaml', use_cache=True) or {}

    def load_knowledge_defaults(self) -> dict:
        """Load knowledge defaults (expert profile template) from ai/knowledge/_defaults.yaml"""
        return self.load_yaml('ai/knowledge/_defaults.yaml', use_cache=True) or {}

    def load_expert_profile(self, expert_id: str) -> Optional[dict]:
        """
        Load expert profile from ai/knowledge/experts/{expert_id}.yaml
        Falls back to _default.yaml if not found
        """
        profile = self.load_yaml(f'ai/knowledge/experts/{expert_id}.yaml')
        if profile:
            return profile

        print(f"[AIConfigLoader] Expert '{expert_id}' not found, using _default.yaml")
        return self.load_yaml('ai/knowledge/experts/_default.yaml')

    def list_expert_profiles(self) -> List[dict]:
        """
        List all available expert profiles from ai/knowledge/experts/
        Returns list of profile summaries for UI
        """
        profiles = []
        for yaml_file in self.list_yaml_files('ai/knowledge/experts'):
            try:
                with open(yaml_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}

                expertise = data.get('expertise', {})
                primary = expertise.get('primary', [])

                profiles.append({
                    'id': data.get('id', yaml_file.stem),
                    'name': data.get('name', ''),
                    'title': data.get('title', ''),
                    'affiliation': data.get('affiliation', ''),
                    'description': ', '.join(primary[:2]) if primary else data.get('role', '')
                })
            except Exception as e:
                print(f"[AIConfigLoader] Error reading {yaml_file}: {e}")
                continue

        return profiles

    def load_user_preferences(self, user_id: str) -> dict:
        """
        Load user AI preferences with override pattern
        ai/users/_defaults.yaml + ai/users/overrides/{user_id}.yaml
        """
        return self.load_with_defaults(
            'ai/users/_defaults.yaml',
            f'ai/users/overrides/{user_id}.yaml'
        )


class RILEYAPIHandler:
    """
    RILEY API Handler v2.0 with Dependency Injection
    Uses AIConfigLoader for all config access
    """

    def __init__(self, active_rileys, ai_agents_config, build_riley_system_prompt):
        """
        Initialize RILEY API Handler

        Args:
            active_rileys: Dictionary of active RILEY instances
            ai_agents_config: AI agents configuration dict
            build_riley_system_prompt: Function to build system prompts
        """
        self.active_rileys = active_rileys
        self.ai_agents_config = ai_agents_config
        self.build_riley_system_prompt = build_riley_system_prompt

        # Initialize AI config loader
        self.config = AIConfigLoader()

        # Load and cache profile template from knowledge/_defaults.yaml
        knowledge_defaults = self.config.load_knowledge_defaults()
        self.profile_template = knowledge_defaults.get('expert_profile_template', {})

        # Create Flask Blueprint
        self.blueprint = Blueprint('riley_api', __name__)
        self._register_routes()

    def _register_routes(self):
        """Register all API routes"""
        self.blueprint.route('/spawn', methods=['POST'])(self.spawn_riley)
        self.blueprint.route('/user_to_riley', methods=['POST'])(self.user_to_riley)
        self.blueprint.route('/riley_to_riley', methods=['POST'])(self.riley_to_riley)
        self.blueprint.route('/list_rileys', methods=['GET'])(self.list_rileys)
        self.blueprint.route('/get_history', methods=['POST'])(self.get_history)
        self.blueprint.route('/list_profiles', methods=['GET'])(self.list_profiles)
        self.blueprint.route('/despawn', methods=['POST'])(self.despawn_riley)
        self.blueprint.route('/export_history', methods=['GET'])(self.export_history)
        self.blueprint.route('/get_user_preferences', methods=['GET'])(self.get_user_preferences)
        # v2.0 new endpoints
        self.blueprint.route('/health', methods=['GET'])(self.health_check)

    def _merge_persona_with_profile(self, base_persona, expert_profile):
        """
        Merge base persona with expert profile to create enriched system prompt
        Uses YAML template (NO HARDCODING)

        Args:
            base_persona: Base RILEY system prompt
            expert_profile: Expert profile dict

        Returns:
            str: Enriched system prompt
        """
        if not expert_profile or not self.profile_template:
            return base_persona

        # Get template sections
        template = self.profile_template
        identity_tmpl = template.get('identity', {})
        expertise_tmpl = template.get('expertise', {})
        projects_tmpl = template.get('projects', {})

        # Build identity section
        identity_format = identity_tmpl.get('format', '')
        identity_header = identity_tmpl.get('header', '')
        identity_text = f"\n{identity_header}\n{identity_format.format(**expert_profile)}"

        # Build expertise section
        expertise = expert_profile.get('expertise', {})
        primary = expertise.get('primary', [])

        expertise_header = expertise_tmpl.get('header', '')
        primary_label = expertise_tmpl.get('primary_areas_label', '')
        item_prefix = expertise_tmpl.get('item_prefix', '')

        expertise_text = f"\n{expertise_header}\n"
        if primary:
            expertise_text += f"{primary_label}\n"
            for area in primary:
                expertise_text += f"{item_prefix}{area}\n"

        # Add research focus if enabled
        if expertise_tmpl.get('research_focus_enabled', False):
            research_focus = expert_profile.get('research_focus', '')
            if research_focus:
                expertise_text += f"\n{research_focus}\n"

        # Build projects section
        key_projects = expert_profile.get('key_projects', [])
        projects_header = projects_tmpl.get('header', '')
        item_format = projects_tmpl.get('item_format', '')
        simple_format = projects_tmpl.get('simple_item_format', '')

        if key_projects:
            projects_text = f"\n{projects_header}\n"
            for project in key_projects:
                if isinstance(project, dict):
                    projects_text += item_format.format(**project) + "\n"
                else:
                    projects_text += simple_format.format(project=project) + "\n"
        else:
            projects_text = ""

        # Get separators and footer from template
        opening_sep = template.get('opening_separator', '')
        closing_sep = template.get('closing_separator', '')
        footer = template.get('footer', '')

        # Merge all sections using template structure
        enriched_prompt = f"""{base_persona}

{opening_sep}
{identity_text}
{expertise_text}
{projects_text}
{footer}
{closing_sep}
"""

        return enriched_prompt

    def spawn_riley(self):
        """
        Spawn a RILEY for a user in a room

        Request:
            {
                "room": "test_room",
                "user_id": "alice",
                "user_name": "Alice" (optional),
                "persona_language": "ko" or "en" (optional, default: "ko"),
                "expert_profile": "hyunglae_lee" (optional, expert profile ID),
                "use_default_mimic": true (optional, use user's default_mimic from ai_user_preferences)
            }

        Response:
            {
                "status": "success",
                "bot_key": "test_room_alice",
                "expert_profile": "hyunglae_lee" (if applied)
            }
        """
        data = request.get_json()
        room = data.get('room')
        user_id = data.get('user_id')
        user_name = data.get('user_name', user_id)
        persona_language = data.get('persona_language', 'ko')
        expert_profile_id = data.get('expert_profile')
        use_default_mimic = data.get('use_default_mimic', False)

        if not room or not user_id:
            return jsonify({'status': 'error', 'message': 'room and user_id required'}), 400

        bot_key = f"{room}_{user_id}"

        # Check if already exists
        if bot_key in self.active_rileys:
            return jsonify({
                'status': 'success',
                'bot_key': bot_key,
                'message': 'RILEY already exists'
            })

        # Create RILEY
        try:
            riley_config = self.ai_agents_config.get('riley', {})

            # Select persona file based on language
            persona_file = 'riley_persona_en.yaml' if persona_language == 'en' else 'riley_persona.yaml'
            system_prompt = self.build_riley_system_prompt(persona_file)

            # If no explicit expert_profile, check user's default_mimic preference
            if not expert_profile_id and use_default_mimic:
                user_prefs = self.config.load_user_preferences(user_id)
                default_mimic = user_prefs.get('default_mimic')
                if default_mimic:
                    expert_profile_id = default_mimic
                    print(f"[RILEYAPIHandler] Using default_mimic '{default_mimic}' for {user_id}")

            # Merge with expert profile if provided (v2: individual files)
            expert_profile = None
            if expert_profile_id:
                expert_profile = self.config.load_expert_profile(expert_profile_id)
                if expert_profile:
                    system_prompt = self._merge_persona_with_profile(system_prompt, expert_profile)
                    print(f"[RILEYAPIHandler] Expert profile '{expert_profile_id}' merged for {bot_key}")
                else:
                    print(f"[RILEYAPIHandler] Expert profile '{expert_profile_id}' not found")

            # Load provider-specific config (v2: ai/providers/)
            provider = riley_config.get('provider', 'ollama')
            provider_config = self.config.load_provider(provider)

            # Load AI defaults for fallback values
            ai_defaults = self.config.load_ai_defaults()
            llm_defaults = ai_defaults.get('llm', {})

            ai_bot = AIBot(
                user_id=bot_key,
                room=room,
                llm_config={
                    'provider': provider,
                    'model': provider_config.get('model', 'llama3.1:8b-instruct-q8_0'),
                    'timeout': provider_config.get('options', {}).get('timeout', llm_defaults.get('timeout', 60)),
                    'url': provider_config.get('url', 'http://host.docker.internal:11434'),
                    'api_key': os.environ.get(provider_config.get('api_key_env', ''), ''),
                    'system_prompt': system_prompt
                },
                zmq_config=riley_config.get('zmq', {}),
                tts_config=riley_config.get('voice', {}),
                actual_user_id=user_id
            )

            ai_bot.start()
            self.active_rileys[bot_key] = ai_bot

            response = {
                'status': 'success',
                'bot_key': bot_key,
                'message': f'RILEY spawned for {user_name}'
            }

            if expert_profile:
                response['expert_profile'] = expert_profile_id
                response['expert_name'] = expert_profile.get('name')

            return jsonify(response)

        except Exception as e:
            return jsonify({
                'status': 'error',
                'message': f'Failed to spawn RILEY: {str(e)}'
            }), 500

    def user_to_riley(self):
        """
        User sends message to their RILEY

        Request:
            {
                "room": "test_room",
                "user_id": "alice",
                "message": "안녕 RILEY"
            }

        Response:
            {
                "status": "success",
                "bot_key": "test_room_alice",
                "response": "안녕하세요! 무엇을 도와드릴까요?"
            }
        """
        data = request.get_json()
        room = data.get('room')
        user_id = data.get('user_id')
        message = data.get('message')

        if not room or not user_id or not message:
            return jsonify({'status': 'error', 'message': 'room, user_id, and message required'}), 400

        bot_key = f"{room}_{user_id}"
        ai_bot = self.active_rileys.get(bot_key)

        if not ai_bot:
            return jsonify({
                'status': 'error',
                'message': f'RILEY not found for {bot_key}. Please spawn first.'
            }), 404

        # Process message
        try:
            response_data = ai_bot.on_human_voice(message)
            return jsonify({
                'status': 'success',
                'bot_key': bot_key,
                'response': response_data.get('text', '')
            })

        except Exception as e:
            return jsonify({
                'status': 'error',
                'message': f'Error processing message: {str(e)}'
            }), 500

    def riley_to_riley(self):
        """
        RILEY sends message to another RILEY (email-like communication)

        Request:
            {
                "room": "test_room",
                "from_user_id": "alice",
                "to_user_ids": ["bob"] or "bob",
                "message": "안녕 Bob의 RILEY야"
            }

        Response:
            {
                "status": "success",
                "from_bot_key": "test_room_alice",
                "responses": {
                    "test_room_bob": "안녕 Alice의 RILEY야"
                }
            }
        """
        data = request.get_json()
        room = data.get('room')
        from_user_id = data.get('from_user_id')
        to_user_ids = data.get('to_user_ids')
        message = data.get('message')

        if not room or not from_user_id or not to_user_ids or not message:
            return jsonify({
                'status': 'error',
                'message': 'room, from_user_id, to_user_ids, and message required'
            }), 400

        # Support single recipient (string) or multiple (list)
        if isinstance(to_user_ids, str):
            to_user_ids = [to_user_ids]

        from_bot_key = f"{room}_{from_user_id}"

        # Check sender exists
        if from_bot_key not in self.active_rileys:
            return jsonify({
                'status': 'error',
                'message': f'Sender RILEY not found: {from_bot_key}'
            }), 404

        # Send to each recipient
        responses = {}
        errors = {}

        for to_user_id in to_user_ids:
            to_bot_key = f"{room}_{to_user_id}"
            to_ai_bot = self.active_rileys.get(to_bot_key)

            if not to_ai_bot:
                errors[to_bot_key] = f'Recipient RILEY not found'
                continue

            # Format message with sender prefix (email From:)
            formatted_message = f"[RILEY_{from_user_id}]: {message}"

            try:
                # Process message (recipient treats it like human voice)
                response_data = to_ai_bot.on_human_voice(formatted_message)
                responses[to_bot_key] = response_data.get('text', '')

            except Exception as e:
                errors[to_bot_key] = f'Error: {str(e)}'

        # Build response
        result = {
            'status': 'success' if responses else 'error',
            'from_bot_key': from_bot_key,
            'responses': responses
        }

        if errors:
            result['errors'] = errors

        return jsonify(result)

    def list_rileys(self):
        """
        List all active RILEYs

        Response:
            {
                "status": "success",
                "rileys": ["test_room_alice", "test_room_bob"],
                "count": 2
            }
        """
        rileys = list(self.active_rileys.keys())
        return jsonify({
            'status': 'success',
            'rileys': rileys,
            'count': len(rileys)
        })

    def get_history(self):
        """
        Get conversation history for a RILEY

        Request:
            {
                "room": "test_room",
                "user_id": "alice"
            }

        Response:
            {
                "status": "success",
                "bot_key": "test_room_alice",
                "history": [
                    {"role": "user", "content": "안녕"},
                    {"role": "assistant", "content": "안녕하세요"}
                ]
            }
        """
        data = request.get_json()
        room = data.get('room')
        user_id = data.get('user_id')

        if not room or not user_id:
            return jsonify({'status': 'error', 'message': 'room and user_id required'}), 400

        bot_key = f"{room}_{user_id}"
        ai_bot = self.active_rileys.get(bot_key)

        if not ai_bot:
            return jsonify({
                'status': 'error',
                'message': f'RILEY not found for {bot_key}'
            }), 404

        return jsonify({
            'status': 'success',
            'bot_key': bot_key,
            'history': ai_bot.conversation_history
        })

    def list_profiles(self):
        """
        List available expert profiles from YAML

        Response:
            {
                "status": "success",
                "profiles": [
                    {
                        "id": "hyunglae_lee",
                        "name": "Prof. Hyunglae Lee",
                        "title": "Associate Professor",
                        "affiliation": "Arizona State University",
                        "description": "Rehabilitation Robotics & Human-Robot Interaction"
                    },
                    ...
                ],
                "count": 3
            }
        """
        # v2: Load from individual files in ai/knowledge/experts/
        try:
            profiles = self.config.list_expert_profiles()
            return jsonify({
                'status': 'success',
                'profiles': profiles,
                'count': len(profiles)
            })
        except Exception as e:
            return jsonify({
                'status': 'error',
                'message': f'Error loading profiles: {str(e)}'
            }), 500

    def despawn_riley(self):
        """
        Despawn (remove) a RILEY instance

        Request:
            {
                "room": "test_room",
                "user_id": "alice"
            }

        Response:
            {
                "status": "success",
                "bot_key": "test_room_alice",
                "message": "RILEY despawned"
            }
        """
        data = request.get_json()
        room = data.get('room')
        user_id = data.get('user_id')

        if not room or not user_id:
            return jsonify({'status': 'error', 'message': 'room and user_id required'}), 400

        bot_key = f"{room}_{user_id}"
        ai_bot = self.active_rileys.get(bot_key)

        if not ai_bot:
            return jsonify({
                'status': 'error',
                'message': f'RILEY not found for {bot_key}'
            }), 404

        try:
            # Stop the bot
            ai_bot.stop()
            del self.active_rileys[bot_key]

            return jsonify({
                'status': 'success',
                'bot_key': bot_key,
                'message': 'RILEY despawned'
            })

        except Exception as e:
            return jsonify({
                'status': 'error',
                'message': f'Error despawning RILEY: {str(e)}'
            }), 500

    def export_history(self):
        """
        Export conversation history for a RILEY to JSON or Markdown

        Query params:
            room: Room ID (required)
            user_id: User/Expert ID (required)
            format: "json" or "md" (default: json)

        Response:
            JSON or Markdown file download
        """
        from flask import Response
        import json
        from datetime import datetime

        room = request.args.get('room')
        user_id = request.args.get('user_id')
        export_format = request.args.get('format', 'json')

        if not room or not user_id:
            return jsonify({'status': 'error', 'message': 'room and user_id required'}), 400

        bot_key = f"{room}_{user_id}"
        ai_bot = self.active_rileys.get(bot_key)

        if not ai_bot:
            return jsonify({
                'status': 'error',
                'message': f'RILEY not found for {bot_key}'
            }), 404

        history = ai_bot.conversation_history

        if export_format == 'md':
            # Markdown format
            md_content = f"# RILEY Conversation History\n\n"
            md_content += f"**Room:** {room}\n"
            md_content += f"**Expert:** {user_id}\n"
            md_content += f"**Exported:** {datetime.now().isoformat()}\n"
            md_content += f"**Messages:** {len(history)}\n\n---\n\n"

            for msg in history:
                role = msg.get('role', 'unknown')
                content = msg.get('content', '')
                emoji = '👤' if role == 'user' else '🤖'
                md_content += f"### {emoji} {role.upper()}\n\n{content}\n\n---\n\n"

            return Response(
                md_content,
                mimetype='text/markdown',
                headers={'Content-Disposition': f'attachment;filename={bot_key}_history.md'}
            )
        else:
            # JSON format
            export_data = {
                'room': room,
                'user_id': user_id,
                'bot_key': bot_key,
                'exported_at': datetime.now().isoformat(),
                'message_count': len(history),
                'conversation': history
            }

            return Response(
                json.dumps(export_data, indent=2, ensure_ascii=False),
                mimetype='application/json',
                headers={'Content-Disposition': f'attachment;filename={bot_key}_history.json'}
            )

    def get_user_preferences(self):
        """
        Get AI preferences for a user

        Query params:
            user_id: User ID (required, e.g., 'dongjune_chang')

        Response:
            {
                "status": "success",
                "user_id": "dongjune_chang",
                "preferences": { ... }
            }
        """
        user_id = request.args.get('user_id')

        if not user_id:
            return jsonify({'status': 'error', 'message': 'user_id required'}), 400

        try:
            # v2: Use config loader
            preferences = self.config.load_user_preferences(user_id)

            return jsonify({
                'status': 'success',
                'user_id': user_id,
                'preferences': preferences
            })

        except Exception as e:
            return jsonify({
                'status': 'error',
                'message': f'Error loading preferences: {str(e)}'
            }), 500

    def health_check(self):
        """
        Health check endpoint for monitoring

        Response:
            {
                "status": "healthy",
                "version": "2.0",
                "active_rileys": 3
            }
        """
        return jsonify({
            'status': 'healthy',
            'version': '2.0',
            'active_rileys': len(self.active_rileys)
        })
