"""
Tracking API Blueprint
Real-time tracking data management
- User tracking
- Robot telemetry
- ZMQ Unity integration
- WebSocket broadcasting
"""

from flask import Blueprint, jsonify, request
from flask_socketio import SocketIO, emit
from pathlib import Path
from datetime import datetime
import json
import yaml
import threading
import time

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False


class TrackingAPIHandler:
    """
    Tracking API Handler with Dependency Injection
    Handles real-time tracking from Unity via ZMQ
    """

    def __init__(self, config_dir: Path, socketio: SocketIO = None):
        """
        Initialize Tracking API Handler

        Args:
            config_dir: Path to config directory
            socketio: Flask-SocketIO instance for broadcasting
        """
        self.config_dir = config_dir
        self.socketio = socketio

        # Tracking state
        self.tracking_data = {
            'active_users': {},
            'active_robots': {},
            'last_update': 0
        }

        # Map data cache
        self._map_data = {}

        # ZMQ state
        self.zmq_context = None
        self.zmq_socket = None
        self.zmq_thread = None

        # Create Flask Blueprint
        self.blueprint = Blueprint('tracking_api', __name__)
        self._register_routes()

    def _register_routes(self):
        """Register all API routes"""
        self.blueprint.route('/data')(self.get_tracking_data)
        self.blueprint.route('/refresh')(self.refresh_tracking)
        self.blueprint.route('/users')(self.get_users)
        self.blueprint.route('/robots')(self.get_robots)
        self.blueprint.route('/status')(self.get_status)

    def load_map_data(self):
        """Load map_data.yaml"""
        map_data_path = self.config_dir / 'map_data.yaml'
        if map_data_path.exists():
            with open(map_data_path, 'r', encoding='utf-8') as f:
                self._map_data = yaml.safe_load(f)
        return self._map_data

    def load_tracking_data(self):
        """Load tracking_data.json if exists"""
        tracking_path = self.config_dir / 'tracking_data.json'
        if tracking_path.exists():
            try:
                with open(tracking_path, 'r', encoding='utf-8') as f:
                    self.tracking_data = json.load(f)
            except Exception as e:
                print(f"[Tracking] Error loading tracking_data.json: {e}")
        return self.tracking_data

    def save_tracking_data(self):
        """Save tracking_data.json"""
        tracking_path = self.config_dir / 'tracking_data.json'
        try:
            with open(tracking_path, 'w', encoding='utf-8') as f:
                json.dump(self.tracking_data, f, indent=2)
        except Exception as e:
            print(f"[Tracking] Error saving tracking_data.json: {e}")

    def get_tracking_data(self):
        """
        Get current tracking data

        Response:
            {
                "active_users": {...},
                "active_robots": {...},
                "last_update": timestamp
            }
        """
        return jsonify(self.tracking_data)

    def refresh_tracking(self):
        """
        Refresh tracking data from file

        Response:
            {
                "status": "success",
                "timestamp": "...",
                "data": {...}
            }
        """
        self.load_tracking_data()
        return jsonify({
            'status': 'success',
            'timestamp': datetime.now().isoformat(),
            'data': self.tracking_data
        })

    def get_users(self):
        """
        Get registered users from config/users/

        Merges _defaults.yaml with overrides/*.yaml for each user
        Returns only users with show_in_html: true

        Response:
            {
                "status": "success",
                "users": {
                    "user_id": {
                        "user_name": "...",
                        "role": "...",
                        ...
                    }
                }
            }
        """
        users = {}
        users_dir = self.config_dir / 'users'
        defaults_path = users_dir / '_defaults.yaml'
        overrides_dir = users_dir / 'overrides'

        if not defaults_path.exists():
            return jsonify({'status': 'success', 'users': {}})

        # Load defaults
        with open(defaults_path, 'r', encoding='utf-8') as f:
            defaults = yaml.safe_load(f) or {}

        # Load and merge each user override
        if overrides_dir.exists():
            for user_file in overrides_dir.glob('*.yaml'):
                user_id = user_file.stem  # filename without .yaml

                with open(user_file, 'r', encoding='utf-8') as f:
                    override = yaml.safe_load(f) or {}

                # Deep merge defaults + override
                merged = self._deep_merge_dict(defaults, override)

                # Only include if show_in_html is true
                if merged.get('show_in_html', False):
                    users[user_id] = merged

        return jsonify({
            'status': 'success',
            'users': users
        })

    def _deep_merge_dict(self, defaults, overrides):
        """Deep merge two dictionaries (override takes precedence)"""
        if not isinstance(defaults, dict) or not isinstance(overrides, dict):
            return overrides if overrides is not None else defaults

        result = dict(defaults)

        for key, override_value in overrides.items():
            if override_value is None:
                continue

            if key not in result:
                result[key] = override_value
                continue

            default_value = result[key]

            # Recursively merge nested dicts
            if isinstance(default_value, dict) and isinstance(override_value, dict):
                result[key] = self._deep_merge_dict(default_value, override_value)
            # Override non-empty values
            elif override_value not in ('', [], {}):
                result[key] = override_value

        return result

    def get_robots(self):
        """
        Get active robots from tracking data

        Response:
            {
                "status": "success",
                "robots": {...}
            }
        """
        return jsonify({
            'status': 'success',
            'robots': self.tracking_data.get('active_robots', {})
        })

    def get_status(self):
        """
        Get system status

        Response:
            {
                "status": "ok",
                "zmq_connected": true/false,
                "active_users": count,
                "active_robots": count,
                "timestamp": "..."
            }
        """
        return jsonify({
            'status': 'ok',
            'zmq_connected': self.zmq_socket is not None,
            'active_users': len(self.tracking_data.get('active_users', {})),
            'active_robots': len(self.tracking_data.get('active_robots', {})),
            'timestamp': datetime.now().isoformat()
        })

    # ==================== ZMQ Integration ====================

    def init_zmq(self, subscribe_port: int = 5565):
        """
        Initialize ZeroMQ subscriber

        Args:
            subscribe_port: Port to subscribe to Unity messages
        """
        if not ZMQ_AVAILABLE:
            print("[Tracking] ZMQ not available")
            return False

        try:
            self.zmq_context = zmq.Context()
            self.zmq_socket = self.zmq_context.socket(zmq.SUB)
            self.zmq_socket.connect(f"tcp://localhost:{subscribe_port}")
            self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            print(f"[Tracking] ZMQ initialized - subscribing to port {subscribe_port}")
            return True
        except Exception as e:
            print(f"[Tracking] ZMQ init error: {e}")
            return False

    def start_zmq_listener(self):
        """Start ZMQ listener thread"""
        if not self.zmq_socket:
            return

        self.zmq_thread = threading.Thread(target=self._zmq_listener, daemon=True)
        self.zmq_thread.start()
        print("[Tracking] ZMQ listener thread started")

    def _zmq_listener(self):
        """ZMQ listener thread function"""
        while True:
            try:
                if self.zmq_socket:
                    message = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
                    data = json.loads(message)
                    self._handle_zmq_message(data)
            except zmq.Again:
                time.sleep(0.01)
            except Exception as e:
                if 'Resource temporarily unavailable' not in str(e):
                    print(f"[Tracking] ZMQ listener error: {e}")
                time.sleep(0.1)

    def _handle_zmq_message(self, data: dict):
        """
        Handle incoming ZMQ message from Unity

        Args:
            data: Message data dict
        """
        msg_type = data.get('type')

        if msg_type == 'user_pose':
            self._handle_user_pose(data)
        elif msg_type == 'qr_update':
            self._handle_qr_update(data)
        elif msg_type == 'robot_spawn':
            self._handle_robot_spawn(data)
        else:
            print(f"[Tracking] Unknown message type: {msg_type}")

    def _handle_user_pose(self, data: dict):
        """Handle user pose update"""
        user_id = data.get('user_id')
        if not user_id:
            return

        self.tracking_data.setdefault('active_users', {})[user_id] = {
            'user_id': user_id,
            'pose': data.get('pose'),
            'position': data.get('position'),
            'rotation': data.get('rotation'),
            'qr_anchor': data.get('qr_anchor'),
            'last_seen': datetime.now().isoformat()
        }
        self.tracking_data['last_update'] = time.time()

        # Broadcast via WebSocket
        if self.socketio:
            self.socketio.emit('tracking_update', self.tracking_data)

    def _handle_qr_update(self, data: dict):
        """Handle QR detection/save event"""
        qr_data = data.get('data', {})
        qr_id = qr_data.get('qr_id')
        user_id = qr_data.get('user_id')

        print(f"[Tracking] QR Update: {qr_id} by {user_id}")

        # Broadcast via WebSocket
        if self.socketio:
            self.socketio.emit('qr_update', qr_data)

        # Save tracking data
        self.save_tracking_data()

    def _handle_robot_spawn(self, data: dict):
        """Handle robot spawn event"""
        robot_id = data.get('robot_id')
        robot_type = data.get('robot_type')
        owner = data.get('owner')

        self.tracking_data.setdefault('active_robots', {})[robot_id] = {
            'robot_id': robot_id,
            'robot_type': robot_type,
            'owner': owner,
            'spawned_at': datetime.now().isoformat()
        }

        # Broadcast via WebSocket
        if self.socketio:
            self.socketio.emit('robot_spawned', {
                'robot_id': robot_id,
                'robot_type': robot_type,
                'owner': owner
            })

        self.save_tracking_data()
