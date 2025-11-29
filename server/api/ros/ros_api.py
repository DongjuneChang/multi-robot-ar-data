"""
ROS2 API Blueprint
Production-grade API for ROS2 service proxy
- Device API (HoloLens/Quest detection)
- TCP Endpoint control
- MoveIt control
- Robot controller
- Pattern test service
- Foxglove status

All API calls proxy to ROS2 server via HTTP
"""

from flask import Blueprint, jsonify, request
from pathlib import Path
import requests
import subprocess
import threading
import os
import signal
import yaml


class ROS2APIHandler:
    """
    ROS2 API Handler with Dependency Injection
    Proxies web panel requests to ROS2 server
    """

    def __init__(self, config_dir: Path, ros_server_url: str = None):
        """
        Initialize ROS2 API Handler

        Args:
            config_dir: Path to config directory
            ros_server_url: ROS2 server URL (loaded from network_config if not provided)
        """
        self.config_dir = config_dir
        self._ros_server_url = ros_server_url
        self._network_config = None

        # Process handles for local services
        self._processes = {}

        # Create Flask Blueprint
        self.blueprint = Blueprint('ros_api', __name__)
        self._register_routes()

    @property
    def ros_server_url(self) -> str:
        """Get default ROS2 server URL from config or constructor"""
        if self._ros_server_url:
            return self._ros_server_url

        config = self._load_network_config()
        servers = config.get('servers', {}).get('ros2', {})

        if not servers:
            raise ValueError("No ROS2 servers defined in network_config.yaml")

        # Get first available server
        for server_id, server_info in servers.items():
            host = server_info.get('host')
            port = server_info.get('port')
            if not host or not port:
                raise ValueError(f"Server {server_id} missing host or port in network_config.yaml")
            return f"http://{host}:{port}"

        raise ValueError("No valid ROS2 server found in network_config.yaml")

    def get_ros_server_url(self, server_id: str = None) -> str:
        """
        Get ROS2 server URL by server ID or from request params

        Args:
            server_id: Server ID from network_config.yaml (e.g., 'chicago_home')
                      If None, tries to get from request args/json

        Returns:
            Server URL (e.g., 'http://192.168.1.4:10000')
        """
        # Check request for server specification
        if server_id is None:
            server_id = request.args.get('server') or (request.json or {}).get('server')

        if not server_id:
            return self.ros_server_url  # Use default

        # Look up server by ID
        config = self._load_network_config()
        servers = config.get('servers', {}).get('ros2', {})

        if server_id in servers:
            server_info = servers[server_id]
            host = server_info.get('host')
            port = server_info.get('port')
            if host and port:
                return f"http://{host}:{port}"

        # If not found by ID, assume it's already a URL or host:port
        if ':' in server_id and not server_id.startswith('http'):
            return f"http://{server_id}"

        return self.ros_server_url  # Fallback to default

    def _load_network_config(self) -> dict:
        """Load network_config.yaml with caching"""
        if self._network_config:
            return self._network_config

        config_path = self.config_dir / 'network_config.yaml'
        if config_path.exists():
            with open(config_path, 'r', encoding='utf-8') as f:
                self._network_config = yaml.safe_load(f) or {}
        else:
            self._network_config = {}

        return self._network_config

    def _register_routes(self):
        """Register all API routes"""
        # Device API
        self.blueprint.route('/device_api/<action>', methods=['POST'])(self.device_api_action)
        self.blueprint.route('/devices/scan', methods=['POST'])(self.scan_devices)
        self.blueprint.route('/devices', methods=['GET'])(self.list_devices)

        # TCP Endpoint
        self.blueprint.route('/tcp_endpoint/<action>', methods=['POST'])(self.tcp_endpoint_action)

        # MoveIt
        self.blueprint.route('/moveit/<action>', methods=['POST'])(self.moveit_action)

        # Controller
        self.blueprint.route('/controller/<action>', methods=['POST'])(self.controller_action)

        # Visualizer
        self.blueprint.route('/visualizer/<action>', methods=['POST'])(self.visualizer_action)

        # World TF
        self.blueprint.route('/world_tf/<action>', methods=['POST'])(self.world_tf_action)

        # Viz System
        self.blueprint.route('/viz_system/<action>', methods=['POST'])(self.viz_system_action)

        # Foxglove
        self.blueprint.route('/foxglove/<action>', methods=['POST'])(self.foxglove_action)
        self.blueprint.route('/foxglove/status', methods=['GET'])(self.foxglove_status)

        # Path Planning
        self.blueprint.route('/path/start', methods=['POST'])(self.path_start)
        self.blueprint.route('/path/stop', methods=['POST'])(self.path_stop)

        # Pattern Test
        self.blueprint.route('/pattern/service/<action>', methods=['POST'])(self.pattern_service_action)
        self.blueprint.route('/pattern/preview', methods=['POST'])(self.pattern_preview)
        self.blueprint.route('/pattern/start', methods=['POST'])(self.pattern_start)
        self.blueprint.route('/pattern/stop', methods=['POST'])(self.pattern_stop)

        # Joints
        self.blueprint.route('/joints/set', methods=['POST'])(self.joints_set)
        self.blueprint.route('/joints/home', methods=['POST'])(self.joints_home)
        self.blueprint.route('/joints/zero', methods=['POST'])(self.joints_zero)

        # System
        self.blueprint.route('/status', methods=['GET'])(self.get_status)
        self.blueprint.route('/system/clean', methods=['POST'])(self.system_clean)

    # ==================== Device API ====================

    def device_api_action(self, action: str):
        """
        Device API actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        if action == 'start':
            return self._start_process('device_api', 'python3 device_api_server.py')
        elif action == 'stop':
            return self._stop_process('device_api')
        else:
            return jsonify({'error': f'Unknown action: {action}'}), 400

    def scan_devices(self):
        """Scan for AR devices (HoloLens/Quest)"""
        try:
            # Proxy to device API server
            response = requests.post(
                f"{self.ros_server_url}/api/devices/scan",
                timeout=15
            )
            return jsonify(response.json()), response.status_code
        except requests.RequestException as e:
            return jsonify({
                'status': 'error',
                'error': str(e),
                'devices': []
            }), 503

    def list_devices(self):
        """List detected devices"""
        try:
            response = requests.get(
                f"{self.ros_server_url}/api/devices",
                timeout=5
            )
            return jsonify(response.json()), response.status_code
        except requests.RequestException as e:
            return jsonify({
                'status': 'error',
                'error': str(e),
                'devices': []
            }), 503

    # ==================== TCP Endpoint ====================

    def tcp_endpoint_action(self, action: str):
        """
        TCP Endpoint actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('tcp_endpoint', action)

    # ==================== MoveIt ====================

    def moveit_action(self, action: str):
        """
        MoveIt actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('moveit', action, request.get_json())

    # ==================== Controller ====================

    def controller_action(self, action: str):
        """
        Controller actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('controller', action)

    # ==================== Visualizer ====================

    def visualizer_action(self, action: str):
        """
        Visualizer actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('visualizer', action)

    # ==================== World TF ====================

    def world_tf_action(self, action: str):
        """
        World TF actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('world_tf', action)

    # ==================== Viz System ====================

    def viz_system_action(self, action: str):
        """
        Viz System actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('viz_system', action)

    # ==================== Foxglove ====================

    def foxglove_action(self, action: str):
        """
        Foxglove Bridge actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('foxglove', action)

    def foxglove_status(self):
        """Get Foxglove Bridge status"""
        try:
            response = requests.get(
                f"{self.ros_server_url}/api/foxglove/status",
                timeout=5
            )
            return jsonify(response.json()), response.status_code
        except requests.RequestException:
            return jsonify({
                'status': 'unknown',
                'connected': False
            })

    # ==================== Path Planning ====================

    def path_start(self):
        """Start path planning"""
        return self._proxy_ros_action('path', 'start')

    def path_stop(self):
        """Stop path planning"""
        return self._proxy_ros_action('path', 'stop')

    # ==================== Pattern Test ====================

    def pattern_service_action(self, action: str):
        """
        Pattern service actions (start/stop)

        Args:
            action: 'start' or 'stop'
        """
        return self._proxy_ros_action('pattern/service', action, request.get_json())

    def pattern_preview(self):
        """Preview pattern"""
        return self._proxy_ros_action('pattern', 'preview', request.get_json())

    def pattern_start(self):
        """Start pattern execution"""
        return self._proxy_ros_action('pattern', 'start', request.get_json())

    def pattern_stop(self):
        """Stop pattern execution"""
        return self._proxy_ros_action('pattern', 'stop')

    # ==================== Joints ====================

    def joints_set(self):
        """Set joint positions"""
        data = request.get_json()
        return self._proxy_ros_action('joints', 'set', data)

    def joints_home(self):
        """Move to home position"""
        return self._proxy_ros_action('joints', 'home')

    def joints_zero(self):
        """Move to zero position"""
        return self._proxy_ros_action('joints', 'zero')

    # ==================== System ====================

    def get_status(self):
        """Get system status (local processes + ROS2 server status)"""
        # Local process status
        local_status = {
            name: proc.poll() is None
            for name, proc in self._processes.items()
        }

        # Try to get ROS2 server status
        ros2_status = {}
        try:
            server_url = self.ros_server_url  # Use default, not request-based
            response = requests.get(f"{server_url}/api/status", timeout=3)
            if response.ok:
                ros2_status = response.json()
        except (requests.RequestException, ValueError):
            pass

        return jsonify({
            'status': 'ok',
            'local_processes': local_status,
            'ros2_server': ros2_status
        })

    def system_clean(self):
        """Clean system (stop all processes)"""
        results = {}
        for name in list(self._processes.keys()):
            result = self._stop_process(name)
            results[name] = result

        return jsonify({
            'status': 'success',
            'cleaned': results
        })

    # ==================== Helper Methods ====================

    def _proxy_ros_action(self, service: str, action: str, data: dict = None):
        """
        Proxy action to ROS2 server

        Args:
            service: Service name
            action: Action name
            data: Optional JSON data (can include 'server' key for server selection)

        Returns:
            JSON response
        """
        try:
            # Get server URL (from data, request args, or default)
            server_url = self.get_ros_server_url()
            url = f"{server_url}/api/{service}/{action}"

            # Remove 'server' from data if present (don't forward to ROS2)
            if data and 'server' in data:
                data = {k: v for k, v in data.items() if k != 'server'}

            if data:
                response = requests.post(url, json=data, timeout=10)
            else:
                response = requests.post(url, timeout=10)

            return jsonify(response.json()), response.status_code

        except requests.RequestException as e:
            return jsonify({
                'status': 'error',
                'error': str(e),
                'service': service,
                'action': action,
                'server_url': server_url
            }), 503

    def _start_process(self, name: str, command: str) -> dict:
        """
        Start a local process

        Args:
            name: Process name
            command: Command to execute

        Returns:
            Status dict
        """
        if name in self._processes and self._processes[name].poll() is None:
            return jsonify({
                'status': 'already_running',
                'name': name
            })

        try:
            process = subprocess.Popen(
                command.split(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid if os.name != 'nt' else None
            )
            self._processes[name] = process

            return jsonify({
                'status': 'started',
                'name': name,
                'pid': process.pid
            })

        except Exception as e:
            return jsonify({
                'status': 'error',
                'error': str(e),
                'name': name
            }), 500

    def _stop_process(self, name: str) -> dict:
        """
        Stop a local process

        Args:
            name: Process name

        Returns:
            Status dict
        """
        if name not in self._processes:
            return jsonify({
                'status': 'not_running',
                'name': name
            })

        process = self._processes[name]
        if process.poll() is not None:
            del self._processes[name]
            return jsonify({
                'status': 'already_stopped',
                'name': name
            })

        try:
            if os.name != 'nt':
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            else:
                process.terminate()

            process.wait(timeout=5)
            del self._processes[name]

            return jsonify({
                'status': 'stopped',
                'name': name
            })

        except Exception as e:
            return jsonify({
                'status': 'error',
                'error': str(e),
                'name': name
            }), 500
