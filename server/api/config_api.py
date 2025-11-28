"""
Config API Blueprint
Production-grade API for config file management
- Config files serving
- Robot config merging (_defaults + override)
- No hardcoding - all paths from constructor
"""

from flask import Blueprint, jsonify, Response
from pathlib import Path
import yaml
import copy


class ConfigAPIHandler:
    """
    Config API Handler with Dependency Injection
    No global variables - all dependencies injected through constructor
    """

    def __init__(self, config_dir: Path, network_robots_dir: Path):
        """
        Initialize Config API Handler

        Args:
            config_dir: Path to config directory (shared_data/config)
            network_robots_dir: Path to network_robots directory
        """
        self.config_dir = config_dir
        self.network_robots_dir = network_robots_dir

        # Create Flask Blueprint
        self.blueprint = Blueprint('config_api', __name__)
        self._register_routes()

    def _register_routes(self):
        """Register all API routes"""
        self.blueprint.route('/<filename>')(self.get_config_file)
        self.blueprint.route('/map_data.yaml')(self.get_map_data)
        self.blueprint.route('/device_config.yaml')(self.get_device_config)
        self.blueprint.route('/network_config.yaml')(self.get_network_config)

    def get_config_file(self, filename: str):
        """
        Get config file (YAML)

        Args:
            filename: Config file name

        Response:
            YAML content with text/yaml mimetype
        """
        filepath = self.config_dir / filename

        if not filepath.exists():
            return jsonify({'error': f'Config not found: {filename}'}), 404

        content = filepath.read_text(encoding='utf-8')
        return Response(content, mimetype='text/yaml')

    def get_map_data(self):
        """Get map_data.yaml"""
        return self.get_config_file('map_data.yaml')

    def get_device_config(self):
        """Get device_config.yaml"""
        return self.get_config_file('device_config.yaml')

    def get_network_config(self):
        """Get network_config.yaml"""
        return self.get_config_file('network_config.yaml')


class RobotConfigAPIHandler:
    """
    Robot Config API Handler
    Handles robot-specific config with merging
    """

    def __init__(self, config_dir: Path, network_robots_dir: Path):
        """
        Initialize Robot Config API Handler

        Args:
            config_dir: Path to config directory
            network_robots_dir: Path to network_robots directory
        """
        self.config_dir = config_dir
        self.network_robots_dir = network_robots_dir

        # Cache for map_data
        self._map_data_cache = None
        self._map_data_mtime = 0

        # Create Flask Blueprint
        self.blueprint = Blueprint('robot_api', __name__)
        self._register_routes()

    def _register_routes(self):
        """Register all API routes"""
        self.blueprint.route('/')(self.list_robots)
        self.blueprint.route('/<robot_type>/config')(self.get_merged_config)
        self.blueprint.route('/<robot_type>/defaults')(self.get_defaults)
        self.blueprint.route('/<robot_type>/override')(self.get_override)

    def _load_map_data(self):
        """Load map_data.yaml with caching"""
        map_data_path = self.config_dir / 'map_data.yaml'

        if not map_data_path.exists():
            return {}

        # Check if file changed
        mtime = map_data_path.stat().st_mtime
        if mtime != self._map_data_mtime:
            with open(map_data_path, 'r', encoding='utf-8') as f:
                self._map_data_cache = yaml.safe_load(f)
            self._map_data_mtime = mtime

        return self._map_data_cache or {}

    def _deep_merge(self, base: dict, override: dict) -> dict:
        """Deep merge two dictionaries"""
        result = copy.deepcopy(base)

        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._deep_merge(result[key], value)
            else:
                result[key] = copy.deepcopy(value)

        return result

    def list_robots(self):
        """
        List available robots from map_data.yaml

        Response:
            {
                "status": "success",
                "robots": ["lite6", "xarm5", "ur5e", ...],
                "count": 11
            }
        """
        map_data = self._load_map_data()
        robot_qr_codes = map_data.get('robot_qr_codes', {})

        # Extract unique robot types
        robot_types = set()
        for qr_info in robot_qr_codes.values():
            if 'robot_type' in qr_info:
                robot_types.add(qr_info['robot_type'])

        robots = sorted(list(robot_types))
        return jsonify({
            'status': 'success',
            'robots': robots,
            'count': len(robots)
        })

    def get_merged_config(self, robot_type: str):
        """
        Get merged robot config (_defaults.yaml + override)

        Args:
            robot_type: Robot type (e.g., 'lite6', 'xarm5')

        Response:
            Merged config as JSON
        """
        defaults_path = self.network_robots_dir / '_defaults.yaml'
        override_path = self.network_robots_dir / 'overrides' / f'{robot_type}.yaml'

        if not defaults_path.exists():
            return jsonify({'error': '_defaults.yaml not found'}), 404
        if not override_path.exists():
            return jsonify({'error': f'Override not found: {robot_type}.yaml'}), 404

        with open(defaults_path, 'r', encoding='utf-8') as f:
            defaults = yaml.safe_load(f)
        with open(override_path, 'r', encoding='utf-8') as f:
            override = yaml.safe_load(f)

        merged = self._deep_merge(defaults, override)
        return jsonify(merged)

    def get_defaults(self, robot_type: str):
        """
        Get _defaults.yaml

        Response:
            YAML content
        """
        filepath = self.network_robots_dir / '_defaults.yaml'
        if not filepath.exists():
            return jsonify({'error': '_defaults.yaml not found'}), 404

        content = filepath.read_text(encoding='utf-8')
        return Response(content, mimetype='text/yaml')

    def get_override(self, robot_type: str):
        """
        Get robot-specific override YAML

        Args:
            robot_type: Robot type

        Response:
            YAML content
        """
        filepath = self.network_robots_dir / 'overrides' / f'{robot_type}.yaml'
        if not filepath.exists():
            return jsonify({'error': f'Override not found: {robot_type}.yaml'}), 404

        content = filepath.read_text(encoding='utf-8')
        return Response(content, mimetype='text/yaml')
