"""
RoutesManager - Singleton for centralized path management

모든 경로는 이 싱글톤을 통해 관리됨:
- 환경변수 또는 설정에서 주입
- 하드코딩된 상대경로 금지
- 프론트엔드에 API로 경로 정보 제공

Usage:
    from routes_manager import RoutesManager

    # 초기화 (한 번만)
    RoutesManager.initialize(shared_data_dir="/app/shared_data")

    # 사용
    routes = RoutesManager.instance()
    config_path = routes.config_dir
    robots = routes.get_api_paths()
"""

import os
import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from flask import Blueprint, jsonify


class RoutesManager:
    """
    Singleton for centralized path and route management

    Principles:
    - No hardcoded relative paths (../../)
    - All paths from environment or config
    - Frontend gets paths via API
    """

    _instance: Optional['RoutesManager'] = None
    _initialized: bool = False

    def __init__(self):
        if RoutesManager._initialized:
            raise RuntimeError("Use RoutesManager.instance() instead of constructor")

        # Will be set by initialize()
        self.shared_data_dir: Optional[Path] = None
        self.config_dir: Optional[Path] = None
        self.web_panel_dir: Optional[Path] = None
        self.server_config: Dict[str, Any] = {}

        # Blueprint for API routes
        self.blueprint = Blueprint('routes', __name__)
        self._register_api_routes()

    @classmethod
    def initialize(cls, shared_data_dir: str = None) -> 'RoutesManager':
        """
        Initialize the singleton with paths

        Args:
            shared_data_dir: Path to shared_data directory.
                            If None, auto-detect from environment.
        """
        if cls._instance is None:
            cls._instance = cls.__new__(cls)
            cls._instance.__init__()

        instance = cls._instance

        # Determine shared_data path
        if shared_data_dir:
            instance.shared_data_dir = Path(shared_data_dir)
        elif os.environ.get('SHARED_DATA_DIR'):
            instance.shared_data_dir = Path(os.environ['SHARED_DATA_DIR'])
        elif os.environ.get('DOCKER_ENV'):
            instance.shared_data_dir = Path('/app/shared_data')
        else:
            # Local development - relative to server/
            instance.shared_data_dir = Path(__file__).parent.parent

        # Derived paths
        instance.config_dir = instance.shared_data_dir / 'config'
        instance.web_panel_dir = instance.shared_data_dir / 'web_panel'

        # Load server config
        instance._load_server_config()

        cls._initialized = True

        print(f"[RoutesManager] Initialized:")
        print(f"  shared_data_dir: {instance.shared_data_dir}")
        print(f"  config_dir: {instance.config_dir}")
        print(f"  web_panel_dir: {instance.web_panel_dir}")

        return instance

    @classmethod
    def instance(cls) -> 'RoutesManager':
        """Get the singleton instance"""
        if cls._instance is None or not cls._initialized:
            raise RuntimeError("RoutesManager not initialized. Call initialize() first.")
        return cls._instance

    def _load_server_config(self):
        """Load server_config.yaml"""
        # Try web_panel location first, then config
        config_paths = [
            self.web_panel_dir / 'server_config.yaml',
            self.config_dir / 'server_config.yaml',
        ]

        for config_path in config_paths:
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    self.server_config = yaml.safe_load(f) or {}
                print(f"[RoutesManager] Loaded config from: {config_path}")
                return

        print("[RoutesManager] Warning: No server_config.yaml found")
        self.server_config = {}

    # =========================================================================
    # Path Getters (for server-side use)
    # =========================================================================

    @property
    def network_robots_dir(self) -> Path:
        """Path to network_robots config directory"""
        return self.config_dir / 'network_robots'

    @property
    def ros2_interface_dir(self) -> Path:
        """Path to ros2_interface config directory"""
        return self.config_dir / 'ros2_interface'

    @property
    def unit_patterns_dir(self) -> Path:
        """Path to unit_patterns config directory"""
        return self.config_dir / 'unit_patterns'

    def get_robot_config_path(self, robot_type: str) -> Path:
        """Get path to specific robot's merged config"""
        return self.network_robots_dir / 'overrides' / f'{robot_type}.yaml'

    # =========================================================================
    # API Paths (for frontend use)
    # =========================================================================

    def get_api_paths(self) -> Dict[str, str]:
        """
        Get API paths for frontend to use

        Returns dict like:
        {
            "config": "/api/config",
            "robots": "/api/robots",
            "ros2_interface": "/api/config/ros2_interface",
            "patterns": "/api/config/unit_patterns",
            ...
        }
        """
        return {
            # Config API paths
            "config": "/api/config",
            "robots": "/api/robots",
            "ros2_interface": "/api/config/ros2_interface",
            "patterns": "/api/config/unit_patterns",

            # Specific endpoints
            "robot_list": "/api/robots/",
            "robot_config": "/api/robots/{robot_type}/config",
            "robot_defaults": "/api/robots/{robot_type}/defaults",

            # ROS2 API
            "ros2": "/api/ros2",
            "moveit": "/api/ros2/moveit",
            "controller": "/api/ros2/controller",
            "foxglove": "/api/ros2/foxglove",

            # Tracking
            "tracking": "/api/tracking",
            "status": "/api/status",
        }

    def get_static_routes(self) -> Dict[str, str]:
        """Get static routes from server_config.yaml"""
        return self.server_config.get('static_routes', {})

    def get_index_redirects(self) -> Dict[str, str]:
        """Get index redirects from server_config.yaml"""
        return self.server_config.get('index_redirects', {})

    # =========================================================================
    # Flask Blueprint Routes
    # =========================================================================

    def _register_api_routes(self):
        """Register API routes for frontend path discovery"""

        @self.blueprint.route('/paths')
        def get_paths():
            """
            GET /api/routes/paths

            Returns all API paths for frontend to use.
            Frontend should call this once on load and cache.
            """
            return jsonify({
                "api_paths": self.get_api_paths(),
                "static_routes": self.get_static_routes(),
                "shared_data_dir": str(self.shared_data_dir),
            })

        @self.blueprint.route('/config')
        def get_config():
            """
            GET /api/routes/config

            Returns server configuration
            """
            return jsonify(self.server_config)
