"""
Multi-HRI Web Server
Flask server for web_panel + config API + tracking

Structure:
    - ConfigAPIHandler: Config file serving
    - RobotConfigAPIHandler: Robot config with merging
    - TrackingAPIHandler: Real-time tracking + ZMQ

Usage:
    python main.py

Docker:
    docker-compose up web-server
"""

import os
import yaml
from pathlib import Path
from flask import Flask, send_from_directory, redirect
from flask_socketio import SocketIO

from api.config_api import ConfigAPIHandler, RobotConfigAPIHandler
from api.tracking_api import TrackingAPIHandler
from api.ros_api import ROS2APIHandler
from routes_manager import RoutesManager


class WebServer:
    """
    Multi-HRI Web Server
    Class-based server with Dependency Injection
    """

    def __init__(self, shared_data_dir: Path = None):
        """
        Initialize Web Server

        Args:
            shared_data_dir: Path to shared_data directory
        """
        # Initialize RoutesManager singleton
        self.routes = RoutesManager.initialize(
            shared_data_dir=str(shared_data_dir) if shared_data_dir else None
        )

        # Get paths from RoutesManager
        self.shared_data_dir = self.routes.shared_data_dir
        self.config_dir = self.routes.config_dir
        self.web_panel_dir = self.routes.web_panel_dir
        self.network_robots_dir = self.routes.network_robots_dir
        self.management_dir = self.shared_data_dir.parent / 'management'

        print(f"[WebServer] Using RoutesManager for path management")

        # Flask app
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'multi-hri-web-2025'

        # Socket.IO
        self.socketio = SocketIO(self.app, cors_allowed_origins='*')

        # API Handlers (Dependency Injection)
        self.config_api = ConfigAPIHandler(
            config_dir=self.config_dir,
            network_robots_dir=self.network_robots_dir
        )

        self.robot_api = RobotConfigAPIHandler(
            config_dir=self.config_dir,
            network_robots_dir=self.network_robots_dir
        )

        self.tracking_api = TrackingAPIHandler(
            config_dir=self.config_dir,
            socketio=self.socketio
        )

        self.ros_api = ROS2APIHandler(
            config_dir=self.config_dir
        )

        # Register routes
        self._register_blueprints()
        self._register_static_routes()
        self._register_socketio_events()

    def _register_blueprints(self):
        """Register API Blueprints"""
        self.app.register_blueprint(self.config_api.blueprint, url_prefix='/api/config')
        self.app.register_blueprint(self.robot_api.blueprint, url_prefix='/api/robots')
        self.app.register_blueprint(self.tracking_api.blueprint, url_prefix='/api/tracking')
        self.app.register_blueprint(self.ros_api.blueprint, url_prefix='/api/ros2')
        self.app.register_blueprint(self.routes.blueprint, url_prefix='/api/routes')

        # Legacy routes for compatibility
        self._register_legacy_routes()

    def _register_legacy_routes(self):
        """Register legacy API routes for compatibility"""

        @self.app.route('/api/status')
        def api_status():
            return self.tracking_api.get_status()

        @self.app.route('/api/users')
        def api_users():
            return self.tracking_api.get_users()

        @self.app.route('/api/map_data')
        def api_map_data():
            self.tracking_api.load_map_data()
            return self.tracking_api._map_data

        @self.app.route('/api/tracking_data')
        def api_tracking_data():
            return self.tracking_api.get_tracking_data()

    def _register_static_routes(self):
        """Register static file routes from server_config.yaml (via RoutesManager)"""
        static_routes = self.routes.get_static_routes()
        index_redirects = self.routes.get_index_redirects()
        default_index = self.routes.server_config.get('default_index', 'index.html')

        print(f"[WebServer] Loading static routes from server_config.yaml")

        # Register index redirects
        for url_path, redirect_to in index_redirects.items():
            def make_redirect(target):
                def handler():
                    return redirect(target)
                return handler
            endpoint_name = f"redirect_{url_path.replace('/', '_')}"
            self.app.add_url_rule(url_path, endpoint_name, make_redirect(redirect_to))
            print(f"  Redirect: {url_path} -> {redirect_to}")

        # Register static routes dynamically
        for url_path, local_dir in static_routes.items():
            full_local_path = self.shared_data_dir / local_dir

            def make_handler(local_path, default_file):
                def handler(filename=default_file):
                    return send_from_directory(str(local_path), filename)
                return handler

            # Register with and without trailing slash
            endpoint_base = url_path.strip('/').replace('/', '_') or 'root'
            handler = make_handler(full_local_path, default_index)

            # Route with trailing slash (for index)
            self.app.add_url_rule(f"{url_path}/", f"{endpoint_base}_index", handler)
            # Route with filename
            self.app.add_url_rule(f"{url_path}/<path:filename>", f"{endpoint_base}_file", handler)

            print(f"  Static: {url_path}/ -> {local_dir}")

    def _register_socketio_events(self):
        """Register Socket.IO events"""

        @self.socketio.on('connect')
        def handle_connect():
            from datetime import datetime
            from flask_socketio import emit
            emit('connected', {
                'message': 'Connected to Multi-HRI Web Server',
                'timestamp': datetime.now().isoformat()
            })

        @self.socketio.on('request_map_update')
        def handle_map_update():
            from flask_socketio import emit
            self.tracking_api.load_map_data()
            emit('map_update', {
                'map_data': self.tracking_api._map_data,
                'tracking_data': self.tracking_api.tracking_data
            })

    def init_zmq(self, subscribe_port: int = 5565):
        """Initialize ZMQ for Unity integration"""
        success = self.tracking_api.init_zmq(subscribe_port)
        if success:
            self.tracking_api.start_zmq_listener()
        return success

    def run(self, host: str = '0.0.0.0', port: int = 5000, debug: bool = True):
        """
        Run the web server

        Args:
            host: Host to bind to
            port: Port to listen on
            debug: Enable debug mode
        """
        print(f"""
    ====================================
    Multi-HRI Web Server
    ====================================

    Endpoints:
    - Main Panel:       http://{host}:{port}/
    - Robot Control:    http://{host}:{port}/robot_control/
    - Settings:         http://{host}:{port}/settings/
    - Management:       http://{host}:{port}/management

    API:
    - /api/config/<file>
    - /api/robots/
    - /api/robots/<type>/config
    - /api/tracking/data
    - /api/status
    - /api/users

    ROS2 API (proxy to ROS2 server):
    - /api/ros2/moveit/<action>
    - /api/ros2/controller/<action>
    - /api/ros2/tcp_endpoint/<action>
    - /api/ros2/devices/scan
    - /api/ros2/foxglove/<action>
    - /api/ros2/pattern/*
    - /api/ros2/system/clean
        """)

        self.socketio.run(
            self.app,
            host=host,
            port=port,
            debug=debug,
            allow_unsafe_werkzeug=True
        )


def main():
    """Main entry point"""
    # Create server
    server = WebServer()

    # Load initial data
    server.tracking_api.load_map_data()
    server.tracking_api.load_tracking_data()

    # Initialize ZMQ (optional - for Unity integration)
    try:
        server.init_zmq()
    except Exception as e:
        print(f"[WebServer] ZMQ initialization skipped: {e}")

    # Run server
    server.run(debug=True)


if __name__ == '__main__':
    main()
