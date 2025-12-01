"""
Projects API Blueprint
CRUD operations for project YAML files in Projects folder

Projects are stored in the projects directory (mounted via docker-compose)
Each project is a YAML file that extends _defaults.yaml
"""

from flask import Blueprint, jsonify, request
from pathlib import Path
import yaml
import os
from datetime import datetime


class ProjectsAPIHandler:
    """
    Projects API Handler
    Manages project YAML files for Multi-HRI sessions
    """

    def __init__(self, projects_dir: Path = None, config_dir: Path = None):
        """
        Initialize Projects API Handler

        Args:
            projects_dir: Path to projects directory (from docker mount or env)
            config_dir: Path to config directory (for _defaults.yaml)
        """
        # Projects directory (external mount)
        if projects_dir:
            self.projects_dir = Path(projects_dir)
        else:
            # Try environment variable first, then fallback
            env_path = os.environ.get('PROJECTS_DIR')
            if env_path:
                self.projects_dir = Path(env_path)
            else:
                # Fallback to config/projects
                self.projects_dir = config_dir / 'projects' if config_dir else Path('/app/projects')

        self.config_dir = config_dir or Path('/app/shared_data/config')

        # Ensure directory exists
        self.projects_dir.mkdir(parents=True, exist_ok=True)

        print(f"[ProjectsAPI] Projects directory: {self.projects_dir}")

        # Create Blueprint
        self.blueprint = Blueprint('projects_api', __name__)
        self._register_routes()

    def _register_routes(self):
        """Register API routes"""
        # List / Create
        self.blueprint.route('/', methods=['GET'])(self.list_projects)
        self.blueprint.route('/', methods=['POST'])(self.create_project)

        # CRUD by ID
        self.blueprint.route('/<project_id>', methods=['GET'])(self.get_project)
        self.blueprint.route('/<project_id>', methods=['PUT'])(self.update_project)
        self.blueprint.route('/<project_id>', methods=['DELETE'])(self.delete_project)

        # Actions
        self.blueprint.route('/<project_id>/activate', methods=['POST'])(self.activate_project)
        self.blueprint.route('/<project_id>/join', methods=['POST'])(self.join_project)

    def _load_defaults(self) -> dict:
        """Load project defaults from _defaults.yaml"""
        defaults_path = self.config_dir / 'projects' / '_defaults.yaml'
        if defaults_path.exists():
            with open(defaults_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
        return {}

    def _load_project(self, project_id: str) -> dict:
        """
        Load a project YAML file with defaults merged

        Args:
            project_id: Project ID (filename without .yaml)

        Returns:
            Project dict with defaults merged
        """
        project_path = self.projects_dir / f"{project_id}.yaml"
        if not project_path.exists():
            return None

        with open(project_path, 'r', encoding='utf-8') as f:
            project = yaml.safe_load(f) or {}

        # Merge with defaults if extends is specified
        if project.get('extends'):
            defaults = self._load_defaults()
            merged = {**defaults, **project}
            # Deep merge for nested dicts
            for key in ['server', 'fusion2', 'permissions']:
                if key in defaults and key in project:
                    merged[key] = {**defaults.get(key, {}), **project.get(key, {})}
            return merged

        return project

    def _save_project(self, project_id: str, data: dict) -> bool:
        """
        Save a project to YAML file

        Args:
            project_id: Project ID
            data: Project data

        Returns:
            True if saved successfully
        """
        project_path = self.projects_dir / f"{project_id}.yaml"

        # Add metadata
        data['id'] = project_id
        if 'created_at' not in data:
            data['created_at'] = datetime.now().isoformat()
        data['updated_at'] = datetime.now().isoformat()

        with open(project_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

        return True

    # ==================== API Endpoints ====================

    def list_projects(self):
        """
        List all projects, optionally filtered by user

        Query params:
            user: Filter by user (server_host or participant)
        """
        user_filter = request.args.get('user')

        projects = []
        for yaml_file in self.projects_dir.glob('*.yaml'):
            if yaml_file.name.startswith('_'):
                continue  # Skip _defaults.yaml

            project_id = yaml_file.stem
            project = self._load_project(project_id)
            if not project:
                continue

            # Filter by user if specified
            if user_filter:
                is_host = project.get('server_host') == user_filter
                is_participant = user_filter in [p.get('user_id') for p in project.get('participants', [])]
                if not (is_host or is_participant):
                    continue

            projects.append({
                'id': project_id,
                'name': project.get('name', project_id),
                'description': project.get('description', ''),
                'server_host': project.get('server_host'),
                'server': project.get('server', {}),
                'robots': project.get('robots', []),
                'status': project.get('status', 'inactive'),
                'created_by': project.get('created_by'),
                'created_at': project.get('created_at')
            })

        return jsonify({
            'status': 'success',
            'projects': projects,
            'count': len(projects)
        })

    def get_project(self, project_id: str):
        """Get a specific project by ID"""
        project = self._load_project(project_id)
        if not project:
            return jsonify({
                'status': 'error',
                'error': f'Project not found: {project_id}'
            }), 404

        return jsonify({
            'status': 'success',
            'project': project
        })

    def create_project(self):
        """Create a new project"""
        data = request.get_json()
        if not data:
            return jsonify({
                'status': 'error',
                'error': 'No data provided'
            }), 400

        # Generate project ID from name if not provided
        project_id = data.get('id')
        if not project_id:
            name = data.get('name', 'untitled')
            project_id = name.lower().replace(' ', '_').replace('-', '_')

        # Check if already exists
        if (self.projects_dir / f"{project_id}.yaml").exists():
            return jsonify({
                'status': 'error',
                'error': f'Project already exists: {project_id}'
            }), 409

        # Add extends for inheritance
        data['extends'] = '_defaults.yaml'

        # Save
        self._save_project(project_id, data)

        return jsonify({
            'status': 'success',
            'project_id': project_id,
            'message': f'Project created: {project_id}'
        }), 201

    def update_project(self, project_id: str):
        """Update an existing project"""
        if not (self.projects_dir / f"{project_id}.yaml").exists():
            return jsonify({
                'status': 'error',
                'error': f'Project not found: {project_id}'
            }), 404

        data = request.get_json()
        if not data:
            return jsonify({
                'status': 'error',
                'error': 'No data provided'
            }), 400

        # Load existing and merge
        existing = self._load_project(project_id) or {}
        merged = {**existing, **data}

        self._save_project(project_id, merged)

        return jsonify({
            'status': 'success',
            'project_id': project_id,
            'message': f'Project updated: {project_id}'
        })

    def delete_project(self, project_id: str):
        """Delete a project"""
        project_path = self.projects_dir / f"{project_id}.yaml"
        if not project_path.exists():
            return jsonify({
                'status': 'error',
                'error': f'Project not found: {project_id}'
            }), 404

        project_path.unlink()

        return jsonify({
            'status': 'success',
            'message': f'Project deleted: {project_id}'
        })

    def activate_project(self, project_id: str):
        """Activate a project (set status to active)"""
        project = self._load_project(project_id)
        if not project:
            return jsonify({
                'status': 'error',
                'error': f'Project not found: {project_id}'
            }), 404

        project['status'] = 'active'
        self._save_project(project_id, project)

        return jsonify({
            'status': 'success',
            'project_id': project_id,
            'message': f'Project activated: {project_id}'
        })

    def join_project(self, project_id: str):
        """Join a project as participant"""
        project = self._load_project(project_id)
        if not project:
            return jsonify({
                'status': 'error',
                'error': f'Project not found: {project_id}'
            }), 404

        data = request.get_json() or {}
        user_id = data.get('user_id')
        if not user_id:
            return jsonify({
                'status': 'error',
                'error': 'user_id required'
            }), 400

        # Add to participants if not already
        participants = project.get('participants', [])
        if not any(p.get('user_id') == user_id for p in participants):
            participants.append({
                'user_id': user_id,
                'role': 'guest',
                'joined_at': datetime.now().isoformat()
            })
            project['participants'] = participants
            self._save_project(project_id, project)

        return jsonify({
            'status': 'success',
            'project_id': project_id,
            'message': f'User {user_id} joined project'
        })
