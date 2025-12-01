"""
Config Utilities
Shared config loading and merging utilities
Mirrors Unity's YamlMerger pattern for consistency
"""

import yaml
import copy
from pathlib import Path
from typing import Dict, Optional, Any, List


def deep_merge(base: dict, override: dict) -> dict:
    """
    Deep merge two dictionaries (Unity YamlMerger pattern)
    Override values take precedence, nested dicts are merged recursively
    Empty values (None, '', [], {}) in override don't overwrite base

    Args:
        base: Base dictionary (defaults)
        override: Override dictionary

    Returns:
        Merged dictionary
    """
    if not override:
        return copy.deepcopy(base) if base else {}
    if not base:
        return copy.deepcopy(override) if override else {}

    result = copy.deepcopy(base)
    for key, value in override.items():
        if value is None:
            continue
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        elif not _is_empty_value(value):
            result[key] = copy.deepcopy(value)
    return result


def _is_empty_value(value) -> bool:
    """Check if value is 'empty' (should not override)"""
    if value is None:
        return True
    if isinstance(value, str) and value == '':
        return True
    if isinstance(value, (list, dict)) and len(value) == 0:
        return True
    return False


class ConfigLoader:
    """
    Centralized config loader with deep merge support
    Mirrors Unity's YamlMerger + AppDataManager pattern
    """

    def __init__(self, base_path: str = '/app/config'):
        """
        Initialize ConfigLoader

        Args:
            base_path: Base path to config directory
        """
        self.base_path = Path(base_path)
        self._cache: Dict[str, Any] = {}

    def load_yaml(self, relative_path: str, use_cache: bool = False) -> Optional[dict]:
        """
        Load a single YAML file

        Args:
            relative_path: Path relative to base_path
            use_cache: Whether to cache the result

        Returns:
            Parsed YAML as dict, or None if not found
        """
        if use_cache and relative_path in self._cache:
            return self._cache[relative_path]

        full_path = self.base_path / relative_path
        try:
            with open(full_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                if use_cache:
                    self._cache[relative_path] = data
                return data
        except FileNotFoundError:
            print(f"[ConfigLoader] File not found: {full_path}")
            return None
        except Exception as e:
            print(f"[ConfigLoader] Error loading {full_path}: {e}")
            return None

    def load_with_defaults(self, defaults_path: str, override_path: str) -> dict:
        """
        Load config with _defaults.yaml + override pattern

        Args:
            defaults_path: Path to defaults file (relative)
            override_path: Path to override file (relative)

        Returns:
            Merged config dict
        """
        defaults = self.load_yaml(defaults_path) or {}
        override = self.load_yaml(override_path) or {}
        return deep_merge(defaults, override)

    def list_yaml_files(self, relative_dir: str, exclude_prefix: str = '_') -> List[Path]:
        """
        List all YAML files in a directory

        Args:
            relative_dir: Directory path relative to base_path
            exclude_prefix: Prefix to exclude (default: '_' for _defaults.yaml etc)

        Returns:
            List of Path objects
        """
        dir_path = self.base_path / relative_dir
        if not dir_path.exists():
            return []

        files = []
        for yaml_file in dir_path.glob('*.yaml'):
            if not yaml_file.name.startswith(exclude_prefix):
                files.append(yaml_file)
        return files

    def clear_cache(self):
        """Clear all cached configs"""
        self._cache.clear()
