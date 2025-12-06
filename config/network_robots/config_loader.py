#!/usr/bin/env python3
"""
Network Robot Config Loader
병합 로더: _defaults.yaml + overrides/{robot}.yaml → 완전한 config

Usage:
    from config_loader import load_robot_config
    config = load_robot_config("lite6")
"""

import os
import copy
import math
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """
    Deep merge override into base.
    override의 값이 base를 덮어씀. nested dict는 재귀적으로 병합.
    """
    result = copy.deepcopy(base)

    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)

    return result


def get_config_dir() -> Path:
    """config 디렉토리 경로 반환"""
    return Path(__file__).parent


def load_yaml(file_path: Path) -> Dict[str, Any]:
    """YAML 파일 로드"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def load_robot_config(robot_model: str, convert_to_radians: bool = False) -> Dict[str, Any]:
    """
    로봇 설정 로드 (_defaults + override 병합)

    Args:
        robot_model: 로봇 모델명 (예: "lite6", "xarm5", "fr3")
        convert_to_radians: True면 joint angles를 radians로 변환

    Returns:
        병합된 완전한 config dict
    """
    config_dir = get_config_dir()

    # Load defaults
    defaults_path = config_dir / "_defaults.yaml"
    if not defaults_path.exists():
        raise FileNotFoundError(f"Defaults file not found: {defaults_path}")
    defaults = load_yaml(defaults_path)

    # Load override
    override_path = config_dir / "overrides" / f"{robot_model}.yaml"
    if not override_path.exists():
        raise FileNotFoundError(f"Override file not found: {override_path}")
    override = load_yaml(override_path)

    # Merge
    config = deep_merge(defaults, override)

    # Convert degrees to radians if requested
    if convert_to_radians:
        config = convert_joints_to_radians(config)

    return config


def convert_joints_to_radians(config: Dict[str, Any]) -> Dict[str, Any]:
    """
    joints.zero, joints.home, joints.limits를 radians로 변환
    """
    result = copy.deepcopy(config)

    if 'joints' in result:
        joints = result['joints']

        # Convert zero
        if 'zero' in joints and joints['zero']:
            joints['zero'] = [math.radians(deg) for deg in joints['zero']]

        # Convert home
        if 'home' in joints and joints['home']:
            joints['home'] = [math.radians(deg) for deg in joints['home']]

        # Convert limits
        if 'limits' in joints:
            for joint_name, limits in joints['limits'].items():
                joints['limits'][joint_name] = [math.radians(deg) for deg in limits]

    return result


def get_available_robots() -> list:
    """사용 가능한 로봇 목록 반환"""
    config_dir = get_config_dir()
    overrides_dir = config_dir / "overrides"

    if not overrides_dir.exists():
        return []

    return [f.stem for f in overrides_dir.glob("*.yaml")]


def load_named_frames(robot_model: str) -> Dict[str, Any]:
    """
    named_frames 섹션만 추출 (FrameManager용)
    """
    config = load_robot_config(robot_model, convert_to_radians=True)
    return config.get('named_frames', {})


def load_admittance_config(robot_model: str) -> Dict[str, Any]:
    """
    admittance 섹션만 추출
    """
    config = load_robot_config(robot_model)
    return config.get('admittance', {})


# CLI test
if __name__ == "__main__":
    import sys
    import json

    if len(sys.argv) < 2:
        print("Usage: python config_loader.py <robot_model> [--radians]")
        print(f"Available robots: {get_available_robots()}")
        sys.exit(1)

    robot_model = sys.argv[1]
    convert_radians = "--radians" in sys.argv

    try:
        config = load_robot_config(robot_model, convert_to_radians=convert_radians)
        print(f"=== {robot_model} config ===")
        print(yaml.dump(config, default_flow_style=False, allow_unicode=True))
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
