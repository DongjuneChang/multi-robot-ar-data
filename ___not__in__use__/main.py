"""
Multi-HRI Web Panel FastAPI Server
Serves web_panel static files and config API endpoints

Usage:
    uvicorn main:app --host 0.0.0.0 --port 5000 --reload

Docker:
    docker-compose up web-server
"""

from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, Response
from pathlib import Path
import yaml

# Paths - Docker 컨테이너 내부 경로
SHARED_DATA = Path("/app/shared_data")
CONFIG_DIR = SHARED_DATA / "config"
WEB_PANEL_DIR = SHARED_DATA / "web_panel"
NETWORK_ROBOTS_DIR = CONFIG_DIR / "network_robots"

app = FastAPI(
    title="Multi-HRI Web Panel Server",
    description="Serves web panel and config API for multi-robot HRI system",
    version="1.0.0"
)

# ==================== Config API ====================

@app.get("/api/config/{filename}")
async def get_config(filename: str):
    """Get config file (YAML)"""
    filepath = CONFIG_DIR / filename
    if not filepath.exists():
        raise HTTPException(status_code=404, detail=f"Config not found: {filename}")

    content = filepath.read_text(encoding="utf-8")
    return Response(content=content, media_type="text/yaml")


@app.get("/api/robots")
async def get_robots():
    """Get list of available robots from map_data.yaml"""
    map_data_path = CONFIG_DIR / "map_data.yaml"
    if not map_data_path.exists():
        raise HTTPException(status_code=404, detail="map_data.yaml not found")

    with open(map_data_path, "r", encoding="utf-8") as f:
        map_data = yaml.safe_load(f)

    # Extract unique robot types from robot_qr_codes
    robot_qr_codes = map_data.get("robot_qr_codes", {})
    robot_types = set()
    for qr_info in robot_qr_codes.values():
        if "robot_type" in qr_info:
            robot_types.add(qr_info["robot_type"])

    return {"robots": sorted(list(robot_types))}


@app.get("/api/robots/{robot_type}/config")
async def get_robot_config(robot_type: str):
    """Get merged robot config (_defaults.yaml + override)"""
    defaults_path = NETWORK_ROBOTS_DIR / "_defaults.yaml"
    override_path = NETWORK_ROBOTS_DIR / "overrides" / f"{robot_type}.yaml"

    if not defaults_path.exists():
        raise HTTPException(status_code=404, detail="_defaults.yaml not found")
    if not override_path.exists():
        raise HTTPException(status_code=404, detail=f"Override not found: {robot_type}.yaml")

    # Load and merge
    with open(defaults_path, "r", encoding="utf-8") as f:
        defaults = yaml.safe_load(f)
    with open(override_path, "r", encoding="utf-8") as f:
        override = yaml.safe_load(f)

    merged = deep_merge(defaults, override)
    return merged


@app.get("/api/robots/{robot_type}/defaults")
async def get_robot_defaults(robot_type: str):
    """Get _defaults.yaml"""
    filepath = NETWORK_ROBOTS_DIR / "_defaults.yaml"
    if not filepath.exists():
        raise HTTPException(status_code=404, detail="_defaults.yaml not found")

    content = filepath.read_text(encoding="utf-8")
    return Response(content=content, media_type="text/yaml")


@app.get("/api/robots/{robot_type}/override")
async def get_robot_override(robot_type: str):
    """Get robot-specific override YAML"""
    filepath = NETWORK_ROBOTS_DIR / "overrides" / f"{robot_type}.yaml"
    if not filepath.exists():
        raise HTTPException(status_code=404, detail=f"Override not found: {robot_type}.yaml")

    content = filepath.read_text(encoding="utf-8")
    return Response(content=content, media_type="text/yaml")


# ==================== Users API ====================

@app.get("/api/users")
async def get_users():
    """Get registered users from map_data.yaml"""
    map_data_path = CONFIG_DIR / "map_data.yaml"
    if not map_data_path.exists():
        raise HTTPException(status_code=404, detail="map_data.yaml not found")

    with open(map_data_path, "r", encoding="utf-8") as f:
        map_data = yaml.safe_load(f)

    return {"users": map_data.get("registered_users", {})}


# ==================== Status API ====================

@app.get("/api/status")
async def get_status():
    """Get system status (placeholder)"""
    return {
        "status": "ok",
        "ros2": False,
        "moveit": False,
        "controller": False,
        "tcp_endpoint": False,
        "device_api": False,
        "pattern_service": False,
        "test_running": False
    }


# ==================== Static Files ====================

# Mount web_panel as static files (after API routes)
app.mount("/", StaticFiles(directory=str(WEB_PANEL_DIR), html=True), name="static")


# ==================== Utilities ====================

def deep_merge(base: dict, override: dict) -> dict:
    """Deep merge two dictionaries"""
    import copy
    result = copy.deepcopy(base)

    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)

    return result


# ==================== Development ====================

if __name__ == "__main__":
    import uvicorn

    # 로컬 개발용 경로 재설정
    import os
    if os.path.exists("../../config"):
        global SHARED_DATA, CONFIG_DIR, WEB_PANEL_DIR, NETWORK_ROBOTS_DIR
        SHARED_DATA = Path("../..").resolve()
        CONFIG_DIR = SHARED_DATA / "config"
        WEB_PANEL_DIR = SHARED_DATA / "web_panel"
        NETWORK_ROBOTS_DIR = CONFIG_DIR / "network_robots"

    uvicorn.run(app, host="0.0.0.0", port=5000, reload=True)
