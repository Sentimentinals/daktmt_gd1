from __future__ import annotations

from .config import Config
from .stair_main import run_terrain_auto


def run_terrain(
    args: Config,
    dashboard,
    camera,
    camera_ready: bool,
    backend,
    sensor_hub,
    fall_safety,
) -> None:
    run_terrain_auto(
        args,
        dashboard,
        camera,
        camera_ready,
        backend,
        sensor_hub,
        fall_safety,
    )
