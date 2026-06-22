"""Config loader for mission_fsm.

Reads areas.yaml from the same directory and exposes
per-area goal coordinates as a plain dict:

    from .config.loader import AREA_GOALS

    goal = AREA_GOALS["area_1"]   # {"x": 0.0, "y": 0.0, "yaw": 0.0}
"""

import os
import yaml

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "areas.yaml")


def _load() -> dict:
    with open(_CONFIG_PATH, "r") as f:
        data = yaml.safe_load(f) or {}
    return data


# Loaded once at import time; restart the node to pick up edits.
AREA_GOALS: dict = _load()
