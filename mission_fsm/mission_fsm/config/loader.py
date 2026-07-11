"""Config loader for mission_fsm.

NOTE: this file is a clean rebuild, not a patch of your original
loader.py (which wasn't available when this was written). It was
reconstructed purely from how it's used elsewhere in the package:
  - area_1.py / area_2.py / area_3.py do  AREA_GOALS["area_1"]  etc.
  - the old dashboard did  open(_CONFIG_PATH, ...)
If your real loader.py does anything else, send it over and this
should be merged rather than replacing it outright.

areas.yaml now holds multiple named configs (config_1, config_2, ...),
each with its own area_1/area_2/area_3. AREA_GOALS always reflects
whichever config is currently active -- the area state modules don't
need to know configs exist at all; they keep reading
AREA_GOALS["area_1"] exactly as before.
"""

from pathlib import Path

import yaml

_CONFIG_PATH = Path(__file__).parent / "areas.yaml"
_REQUIRED_AREAS = ("area_1", "area_2", "area_3")

# Keys each area block must define. Only Area 1's move params are validated at
# key level (a missing one is a startup error rather than a silent
# wrong-distance move at runtime).
_REQUIRED_KEYS = {
    "area_1": ("move_x", "move_y", "prox_forward_x"),
}


def _load_raw() -> dict:
    with open(_CONFIG_PATH, "r") as f:
        return yaml.safe_load(f) or {}


def _validate(name: str, goals: dict) -> None:
    missing = [a for a in _REQUIRED_AREAS if a not in goals]
    if missing:
        raise ValueError(f"Config '{name}' is missing areas: {missing}")
    for area, keys in _REQUIRED_KEYS.items():
        block = goals.get(area) or {}
        missing_keys = [k for k in keys if k not in block]
        if missing_keys:
            raise ValueError(
                f"Config '{name}' area '{area}' is missing keys: {missing_keys}"
            )


_raw = _load_raw()
CONFIGS: dict = _raw.get("configs", {})

if not CONFIGS:
    raise ValueError(
        f"No 'configs:' section found in {_CONFIG_PATH}. "
        f"Expected configs: {{config_1: {{area_1: {{x, y, yaw}}, ...}}, ...}}"
    )
for _name, _goals in CONFIGS.items():
    _validate(_name, _goals)

ACTIVE_CONFIG: str = _raw.get("active_config") or next(iter(CONFIGS))
if ACTIVE_CONFIG not in CONFIGS:
    raise ValueError(
        f"active_config '{ACTIVE_CONFIG}' not found in configs: {list(CONFIGS)}"
    )

# Public dict read by area_1.py / area_2.py / area_3.py via
# AREA_GOALS["area_1"], etc. Mutated in place (never rebound) by
# set_active_config() so any cached reference to this exact dict object
# stays valid.
AREA_GOALS: dict = dict(CONFIGS[ACTIVE_CONFIG])


def available_configs() -> list:
    """Names of all configs defined in areas.yaml, in file order."""
    return list(CONFIGS.keys())


def update_active(area: str, key: str, value) -> None:
    """Override a single value in the currently active config.

    In-memory only -- does NOT rewrite areas.yaml, so edits are lost on
    restart. The change persists across a set_active_config() away-and-back
    because it's written into CONFIGS[ACTIVE_CONFIG] as well as AREA_GOALS.
    """
    AREA_GOALS.setdefault(area, {})[key] = value
    CONFIGS[ACTIVE_CONFIG].setdefault(area, {})[key] = value


def set_active_config(name: str) -> dict:
    """Switch AREA_GOALS to a different config by name (in-memory only;
    does not rewrite areas.yaml). Returns the new AREA_GOALS dict.

    Raises KeyError if `name` isn't defined in areas.yaml.
    """
    global ACTIVE_CONFIG
    if name not in CONFIGS:
        raise KeyError(f"Unknown config '{name}'. Available: {list(CONFIGS)}")
    ACTIVE_CONFIG = name
    AREA_GOALS.clear()
    AREA_GOALS.update(CONFIGS[name])
    return AREA_GOALS