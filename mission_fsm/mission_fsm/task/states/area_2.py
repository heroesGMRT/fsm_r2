"""Area 2 state for mission_fsm."""

from ..base_state import BaseState
from ...config.loader import AREA_GOALS

# ── Goal for Area 2 ────────────────────────────────────────────
# To change the target position, edit config/areas.yaml → area_2
# ──────────────────────────────────────────────────────────────
_GOAL = AREA_GOALS["area_2"]   # {x, y, yaw}


class Area2State(BaseState):
    """Handles all robot behaviour for Area 2.

    Sends the robot to the coordinates defined in config/areas.yaml
    under the ``area_2`` key, then waits for navigation to finish
    before transitioning to AREA_3.
    """

    def execute(self, node):
        node.get_logger().info(
            f"Area 2 → navigating to "
            f"x={_GOAL['x']}, y={_GOAL['y']}, yaw={_GOAL['yaw']}",
            once=True,
        )
        node.nav.send_goal([_GOAL["x"], _GOAL["y"], _GOAL["yaw"]])

    def check_transition(self, node):
        if node.area_complete:
            node.area_complete = False
            node.get_logger().info("Area 2 complete. Transitioning to AREA_3.")
            return "AREA_3"
        return None
