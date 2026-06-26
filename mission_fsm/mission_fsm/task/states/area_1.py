"""Area 1 state for mission_fsm."""

import json

from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from ..base_state import BaseState
from ...config.loader import AREA_GOALS

# ── Goal for Area 1 ────────────────────────────────────────────
# To change the target position, edit config/areas.yaml → area_1
# ──────────────────────────────────────────────────────────────
_GOAL = AREA_GOALS["area_1"]   # {x, y, yaw}


class Area1State(BaseState):
    """Handles all robot behaviour for Area 1 (spearhead visual servo).

    Sends the robot to the coordinates defined in config/areas.yaml
    under the ``area_1`` key. Once nav reports arrival, publishes a
    one-shot start command to the vision executor node over
    /fsm/area_command, then -- like every other area -- just waits for
    /fsm/signal -> area_complete before transitioning to AREA_2.

    This state does NOT run align() itself: SpearheadVisualServo.align()
    blocks for several seconds (camera I/O, YOLO inference, motion
    waits) and must never run inside this FSM's 10Hz tick loop. That
    work lives in vision_executor_node.py instead.
    """

    def execute(self, node):
        node.get_logger().info(
            f"Area 1 → navigating to "
            f"x={_GOAL['x']}, y={_GOAL['y']}, yaw={_GOAL['yaw']}",
            once=True,
        )
        node.nav.send_goal([_GOAL["x"], _GOAL["y"], _GOAL["yaw"]])

        # Fire the visual-servo task exactly once, as soon as nav arrives.
        # node.align_triggered is reset by trigger_start/trigger_reset/
        # trigger_retry_area(1) so RETRY re-arms this.
        if not node.align_triggered and node.nav.is_goal_done():
            if node.nav.status == GoalStatus.STATUS_SUCCEEDED:
                self._start_align(node)
            else:
                node.get_logger().error(
                    "Area 1: nav did not succeed reaching the spearhead "
                    "vantage point yet, will retry once it does.",
                    throttle_duration_sec=2.0,
                )

    def _start_align(self, node):
        msg = String()
        msg.data = json.dumps({"command": "start", "area": "AREA_1"})
        node.area_cmd_pub.publish(msg)
        node.get_logger().info("Area 1: sent visual-servo start command.")
        node.align_triggered = True

    def check_transition(self, node):
        if node.area_complete:
            node.area_complete = False
            node.get_logger().info("Area 1 complete. Transitioning to AREA_2.")
            return "AREA_2"
        return None