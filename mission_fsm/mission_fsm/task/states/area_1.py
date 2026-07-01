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
    """Handles all robot behaviour for Area 1 (spearhead visual servo + green detection).

    Navigates to the goal in config/areas.yaml, then kicks off two
    sub-tasks **in sequence**:
      1. Spearhead / visual-servo align (via visual executor)
      2. Green detection (via green_detection_node)
    """

    def execute(self, node):
        node.get_logger().info(
            f"Area 1 → Mundur  "
            f"x={_GOAL['x']}, y={_GOAL['y']}, yaw={_GOAL['yaw']}",
            once=True,
        )
        node.nav.send_goal([_GOAL["x"], _GOAL["y"], _GOAL["yaw"]])

        # ── Step 1: Fire visual-servo when nav arrives ────────────────
        if not node.align_triggered and node.nav.is_goal_done():
            if node.nav.status == GoalStatus.STATUS_SUCCEEDED:
                self._start_align(node)
            else:
                node.get_logger().error(
                    "Area 1: nav did not succeed reaching the spearhead "
                    "vantage point yet, will retry once it does.",
                    throttle_duration_sec=2.0,
                )

        # ── Step 2: Start green detection after align is done ────────
        if (
            node.align_triggered
            and not getattr(node, "green_detection_triggered", False)
            and node.nav.is_goal_done()
        ):
            self._start_green_detection(node)

    def _start_align(self, node):
        msg = String()
        msg.data = json.dumps({"command": "start", "area": "AREA_1"})
        node.area_cmd_pub.publish(msg)
        node.get_logger().info("Area 1: sent visual-servo start command.")
        node.align_triggered = True
        
    def execute(self, node):
        node.get_logger().info(
            f"Area 1 → Mundur  "
            f"x={_GOAL['x']}, y={_GOAL['y']}, yaw={_GOAL['yaw']}",
            once=True,
        )
        node.nav.send_goal([0.3, -3.94351, -1.54422])

    def _start_green_detection(self, node):
        """Publish start command to green detection node.

        Starts a dedicated green detection node that subscribes to
        /camera/image_raw, runs colour / YOLO detection for green
        objects, and publishes results back to the FSM.
        """
        msg = String()
        msg.data = json.dumps({
            "command": "start",
            "area": "AREA_1",
            "task": "green_detection",
        })
        node.area_cmd_pub.publish(msg)
        node.get_logger().info("Area 1: sent green-detection start command.")
        node.green_detection_triggered = True

    def check_transition(self, node):
        if node.area_complete:
            node.area_complete = False
            node.get_logger().info("Area 1 complete. Transitioning to AREA_2.")
            return "AREA_2"
        return None