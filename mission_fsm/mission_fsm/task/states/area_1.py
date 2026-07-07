"""Area 1 state for mission_fsm."""

import json
import math

from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from ..base_state import BaseState
from ...config.loader import AREA_GOALS

# ── Goal for Area 1 ────────────────────────────────────────────
# To change the target position, edit config/areas.yaml → area_1
# ──────────────────────────────────────────────────────────────
_GOAL = AREA_GOALS["area_1"]   # {x, y, yaw}

# After visual servo: move back 0.5 m directly (reverse without rotating yaw)
_BACK_OFFSET = 0.5   # metres to reverse
_POST_ALIGN_GOAL = {
    "x":   _GOAL["x"] - _BACK_OFFSET * math.cos(_GOAL["yaw"]),
    "y":   _GOAL["y"] - _BACK_OFFSET * math.sin(_GOAL["yaw"]),
    "yaw": _GOAL["yaw"],   # keep the same heading
}


class Area1State(BaseState):
    """Handles all robot behaviour for Area 1 (spearhead visual servo + green detection).

    Navigates to the goal in config/areas.yaml, then executes three
    sub-tasks **in sequence**:
      1. Spearhead / visual-servo align (via visual executor)
      2. Move back and rotate 180 ° (post-align repositioning)
      3. Green detection (via green_detection_node)
    """

    def execute(self, node):
        node.get_logger().info(
            f"Area 1 → Navigate  "
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

        # ── Step 2: After visual servo complete → move back + rotate 180° ─
        # Wait for align_complete (set when /fsm/signal→area_complete arrives
        # from spearhead_vision_node), NOT just nav.is_goal_done() which is
        # still True from the initial nav goal at this point.
        if (
            node.align_triggered
            and getattr(node, "align_complete", False)
            and not getattr(node, "post_align_nav_triggered", False)
        ):
            self._start_post_align_nav(node)

        # ── Step 3: Start green detection after repositioning done ────
        if (
            getattr(node, "post_align_nav_triggered", False)
            and not getattr(node, "green_detection_triggered", False)
            and node.nav.is_goal_done()
            and node.nav.status == GoalStatus.STATUS_SUCCEEDED
        ):
            self._start_green_detection(node)

    def _start_align(self, node):
        msg = String()
        msg.data = json.dumps({"command": "start", "area": "AREA_1"})
        node.area_cmd_pub.publish(msg)
        node.get_logger().info("Area 1: sent visual-servo start command.")
        node.align_triggered = True

    def _start_post_align_nav(self, node):
        """Cancel any completed goal tracking so nav_interface sends the new goal,
        then navigate back directly (reversing without rotating)."""
        # Reset nav state so the interface doesn't skip the new goal as a duplicate
        node.nav.completed_goal = None
        node.nav.status = GoalStatus.STATUS_UNKNOWN

        node.get_logger().info(
            f"Area 1: visual servo done → moving back directly  "
            f"x={_POST_ALIGN_GOAL['x']:.3f}, y={_POST_ALIGN_GOAL['y']:.3f}, "
            f"yaw={_POST_ALIGN_GOAL['yaw']:.4f} rad"
        )
        node.nav.send_goal(
            [_POST_ALIGN_GOAL["x"], _POST_ALIGN_GOAL["y"], _POST_ALIGN_GOAL["yaw"]]
        )
        node.post_align_nav_triggered = True

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
        # area_complete fires twice in this state:
        #   1st time → visual servo done  (captured as align_complete)
        #   2nd time → green detection done (triggers AREA_2 transition)
        if node.area_complete:
            node.area_complete = False
            if not getattr(node, "align_complete", False):
                # First area_complete: visual servo finished
                node.align_complete = True
                node.get_logger().info(
                    "Area 1: visual servo align complete — will reposition then start green detection."
                )
                return None  # stay in AREA_1
            # Second area_complete: green detection finished
            node.get_logger().info("Area 1 complete. Transitioning to AREA_2.")
            return "AREA_2"
        return None