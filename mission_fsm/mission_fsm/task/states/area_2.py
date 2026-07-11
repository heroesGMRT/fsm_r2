"""Area 2 state for mission_fsm."""

import json

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

from ..base_state import BaseState
from ...config.loader import AREA_GOALS

# ── Goal for Area 2 ────────────────────────────────────────────
# To change the target position, edit config/areas.yaml → area_2
# ──────────────────────────────────────────────────────────────
_GOAL = AREA_GOALS["area_2"]   # {x, y, yaw}


class Area2State(BaseState):
    """Handles all robot behaviour for Area 2 (the Forest task).

    On a normal mission entry, the state goes directly to nav.

    On a RETRY (triggered by the dashboard Retry button):
      Phase R0: publish /relative_move (values from areas.yaml
                area_2.retry_move_x/y/wait_sec) and wait for the
                timer to expire.  This lets the robot reposition
                itself before nav takes over.
      Phase R1 onwards: same as the normal mission path.

    Sends the robot to the coordinates defined in config/areas.yaml
    under the ``area_2`` key.  Once nav reports it has arrived, this
    state publishes the operator-entered Forest block state (set via
    node.set_forest_state(...) from the dashboard) to the Forest
    executor node over /fsm/area_command, then -- exactly like every
    other area -- just waits for /fsm/signal -> area_complete.

    This state does NOT plan or run the rotate/climb/pick sequence
    itself; that lives in the separate Forest executor node, which is
    expected to publish "area_complete" on /fsm/signal when its own
    action queue finishes.
    """

    def execute(self, node):
        # ── PHASE R0: Retry-only pre-nav /relative_move ────────────────────
        # This phase is only active when trigger_retry_area(2) has been
        # called (it sets area2_retry_move_triggered = False on the node).
        # During a normal mission the attribute is absent, so we skip it.
        if hasattr(node, "area2_retry_move_triggered"):
            if not node.area2_retry_move_triggered:
                # First tick of retry: fire the move and start the timer.
                node.area2_retry_move_triggered = True

                cfg = AREA_GOALS.get("area_2", {})
                move_x        = float(cfg.get("retry_move_x",        0.0))
                move_y        = float(cfg.get("retry_move_y",        0.0))
                move_wait_sec = float(cfg.get("retry_move_wait_sec", 2.0))

                msg = Vector3()
                msg.x = move_x
                msg.y = move_y
                msg.z = 0.0
                node.relative_move_pub.publish(msg)
                node.get_logger().info(
                    f"Area 2 RETRY: /relative_move x={move_x} y={move_y}, "
                    f"waiting {move_wait_sec}s before nav."
                )
                node.area2_retry_move_finish_ts = (
                    node.get_clock().now().nanoseconds
                    + int(move_wait_sec * 1e9)
                )
                return  # wait until next tick

            # Still waiting for the retry move timer to expire.
            if not getattr(node, "area2_retry_move_complete", False):
                if node.get_clock().now().nanoseconds < node.area2_retry_move_finish_ts:
                    return  # still waiting — don't touch nav yet
                node.area2_retry_move_complete = True
                node.get_logger().info("Area 2 RETRY: pre-nav move complete.")

        # ── Normal nav + task path ──────────────────────────────────────────
        node.get_logger().info(
            f"Area 2 → navigating to "
            f"x={_GOAL['x']}, y={_GOAL['y']}, yaw={_GOAL['yaw']}",
            once=True,
        )
        node.nav.send_goal([_GOAL["x"], _GOAL["y"], _GOAL["yaw"]])

        # Fire the Forest task exactly once, as soon as nav arrives.
        # node.forest_triggered is reset by trigger_start/trigger_reset/
        # trigger_retry_area(2) so RETRY re-arms this.
        if not node.forest_triggered and node.nav.is_goal_done():
            if node.nav.status == GoalStatus.STATUS_SUCCEEDED:
                self._start_forest_task(node)
            else:
                node.get_logger().error(
                    "Area 2: nav did not succeed reaching the Forest "
                    "entrance yet, will retry once it does.",
                    throttle_duration_sec=2.0,
                )

    def _start_forest_task(self, node):
        if not node.r1_blocks or not node.r2_blocks or not node.fake_block:
            node.get_logger().error(
                "Area 2: Forest state not set yet (r1/r2/fake blocks are "
                "empty) -- operator must enter it on the dashboard.",
                throttle_duration_sec=2.0,
            )
            return  # leave forest_triggered False, so we retry next tick

        msg = String()
        msg.data = json.dumps({
            "command": "start",
            "area": "AREA_2",
            "r1_blocks": node.r1_blocks,
            "r2_blocks": node.r2_blocks,
            "fake_block": node.fake_block,
            "no_downward_pick": bool(getattr(node, "no_downward_pick", False)),
        })
        node.area_cmd_pub.publish(msg)
        node.get_logger().info(f"Area 2: sent Forest start command: {msg.data}")
        node.forest_triggered = True

    def check_transition(self, node):
        if node.area_complete:
            node.area_complete = False
            node.get_logger().info("Area 2 complete. Transitioning to AREA_3.")
            return "AREA_3"
        return None