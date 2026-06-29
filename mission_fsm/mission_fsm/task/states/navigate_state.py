"""Navigation state with automatic retry on Nav2 failure."""

from action_msgs.msg import GoalStatus
from ..base_state import BaseState


class NavigateState(BaseState):
    """Send a NavigateToPose goal and wait for Nav2 to finish.

    Automatically retries up to ``max_retries`` times with
    ``retry_delay_sec`` seconds between attempts.  Only returns
    ``failed`` when all retries are exhausted.

    Unlike the old Area1/2/3 states, this does **not** require an
    external ``area_complete`` signal — it monitors the action
    result directly.
    """

    def __init__(
        self,
        x: float,
        y: float,
        yaw: float,
        max_retries: int = 3,
        retry_delay_sec: float = 2.0,
    ):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.max_retries = max_retries
        self.retry_delay_sec = retry_delay_sec

        # runtime state
        self._sent = False
        self._retry_count = 0
        self._retry_wait_start = None

    @classmethod
    def from_areas_yaml(
        cls,
        key: str,
        max_retries: int = 3,
        retry_delay_sec: float = 2.0,
    ):
        """Factory: build from a key in areas.yaml (e.g. 'area_1')."""
        from ...config.loader import AREA_GOALS
        g = AREA_GOALS[key]
        return cls(g["x"], g["y"], g["yaw"], max_retries, retry_delay_sec)

    def execute(self, node):
        # ── delay between retries ───────────────────────────────────────
        if self._retry_wait_start is not None:
            elapsed = (node.get_clock().now() - self._retry_wait_start).nanoseconds / 1e9
            if elapsed < self.retry_delay_sec:
                return  # still cooling down
            self._retry_wait_start = None  # cooldown finished

        # ── send goal (if not already in-flight) ────────────────────────
        if not self._sent:
            node.get_logger().info(
                f"Navigate → goal x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f}"
                f" (retry {self._retry_count}/{self.max_retries})",
                once=True,
            )
            ok = node.nav.send_goal([self.x, self.y, self.yaw])
            if ok:
                self._sent = True

    def check_transition(self, node):
        if not self._sent:
            return None

        if not node.nav.is_goal_done():
            return None

        # ── goal finished ───────────────────────────────────────────────
        status = node.nav.status
        self._sent = False  # re-arm so execute can send again

        if status == GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().info("Navigate → reached goal.")
            self.reset()
            return "done"

        # ── failed → retry or give up ───────────────────────────────────
        self._retry_count += 1

        if self._retry_count > self.max_retries:
            node.get_logger().error(
                f"Navigate → failed after {self.max_retries} retries. Giving up."
            )
            self.reset()
            return "failed"

        node.get_logger().warn(
            f"Navigate → failed (status {status}), "
            f"retry {self._retry_count}/{self.max_retries} "
            f"in {self.retry_delay_sec}s..."
        )
        self._retry_wait_start = node.get_clock().now()
        return None

    def reset(self):
        self._sent = False
        self._retry_count = 0
        self._retry_wait_start = None
