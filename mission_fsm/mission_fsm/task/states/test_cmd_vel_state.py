"""Test state that immediately publishes /cmd_vel for verification.

Use this to verify the cmd_vel pipeline works without waiting for Nav2.
"""

from ..base_state import BaseState


class TestCmdVelState(BaseState):
    """Immediately drive forward for ``duration_sec`` seconds.

    This bypasses Nav2 entirely and is useful for confirming that
    ``/cmd_vel`` messages are actually reaching your robot.
    """

    def __init__(self, duration_sec: float = 3.0, speed: float = 0.15):
        self.duration_sec = duration_sec
        self.speed = speed
        self._start_time = None

    def execute(self, node):
        now = node.get_clock().now()
        if self._start_time is None:
            self._start_time = now
            node.get_logger().warn(
                f"🧪 TEST CMD_VEL → driving forward {self.duration_sec}s @ {self.speed} m/s"
            )
        elapsed = (now - self._start_time).nanoseconds / 1e9
        if elapsed < self.duration_sec:
            node.publish_cmd_vel(linear_x=self.speed, angular_z=0.0)
        else:
            node.stop_cmd_vel()

    def check_transition(self, node):
        if self._start_time is None:
            return None
        elapsed = (node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self.duration_sec:
            node.stop_cmd_vel()
            node.get_logger().warn("🧪 TEST CMD_VEL → finished.")
            self._start_time = None
            return "done"
        return None
