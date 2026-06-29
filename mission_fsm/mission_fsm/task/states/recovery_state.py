"""Recovery state executed before a RetryState re-attempts.

Typical use: back away a little so the next attempt starts fresh."""

from ..base_state import BaseState


class RecoveryState(BaseState):
    """Simple recovery: drive backward briefly then stop.

    Can be used standalone or handed to :class:`RetryState`.

    Args:
        back_duration: how long to drive backward (seconds).
        back_speed:    linear speed while reversing (positive number,
                       internally negated so robot backs up).
        wait_after:    extra wait after stopping (seconds).
    """

    def __init__(self, back_duration: float = 1.0, back_speed: float = 0.12, wait_after: float = 0.5):
        self.back_duration = back_duration
        self.back_speed = abs(back_speed)
        self.wait_after = wait_after
        self._start_time = None

    def execute(self, node):
        now = node.get_clock().now()
        if self._start_time is None:
            self._start_time = now
            node.get_logger().info(
                f"Recovery → backing {self.back_duration}s @ {self.back_speed} m/s"
            )
        elapsed = (now - self._start_time).nanoseconds / 1e9
        if elapsed < self.back_duration:
            node.publish_cmd_vel(linear_x=-self.back_speed, angular_z=0.0)
        else:
            node.stop_cmd_vel()

    def check_transition(self, node):
        if self._start_time is None:
            return None
        elapsed = (node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self.back_duration + self.wait_after:
            node.stop_cmd_vel()
            self._start_time = None
            return "done"
        return None

    def reset(self):
        self._start_time = None
