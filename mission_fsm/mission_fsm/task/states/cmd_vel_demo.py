"""Example states that drive the robot using /cmd_vel directly.

These bypass Nav2 and send velocity commands in open-loop.
Useful for short, timed manoeuvres (e.g. align, creep forward).
"""

from ..base_state import BaseState


class DriveForwardState(BaseState):
    """Move straight forward for a fixed duration using /cmd_vel.

    Transitions to ``next_state`` once the timer expires.
    """

    def __init__(self, duration_sec: float = 2.0, speed: float = 0.15, next_state: str = "DONE"):
        self.duration_sec = duration_sec
        self.speed = speed
        self.next_state = next_state
        self._start_time = None

    def execute(self, node):
        now = node.get_clock().now()

        if self._start_time is None:
            self._start_time = now
            node.get_logger().info(
                f"DriveForward: started (speed={self.speed} m/s, "
                f"duration={self.duration_sec} s)"
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
            node.get_logger().info("DriveForward: finished.")
            self._start_time = None  # reset for next time
            return self.next_state
        return None


class RotateState(BaseState):
    """Rotate in place for a fixed duration using /cmd_vel.

    Transitions to ``next_state`` once the timer expires.
    """

    def __init__(self, duration_sec: float = 2.0, angular_speed: float = 0.5, next_state: str = "DONE"):
        self.duration_sec = duration_sec
        self.angular_speed = angular_speed
        self.next_state = next_state
        self._start_time = None

    def execute(self, node):
        now = node.get_clock().now()

        if self._start_time is None:
            self._start_time = now
            node.get_logger().info(
                f"Rotate: started (omega={self.angular_speed} rad/s, "
                f"duration={self.duration_sec} s)"
            )

        elapsed = (now - self._start_time).nanoseconds / 1e9

        if elapsed < self.duration_sec:
            node.publish_cmd_vel(linear_x=0.0, angular_z=self.angular_speed)
        else:
            node.stop_cmd_vel()

    def check_transition(self, node):
        if self._start_time is None:
            return None
        elapsed = (node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self.duration_sec:
            node.get_logger().info("Rotate: finished.")
            self._start_time = None
            return self.next_state
        return None
