"""Open-loop velocity states using /cmd_vel.

All timed states support an optional ``verify_attr``.
If provided, after the motion finishes the state checks
``getattr(node, verify_attr, True)``.  When that attribute is
``False`` the state returns ``"failed"`` instead of ``"done"``,
so a :class:`RetryState` wrapper can re-run it.
"""

from ..base_state import BaseState


class _TimedCmdVelState(BaseState):
    """Base class for timed /cmd_vel motions with optional post-check."""

    def __init__(
        self,
        duration_sec: float,
        linear_x: float,
        angular_z: float,
        verify_attr: str | None = None,
    ):
        self.duration_sec = duration_sec
        self.linear_x = linear_x
        self.angular_z = angular_z
        self.verify_attr = verify_attr
        self._start_time = None

    def execute(self, node):
        now = node.get_clock().now()
        if self._start_time is None:
            self._start_time = now
            extra = f" verify={self.verify_attr}" if self.verify_attr else ""
            node.get_logger().info(
                f"{self._label()}: lin={self.linear_x:.2f} ang={self.angular_z:.2f} "
                f"for {self.duration_sec:.1f}s{extra}"
            )
        elapsed = (now - self._start_time).nanoseconds / 1e9
        if elapsed < self.duration_sec:
            node.publish_cmd_vel(linear_x=self.linear_x, angular_z=self.angular_z)
        else:
            node.stop_cmd_vel()

    def check_transition(self, node):
        if self._start_time is None:
            return None
        elapsed = (node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self.duration_sec:
            node.stop_cmd_vel()
            self._start_time = None

            # Optional verification step (e.g. sensor check after motion)
            if self.verify_attr is not None:
                ok = getattr(node, self.verify_attr, True)
                if not ok:
                    node.get_logger().warn(
                        f"{self._label()}: verify_attr '{self.verify_attr}' = False → FAILED"
                    )
                    return "failed"

            return "done"
        return None

    def reset(self):
        self._start_time = None

    def _label(self):
        return self.__class__.__name__


class DriveForwardState(_TimedCmdVelState):
    """Drive straight forward for a fixed duration.

    Args:
        duration_sec: how long to drive.
        speed: linear.x speed (m/s).  Positive = forward.
        verify_attr: optional node attribute name to check after the motion.
    """
    def __init__(self, duration_sec: float = 2.0, speed: float = 0.15, verify_attr: str | None = None):
        super().__init__(duration_sec, linear_x=speed, angular_z=0.0, verify_attr=verify_attr)


class DriveBackwardState(_TimedCmdVelState):
    """Drive straight backward for a fixed duration."""
    def __init__(self, duration_sec: float = 2.0, speed: float = 0.15, verify_attr: str | None = None):
        super().__init__(duration_sec, linear_x=-abs(speed), angular_z=0.0, verify_attr=verify_attr)


class RotateLeftState(_TimedCmdVelState):
    """Rotate counter-clockwise (CCW / kiri) in place."""
    def __init__(self, duration_sec: float = 1.6, angular_speed: float = 0.5, verify_attr: str | None = None):
        super().__init__(duration_sec, linear_x=0.0, angular_z=abs(angular_speed), verify_attr=verify_attr)


class RotateRightState(_TimedCmdVelState):
    """Rotate clockwise (CW / kanan) in place."""
    def __init__(self, duration_sec: float = 1.6, angular_speed: float = 0.5, verify_attr: str | None = None):
        super().__init__(duration_sec, linear_x=0.0, angular_z=-abs(angular_speed), verify_attr=verify_attr)


class CurveState(_TimedCmdVelState):
    """Drive in an arc (both linear + angular)."""
    def __init__(self, duration_sec: float = 2.0, linear_x: float = 0.15, angular_z: float = 0.3, verify_attr: str | None = None):
        super().__init__(duration_sec, linear_x=linear_x, angular_z=angular_z, verify_attr=verify_attr)


class StopState(BaseState):
    """Immediately publish zero velocity and finish."""
    def execute(self, node):
        node.stop_cmd_vel()

    def check_transition(self, node):
        return "done"
