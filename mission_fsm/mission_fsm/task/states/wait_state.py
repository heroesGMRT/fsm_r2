"""Simple wait/delay state."""

from ..base_state import BaseState


class WaitState(BaseState):
    """Do nothing for ``duration_sec`` seconds, then finish."""

    def __init__(self, duration_sec: float = 1.0):
        self.duration_sec = duration_sec
        self._start_time = None

    def execute(self, node):
        if self._start_time is None:
            self._start_time = node.get_clock().now()
            node.get_logger().info(f"Wait → {self.duration_sec:.1f}s ...")

    def check_transition(self, node):
        if self._start_time is None:
            return None
        elapsed = (node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self.duration_sec:
            self._start_time = None
            return "done"
        return None

    def reset(self):
        self._start_time = None
