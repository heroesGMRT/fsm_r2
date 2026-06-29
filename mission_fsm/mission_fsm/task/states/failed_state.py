"""Terminal failure state."""

from ..base_state import BaseState


class FailedState(BaseState):
    """Terminal state reached when a critical step fails after all retries.

    Logs an error and stops the robot permanently until RESET is pressed.
    """

    def execute(self, node):
        node.get_logger().error(
            "=== MISSION FAILED === "
            "A critical step failed after maximum retries. "
            "Press RESET to restart.",
            once=True,
        )
        node.stop_cmd_vel()

    def check_transition(self, node):
        return None
