"""Idle state for mission_fsm."""

from ..base_state import BaseState


class IdleState(BaseState):
    """Initial state — waiting for the dashboard START command.

    ``execute`` logs that FSM is waiting for start.
    ``check_transition`` returns ``None`` since transitions out of IDLE
    are driven by external events (e.g. dashboard pressing START, which
    directly sets self.current_state to AREA_1).
    """

    def execute(self, node):
        node.get_logger().info("FSM IDLE. Waiting for START command...", once=True)

    def check_transition(self, node):
        return None
