"""Done state for mission_fsm."""

from ..base_state import BaseState


class DoneState(BaseState):
    """Terminal state — the mission has completed.

    ``execute`` logs a one-shot message so the operator can confirm the
    FSM reached the end.  ``check_transition`` always returns ``None``
    because there is no further state to move to.
    """

    def execute(self, node):
        node.get_logger().info("Mission complete. FSM in DONE state.", once=True)

    def check_transition(self, node):
        return None
