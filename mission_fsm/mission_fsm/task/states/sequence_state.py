"""SequenceState — run a list of child states in order."""

from ..base_state import BaseState


class SequenceState(BaseState):
    """Execute a list of child states sequentially.

    Each child is ticked until its ``check_transition`` returns non-None,
    then the sequence advances to the next child.  When all children finish,
    the sequence itself returns ``next_state``.

    Example::

        SequenceState([
            NavigateState.from_areas_yaml("area_1"),
            WaitState(1.0),
            DriveForwardState(2.0, 0.15),
        ], next_state="DONE")
    """

    def __init__(self, states: list, next_state: str = "DONE", name: str = "Sequence"):
        self.states = states
        self.next_state = next_state
        self.name = name
        self.index = 0

    def execute(self, node):
        if self.index < len(self.states):
            self.states[self.index].execute(node)
        else:
            node.stop_cmd_vel()

    def check_transition(self, node):
        if self.index >= len(self.states):
            return self.next_state

        child_result = self.states[self.index].check_transition(node)
        if child_result is not None:
            node.stop_cmd_vel()

            # If a child reports failure, abort the whole sequence
            if child_result == "failed":
                node.get_logger().error(
                    f"{self.name}: child {self.index} failed → aborting mission."
                )
                return "failed"

            self.index += 1
            if self.index >= len(self.states):
                node.get_logger().info(f"{self.name}: sequence complete.")
                return self.next_state
        return None

    def reset(self):
        self.index = 0
        for s in self.states:
            if hasattr(s, "reset"):
                s.reset()
