"""Wait for an external boolean flag on the node to become True."""

from ..base_state import BaseState


class WaitForSignalState(BaseState):
    """Block until ``node.<attr_name>`` becomes True, then clear it.

    Default waits for the standard ``area_complete`` flag that external
    nodes (or the test publisher) set via ``/fsm/signal``.
    """

    def __init__(self, attr_name: str = "area_complete"):
        self.attr_name = attr_name

    def execute(self, node):
        pass

    def check_transition(self, node):
        if getattr(node, self.attr_name, False):
            setattr(node, self.attr_name, False)
            node.get_logger().info(f"WaitForSignal → '{self.attr_name}' received.")
            return "done"
        return None
