"""Pickup state for mission_fsm."""

from ..base_state import BaseState


class PickupState(BaseState):
    """State that handles pickup behavior."""

    def execute(self, node):
        """Execute pickup behavior."""
        pass

    def check_transition(self, node):
        """Check for state transitions."""
        return None
