"""Task manager for mission_fsm."""

from .states.idle import IdleState
from .states.area_1 import Area1State
from .states.area_2 import Area2State
from .states.area_3 import Area3State
from .states.done import DoneState


class TaskManager:

    def __init__(self, node):

        self.node = node

        self.current_state = "IDLE"

        self.states = {
            "IDLE":   IdleState(),
            "AREA_1": Area1State(),
            "AREA_2": Area2State(),
            "AREA_3": Area3State(),
            "DONE":   DoneState(),
        }

    def update(self):

        state = self.states[self.current_state]

        state.execute(self.node)

        next_state = state.check_transition(self.node)

        if next_state:
            self.node.get_logger().info(
                f"FSM transition: {self.current_state} -> {next_state}"
            )
            self.current_state = next_state