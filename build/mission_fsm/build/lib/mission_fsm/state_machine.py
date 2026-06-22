"""State machine for mission_fsm."""

from .task.states.area_1 import Area1State
from .task.states.area_2 import Area2State
from .task.states.area_3 import Area3State
from .task.states.done import DoneState


class StateMachine:

    def __init__(self, node):

        self.node = node

        self.states = {
            "AREA_1": Area1State(),
            "AREA_2": Area2State(),
            "AREA_3": Area3State(),
            "DONE":   DoneState(),
        }

        self.current_state = "AREA_1"

    def update(self):

        state = self.states[self.current_state]

        state.execute(self.node)

        next_state = state.check_transition(self.node)

        if next_state:

            self.node.get_logger().info(
                f"{self.current_state} -> {next_state}"
            )

            self.current_state = next_state