from ..base_state import BaseState

class RecoveryState(BaseState):

    def execute(self, node):

        node.get_logger().info(
            "Recovery behavior"
        )

    def check_transition(self, node):

        if node.recovery_done:
            return "SEARCH"

        return None