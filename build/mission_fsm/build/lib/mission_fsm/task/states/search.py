from ..base_state import BaseState

class SearchState(BaseState):

    def execute(self, node):

        node.get_logger().info(
            "Searching for spear"
        )

    def check_transition(self, node):

        if node.spear_found:
            return "PICKUP"

        if node.search_failed:
            return "RECOVERY"

        return None