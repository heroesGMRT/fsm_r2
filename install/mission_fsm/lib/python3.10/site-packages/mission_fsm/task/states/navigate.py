"""Navigate state for mission_fsm."""

from ..base_state import BaseState


class NavigateState(BaseState):

    def execute(self, node):

        area = node.mission.current_area

        goal = node.area_config[f"area_{area}"]["search_pose"]

        node.nav.send_goal(goal)

    def check_transition(self, node):
        """Check for state transitions."""
        if node.nav.is_goal_done():
            from action_msgs.msg import GoalStatus
            if node.nav.status == GoalStatus.STATUS_SUCCEEDED:
                node.get_logger().info("Goal reached successfully! Transitioning to SEARCH.")
                return "SEARCH"
            else:
                node.get_logger().error("Goal failed or was canceled! Transitioning to RECOVERY.")
                return "RECOVERY"
        return None