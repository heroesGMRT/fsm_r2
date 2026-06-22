"""Navigation interface for mission_fsm using Nav2."""

import math
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavInterface:
    """Interface for navigation actions using Nav2."""

    def __init__(self, node):
        """Initialize the navigation interface.

        Args:
            node: The ROS 2 node instance to use for action client creation and logging.
        """
        self.node = node
        self.logger = node.get_logger()
        
        self.logger.info("Initializing Nav2 Action Client on 'navigate_to_pose'...")
        self._action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        
        # Keep track of active goal and state status
        self.status = GoalStatus.STATUS_UNKNOWN
        self.current_feedback = None
        self.result = None
        self.active_goal = None

    def navigate(self, destination):
        """Send navigation command to the robot.

        Alias for send_goal to maintain compatibility with the original interface.
        """
        return self.send_goal(destination)

    def send_goal(self, goal):
        """Send navigation goal to the robot asynchronously.

        Args:
            goal: A PoseStamped message, a dictionary, list/tuple, or other representation of pose.
        
        Returns:
            bool: True if the goal request was successfully sent, False otherwise.
        """
        # Non-blocking check — never call wait_for_server() inside a ROS
        # timer/subscription callback because it blocks the executor and
        # corrupts the action-client C++ internals (stack smashing crash).
        if not self._action_client.server_is_ready():
            self.logger.warn("NavigateToPose action server not available yet, retrying next tick...")
            return False

        goal_msg = NavigateToPose.Goal()

        # Parse and construct the PoseStamped goal message
        if isinstance(goal, PoseStamped):
            goal_msg.pose = goal
        else:
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            
            if isinstance(goal, dict):
                pos = goal.get('position', goal)
                ori = goal.get('orientation', goal)
                
                # Check for flat dict vs nested dict
                if isinstance(pos, dict):
                    goal_msg.pose.pose.position.x = float(pos.get('x', 0.0))
                    goal_msg.pose.pose.position.y = float(pos.get('y', 0.0))
                    goal_msg.pose.pose.position.z = float(pos.get('z', 0.0))
                else:
                    goal_msg.pose.pose.position.x = float(goal.get('x', 0.0))
                    goal_msg.pose.pose.position.y = float(goal.get('y', 0.0))
                    goal_msg.pose.pose.position.z = float(goal.get('z', 0.0))

                if 'yaw' in goal:
                    yaw = float(goal['yaw'])
                    goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                    goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
                elif isinstance(ori, dict):
                    goal_msg.pose.pose.orientation.x = float(ori.get('x', 0.0))
                    goal_msg.pose.pose.orientation.y = float(ori.get('y', 0.0))
                    goal_msg.pose.pose.orientation.z = float(ori.get('z', 0.0))
                    goal_msg.pose.pose.orientation.w = float(ori.get('w', 1.0))
                else:
                    goal_msg.pose.pose.orientation.x = float(goal.get('qx', goal.get('x', 0.0)))
                    goal_msg.pose.pose.orientation.y = float(goal.get('qy', goal.get('y', 0.0)))
                    goal_msg.pose.pose.orientation.z = float(goal.get('qz', goal.get('z', 0.0)))
                    goal_msg.pose.pose.orientation.w = float(goal.get('qw', goal.get('w', 1.0)))
            elif isinstance(goal, (list, tuple)):
                if len(goal) >= 2:
                    goal_msg.pose.pose.position.x = float(goal[0])
                    goal_msg.pose.pose.position.y = float(goal[1])
                if len(goal) >= 3:
                    yaw = float(goal[2])
                    goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                    goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
                else:
                    goal_msg.pose.pose.orientation.w = 1.0
            else:
                self.logger.error(f"Unsupported goal type: {type(goal)}")
                return False

        # Extract target coordinates rounded to 5 decimal places for robust comparison
        target_coords = (
            round(float(goal_msg.pose.pose.position.x), 5),
            round(float(goal_msg.pose.pose.position.y), 5),
            round(float(goal_msg.pose.pose.position.z), 5),
            round(float(goal_msg.pose.pose.orientation.x), 5),
            round(float(goal_msg.pose.pose.orientation.y), 5),
            round(float(goal_msg.pose.pose.orientation.z), 5),
            round(float(goal_msg.pose.pose.orientation.w), 5)
        )

        # Prevent sending duplicate goal if the same target is currently in-progress
        if self.active_goal == target_coords and self.is_active():
            return True

        self.logger.info(
            f"Sending NavigateToPose goal: x={goal_msg.pose.pose.position.x:.2f}, "
            f"y={goal_msg.pose.pose.position.y:.2f}"
        )

        self.active_goal = target_coords
        self.status = GoalStatus.STATUS_UNKNOWN
        self.current_feedback = None
        self.result = None

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _feedback_callback(self, feedback_msg):
        self.current_feedback = feedback_msg.feedback

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.logger.error("Goal rejected by Nav2 server!")
            self.status = GoalStatus.STATUS_ABORTED
            self.active_goal = None
            return

        self.logger.info("Goal accepted by Nav2 server.")
        self.status = GoalStatus.STATUS_EXECUTING
        
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result_response = future.result()
        self.status = result_response.status
        self.result = result_response.result
        self.active_goal = None
        
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info("Nav2 navigation succeeded!")
        elif self.status == GoalStatus.STATUS_CANCELED:
            self.logger.warn("Nav2 navigation canceled.")
        else:
            self.logger.error(f"Nav2 navigation failed with status code: {self.status}")

    def cancel_goal(self):
        """Cancel the active navigation goal."""
        if self._goal_handle is not None:
            self.logger.info("Canceling active navigation goal...")
            self._goal_handle.cancel_goal_async()
            return True
        return False

    def is_goal_done(self):
        """Check if the goal execution is complete."""
        return self.status in [
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED
        ]

    def is_navigating(self):
        """Check if the robot is currently navigating."""
        return self.status == GoalStatus.STATUS_EXECUTING

    def is_active(self):
        """Check if a goal is currently in-flight, accepted, or executing."""
        return self.status in [
            GoalStatus.STATUS_UNKNOWN,
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING
        ]

