"""Mock NavigateToPose Action Server for mission_fsm testing."""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class MockNavServer(Node):

    def __init__(self):
        super().__init__("mock_nav_server")
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            self.execute_callback
        )
        self.get_logger().info("Mock NavigateToPose Action Server initialized.")

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            f"Received goal: x={goal_handle.request.pose.pose.position.x:.2f}, "
            f"y={goal_handle.request.pose.pose.position.y:.2f}"
        )
        
        # Simulate robot traveling to destination
        for i in range(3):
            self.get_logger().info(f"Navigating... {3 - i}s remaining")
            time.sleep(1.0)

        goal_handle.succeed()
        self.get_logger().info("Goal completed successfully!")
        
        result = NavigateToPose.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockNavServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
