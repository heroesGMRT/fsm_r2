"""Placeholder Teensy command bridge.

Later this node should translate high-level command names into the real serial
protocol for the Teensy. For now it only logs the command it would send.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TeensyCommandNode(Node):
    """Receive high-level robot commands and placeholder-send them to Teensy."""

    def __init__(self):
        super().__init__("teensy_command")
        self._command_sub = self.create_subscription(
            String,
            "/teensy/command",
            self._command_callback,
            10,
        )
        self.get_logger().info(
            "TeensyCommand placeholder ready. Listening on /teensy/command"
        )

    def _command_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"command": msg.data, "comment": ""}

        command = payload.get("command", "")
        sequence = payload.get("sequence", "?")
        total = payload.get("total", "?")
        comment = payload.get("comment", "")

        self.get_logger().info(
            f"[PLACEHOLDER] Teensy command {sequence}/{total}: {command}"
            f"{(' - ' + comment) if comment else ''}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TeensyCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
