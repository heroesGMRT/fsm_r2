"""Placeholder Teensy command bridge.

Later this node should translate high-level command names into the real
serial / micro-ROS protocol for the Teensy. For now it logs the command and
IMMEDIATELY acks it on /teensy/ack, so the forest executor's sequencer can
be dry-run end-to-end without hardware.

Ack contract the real bridge MUST honour: after a command from
/teensy/command has PHYSICALLY COMPLETED (not merely been sent), publish on
/teensy/ack::

    {"sequence": <sequence from the command>, "status": "done"}

or ``"status": "error"`` if the motion failed.
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
        self._ack_pub = self.create_publisher(String, "/teensy/ack", 10)
        self.get_logger().info(
            "TeensyCommand placeholder ready. Listening on /teensy/command; "
            "acking instantly on /teensy/ack"
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

        # Instant ack — the real bridge must ack only on motion completion.
        if isinstance(sequence, int):
            ack = String()
            ack.data = json.dumps({"sequence": sequence, "status": "done"})
            self._ack_pub.publish(ack)


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
