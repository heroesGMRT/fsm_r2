"""Forest executor node for Area 2.

This node consumes the FSM Area 2 start command, runs the planner in
``path.py``, and dispatches the generated high-level action sequence to the
Teensy command bridge. The Teensy bridge is still a placeholder, so for now
commands are fire-and-forget and Area 2 is marked complete after dispatch.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import path as forest_path


class ForestExecutorNode(Node):
    """Run the Area 2 Forest planner and publish commands for the Teensy."""

    def __init__(self):
        super().__init__("forest_executor")

        self._area_cmd_sub = self.create_subscription(
            String,
            "/fsm/area_command",
            self._area_command_callback,
            10,
        )
        self._teensy_cmd_pub = self.create_publisher(
            String,
            "/teensy/command",
            10,
        )
        self._fsm_signal_pub = self.create_publisher(
            String,
            "/fsm/signal",
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            "/fsm/area_status",
            10,
        )

        self._running = False
        self.get_logger().info(
            "ForestExecutor ready. Waiting for AREA_2 commands on /fsm/area_command"
        )

    def _area_command_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid area command JSON: {exc}")
            return

        if payload.get("command") != "start" or payload.get("area") != "AREA_2":
            return

        if self._running:
            self.get_logger().warn("Forest task already running; ignoring duplicate start.")
            return

        self._running = True
        try:
            self._run_forest_task(payload)
        finally:
            self._running = False

    def _run_forest_task(self, payload: dict):
        try:
            r1_blocks = self._read_blocks(payload, "r1_blocks", 3)
            r2_blocks = self._read_blocks(payload, "r2_blocks", 4)
            fake_block = int(payload["fake_block"])
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_status("error", f"Invalid Forest payload: {exc}")
            return

        self.get_logger().info(
            f"Planning Forest task: r1={r1_blocks}, r2={r2_blocks}, fake={fake_block}"
        )
        result = forest_path.plan(r1_blocks, r2_blocks, fake_block)

        if result["status"] == "impossible":
            self._publish_status(
                "impossible",
                "No legal R2 Forest route, even if R1 clears all of its KFS.",
            )
            return

        route = result["route"]
        actions = forest_path.generate_actions(route)
        clear_set = result["clear_set"]

        self._publish_status(
            "planned",
            {
                "status": result["status"],
                "clear_set": clear_set,
                "needs_extra_r1_trip": result["needs_extra_r1_trip"],
                "fallback_collect": result["fallback_collect"],
                "want_used": result["want_used"],
                "r2_collected": route["collected"],
                "exit_block": route["exit_block"],
                "positions": route["positions"],
                "action_count": len(actions),
            },
        )

        if clear_set:
            self.get_logger().warn(
                f"R1 must clear these blocks before/while R2 executes: {clear_set}"
            )

        for index, (action, comment) in enumerate(actions, start=1):
            self._publish_teensy_command(index, len(actions), action, comment)

        self._publish_status("complete", f"Dispatched {len(actions)} Forest actions.")
        self._publish_area_complete()

    @staticmethod
    def _read_blocks(payload: dict, key: str, expected_count: int) -> list[int]:
        blocks = [int(value) for value in payload[key]]
        if len(blocks) != expected_count:
            raise ValueError(f"{key} must contain {expected_count} blocks")
        return blocks

    def _publish_teensy_command(self, index: int, total: int, action: str, comment: str):
        msg = String()
        msg.data = json.dumps(
            {
                "source": "forest_executor",
                "sequence": index,
                "total": total,
                "command": action,
                "comment": comment,
            }
        )
        self._teensy_cmd_pub.publish(msg)
        self.get_logger().info(f"Forest command {index}/{total}: {action} - {comment}")

    def _publish_status(self, status: str, detail):
        msg = String()
        msg.data = json.dumps(
            {
                "area": "AREA_2",
                "status": status,
                "detail": detail,
            }
        )
        self._status_pub.publish(msg)
        if status in ("error", "impossible"):
            self.get_logger().error(f"Area 2 {status}: {detail}")
        else:
            self.get_logger().info(f"Area 2 {status}: {detail}")

    def _publish_area_complete(self):
        msg = String()
        msg.data = "area_complete"
        self._fsm_signal_pub.publish(msg)
        self.get_logger().info("Published area_complete for Area 2.")


def main(args=None):
    rclpy.init(args=args)
    node = ForestExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
