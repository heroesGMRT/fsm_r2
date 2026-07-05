"""Forest executor node for Area 2.

This node consumes the FSM Area 2 start command, runs the planner in
``path.py``, and dispatches the generated high-level action sequence as
Int32 commands on /fsm_command — the same topic used by the keyboard
teleop node.  Integer values are taken directly from the HOTKEYS table
in keyboard_teleop_node.py.  Actions that have no assigned integer (e.g.
STRAFE_LEFT/RIGHT, ROTATE_270) are logged as warnings and skipped so
the rest of the sequence can still execute.
"""

import json
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # used for area_command / area_status / fsm_signal

from . import path as forest_path

# How fast and how long the robot moves forward for each DESCENT_FORWARD step.
# Tune these to match the physical distance needed between wheel extensions.
DESCENT_FORWARD_SPEED = 0.15   # m/s  (slow and controlled)
DESCENT_FORWARD_SECS  = 1.0    # seconds per forward burst

# ---------------------------------------------------------------------------
# Action-string → /fsm_command integer mapping
# Integer values match the HOTKEYS table in keyboard_teleop_node.py.
# Add new entries here as more Teensy commands are defined.
# ---------------------------------------------------------------------------
ACTION_TO_CMD: dict[str, int | None] = {
    # ── Approach & entry ──────────────────────────────────────────────────────
    "FORWARD_INIT":         40,   # Start autonomous drive sequence (approach block 12)
    "LIFT_CROSS_SEQUENCE":  300,  # lift↑ → fwd → front↓ → fwd → back↓ → fwd  (climb up to block 12)
    # ── Forest traversal ─────────────────────────────────────────────────────
    "CLIMB_UP":             104,  # BOTH lifts UP   (encoder/limit-switch)
    "CLIMB_DOWN":           105,  # BOTH lifts DOWN (encoder/limit-switch)
    "PICK_BLOCK_UP":        51,   # Arm Sequence S1
    "PICK_BLOCK_DOWN":      52,   # Arm Sequence S2
    "VISUAL_SERVO_BLOCK":   53,   # Arm Sequence S3
    "ROTATE_90":            201,  # Chassis Macro M
    "ROTATE_180":           202,  # Chassis Macro N
    # ── Exit descent sequence (block 12 → ground) ─────────────────────────────
    # fwd → extend front → fwd → extend back → fwd → retract both
    "DESCENT_FORWARD":      "CMD_VEL",  # publish forward Twist (see DESCENT_FORWARD_SPEED/SECS)
    "EXTEND_FRONT_WHEEL":   1000,   # front wheel extend
    "EXTEND_BACK_WHEEL":    101,    # back wheel extend
    "RETRACT_BOTH_WHEELS":  105,    # retract both wheels
    # ── No hotkey equivalent yet ─────────────────────────────────────────────
    "ROTATE_270":           None,  # TODO: assign integer when firmware ready
    "STRAFE_LEFT":          None,  # TODO: assign integer when firmware ready
    "STRAFE_RIGHT":         None,  # TODO: assign integer when firmware ready
}


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
        self._fsm_cmd_pub = self.create_publisher(
            Int32,
            "/fsm_command",
            10,
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
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
            "ForestExecutor ready. "
            "Waiting for AREA_2 commands on /fsm/area_command  "
            "(actions → /fsm_command Int32)"
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
        cmd_int = ACTION_TO_CMD.get(action)

        # DESCENT_FORWARD is a plain cmd_vel forward burst, not a Teensy integer.
        if cmd_int == "CMD_VEL":
            self._publish_forward(index, total, comment)
            return

        if cmd_int is None:
            self.get_logger().warn(
                f"Forest command {index}/{total}: {action!r} has no integer mapping "
                f"— skipping.  ({comment})"
            )
            return
        msg = Int32()
        msg.data = cmd_int
        self._fsm_cmd_pub.publish(msg)
        self.get_logger().info(
            f"Forest command {index}/{total}: {action} → {cmd_int}  # {comment}"
        )

    def _publish_forward(self, index: int, total: int, comment: str):
        """Publish a forward Twist for DESCENT_FORWARD_SECS seconds, then stop."""
        self.get_logger().info(
            f"Forest command {index}/{total}: DESCENT_FORWARD "
            f"→ cmd_vel {DESCENT_FORWARD_SPEED} m/s × {DESCENT_FORWARD_SECS}s  # {comment}"
        )
        fwd = Twist()
        fwd.linear.x = DESCENT_FORWARD_SPEED
        stop = Twist()  # all zeros

        self._cmd_vel_pub.publish(fwd)
        time.sleep(DESCENT_FORWARD_SECS)
        self._cmd_vel_pub.publish(stop)

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
