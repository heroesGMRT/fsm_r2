"""Forest executor node for Area 2.

Consumes the FSM Area 2 start command, runs the planner in ``path.py``, and
executes the generated action sequence ONE PRIMITIVE AT A TIME:

* ``VISUAL_SERVO_BLOCK`` → calls the ``align_and_pick`` action
  (r2_servo/pick_servo_node) with the target block id and height, and waits
  for the alignment result. One retry on failure.
* every other primitive → published as JSON on ``/teensy/command`` (and also
  as an Int32 command on ``/fsm_command`` if an integer mapping exists)
  and the executor waits for a matching completion ack on ``/teensy/ack``:
  ``{"sequence": <int>, "status": "done" | "error"}``. The placeholder
  Teensy bridge acks instantly; the real bridge must ack when the motion
  actually finishes.

All waiting is timer-driven (10 Hz tick) — nothing blocks the executor, in
keeping with the rest of mission_fsm.
"""

import json
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from r2_interfaces.action import AlignAndPick

from . import path as forest_path

# How fast and how long the robot moves forward for each DESCENT_FORWARD step.
# Tune these to match the physical distance needed between wheel extensions.
DESCENT_FORWARD_SPEED = 0.15   # m/s  (slow and controlled)
DESCENT_FORWARD_SECS  = 1.0    # seconds per forward burst

# Planner block heights 'A'/'B'/'C' → AlignAndPick height enum.
_HEIGHT_ENUM = {
    'A': AlignAndPick.Goal.HEIGHT_A,
    'B': AlignAndPick.Goal.HEIGHT_B,
    'C': AlignAndPick.Goal.HEIGHT_C,
}

# Sequencer states
_IDLE = 'IDLE'
_DISPATCH = 'DISPATCH'
_WAIT_ACK = 'WAIT_ACK'
_WAIT_SERVO = 'WAIT_SERVO'

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
    """Run the Area 2 Forest planner and sequence its primitives."""

    def __init__(self):
        super().__init__("forest_executor")

        self.declare_parameter('ack_timeout_s', 30.0)
        self.declare_parameter('servo_timeout_s', 90.0)
        self.declare_parameter('servo_retries', 1)

        self._area_cmd_sub = self.create_subscription(
            String, "/fsm/area_command", self._area_command_callback, 10)
        self._ack_sub = self.create_subscription(
            String, "/teensy/ack", self._ack_callback, 10)
        self._teensy_cmd_pub = self.create_publisher(String, "/teensy/command", 10)
        self._fsm_cmd_pub = self.create_publisher(Int32, "/fsm_command", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._fsm_signal_pub = self.create_publisher(String, "/fsm/signal", 10)
        self._status_pub = self.create_publisher(String, "/fsm/area_status", 10)

        self._servo_client = ActionClient(self, AlignAndPick, 'align_and_pick')

        # Sequencer state
        self._state = _IDLE
        self._actions = []
        self._index = 0
        self._wait_deadline = None
        self._server_wait_deadline = None
        self._acked_sequence = None
        self._ack_error = None
        self._servo_status = None          # GoalStatus once the result lands
        self._servo_result = None
        self._servo_goal_handle = None
        self._servo_attempts = 0

        self._tick_timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            "ForestExecutor ready. "
            "Waiting for AREA_2 commands on /fsm/area_command "
            "(actions -> /fsm_command Int32 & /teensy/command JSON)"
        )

    def _area_command_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid area command JSON: {exc}")
            return

        if payload.get("command") != "start" or payload.get("area") != "AREA_2":
            return

        if self._state != _IDLE:
            self.get_logger().warn(
                "Forest task already running; ignoring duplicate start.")
            return

        self._plan_forest_task(payload)

    def _ack_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            sequence = int(payload["sequence"])
            status = str(payload.get("status", "done"))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"Ignoring invalid Teensy ack: {exc}")
            return
        self._acked_sequence = sequence
        self._ack_error = None if status == "done" else status

    # ── Planning ─────────────────────────────────────────────────────────

    def _plan_forest_task(self, payload: dict):
        try:
            r1_blocks = self._read_blocks(payload, "r1_blocks", 3)
            r2_blocks = self._read_blocks(payload, "r2_blocks", 4)
            fake_block = int(payload["fake_block"])
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_status("error", f"Invalid Forest payload: {exc}")
            return

        self.get_logger().info(
            f"Planning Forest task: r1={r1_blocks}, r2={r2_blocks}, fake={fake_block}")
        result = forest_path.plan(r1_blocks, r2_blocks, fake_block)

        if result["status"] == "impossible":
            self._publish_status(
                "impossible",
                "No legal R2 Forest route, even if R1 clears all of its KFS.")
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
                f"R1 must clear these blocks before/while R2 executes: {clear_set}")

        self._actions = actions
        self._index = 0
        self._state = _DISPATCH

    @staticmethod
    def _read_blocks(payload: dict, key: str, expected_count: int) -> list[int]:
        blocks = [int(value) for value in payload[key]]
        if len(blocks) != expected_count:
            raise ValueError(f"{key} must contain {expected_count} blocks")
        return blocks
    # ── Sequencer tick ───────────────────────────────────────────────────

    def _tick(self):
        if self._state == _DISPATCH:
            self._dispatch_next()
        elif self._state == _WAIT_ACK:
            self._check_ack()
        elif self._state == _WAIT_SERVO:
            self._check_servo()

    def _dispatch_next(self):
        if self._index >= len(self._actions):
            self._publish_status(
                "complete", f"Executed {len(self._actions)} Forest actions.")
            self._publish_area_complete()
            self._state = _IDLE
            return

        action, comment, meta = self._actions[self._index]
        if action == 'VISUAL_SERVO_BLOCK':
            self._servo_attempts = 0
            self._start_servo(meta, comment)
        else:
            self._publish_teensy_command(
                self._index + 1, len(self._actions), action, comment, meta)
            self._acked_sequence = None
            self._ack_error = None
            self._wait_deadline = self.get_clock().now().nanoseconds / 1e9 \
                + float(self.get_parameter('ack_timeout_s').value)
            self._state = _WAIT_ACK

    def _check_ack(self):
        if self._acked_sequence == self._index + 1:
            if self._ack_error is not None:
                self._fail(
                    f"Teensy reported '{self._ack_error}' for "
                    f"{self._actions[self._index][0]} (step {self._index + 1})")
                return
            self._advance()
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s > self._wait_deadline:
            self._fail(
                f"No Teensy ack for {self._actions[self._index][0]} "
                f"(step {self._index + 1}) within "
                f"{self.get_parameter('ack_timeout_s').value}s")

    def _advance(self):
        self._index += 1
        self._state = _DISPATCH

    def _fail(self, detail: str):
        self._publish_status("error", detail)
        self._state = _IDLE
        self._actions = []
        self._index = 0

    # ── Visual servo (AlignAndPick) ──────────────────────────────────────

    def _start_servo(self, meta: dict, comment: str):
        if not self._servo_client.server_is_ready():
            # Same non-blocking rule as NavInterface: never wait_for_server()
            # inside a timer callback. Retry next tick until the deadline.
            if self._server_wait_deadline is None:
                self._server_wait_deadline = \
                    self.get_clock().now().nanoseconds / 1e9 + 10.0
                self.get_logger().warn(
                    "align_and_pick server not ready, waiting up to 10 s...")
            elif self.get_clock().now().nanoseconds / 1e9 > self._server_wait_deadline:
                self._server_wait_deadline = None
                self._fail("align_and_pick action server unavailable")
            return

        self._server_wait_deadline = None
        self._wait_deadline = self.get_clock().now().nanoseconds / 1e9 \
            + float(self.get_parameter('servo_timeout_s').value)
        self._servo_status = None
        self._servo_result = None
        self._servo_goal_handle = None
        self._servo_attempts += 1

        goal = AlignAndPick.Goal()
        goal.block_id = int(meta.get('block', 0))
        goal.block_height = _HEIGHT_ENUM.get(meta.get('height', 'A'),
                                             AlignAndPick.Goal.HEIGHT_A)
        self.get_logger().info(
            f"Visual servo: {comment} (attempt {self._servo_attempts})")

        future = self._servo_client.send_goal_async(
            goal, feedback_callback=self._servo_feedback_callback)
        future.add_done_callback(self._servo_goal_response)
        self._state = _WAIT_SERVO

    def _servo_feedback_callback(self, fb):
        self.get_logger().info(
            f"servo: {fb.feedback.state} "
            f"offset={fb.feedback.current_offset_mm:.1f}mm",
            throttle_duration_sec=1.0)

    def _servo_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self._servo_status = GoalStatus.STATUS_ABORTED
            return
        self._servo_goal_handle = handle
        handle.get_result_async().add_done_callback(self._servo_result_callback)

    def _servo_result_callback(self, future):
        response = future.result()
        self._servo_status = response.status
        self._servo_result = response.result

    def _check_servo(self):
        if self._servo_status is not None:
            success = (
                self._servo_status == GoalStatus.STATUS_SUCCEEDED
                and self._servo_result is not None
                and self._servo_result.success
            )
            if success:
                self.get_logger().info(
                    "Visual servo aligned: final offset "
                    f"{self._servo_result.final_offset_mm:.1f} mm")
                self._advance()
                return
            detail = (self._servo_result.message
                      if self._servo_result else f"status {self._servo_status}")
            retries = int(self.get_parameter('servo_retries').value)
            if self._servo_attempts <= retries:
                self.get_logger().warn(
                    f"Visual servo failed ({detail}); retrying...")
                _action, comment, meta = self._actions[self._index]
                self._start_servo(meta, comment)
            else:
                self._fail(f"Visual servo failed after "
                           f"{self._servo_attempts} attempt(s): {detail}")
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s > self._wait_deadline:
            if self._servo_goal_handle is not None:
                self._servo_goal_handle.cancel_goal_async()
            self._fail(
                f"Visual servo timed out after "
                f"{self.get_parameter('servo_timeout_s').value}s")

    # ── Outbound ─────────────────────────────────────────────────────────

    def _publish_teensy_command(self, index: int, total: int, action: str,
                                comment: str, meta: dict):
        cmd_int = ACTION_TO_CMD.get(action)

        # DESCENT_FORWARD is a plain cmd_vel forward burst, not a Teensy integer.
        if cmd_int == "CMD_VEL":
            self._publish_forward(index, total, comment)
            return

        # 1. Publish to /fsm_command (Int32) if mapping exists
        if cmd_int is not None:
            msg_fsm = Int32()
            msg_fsm.data = cmd_int
            self._fsm_cmd_pub.publish(msg_fsm)
            self.get_logger().info(
                f"Forest command {index}/{total}: {action} -> /fsm_command {cmd_int}  # {comment}"
            )
        else:
            self.get_logger().warn(
                f"Forest command {index}/{total}: {action!r} has no integer mapping for /fsm_command. ({comment})"
            )

        # 2. Publish to /teensy/command (JSON String)
        msg_json = String()
        msg_json.data = json.dumps(
            {
                "source": "forest_executor",
                "sequence": index,
                "total": total,
                "command": action,
                "comment": comment,
                "meta": meta or {},
            }
        )
        self._teensy_cmd_pub.publish(msg_json)

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
