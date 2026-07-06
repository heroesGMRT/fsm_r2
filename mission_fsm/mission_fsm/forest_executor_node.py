"""Forest executor node for Area 2.

Consumes the FSM Area 2 start command, runs the planner in ``path.py``, and
executes the generated action sequence ONE PRIMITIVE AT A TIME: every
primitive is published as JSON on ``/teensy/command`` and the executor waits
for a matching completion ack on ``/teensy/ack``::

    {"sequence": <int>, "status": "done" | "error"}

The Teensy bridge must ack only when the motion actually finishes.

The KFS visual servo is dropped from the flow for now (UPDATE.md A6): the
planner emits no ``VISUAL_SERVO_BLOCK`` and this node never calls the
``align_and_pick`` action server. The servo stack (``r2_stack/``) stays in
the tree, dormant, for later re-enablement. Picks are positioned purely by
the climb-on drive (odometry/IR).

Dev/free-path mode (UPDATE.md A5): a separate ``dev_free_path`` command on
``/fsm/area_command`` compiles an operator-given block route into climb
primitives (no picks, no rules) for bench-testing the climb choreography.
It never publishes ``area_complete``, so it cannot advance the mission FSM.

All waiting is timer-driven (10 Hz tick) — nothing blocks the executor, in
keeping with the rest of mission_fsm.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import path as forest_path

# Sequencer states
_IDLE = 'IDLE'
_DISPATCH = 'DISPATCH'
_WAIT_ACK = 'WAIT_ACK'


class ForestExecutorNode(Node):
    """Run the Area 2 Forest planner and sequence its primitives."""

    def __init__(self):
        super().__init__("forest_executor")

        self.declare_parameter('ack_timeout_s', 30.0)

        self._area_cmd_sub = self.create_subscription(
            String, "/fsm/area_command", self._area_command_callback, 10)
        self._ack_sub = self.create_subscription(
            String, "/teensy/ack", self._ack_callback, 10)
        self._teensy_cmd_pub = self.create_publisher(String, "/teensy/command", 10)
        self._fsm_signal_pub = self.create_publisher(String, "/fsm/signal", 10)
        self._status_pub = self.create_publisher(String, "/fsm/area_status", 10)

        # Sequencer state
        self._state = _IDLE
        self._actions = []
        self._index = 0
        self._wait_deadline = None
        self._acked_sequence = None
        self._ack_error = None
        self._dev_mode = False

        self._tick_timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            "ForestExecutor ready. Waiting for AREA_2 commands on /fsm/area_command")

    # ── Inbound: start command + acks ────────────────────────────────────

    def _area_command_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid area command JSON: {exc}")
            return

        command = payload.get("command")
        is_start = command == "start" and payload.get("area") == "AREA_2"
        is_dev = command == "dev_free_path"
        if not (is_start or is_dev):
            return

        if self._state != _IDLE:
            self.get_logger().warn(
                "Forest task already running; ignoring duplicate start.")
            return

        if is_dev:
            self._plan_dev_free_path(payload)
        else:
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
        no_downward_pick = bool(payload.get("no_downward_pick", False))

        self.get_logger().info(
            f"Planning Forest task: r1={r1_blocks}, r2={r2_blocks}, "
            f"fake={fake_block}, no_downward_pick={no_downward_pick}")
        result = forest_path.plan(r1_blocks, r2_blocks, fake_block,
                                  no_downward_pick=no_downward_pick)

        if result["status"] == "impossible":
            detail = "No legal R2 Forest route, even if R1 clears all of its KFS."
            if no_downward_pick:
                detail += (" NOTE: no_downward_pick is ON; the layout may be"
                           " solvable with downward picks allowed.")
            self._publish_status("impossible", detail)
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
                "no_downward_pick": no_downward_pick,
                "r2_collected": route["collected"],
                "exit_block": route["exit_block"],
                "positions": route["positions"],
                "action_count": len(actions),
            },
        )

        if clear_set:
            self.get_logger().warn(
                f"R1 must clear these blocks before/while R2 executes: {clear_set}")

        self._begin(actions, dev_mode=False)

    def _plan_dev_free_path(self, payload: dict):
        """UPDATE.md A5: bench-test climb choreography along an operator
        route — no picks, no rule checks, isolated from the competition
        planner. Payload: {"command": "dev_free_path", "blocks": [2, 5, ...],
        "descend_exit": bool}."""
        try:
            blocks = [int(b) for b in payload["blocks"]]
            descend_exit = bool(payload.get("descend_exit", False))
            actions = forest_path.dev_free_path_actions(
                blocks, descend_exit=descend_exit)
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_status("error", f"Invalid dev_free_path payload: {exc}")
            return

        self.get_logger().warn(
            f"DEV FREE-PATH mode: route {blocks} (descend_exit={descend_exit}) "
            f"— no picks, no rule checks, area_complete will NOT be signalled")
        self._publish_status(
            "dev_free_path_planned",
            {"blocks": blocks, "descend_exit": descend_exit,
             "action_count": len(actions)},
        )
        self._begin(actions, dev_mode=True)

    def _begin(self, actions, dev_mode):
        self._actions = actions
        self._index = 0
        self._dev_mode = dev_mode
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

    def _dispatch_next(self):
        if self._index >= len(self._actions):
            self._publish_status(
                "complete", f"Executed {len(self._actions)} Forest actions."
                + (" (dev_free_path)" if self._dev_mode else ""))
            if not self._dev_mode:
                self._publish_area_complete()
            self._state = _IDLE
            return

        action, comment, meta = self._actions[self._index]
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
        self._dev_mode = False

    # ── Outbound ─────────────────────────────────────────────────────────

    def _publish_teensy_command(self, index: int, total: int, action: str,
                                comment: str, meta: dict):
        msg = String()
        msg.data = json.dumps(
            {
                "source": "forest_executor",
                "sequence": index,
                "total": total,
                "command": action,
                "comment": comment,
                "meta": meta or {},
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
