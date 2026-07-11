"""FSM node for mission_fsm."""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from .interfaces.nav_interface import NavInterface
from .task.task_manager import TaskManager


class FSMNode(Node):

    def __init__(self):

        super().__init__("mission_fsm")

        # Navigation interface (used by area states via node.nav)
        self.nav = NavInterface(self)

        self.task = TaskManager(self)

        # Flag set to True when the current area is finished
        self.area_complete = False

        # Forest (Area 2) state, set by the dashboard via set_forest_state()
        self.r1_blocks = []
        self.r2_blocks = []
        self.fake_block = 0
        self.no_downward_pick = False
        self.area_status = {}

        # One-shot guards so Area1State/Area2State only send their start
        # command once per arrival; reset on START/RESET/RETRY.
        self.align_triggered = False
        self.forest_triggered = False
        self.proximity_done = False

        # Subscribe to incoming signals from external nodes
        self.create_subscription(
            String,
            '/fsm/signal',
            self._signal_callback,
            10
        )

        self.create_subscription(
            String,
            '/fsm/area_status',
            self._area_status_callback,
            10
        )

        # Sequence instance
        from .sequences.lift_cross_sequence import LiftCrossSequence
        self._lift_cross_seq = LiftCrossSequence()

        # Subscribe to incoming fsm commands (to easily call sequences)
        self.create_subscription(
            Int32,
            '/fsm_command',
            self._fsm_command_callback,
            10
        )

        # Outbound: tells area-specific executor nodes (e.g. the Forest
        # executor) to start their task. Payload is a JSON string so no
        # custom .msg is needed; each executor filters on "area".
        self.area_cmd_pub = self.create_publisher(String, '/fsm/area_command', 10)

        # Used by Area1State to reverse after visual-servo alignment
        from geometry_msgs.msg import Vector3
        self.relative_move_pub = self.create_publisher(Vector3, '/relative_move', 10)

        # Bench: fire a single climb primitive straight at the Teensy bridge
        # (same wire the Forest executor uses), for testing the choreography
        # from the dashboard without a full mission. The bridge is single-
        # flight, so this is safe to fire one at a time.
        self.teensy_cmd_pub = self.create_publisher(String, '/teensy/command', 10)
        self._bench_seq = 0

        self.get_logger().info("FSMNode ready. Listening on /fsm/signal")

        self.timer = self.create_timer(0.1, self.loop)

    # ── Signal handler ────────────────────────────────────────────────────────

    def _signal_callback(self, msg: String):
        """Handle incoming signal strings from external nodes.

        Supported signals:
            area_complete  - Mark the current area as done (advance to next area)
        """
        signal = msg.data.strip().lower()
        self.get_logger().info(f"Received signal: '{signal}'")

        if signal == "area_complete":
            self.area_complete = True
        elif signal == "31":
            self.proximity_done = True
            self.get_logger().info("Proximity task complete (signal 31).")
        else:
            self.get_logger().warn(f"Unknown signal received: '{signal}'")

    def _fsm_command_callback(self, msg: Int32):
        """Handle incoming FSM command integers.

        Commands:
            300 - Trigger the Lift-Cross sequence
            99  - Emergency stop (resets sequence)
            Other numbers (e.g. 31) - routed to signal callback (e.g. proximity done)
        """
        cmd = msg.data
        if cmd == 300:
            self.get_logger().info("Received FSM command 300: Starting Lift-Cross sequence.")
            self._lift_cross_seq.start(self)
        elif cmd == 99:
            if self._lift_cross_seq.is_running():
                self.get_logger().warn("Received FSM command 99 (Emergency Stop): Resetting Lift-Cross sequence.")
                self._lift_cross_seq.reset()
        else:
            signal = str(cmd).strip()
            self.get_logger().info(f"Received /fsm_command: {signal}")
            fake_msg = type('FakeMsg', (), {'data': signal})()
            self._signal_callback(fake_msg)

    def _area_status_callback(self, msg: String):
        """Store the latest area-executor status for dashboard display."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid area status JSON: {exc}")
            return

        area = payload.get("area", "UNKNOWN")
        self.area_status[area] = payload

    # ── Trigger methods (called by the dashboard UI) ──────────────────────────

    def set_forest_state(self, r1_blocks, r2_blocks, fake_block, no_downward_pick=False):
        """Called by the dashboard once the operator types in the Forest
        state for Area 2 (3 R1 KFS blocks, 4 R2 KFS blocks, 1 Fake block).
        ``no_downward_pick`` forwards the planner bench-safety toggle
        (every KFS collected via an upward approach).

        Can be called any time before or during Area 2 -- Area2State will
        pick it up on its next tick and only fires the start command once
        nav has also arrived.
        """
        self.r1_blocks = list(r1_blocks)
        self.r2_blocks = list(r2_blocks)
        self.fake_block = int(fake_block)
        self.no_downward_pick = bool(no_downward_pick)
        self.forest_triggered = False
        self.get_logger().info(
            f"Dashboard → Forest state set: r1={self.r1_blocks} "
            f"r2={self.r2_blocks} fake={self.fake_block} "
            f"no_downward_pick={self.no_downward_pick}"
        )

    def trigger_dev_free_path(self, blocks, descend_exit=False):
        """Called by the dashboard: send a dev/free-path route straight to
        the Forest executor (bench mode -- no picks, no rule checks, never
        signals area_complete, so it cannot advance the mission FSM)."""
        msg = String()
        msg.data = json.dumps({
            "command": "dev_free_path",
            "blocks": [int(b) for b in blocks],
            "descend_exit": bool(descend_exit),
        })
        self.area_cmd_pub.publish(msg)
        self.get_logger().warn(
            f"Dashboard → DEV FREE PATH: {msg.data}"
        )

    def trigger_climb_primitive(self, command, meta=None):
        """Called by the dashboard bench panel: fire ONE Forest primitive
        (CLIMB_UP/CLIMB_DOWN/PICK_BLOCK_*/FORWARD_INIT/ROTATE_*) directly at
        the teensy_command bridge and let it run the IR-gated choreography.
        Bypasses the mission FSM entirely — bench testing only."""
        self._bench_seq += 1
        msg = String()
        msg.data = json.dumps({
            "source": "dashboard_bench",
            "sequence": self._bench_seq,
            "total": 1,
            "command": str(command),
            "comment": "dashboard bench",
            "meta": meta or {},
        })
        self.teensy_cmd_pub.publish(msg)
        self.get_logger().warn(
            f"Dashboard → BENCH primitive {command} (seq {self._bench_seq})"
            + (f" meta={meta}" if meta else ""))

    def trigger_start(self):
        """Begin mission from Area 1."""
        self.get_logger().info("Dashboard → START")
        self.task.current_state = "AREA_1"
        self.area_complete = False
        self.align_triggered = False
        self.forest_triggered = False
        self.proximity_done = False
        self.post_align_nav_triggered = False

    def trigger_stop(self):
        """Emergency stop: cancel active nav goal and hold current state."""
        self.get_logger().warn("Dashboard → EMERGENCY STOP")
        self.nav.cancel_goal()

    def trigger_reset(self):
        """Reset FSM back to IDLE and clear all flags."""
        self.get_logger().info("Dashboard → RESET")
        self.nav.cancel_goal()
        self.task.current_state = "IDLE"
        self.area_complete = False
        self.align_triggered = False
        self.forest_triggered = False
        self.post_align_move_complete = False
        self.proximity_done = False
        self.post_align_nav_triggered = False

    def trigger_retry_area(self, area_id: int):
        """Jump directly to a specific area (1, 2, or 3)."""
        key = f"AREA_{area_id}"
        if key not in self.task.states:
            self.get_logger().error(f"Dashboard → RETRY: unknown area {area_id}")
            return
        self.get_logger().info(f"Dashboard → RETRY AREA {area_id}")
        self.nav.cancel_goal()
        self.area_complete = False
        self.align_triggered = False
        self.forest_triggered = False
        self.post_align_move_complete = False
        self.proximity_done = False
        self.post_align_nav_triggered = False
        self.task.current_state = key

    # ── Main loop ─────────────────────────────────────────────────────────────

    def loop(self):
        self.task.update()
        if self._lift_cross_seq.is_running():
            self._lift_cross_seq.tick(self)


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()

    # Launch dashboard on the main thread (blocks until window is closed).
    # The dashboard event loop periodically spins the ROS 2 node synchronously,
    # ensuring absolute thread safety and preventing stack-smashing crashes.
    from .ui.dashboard import run_dashboard_main_thread
    run_dashboard_main_thread(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()