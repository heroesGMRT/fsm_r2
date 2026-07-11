"""FSM node for mission_fsm."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .interfaces.nav_interface import NavInterface
from .task.task_manager import TaskManager
from .task.states.area_1 import kill_proxymity_process


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
        # One-shot guards so Area1State/Area2State only send their start
        # command once per arrival; reset on START/RESET/RETRY.
        self.align_triggered = False
        self.forest_triggered = False
        self.proximity_done = False

        # Handle to the proxymity_launch.py subprocess spawned by Area1State
        # (Phase 3). Killed on Area 1 completion and on reset/retry so a
        # fresh instance can always launch cleanly next time.
        self.proxymity_process = None

        # Subscribe to incoming signals from external nodes
        self.create_subscription(
            String,
            '/fsm/signal',
            self._signal_callback,
            10
        )

        # Also listen on /fsm_command so nodes like proxymity_controller_node
        # that publish there are also handled (proxymity publishes "31" here).
        from std_msgs.msg import Int32
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

    def _fsm_command_callback(self, msg):
        """Handle integer commands from nodes publishing to /fsm_command
        (e.g. proxymity_controller_node publishes 31 when done).
        Routes into _signal_callback by converting the int to a string."""
        signal = str(msg.data).strip()
        self.get_logger().info(f"Received /fsm_command: {signal}")
        fake_msg = type('FakeMsg', (), {'data': signal})()
        self._signal_callback(fake_msg)

    # ── Trigger methods (called by the dashboard UI) ──────────────────────────

    def set_forest_state(self, r1_blocks, r2_blocks, fake_block):
        """Called by the dashboard once the operator types in the Forest
        state for Area 2 (3 R1 KFS blocks, 4 R2 KFS blocks, 1 Fake block).

        Can be called any time before or during Area 2 -- Area2State will
        pick it up on its next tick and only fires the start command once
        nav has also arrived.
        """
        self.r1_blocks = list(r1_blocks)
        self.r2_blocks = list(r2_blocks)
        self.fake_block = int(fake_block)
        self.forest_triggered = False
        self.get_logger().info(
            f"Dashboard → Forest state set: r1={self.r1_blocks} "
            f"r2={self.r2_blocks} fake={self.fake_block}"
        )

    def trigger_start(self):
        """Begin mission from Area 1."""
        self.get_logger().info("Dashboard → START")
        kill_proxymity_process(self)  # defensive: clear any stale instance before a fresh run
        self.task.current_state = "AREA_1"
        self.area_complete = False
        self.align_triggered = False
        self.forest_triggered = False
        self.proximity_done = False
        self.post_align_nav_triggered = False
        self.area2_retry_move_triggered = False

    def trigger_set_move(self, x: float, y: float):
        """Override Area 1's first move (move_x/move_y) in the active config at
        runtime. Takes effect on the next Area 1 run (in-memory only)."""
        from .config import loader
        loader.update_active("area_1", "move_x", float(x))
        loader.update_active("area_1", "move_y", float(y))
        self.get_logger().info(f"Dashboard → AREA 1 MOVE x={x} y={y}")

    def trigger_set_prox_forward_x(self, value: float):
        """Override the proximity node's forward_relative_x in the active
        config. Applied when Area 1 next launches proximity (in-memory only)."""
        from .config import loader
        loader.update_active("area_1", "prox_forward_x", float(value))
        self.get_logger().info(f"Dashboard → PROX FORWARD X {value}")

    def trigger_stop(self):
        """Emergency stop: cancel active nav goal and hold current state."""
        self.get_logger().warn("Dashboard → EMERGENCY STOP")
        self.nav.cancel_goal()

    def trigger_reset(self):
        """Reset FSM back to IDLE and clear all flags."""
        self.get_logger().info("Dashboard → RESET")
        self.nav.cancel_goal()
        kill_proxymity_process(self)
        self.task.current_state = "IDLE"
        self.area_complete = False
        self.align_triggered = False
        self.forest_triggered = False
        self.post_align_move_complete = False
        self.proximity_done = False
        self.post_align_nav_triggered = False
        self.area2_retry_move_triggered = False

    def trigger_retry_area(self, area_id: int):
        """Jump directly to a specific area (1, 2, or 3)."""
        key = f"AREA_{area_id}"
        if key not in self.task.states:
            self.get_logger().error(f"Dashboard → RETRY: unknown area {area_id}")
            return
        self.get_logger().info(f"Dashboard → RETRY AREA {area_id}")
        self.nav.cancel_goal()
        kill_proxymity_process(self)
        self.area_complete = False
        self.align_triggered = False
        self.forest_triggered = False
        self.post_align_move_complete = False
        self.proximity_done = False
        self.post_align_nav_triggered = False
        
        if area_id == 2:
            self.area2_retry_move_triggered = False
            self.area2_retry_move_complete = False
            
        self.task.current_state = key

    # ── Main loop ─────────────────────────────────────────────────────────────

    def loop(self):
        self.task.update()


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()

    # Launch dashboard on the main thread (blocks until window is closed).
    # The dashboard event loop periodically spins the ROS 2 node synchronously,
    # ensuring absolute thread safety and preventing stack-smashing crashes.
    from .ui.dashboard import run_dashboard_main_thread
    try:
        run_dashboard_main_thread(node)
    finally:
        # Tear down any proximity stack we spawned, even on Ctrl+C / SIGTERM /
        # crash. It runs in its own session (os.setsid), so the OS won't
        # cascade-kill it when we exit — without this it orphans and keeps
        # holding GPIO/camera (the "stale proximity nodes" problem).
        from .task.states.area_1 import kill_proxymity_process
        kill_proxymity_process(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()