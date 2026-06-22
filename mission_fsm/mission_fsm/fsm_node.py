"""FSM node for mission_fsm."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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

        # Subscribe to incoming signals from external nodes
        self.create_subscription(
            String,
            '/fsm/signal',
            self._signal_callback,
            10
        )

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
        else:
            self.get_logger().warn(f"Unknown signal received: '{signal}'")

    # ── Trigger methods (called by the dashboard UI) ──────────────────────────

    def trigger_start(self):
        """Begin mission from Area 1."""
        self.get_logger().info("Dashboard → START")
        self.task.current_state = "AREA_1"
        self.area_complete = False

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

    def trigger_retry_area(self, area_id: int):
        """Jump directly to a specific area (1, 2, or 3)."""
        key = f"AREA_{area_id}"
        if key not in self.task.states:
            self.get_logger().error(f"Dashboard → RETRY: unknown area {area_id}")
            return
        self.get_logger().info(f"Dashboard → RETRY AREA {area_id}")
        self.nav.cancel_goal()
        self.area_complete = False
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
    run_dashboard_main_thread(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()