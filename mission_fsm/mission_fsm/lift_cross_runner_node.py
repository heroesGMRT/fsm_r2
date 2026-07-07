"""Standalone Lift-Cross sequence runner node.

Subscribes to /fsm_command and drives LiftCrossSequence when it receives
command 300 (the 'P' key in keyboard_teleop).  Designed for bench-testing
the sequence without launching the full fsm_node + dashboard.

Run with:
    ros2 run mission_fsm lift_cross_runner

Then in another terminal:
    ros2 run mission_fsm keyboard_teleop    # press P to trigger
    # or:
    ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "data: 300"

Commands handled:
    300  — Start the lift-cross sequence
     99  — Emergency stop (reset sequence mid-run)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from .sequences.lift_cross_sequence import LiftCrossSequence

CMD_LIFT_CROSS = 300
CMD_ESTOP      = 99
TICK_HZ        = 10.0   # how often to call seq.tick()


class LiftCrossRunnerNode(Node):
    """Thin runner that wires /fsm_command → LiftCrossSequence."""

    def __init__(self):
        super().__init__("lift_cross_runner")

        # Sequence with default durations — tweak args as needed
        self._seq = LiftCrossSequence(
            drive_duration_1=3.0,
            drive_duration_2=2.0,
            drive_duration_3=2.0,
            lift_settle_time=1.5,
        )

        self.create_subscription(Int32, "/fsm_command", self._cmd_cb, 10)
        self.create_timer(1.0 / TICK_HZ, self._tick)

        self.get_logger().info(
            "LiftCrossRunner ready.\n"
            "  Press P in keyboard_teleop (cmd 300) to start.\n"
            "  Press x (cmd 99) for emergency stop."
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Int32) -> None:
        cmd = msg.data

        if cmd == CMD_LIFT_CROSS:
            if self._seq.is_running():
                self.get_logger().warn(
                    "Lift-cross already running — ignoring duplicate cmd 300."
                )
                return
            self.get_logger().info("CMD 300 → Starting lift-cross sequence.")
            self._seq.start(self)

        elif cmd == CMD_ESTOP:
            if self._seq.is_running():
                self.get_logger().warn("CMD 99 (E-STOP) → Resetting lift-cross sequence.")
                self._seq.reset()

    def _tick(self) -> None:
        if self._seq.is_running():
            done = self._seq.tick(self)
            if done:
                self.get_logger().info("Lift-cross sequence finished ✓")


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LiftCrossRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
