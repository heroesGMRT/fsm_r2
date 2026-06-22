"""Interactive test publisher to simulate external packages sending signals to the FSM.

Run this in a separate terminal while fsm_node is running:

    python3 -m mission_fsm.test_signal_pub

Or after installing the package:

    ros2 run mission_fsm test_signal_pub

Available signals to send:
    spear_found    -> Triggers SEARCH -> PICKUP transition
    search_failed  -> Triggers SEARCH -> RECOVERY transition
    recovery_done  -> Triggers RECOVERY -> SEARCH transition
    task_complete  -> Advances to next mission area
    navigate       -> Forces FSM back to NAVIGATE state
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

 
HELP_TEXT = """
╔══════════════════════════════════════════════╗
║        FSM Signal Test Publisher             ║
╠══════════════════════════════════════════════╣
║  Publishes to: /fsm/signal                   ║
║                                              ║
║  Available signals:                          ║
║    spear_found   -> SEARCH -> PICKUP         ║
║    search_failed -> SEARCH -> RECOVERY       ║
║    recovery_done -> RECOVERY -> SEARCH       ║
║    task_complete -> Advance mission area     ║
║    navigate      -> Force NAVIGATE state     ║
║                                              ║
║  Type a signal name and press Enter.         ║
║  Type 'quit' or Ctrl+C to exit.              ║
╚══════════════════════════════════════════════╝
"""

VALID_SIGNALS = {
    "spear_found",
    "search_failed",
    "recovery_done",
    "task_complete",
    "navigate",
}


class SignalPublisher(Node):

    def __init__(self):
        super().__init__("fsm_signal_publisher")
        self._pub = self.create_publisher(String, '/fsm/signal', 10)
        self.get_logger().info("Signal publisher ready on /fsm/signal")

    def send(self, signal: str):
        msg = String()
        msg.data = signal
        self._pub.publish(msg)
        self.get_logger().info(f"Published signal: '{signal}'")


def main(args=None):
    rclpy.init(args=args)
    node = SignalPublisher()

    print(HELP_TEXT)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            try:
                signal = input("Signal > ").strip().lower()
            except EOFError:
                break

            if signal in ("quit", "exit", "q"):
                print("Exiting signal publisher.")
                break

            if signal == "":
                continue

            if signal not in VALID_SIGNALS:
                print(f"Unknown signal '{signal}'. Valid signals: {', '.join(sorted(VALID_SIGNALS))}")
                continue

            node.send(signal)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
