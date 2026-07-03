"""Keyboard teleop node for publishing FSM integer commands.

Publishes ``std_msgs/msg/Int32`` messages to ``/fsm_command`` so operators can
test micro-ROS command handling from a terminal without typing ``ros2 topic
pub`` repeatedly.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


TOPIC_NAME = "/fsm_command"
MAX_ANGLE = 300
SERVO_STEP = 5
SYNC_STEP = 5


HOTKEYS = {
    "q": (None, "Quit"),
    "i": (None, "Enter an exact integer command"),
    "r": (20, "Odometry re-zero"),
    "x": (99, "Emergency stop toggle"),
    "a": (40, "Start autonomous drive sequence"),
    "m": (201, "Chassis Macro M"),
    "n": (202, "Chassis Macro N"),
    "1": (51, "Arm sequence S1"),
    "2": (52, "Arm sequence S2"),
    "3": (53, "Arm sequence S3"),
    "4": (54, "Arm sequence S4"),
    "5": (55, "Arm sequence S5"),
    "6": (56, "Arm sequence S6"),
    "t": (100, "Front lift up sequence"),
    "y": (101, "Back lift up sequence"),
    "g": (102, "Front lift down sequence"),
    "h": (103, "Back lift down sequence"),
    "u": (104, "Both lifts up sequence"),
    "j": (105, "Both lifts down sequence"),
    "T": (110, "Front lift up timed"),
    "Y": (111, "Back lift up timed"),
    "G": (112, "Front lift down timed"),
    "H": (113, "Back lift down timed"),
    "U": (114, "Both lifts up timed"),
    "J": (115, "Both lifts down timed"),
    "s": (60, "M5 stop"),
    "w": (61, "M5 CW"),
    "e": (62, "M5 CCW"),
    "d": (70, "M6 stop"),
    "f": (71, "M6 CW"),
    "c": (72, "M6 CCW"),
    "z": (4000, "Front DC lift stop"),
    "v": (4001, "Front DC lift up"),
    "b": (4002, "Front DC lift down"),
    "Z": (4010, "Back DC lift stop"),
    "V": (4011, "Back DC lift up"),
    "B": (4012, "Back DC lift down"),
    "[": (4020, "Front lift pneumatic release"),
    "]": (4021, "Front lift pneumatic deploy"),
    ";": (4030, "Back lift pneumatic release"),
    "'": (4031, "Back lift pneumatic deploy"),
    "7": (10, "Front base pneumatic high"),
    "8": (11, "Front base pneumatic low"),
    "9": (12, "Back base pneumatic high"),
    "0": (13, "Back base pneumatic low"),
    "-": (14, "Both base pneumatics high"),
    "=": (15, "Both base pneumatics low"),
    "o": (30, "Base servo preset 10"),
    "p": (31, "Base servo preset 20"),
    "k": (32, "Base servo preset 30"),
    "l": (33, "Base servo preset 40"),
    "/": (34, "Base servo preset 90"),
}


HELP_TEXT = """
FSM Keyboard Teleop
Topic: /fsm_command
Message type: std_msgs/msg/Int32

Common hotkeys
  r  -> 20    odometry re-zero
  x  -> 99    emergency stop
  a  -> 40    autonomous drive
  m  -> 201   chassis macro M
  n  -> 202   chassis macro N

Arm sequences
  1..6 -> 51..56

Lift sequences
  t/y/g/h/u/j -> 100/101/102/103/104/105
  T/Y/G/H/U/J -> 110/111/112/113/114/115

Manual overrides
  M5: s/w/e -> 60/61/62
  M6: d/f/c -> 70/71/72
  Front DC lift: z/v/b -> 4000/4001/4002
  Back  DC lift: Z/V/B -> 4010/4011/4012

Pneumatics
  [/ ] -> 4020 / 4021   front lift release / deploy
  ;/ ' -> 4030 / 4031   back lift release / deploy
  7/8  -> 10 / 11       front base high / low
  9/0  -> 12 / 13       back base high / low
  -/=  -> 14 / 15       both base high / low

Base servo presets
  o/p/k/l// -> 30/31/32/33/34

Dynamic servo angles
  , and .   decrease/increase ASME angle by 5 deg, publish 1000 + angle
  < and >   decrease/increase sync angle by 5 deg, publish 2000 + angle

Exact integer mode
  i         type any integer command, then press Enter

Other
  ?         show help
  q         quit
"""


class KeyboardTeleopNode(Node):
    """Publish integer FSM commands from keyboard input."""

    def __init__(self):
        super().__init__("fsm_keyboard_teleop")
        self._publisher = self.create_publisher(Int32, TOPIC_NAME, 10)
        self._stdin_fd = sys.stdin.fileno()
        self._old_term_settings = termios.tcgetattr(self._stdin_fd)
        self._line_mode = False
        self._numeric_buffer = ""
        self._servo_angle = 0
        self._sync_angle = 0

        tty.setcbreak(self._stdin_fd)
        self.create_timer(0.05, self._poll_keyboard)

        self.get_logger().info(
            f"Keyboard teleop ready. Publishing Int32 commands on {TOPIC_NAME}"
        )
        print(HELP_TEXT)
        print("Current ASME angle: 0 deg (command 1000)")
        print("Current sync angle: 0 deg (command 2000)")
        print("Waiting for keypresses...")
        sys.stdout.flush()

    def destroy_node(self):
        self._restore_terminal()
        return super().destroy_node()

    def _restore_terminal(self):
        if self._old_term_settings is not None:
            termios.tcsetattr(
                self._stdin_fd,
                termios.TCSADRAIN,
                self._old_term_settings,
            )
            self._old_term_settings = None

    def _poll_keyboard(self):
        if not self._stdin_ready():
            return

        key = os.read(self._stdin_fd, 1).decode(errors="ignore")
        if not key:
            return

        if self._line_mode:
            self._handle_numeric_mode_key(key)
            return

        if key == ",":
            self._adjust_servo_angle(-SERVO_STEP)
            return
        if key == ".":
            self._adjust_servo_angle(SERVO_STEP)
            return
        if key == "<":
            self._adjust_sync_angle(-SYNC_STEP)
            return
        if key == ">":
            self._adjust_sync_angle(SYNC_STEP)
            return
        if key == "i":
            self._enter_numeric_mode()
            return
        if key == "?":
            print(HELP_TEXT)
            sys.stdout.flush()
            return
        if key == "q":
            self.get_logger().info("Quit requested from keyboard.")
            raise KeyboardInterrupt

        command_entry = HOTKEYS.get(key)
        if command_entry is None:
            printable = repr(key)
            print(f"Unknown key {printable}. Press ? for help.")
            sys.stdout.flush()
            return

        command, label = command_entry
        if command is None:
            print(f"Key {key!r}: {label}")
            sys.stdout.flush()
            return
        self._publish_command(command, label)

    def _stdin_ready(self) -> bool:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        return bool(readable)

    def _enter_numeric_mode(self):
        self._line_mode = True
        self._numeric_buffer = ""
        print("\nExact integer mode. Type a command and press Enter. Esc cancels.")
        print("> ", end="", flush=True)

    def _handle_numeric_mode_key(self, key: str):
        if key in ("\r", "\n"):
            print()
            self._submit_numeric_buffer()
            return
        if key == "\x1b":
            print("\nCanceled exact integer mode.")
            self._line_mode = False
            self._numeric_buffer = ""
            sys.stdout.flush()
            return
        if key in ("\x7f", "\b"):
            if self._numeric_buffer:
                self._numeric_buffer = self._numeric_buffer[:-1]
                print("\b \b", end="", flush=True)
            return
        if key.isdigit() or (key == "-" and not self._numeric_buffer):
            self._numeric_buffer += key
            print(key, end="", flush=True)
            return

    def _submit_numeric_buffer(self):
        raw = self._numeric_buffer.strip()
        self._line_mode = False
        self._numeric_buffer = ""

        if not raw:
            print("No command entered.")
            sys.stdout.flush()
            return

        try:
            command = int(raw)
        except ValueError:
            print(f"Invalid integer: {raw}")
            sys.stdout.flush()
            return

        self._publish_command(command, "Exact integer input")

    def _adjust_servo_angle(self, delta: int):
        self._servo_angle = min(MAX_ANGLE, max(0, self._servo_angle + delta))
        self._publish_command(
            1000 + self._servo_angle,
            f"ASME servo angle -> {self._servo_angle} deg",
        )

    def _adjust_sync_angle(self, delta: int):
        self._sync_angle = min(MAX_ANGLE, max(0, self._sync_angle + delta))
        self._publish_command(
            2000 + self._sync_angle,
            f"Sync angle -> {self._sync_angle} deg",
        )

    def _publish_command(self, command: int, label: str):
        msg = Int32()
        msg.data = command
        self._publisher.publish(msg)
        print(f"Published {command}: {label}")
        sys.stdout.flush()
        self.get_logger().info(f"Published command {command} ({label})")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
