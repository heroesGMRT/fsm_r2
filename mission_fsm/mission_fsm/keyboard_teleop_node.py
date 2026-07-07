"""Keyboard teleop node for publishing FSM integer commands and cmd_vel.

Publishes:
  - std_msgs/msg/Int32      -> /fsm_command  (FSM integer commands)
  - geometry_msgs/msg/Twist -> /cmd_vel      (arrow-key velocity control)

Run with:
  ros2 run mission_fsm keyboard_teleop_node
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
from geometry_msgs.msg import Twist


TOPIC_FSM     = "/fsm_command"
TOPIC_VEL     = "/cmd_vel"
MAX_ANGLE     = 300
SERVO_STEP    = 5
SYNC_STEP     = 5
LINEAR_SPEED  = 0.3   # m/s
ANGULAR_SPEED = 0.8   # rad/s


# key -> (fsm_command_int_or_None, description)
HOTKEYS: dict[str, tuple[int | None, str]] = {
    # ── System ────────────────────────────────────────────────────────────────
    "r": (20,   "Odometry re-zero  — reset odom x/y/θ → 0.0"),
    "x": (99,   "Emergency stop toggle  — kill M5/M6 & reset sequence"),

    # ── Autonomous / Chassis macros ───────────────────────────────────────────
    "a": (40,   "Start autonomous drive sequence"),
    "m": (201,  "Chassis Macro M  (odom-based position sequence)"),
    "n": (202,  "Chassis Macro N  (odom-based position sequence)"),
    "P": (300,  "Lift-cross sequence  (lift up → fwd → front down → fwd → back down → fwd again)"),

    # ── Arm sequences S1-S6 (safe: only runs when robot is IDLE) ─────────────
    "1": (51,   "Arm Sequence S1  [safe — runs only when IDLE]"),
    "2": (52,   "Arm Sequence S2  [safe — runs only when IDLE]"),
    "3": (53,   "Arm Sequence S3  [safe — runs only when IDLE]"),
    "4": (54,   "Arm Sequence S4  [safe — runs only when IDLE]"),
    "5": (55,   "Arm Sequence S5  [safe — runs only when IDLE]"),
    "6": (56,   "Arm Sequence S6  [safe — runs only when IDLE]"),

    # ── Lift sequences — encoder / limit-switch based ─────────────────────────
    "t": (100,  "Front lift UP    (encoder/limit-switch sequence)"),
    "y": (101,  "Back lift UP     (encoder/limit-switch sequence)"),
    "g": (102,  "Front lift DOWN  (encoder/limit-switch sequence)"),
    "h": (103,  "Back lift DOWN   (encoder/limit-switch sequence)"),
    "u": (104,  "BOTH lifts UP    (encoder/limit-switch sequence)"),
    "j": (105,  "BOTH lifts DOWN  (encoder/limit-switch sequence)"),

    # ── Lift sequences — timed fallback ──────────────────────────────────────
    "T": (110,  "Front lift UP    (timed fallback sequence)"),
    "Y": (111,  "Back lift UP     (timed fallback sequence)"),
    "G": (112,  "Front lift DOWN  (timed fallback sequence)"),
    "H": (113,  "Back lift DOWN   (timed fallback sequence)"),
    "U": (114,  "BOTH lifts UP    (timed fallback sequence)"),
    "J": (115,  "BOTH lifts DOWN  (timed fallback sequence)"),

    # ── M5 — arm lift motor ───────────────────────────────────────────────────
    "s": (60,   "M5 STOP"),
    "w": (61,   "M5 Rotate CW"),
    "e": (62,   "M5 Rotate CCW"),

    # ── M6 — gripper spin motor ───────────────────────────────────────────────
    "d": (70,   "M6 STOP"),
    "f": (71,   "M6 Rotate CW"),
    "c": (72,   "M6 Rotate CCW"),

    # ── DC lift motors ────────────────────────────────────────────────────────
    "z": (4000, "Front DC Lift  STOP"),
    "v": (4001, "Front DC Lift  RAISE"),
    "b": (4002, "Front DC Lift  LOWER"),
    "Z": (4010, "Back DC Lift   STOP"),
    "V": (4011, "Back DC Lift   RAISE"),
    "B": (4012, "Back DC Lift   LOWER"),

    # ── Pneumatics ────────────────────────────────────────────────────────────
    "[": (4020, "Front lift pneumatic  RELEASE"),
    "]": (4021, "Front lift pneumatic  DEPLOY"),
    ";": (4030, "Back lift pneumatic   RELEASE"),
    "'": (4031, "Back lift pneumatic   DEPLOY"),
    "7": (10,   "Front base pneumatic  HIGH"),
    "8": (11,   "Front base pneumatic  LOW"),
    "9": (12,   "Back base pneumatic   HIGH"),
    "0": (13,   "Back base pneumatic   LOW"),
    "-": (14,   "BOTH base pneumatics  HIGH"),
    "=": (15,   "BOTH base pneumatics  LOW"),

    # ── Base servo presets ────────────────────────────────────────────────────
    "o": (30,   "Base servo preset → state 10"),
    "p": (31,   "Base servo preset → state 20"),
    "k": (32,   "Base servo preset → state 30"),
    "l": (33,   "Base servo preset → state 40"),
    "/": (34,   "Base servo preset → state 90"),
}


HELP_TEXT = """\
╔══════════════════════════════════════════════════════════════════════════════╗
║           FSM Keyboard Teleop — /fsm_command (Int32) & /cmd_vel (Twist)     ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  SYSTEM CONTROL                                                              ║
║    r → 20    Odometry re-zero  (reset odom x/y/θ → 0.0)                    ║
║    x → 99    Emergency stop toggle  (kills M5/M6, resets sequence)          ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  AUTONOMOUS / CHASSIS MACROS                                                 ║
║    a → 40    Start autonomous drive sequence                                 ║
║    m → 201   Chassis Macro M  (odom-based position sequence)                ║
║    n → 202   Chassis Macro N  (odom-based position sequence)                ║
║    P → 300   Lift-cross sequence (lift↑ → fwd → front↓ → fwd → back↓ → fwd)  ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  ARM SEQUENCES  [safe — only runs when robot is IDLE]                        ║
║    1 → 51    Arm Sequence S1                                                 ║
║    2 → 52    Arm Sequence S2                                                 ║
║    3 → 53    Arm Sequence S3                                                 ║
║    4 → 54    Arm Sequence S4                                                 ║
║    5 → 55    Arm Sequence S5                                                 ║
║    6 → 56    Arm Sequence S6                                                 ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  LIFT SEQUENCES — Encoder / Limit-switch based                               ║
║    t → 100   Front lift UP                                                   ║
║    y → 101   Back lift UP                                                    ║
║    g → 102   Front lift DOWN (retract)                                       ║
║    h → 103   Back lift DOWN  (retract)                                       ║
║    u → 104   BOTH lifts UP                                                   ║
║    j → 105   BOTH lifts DOWN                                                 ║
║  LIFT SEQUENCES — Timed fallback                                              ║
║    T → 110   Front lift UP   (timed)                                         ║
║    Y → 111   Back lift UP    (timed)                                         ║
║    G → 112   Front lift DOWN (timed)                                         ║
║    H → 113   Back lift DOWN  (timed)                                         ║
║    U → 114   BOTH lifts UP   (timed)                                         ║
║    J → 115   BOTH lifts DOWN (timed)                                         ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  MANUAL MOTOR OVERRIDES                                                      ║
║    M5 (arm lift motor)     s→60 STOP  | w→61 CW  | e→62 CCW                ║
║    M6 (gripper spin)       d→70 STOP  | f→71 CW  | c→72 CCW                ║
║    Front DC lift           z→4000 STOP | v→4001 UP | b→4002 DOWN            ║
║    Back DC lift            Z→4010 STOP | V→4011 UP | B→4012 DOWN            ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  PNEUMATICS                                                                  ║
║    [ → 4020  Front lift pneumatic RELEASE                                   ║
║    ] → 4021  Front lift pneumatic DEPLOY                                    ║
║    ; → 4030  Back lift pneumatic RELEASE                                    ║
║    ' → 4031  Back lift pneumatic DEPLOY                                     ║
║    7 → 10    Front base pneumatic HIGH                                       ║
║    8 → 11    Front base pneumatic LOW                                        ║
║    9 → 12    Back base pneumatic HIGH                                        ║
║    0 → 13    Back base pneumatic LOW                                         ║
║    - → 14    BOTH base pneumatics HIGH                                       ║
║    = → 15    BOTH base pneumatics LOW                                        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  BASE SERVO PRESETS                                                          ║
║    o→30 (st.10) | p→31 (st.20) | k→32 (st.30) | l→33 (st.40) | /→34(90)  ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  DYNAMIC ASME SERVO  (publishes 1000 + angle, range 0°-300°)                ║
║    ,  decrease angle by 5°        .  increase angle by 5°                   ║
║  SYNC TRACKING SERVO (publishes 2000 + angle, range 0°-300°)                ║
║    <  decrease angle by 5°        >  increase angle by 5°                   ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  CMD_VEL (arrow keys)                                                        ║
║    ↑  Forward  (+linear.x)        ↓  Backward (-linear.x)                  ║
║    ←  Turn left (+angular.z)      →  Turn right (-angular.z)               ║
║    SPACE  Stop (zero twist)                                                  ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  OTHER                                                                       ║
║    i   Enter exact integer command, then press Enter  (ESC cancels)         ║
║    ?   Show this help                                                        ║
║    q   Quit                                                                  ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""


class KeyboardTeleopNode(Node):
    """Publish integer FSM commands and Twist velocities from keyboard input."""

    # ANSI escape prefix (sent by arrow keys)
    _ARROW_ESCAPE = "\x1b["

    def __init__(self):
        super().__init__("fsm_keyboard_teleop")
        self._pub_fsm = self.create_publisher(Int32, TOPIC_FSM, 10)
        self._pub_vel = self.create_publisher(Twist, TOPIC_VEL, 10)

        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

        self._line_mode = False
        self._numeric_buf = ""
        self._servo_angle = 0
        self._sync_angle  = 0

        # arrow-key multi-byte reader state
        self._esc_buf = ""

        self.create_timer(0.02, self._poll_keyboard)

        self.get_logger().info(
            f"Keyboard teleop ready — FSM: {TOPIC_FSM}  VEL: {TOPIC_VEL}"
        )
        print(HELP_TEXT)
        print(f"ASME angle: {self._servo_angle}°  (cmd {1000+self._servo_angle})")
        print(f"Sync angle: {self._sync_angle}°  (cmd {2000+self._sync_angle})")
        print("Waiting for keypresses…  (? for help, q to quit)\n")
        sys.stdout.flush()

    # ── lifecycle ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._restore_terminal()
        return super().destroy_node()

    def _restore_terminal(self):
        if self._old_term is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
            self._old_term = None

    # ── keyboard polling ───────────────────────────────────────────────────────

    def _poll_keyboard(self):
        while self._stdin_ready():
            ch = os.read(self._stdin_fd, 1).decode(errors="ignore")
            if not ch:
                return
            self._process_char(ch)

    def _stdin_ready(self) -> bool:
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        return bool(r)

    def _process_char(self, ch: str):
        # ── collect ANSI escape sequences (arrow keys) ─────────────────────
        if self._esc_buf or ch == "\x1b":
            self._esc_buf += ch
            # wait for a letter to terminate the sequence
            if len(self._esc_buf) >= 3:
                seq = self._esc_buf
                self._esc_buf = ""
                self._handle_escape(seq)
            return

        if self._line_mode:
            self._handle_numeric_key(ch)
            return

        # ── space → stop vel ───────────────────────────────────────────────
        if ch == " ":
            self._publish_vel(0.0, 0.0, "STOP cmd_vel")
            return

        # ── dynamic servo adjust ───────────────────────────────────────────
        if ch == ",":
            self._servo_angle = max(0, self._servo_angle - SERVO_STEP)
            self._publish_fsm(1000 + self._servo_angle,
                              f"ASME servo → {self._servo_angle}°")
            return
        if ch == ".":
            self._servo_angle = min(MAX_ANGLE, self._servo_angle + SERVO_STEP)
            self._publish_fsm(1000 + self._servo_angle,
                              f"ASME servo → {self._servo_angle}°")
            return
        if ch == "<":
            self._sync_angle = max(0, self._sync_angle - SYNC_STEP)
            self._publish_fsm(2000 + self._sync_angle,
                              f"Sync angle → {self._sync_angle}°")
            return
        if ch == ">":
            self._sync_angle = min(MAX_ANGLE, self._sync_angle + SYNC_STEP)
            self._publish_fsm(2000 + self._sync_angle,
                              f"Sync angle → {self._sync_angle}°")
            return

        # ── meta keys ──────────────────────────────────────────────────────
        if ch == "i":
            self._enter_numeric_mode()
            return
        if ch == "?":
            print(HELP_TEXT)
            sys.stdout.flush()
            return
        if ch == "q":
            self.get_logger().info("Quit requested.")
            raise KeyboardInterrupt

        # ── hotkeys ────────────────────────────────────────────────────────
        entry = HOTKEYS.get(ch)
        if entry is None:
            print(f"Unknown key {ch!r}.  Press ? for help.")
            sys.stdout.flush()
            return
        cmd, label = entry
        self._publish_fsm(cmd, label)

    def _handle_escape(self, seq: str):
        """Decode ANSI arrow key sequences → cmd_vel."""
        mapping = {
            "\x1b[A": (LINEAR_SPEED,   0.0,           "FORWARD"),
            "\x1b[B": (-LINEAR_SPEED,  0.0,           "BACKWARD"),
            "\x1b[D": (0.0,            ANGULAR_SPEED, "TURN LEFT"),
            "\x1b[C": (0.0,           -ANGULAR_SPEED, "TURN RIGHT"),
        }
        vel = mapping.get(seq)
        if vel:
            lin, ang, label = vel
            self._publish_vel(lin, ang, label)
        # else: unknown escape sequence — silently ignore

    # ── numeric input mode ────────────────────────────────────────────────────

    def _enter_numeric_mode(self):
        self._line_mode = True
        self._numeric_buf = ""
        print("\nExact integer mode — type a number and press Enter  (ESC cancels)")
        print("> ", end="", flush=True)

    def _handle_numeric_key(self, ch: str):
        if ch in ("\r", "\n"):
            print()
            self._submit_numeric()
            return
        if ch == "\x1b":
            print("\nCancelled.")
            self._line_mode = False
            self._numeric_buf = ""
            sys.stdout.flush()
            return
        if ch in ("\x7f", "\b"):
            if self._numeric_buf:
                self._numeric_buf = self._numeric_buf[:-1]
                print("\b \b", end="", flush=True)
            return
        if ch.isdigit() or (ch == "-" and not self._numeric_buf):
            self._numeric_buf += ch
            print(ch, end="", flush=True)

    def _submit_numeric(self):
        raw = self._numeric_buf.strip()
        self._line_mode = False
        self._numeric_buf = ""
        if not raw:
            print("No command entered.")
            sys.stdout.flush()
            return
        try:
            cmd = int(raw)
        except ValueError:
            print(f"Invalid integer: {raw!r}")
            sys.stdout.flush()
            return
        self._publish_fsm(cmd, "Exact integer input")

    # ── publishers ────────────────────────────────────────────────────────────

    def _publish_fsm(self, command: int, label: str):
        msg = Int32()
        msg.data = command
        self._pub_fsm.publish(msg)
        print(f"[FSM]  {command:>5}  {label}")
        sys.stdout.flush()
        self.get_logger().info(f"FSM cmd {command} ({label})")

    def _publish_vel(self, lin: float, ang: float, label: str):
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._pub_vel.publish(msg)
        print(f"[VEL]  lin={lin:+.2f}  ang={ang:+.2f}  {label}")
        sys.stdout.flush()
        self.get_logger().info(f"Twist lin={lin} ang={ang} ({label})")


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
