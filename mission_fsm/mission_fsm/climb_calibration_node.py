"""Operator-driven, step-by-step climb CALIBRATION script (handoff v3 §8).

Walks the Forest climb primitives ONE operator-advance apart against the
running ``teensy_command`` bridge, pushing a labelled block of tunables onto
the bridge first so tuned values transfer straight into the §3/§4 sequences
(they read the SAME bridge parameters at runtime — there is no second copy to
keep in sync). Goal for the ``up`` run: R2 starts a configurable ``START_GAP``
in front of block 2 and ends CENTERED on block 2.

It reuses ``climb_test``'s wire taps, so every /ir_sensors transition,
/fsm_command code and /relative_move the bridge issues is printed with a
timing timeline — read the real durations off it and retune.

Usage (bridge running: ``ros2 run mission_fsm teensy_command``):

    ros2 run mission_fsm climb_calibration up      # approach + climb onto block 2
    ros2 run mission_fsm climb_calibration down     # climb down onto the lower block

Motion is ``/relative_move`` only. Press Enter to fire the next primitive;
Ctrl+C publishes ``{0,0,0}`` (STOP) — but note the bridge does the driving, so
the robot may finish its current move; kill the bridge / e-stop for a hard stop.
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3

from .climb_test_node import ClimbTestNode


# ── CALIBRATION CONSTANTS (tune here; pushed onto the bridge before the run) ──
# START_GAP: how far in front of block 2 R2 starts (fed to forward_init_m).
START_GAP_M = 0.30

# Bridge params mirrored into the §3/§4 sequences. Keep this the single place
# you tune; the climb primitives read these exact parameters at runtime.
BRIDGE_TUNABLES = {
    "motion_backend":        "relative_move",  # /relative_move only (§1)
    "creep_backend":         "ceiling",        # ceiling + {0,0,0} cut (§1)
    "creep_ceiling_m":       0.9,              # generous creep ceiling
    "creep_ceiling_short_m": 0.25,             # SHORT ceiling at a tipping edge
    "relmove_speed_est":     0.1,              # m/s, feeds the settle wait
    "relmove_settle_s":      1.0,              # settle margin after each move
    "mech_dwell_s":          1.5,              # dwell after every mech code (§)
    "d_center_up_m":         0.20,             # §3.8 centering on block 2
    "d_center_down_m":       0.20,             # §4.7 centering on lower block
    "enable_pick":           False,            # calibration skips the pick (§8)
}

# Primitive walk per mode: (human description, bridge command, meta).
_WALKS = {
    "up": [
        ("approach block-2 face (START_GAP)", "FORWARD_INIT", {}),
        ("climb UP onto block 2",             "CLIMB_UP",     {}),
    ],
    "down": [
        ("climb DOWN onto the lower block",   "CLIMB_DOWN",   {}),
    ],
}


class ClimbCalibrationNode(ClimbTestNode):
    """climb_test wire-tap + a best-effort {0,0,0} STOP publisher for abort."""

    def __init__(self):
        super().__init__()
        self._relmove_pub = self.create_publisher(Vector3, "/relative_move", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_stop(self):
        # {0,0,0} on /relative_move is the documented STOP (§0/§1); also zero
        # cmd_vel. Best effort — the bridge owns the move in flight.
        self._relmove_pub.publish(Vector3())
        self._cmd_vel_pub.publish(Twist())


def _spin_until_ack(node, timeout_s):
    deadline = time.monotonic() + timeout_s
    while node.ack is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.ack


def main(args=None):
    mode = (sys.argv[1] if len(sys.argv) > 1 else "up").lower()
    if mode not in _WALKS:
        print(f"usage: climb_calibration [up|down]  (got '{mode}')",
              file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = ClimbCalibrationNode()
    exit_code = 0
    try:
        deadline = time.monotonic() + 3.0
        while not node.bridge_listening() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.bridge_listening():
            print("ERROR: nothing subscribed to /teensy/command — start the "
                  "bridge first: ros2 run mission_fsm teensy_command",
                  file=sys.stderr)
            return 1

        assignments = [f"{k}={v}" for k, v in {
            **BRIDGE_TUNABLES, "forward_init_m": START_GAP_M}.items()]
        print("Pushing calibration constants onto the bridge:")
        if not node.push_params(assignments):
            print("ERROR: could not set bridge parameters", file=sys.stderr)
            return 1

        walk = _WALKS[mode]
        print(f"\n=== climb calibration: {mode.upper()} "
              f"({len(walk)} primitive(s), START_GAP={START_GAP_M} m) ===")
        for i, (desc, command, meta) in enumerate(walk, start=1):
            try:
                input(f"\n[{i}/{len(walk)}] Press Enter to fire: {desc} "
                      f"({command})   (Ctrl+C aborts) ...")
            except EOFError:
                print("\n(no TTY) — aborting the walk.")
                break
            node.ack = None
            node.send(i, command, meta)
            ack = _spin_until_ack(node, timeout_s=120.0)
            if ack is None:
                print("\nTIMEOUT waiting for ack — the bridge may still be "
                      "executing. Stopping.")
                exit_code = 1
                break
            if ack[1] != "done":
                print(f"\nBridge reported {ack[1].upper()} — stopping the walk.")
                exit_code = 1
                break
            print(f"--- '{desc}' complete. Measure/tune, then advance. ---")

        print("\nDone. Retune in this file's CONSTANTS block (they push onto "
              "the bridge), or live with: ros2 param set /teensy_command "
              "<name> <value>.")
    except KeyboardInterrupt:
        node.publish_stop()
        print("\nAborted — published {0,0,0} STOP. WARNING: the bridge may "
              "still be driving; kill it or use the e-stop for a hard stop.",
              file=sys.stderr)
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
