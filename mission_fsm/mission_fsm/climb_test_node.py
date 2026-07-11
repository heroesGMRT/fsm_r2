"""Bench-test driver for the Teensy bridge climb choreography.

Sends ONE Forest primitive at a time to the real ``teensy_command`` bridge
(which must be running) and prints a timestamped timeline of everything
observable on the wire:

  * /fsm_command   — every mechanism code the bridge fires (decoded label)
  * /relative_move — every odometry-closed move/nudge the bridge issues
  * /cmd_vel       — drive phase changes (start/stop/direction, not every msg)
  * /ir_sensors   — every IR transition, bits decoded
  * /teensy/ack   — done/error + total elapsed time

so you can read off the real durations and tune the bridge parameters
(``forward_init_m``, ``d_center_up_m``, ``d_center_down_m``,
``creep_ceiling_m``, ``creep_ceiling_short_m``, ``mech_dwell_s``, ...).

Usage (bridge running in another terminal: ``ros2 run mission_fsm teensy_command``):

    ros2 run mission_fsm climb_test climb_up
    ros2 run mission_fsm climb_test climb_down
    ros2 run mission_fsm climb_test pick_up            # end_on=True
    ros2 run mission_fsm climb_test pick_up --reverse  # end_on=False
    ros2 run mission_fsm climb_test pick_down
    ros2 run mission_fsm climb_test forward_init
    ros2 run mission_fsm climb_test rotate --deg 90
    ros2 run mission_fsm climb_test climb_up --repeat 3 --pause 5
    ros2 run mission_fsm climb_test climb_up --set creep_speed=0.03 mech_dwell_s=2.0

``--set name=value ...`` pushes parameters onto the bridge node BEFORE the
run (via its set_parameters service), so you can sweep timings without
restarting it.

SAFETY: this script only *requests* the maneuver — the bridge does the
driving. Ctrl+C here does NOT stop the robot; kill the bridge node or use
the e-stop.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# maneuver name -> (bridge command, meta)
_MANEUVERS = {
    'climb_up':     ('CLIMB_UP', {}),
    'climb_down':   ('CLIMB_DOWN', {}),
    'pick_up':      ('PICK_BLOCK_UP', {'end_on': True}),
    'pick_down':    ('PICK_BLOCK_DOWN', {'end_on': True}),
    'forward_init': ('FORWARD_INIT', {}),
    'rotate':       (None, {}),  # resolved from --deg
}

# /fsm_command decode (keyboard_teleop_node.py palette, relevant subset)
_MECH_LABELS = {
    10: 'front base pneumatic high', 11: 'front base pneumatic low',
    12: 'back base pneumatic high', 13: 'back base pneumatic low',
    14: 'both base pneumatics high', 15: 'both base pneumatics low',
    100: 'front lift up sequence', 101: 'back lift up sequence',
    102: 'front lift down sequence', 103: 'back lift down sequence',
    104: 'both lifts up sequence', 105: 'both lifts down sequence',
    110: 'front lift up TIMED', 111: 'back lift up TIMED',
    112: 'front lift down TIMED', 113: 'back lift down TIMED',
    114: 'both lifts up TIMED', 115: 'both lifts down TIMED',
    4000: 'front DC lift stop', 4001: 'front DC lift up',
    4002: 'front DC lift down', 4010: 'back DC lift stop',
    4011: 'back DC lift up', 4012: 'back DC lift down',
    4020: 'front lift pneumatic release', 4021: 'front lift pneumatic deploy',
    4030: 'back lift pneumatic release', 4031: 'back lift pneumatic deploy',
}

_IR_BIT_NAMES = ['front-deadwheel back', 'middle-deadwheel front',
                 'middle-deadwheel back', 'back-deadwheel front']


def _ir_repr(value: int) -> str:
    bits = f"{value & 0b1111:04b}"
    on = [_IR_BIT_NAMES[b] for b in range(4) if value & (1 << b)]
    return f"0b{bits}" + (f"  ({', '.join(on)})" if on else "  (all clear)")


class ClimbTestNode(Node):
    """Fire one primitive at the bridge and log the wire traffic."""

    def __init__(self):
        super().__init__('climb_test')
        self._cmd_pub = self.create_publisher(String, '/teensy/command', 10)
        self.create_subscription(String, '/teensy/ack', self._on_ack, 10)
        self.create_subscription(Int32, '/ir_sensors', self._on_ir, 10)
        self.create_subscription(Int32, '/fsm_command', self._on_mech, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_vel, 10)
        self.create_subscription(Vector3, '/relative_move', self._on_move, 10)

        self._t0 = None
        self._t_last = None
        self._last_ir = None
        self._last_vel = None
        self.ack = None  # (sequence, status)

    # ── timeline ─────────────────────────────────────────────────────────

    def _log(self, kind: str, text: str):
        now = time.monotonic()
        if self._t0 is None:
            stamp = '   before'
            delta = ''
        else:
            stamp = f"+{now - self._t0:7.2f}s"
            delta = (f"  (Δ {now - self._t_last:5.2f}s)"
                     if self._t_last is not None else '')
            self._t_last = now
        print(f"[{stamp}] {kind:<4} {text}{delta}", flush=True)

    def start_clock(self):
        self._t0 = time.monotonic()
        self._t_last = self._t0

    # ── wire taps ────────────────────────────────────────────────────────

    def _on_ack(self, msg: String):
        try:
            payload = json.loads(msg.data)
            seq = int(payload['sequence'])
            status = str(payload.get('status', '?'))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        self.ack = (seq, status)
        self._log('ACK', f"seq {seq}: {status.upper()}")

    def _on_ir(self, msg: Int32):
        value = msg.data & 0b1111
        if value == self._last_ir:
            return
        self._last_ir = value
        self._log('IR', _ir_repr(value))

    def _on_mech(self, msg: Int32):
        label = _MECH_LABELS.get(msg.data, 'UNKNOWN code')
        self._log('MECH', f"{msg.data}  ({label})")

    def _on_vel(self, msg: Twist):
        vel = (round(msg.linear.x, 3), round(msg.linear.y, 3),
               round(msg.angular.z, 3))
        if vel == self._last_vel:
            return  # bridge republishes at 20 Hz; only log phase changes
        self._last_vel = vel
        if vel == (0.0, 0.0, 0.0):
            self._log('VEL', 'STOP')
        else:
            self._log('VEL', f"vx={vel[0]:+.3f}  vy={vel[1]:+.3f}  "
                             f"wz={vel[2]:+.3f}")

    def _on_move(self, msg: Vector3):
        text = f"x={msg.x:+.3f}m  y={msg.y:+.3f}m"
        if msg.z:
            text += f"  z={msg.z:+.3f}"
        self._log('MOVE', text)

    # ── actions ──────────────────────────────────────────────────────────

    def send(self, sequence: int, command: str, meta: dict):
        msg = String()
        msg.data = json.dumps({
            'source': 'climb_test',
            'sequence': sequence,
            'total': 1,
            'command': command,
            'comment': 'bench test',
            'meta': meta,
        })
        self.start_clock()
        self._cmd_pub.publish(msg)
        self._log('CMD', f"→ {command} (seq {sequence})"
                         + (f" meta={meta}" if meta else ''))

    def bridge_listening(self) -> bool:
        return self.count_subscribers('/teensy/command') > 0

    def push_params(self, assignments: list[str]) -> bool:
        """--set name=value: push parameters onto the bridge before the run."""
        client = self.create_client(
            SetParameters, '/teensy_command/set_parameters')
        if not client.wait_for_service(timeout_sec=3.0):
            print('ERROR: /teensy_command/set_parameters not available — '
                  'is the bridge running?', file=sys.stderr)
            return False
        request = SetParameters.Request()
        for item in assignments:
            name, _, raw = item.partition('=')
            if not name or not raw:
                print(f"ERROR: bad --set item '{item}' (want name=value)",
                      file=sys.stderr)
                return False
            request.parameters.append(
                Parameter(name=name, value=_param_value(raw)))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            print('ERROR: set_parameters call timed out', file=sys.stderr)
            return False
        for item, result in zip(assignments, future.result().results):
            state = 'ok' if result.successful else f"FAILED: {result.reason}"
            print(f"  param {item}  ->  {state}")
        return all(r.successful for r in future.result().results)


def _param_value(raw: str) -> ParameterValue:
    value = ParameterValue()
    lowered = raw.lower()
    if lowered in ('true', 'false'):
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = lowered == 'true'
        return value
    try:
        value.type = ParameterType.PARAMETER_INTEGER
        value.integer_value = int(raw)
        return value
    except ValueError:
        pass
    try:
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = float(raw)
        return value
    except ValueError:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = raw
        return value


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='climb_test',
        description='Bench-test one Forest climb primitive against the '
                    'running teensy_command bridge and log a timing timeline.')
    parser.add_argument('maneuver', choices=sorted(_MANEUVERS),
                        help='primitive to fire')
    parser.add_argument('--deg', type=int, default=90,
                        choices=(90, 180, 270),
                        help='rotation angle for the rotate maneuver')
    parser.add_argument('--reverse', action='store_true',
                        help='pick_up only: end_on=False '
                             '(pick then reverse off the block)')
    parser.add_argument('--repeat', type=int, default=1,
                        help='run the maneuver N times (default 1)')
    parser.add_argument('--pause', type=float, default=3.0,
                        help='seconds between repeats (default 3)')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='max seconds to wait for the ack (default 120)')
    parser.add_argument('--set', nargs='*', default=[], metavar='NAME=VALUE',
                        help='bridge parameters to set before the run, e.g. '
                             '--set creep_speed=0.03 mech_dwell_s=2.0')
    return parser.parse_args(argv)


def main(args=None):
    ns = _parse_args(sys.argv[1:] if args is None else args)

    command, meta = _MANEUVERS[ns.maneuver]
    if ns.maneuver == 'rotate':
        command = f'ROTATE_{ns.deg}'
    if ns.maneuver == 'pick_up' and ns.reverse:
        meta = {'end_on': False}

    rclpy.init(args=None)
    node = ClimbTestNode()
    exit_code = 0
    try:
        # let discovery settle so we see the bridge's subscriber
        deadline = time.monotonic() + 3.0
        while not node.bridge_listening() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.bridge_listening():
            print('ERROR: nothing subscribed to /teensy/command — start the '
                  'bridge first: ros2 run mission_fsm teensy_command',
                  file=sys.stderr)
            return 1

        if ns.set and not node.push_params(ns.set):
            return 1

        results = []
        for run in range(1, ns.repeat + 1):
            print(f"\n=== run {run}/{ns.repeat}: {command} ===")
            node.ack = None
            node.send(run, command, meta)
            deadline = time.monotonic() + ns.timeout
            while node.ack is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.ack is None:
                print(f"\nTIMEOUT: no ack within {ns.timeout}s — the bridge "
                      f"may still be executing (its own ir_timeout_s will "
                      f"eventually abort IR waits). NOT sending more runs.")
                exit_code = 1
                break
            elapsed = time.monotonic() - node._t0
            results.append((run, node.ack[1], elapsed))
            if node.ack[1] != 'done':
                print('\nBridge reported ERROR — stopping the repeat loop.')
                exit_code = 1
                break
            if run < ns.repeat:
                print(f"--- pausing {ns.pause}s before next run ---")
                pause_end = time.monotonic() + ns.pause
                while time.monotonic() < pause_end:
                    rclpy.spin_once(node, timeout_sec=0.1)

        if results:
            print('\n=== summary ===')
            for run, status, elapsed in results:
                print(f"  run {run}: {status.upper():5s}  total {elapsed:6.2f}s")
            done = [e for _, s, e in results if s == 'done']
            if len(done) > 1:
                print(f"  mean of successful runs: {sum(done)/len(done):.2f}s")
        print('\nTune with e.g.: ros2 param set /teensy_command '
              'd_center_up_m 0.25   (or rerun with --set ...)')
    except KeyboardInterrupt:
        print('\nInterrupted. WARNING: the bridge may STILL be driving the '
              'robot — this script cannot stop it. Kill the bridge node or '
              'use the e-stop.', file=sys.stderr)
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
