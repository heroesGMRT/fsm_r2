"""Teensy command bridge — real motion choreography (UPDATE.md Layer B).

Translates the forest executor's high-level primitives from
``/teensy/command`` into:

* open-loop ``geometry_msgs/Twist`` creeps on ``/cmd_vel``, gated by
  ``/ir_sensors`` milestones for the climb FSMs;
* ``std_msgs/Int32`` mechanism codes on ``/fsm_command`` (same palette as
  ``keyboard_teleop_node.py``), each followed by a timed dwell (the Teensy
  gives no completion feedback).

Ack contract: ``{"sequence": n, "status": "done"}`` on ``/teensy/ack`` only
after the FULL step list of a command has physically finished; ``"error"``
on unknown commands, IR timeouts, or failed preconditions.

IR sensor model (``/ir_sensors``, Int32 0..7, 3 bits)
-----------------------------------------------------
All three sensors are clustered mid-chassis and report where the block edge
has reached under the robot (NOT one-sensor-per-wheel). Wheel order
front->back: front deadwheel, front lift wheel, middle deadwheel, back lift
wheel, back deadwheel. Sensor mounting:

* bit 0 (value 1): back of the FRONT deadwheel
* bit 1 (value 2): FRONT side of the MIDDLE deadwheel
* bit 2 (value 4): BACK of the MIDDLE deadwheel (set == middle deadwheel
  has landed)

There is NO sensor on the lift wheels or the back deadwheel — those steps
are TIMED. The same integer means different things in climb-up vs
climb-down, so each maneuver decodes it in its own context (per-maneuver
predicates below) — there is deliberately NO global "IR value -> action"
table.

OPEN ITEMS to confirm on hardware (all mapped to parameters):
1. Which ``/fsm_command`` layer (base pneumatics 10-15 vs lift pneumatics
   40xx vs lift sequences 100-105) is the CLIMB mechanism vs the KFS-lift
   mechanism — the ``*_cmd`` parameter defaults below are BEST GUESSES.
2. Climb-down post-110 IR progression + front/back firing order (the
   re-trigger predicates encode the agreed intent, unverified).
3. Descent tipping safety: the front lift wheel must reach the lower block
   BEFORE the CoM crosses the pillar edge. Keep ``creep_speed`` LOW.
4. The 1 s descent dwell and both TIMED settle intervals (up and down).
5. Whether pick-at-011 works (``enable_pick_at_011``, default off).
"""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

import math


class _StepError(Exception):
    """A step failed; the whole command is acked as an error."""


# ---------------------------------------------------------------------------
# IR predicates (context-aware, created fresh per command)
# ---------------------------------------------------------------------------
def _bit_set(bit):
    def pred(ir, state):
        return bool((ir >> bit) & 1)
    return pred


def _bit_cleared(bit):
    def pred(ir, state):
        return not ((ir >> bit) & 1)
    return pred


def _bit_retriggered(bit):
    """True once `bit` goes LOW (over the edge / in air) and then HIGH again
    (landed on the lower block). PROVISIONAL: encodes the agreed climb-down
    intent while the exact post-110 IR progression is unconfirmed."""
    def pred(ir, state):
        if not ((ir >> bit) & 1):
            state['seen_low'] = True
            return False
        return state.get('seen_low', False)
    return pred


# ---------------------------------------------------------------------------
# Step primitives for the sequencer
# ---------------------------------------------------------------------------
class _MechStep:
    """Publish one /fsm_command code, then dwell (Teensy gives no feedback)."""

    def __init__(self, code, dwell_s, label):
        self.code, self.dwell_s, self.label = code, dwell_s, label
        self._deadline = None

    def start(self, node, now):
        node.publish_mech(self.code, self.label)
        self._deadline = now + self.dwell_s

    def tick(self, node, now, ir):
        return now >= self._deadline


class _DriveTimedStep:
    """Open-loop Twist for a fixed duration, then stop."""

    def __init__(self, vx, vy, wz, duration_s, label):
        self.vx, self.vy, self.wz = vx, vy, wz
        self.duration_s, self.label = duration_s, label
        self._deadline = None

    def start(self, node, now):
        node.get_logger().info(f"drive (timed {self.duration_s:.2f}s): {self.label}")
        self._deadline = now + self.duration_s

    def tick(self, node, now, ir):
        if now >= self._deadline:
            node.publish_stop()
            return True
        node.publish_twist(self.vx, self.vy, self.wz)
        return False


class _DriveUntilIrStep:
    """Creep until an IR predicate is satisfied; error on timeout."""

    def __init__(self, vx, predicate, timeout_s, label, stop_on_done=True):
        self.vx, self.predicate = vx, predicate
        self.timeout_s, self.label = timeout_s, label
        self.stop_on_done = stop_on_done
        self._deadline = None
        self._state = {}

    def start(self, node, now):
        node.get_logger().info(f"creep until IR: {self.label}")
        self._deadline = now + self.timeout_s
        self._state = {}

    def tick(self, node, now, ir):
        if self.predicate(ir, self._state):
            if self.stop_on_done:
                node.publish_stop()
            return True
        if now >= self._deadline:
            node.publish_stop()
            raise _StepError(
                f"IR timeout ({self.timeout_s:.0f}s) waiting for: {self.label} "
                f"(last /ir_sensors={ir:03b})")
        node.publish_twist(self.vx, 0.0, 0.0)
        return False


class _WaitStep:
    def __init__(self, duration_s, label):
        self.duration_s, self.label = duration_s, label
        self._deadline = None

    def start(self, node, now):
        node.get_logger().info(f"wait {self.duration_s:.2f}s: {self.label}")
        self._deadline = now + self.duration_s

    def tick(self, node, now, ir):
        return now >= self._deadline


class _CheckIrStep:
    """Instant precondition check on the current IR reading."""

    def __init__(self, predicate, label, strict):
        self.predicate, self.label, self.strict = predicate, label, strict

    def start(self, node, now):
        pass

    def tick(self, node, now, ir):
        if self.predicate(ir, {}):
            return True
        if self.strict:
            raise _StepError(f"precondition failed: {self.label} "
                             f"(/ir_sensors={ir:03b})")
        node.get_logger().warn(
            f"precondition NOT met (continuing, strict check off): "
            f"{self.label} (/ir_sensors={ir:03b})")
        return True


class _WarnStep:
    def __init__(self, message):
        self.message = message

    def start(self, node, now):
        node.get_logger().warn(self.message)

    def tick(self, node, now, ir):
        return True


class TeensyCommandNode(Node):
    """IR-gated per-maneuver FSMs for the Forest primitives."""

    def __init__(self):
        super().__init__("teensy_command")

        # --- speeds / rates ------------------------------------------------
        self.declare_parameter('creep_speed', 0.05)        # m/s, climb creeps
        self.declare_parameter('transit_speed', 0.15)      # m/s, FORWARD_INIT
        self.declare_parameter('rotate_rate', 0.5)         # rad/s, ROTATE_*
        # --- timed intervals (ALL need bench calibration) --------------------
        self.declare_parameter('forward_init_s', 6.0)      # ~1 m approach
        self.declare_parameter('ir_timeout_s', 15.0)       # creep safety cap
        self.declare_parameter('mech_dwell_s', 1.0)        # after each code
        self.declare_parameter('climb_up_seat_s', 2.0)     # B1 step 7 (timed)
        self.declare_parameter('climb_down_dwell_s', 1.0)  # B2 step 3 MAGIC NUMBER
        self.declare_parameter('climb_down_clear_s', 3.0)  # B2 step 5 (timed)
        self.declare_parameter('pick_reverse_settle_s', 1.5)
        # --- mechanism codes (OPEN item 1: verify the layer mapping!) --------
        self.declare_parameter('climb_extend_both_cmd', 14)   # both base pneu HIGH
        self.declare_parameter('front_lift_retract_cmd', 102)  # front lift down/retract seq
        self.declare_parameter('back_lift_retract_cmd', 103)   # back lift down/retract seq
        self.declare_parameter('front_lift_extend_cmd', 100)   # front lift up seq
        self.declare_parameter('back_lift_extend_cmd', 101)    # back lift up seq
        self.declare_parameter('both_lift_retract_cmd', 105)   # both lifts down seq
        # --- provisional pick-at-011 (UNTESTED, default off) -----------------
        self.declare_parameter('enable_pick_at_011', False)
        self.declare_parameter('pick_sequence_cmds', [0])   # /fsm_command codes
        self.declare_parameter('pick_step_dwell_s', 2.0)
        # --- safety toggles ---------------------------------------------------
        self.declare_parameter('strict_climb_down_precheck', True)

        self._command_sub = self.create_subscription(
            String, "/teensy/command", self._command_callback, 10)
        self._ir_sub = self.create_subscription(
            Int32, "/ir_sensors", self._ir_callback, 10)
        self._ack_pub = self.create_publisher(String, "/teensy/ack", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._mech_pub = self.create_publisher(Int32, "/fsm_command", 10)

        self._ir = 0
        self._steps = []
        self._step_idx = 0
        self._step_started = False
        self._sequence = None
        self._command_name = None

        self._tick_timer = self.create_timer(0.05, self._tick)  # 20 Hz

        self.get_logger().info(
            "TeensyCommand bridge ready: /teensy/command -> IR-gated "
            "/cmd_vel + /fsm_command choreography, acks on /teensy/ack")

    # ── param shorthand ──────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ── inbound ──────────────────────────────────────────────────────────

    def _ir_callback(self, msg: Int32):
        self._ir = msg.data & 0b111

    def _command_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            command = str(payload["command"])
            sequence = int(payload["sequence"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"Ignoring invalid Teensy command: {exc}")
            return
        meta = payload.get("meta") or {}

        if self._sequence is not None:
            self.get_logger().error(
                f"Command {command} (seq {sequence}) received while seq "
                f"{self._sequence} ({self._command_name}) is still running")
            self._ack(sequence, "error")
            return

        try:
            steps = self._build_steps(command, meta)
        except _StepError as exc:
            self.get_logger().error(f"{command}: {exc}")
            self._ack(sequence, "error")
            return
        if steps is None:
            self.get_logger().error(f"Unknown command: {command}")
            self._ack(sequence, "error")
            return

        self.get_logger().info(
            f"Executing {command} (seq {sequence}, {len(steps)} steps)"
            f"{' meta=' + str(meta) if meta else ''}")
        self._steps = steps
        self._step_idx = 0
        self._step_started = False
        self._sequence = sequence
        self._command_name = command

    # ── sequencer ────────────────────────────────────────────────────────

    def _tick(self):
        if self._sequence is None:
            return
        now = self._now()
        while self._sequence is not None:
            step = self._steps[self._step_idx]
            try:
                if not self._step_started:
                    step.start(self, now)
                    self._step_started = True
                if not step.tick(self, now, self._ir):
                    return
            except _StepError as exc:
                self._abort(str(exc))
                return
            self._step_idx += 1
            self._step_started = False
            if self._step_idx >= len(self._steps):
                self._finish()
                return

    def _finish(self):
        self.publish_stop()
        self.get_logger().info(
            f"{self._command_name} (seq {self._sequence}) done")
        self._ack(self._sequence, "done")
        self._reset()

    def _abort(self, detail: str):
        self.publish_stop()
        self.get_logger().error(
            f"{self._command_name} (seq {self._sequence}) FAILED: {detail}")
        self._ack(self._sequence, "error")
        self._reset()

    def _reset(self):
        self._steps = []
        self._step_idx = 0
        self._step_started = False
        self._sequence = None
        self._command_name = None

    # ── outbound ─────────────────────────────────────────────────────────

    def publish_twist(self, vx, vy, wz):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self._cmd_vel_pub.publish(twist)

    def publish_stop(self):
        self._cmd_vel_pub.publish(Twist())

    def publish_mech(self, code, label):
        msg = Int32()
        msg.data = int(code)
        self._mech_pub.publish(msg)
        self.get_logger().info(f"/fsm_command {code}: {label}")

    def _ack(self, sequence, status):
        ack = String()
        ack.data = json.dumps({"sequence": sequence, "status": status})
        self._ack_pub.publish(ack)

    # ── command -> step list ─────────────────────────────────────────────

    def _build_steps(self, command, meta):
        if command == 'FORWARD_INIT':
            return [_DriveTimedStep(self._p('transit_speed'), 0.0, 0.0,
                                    self._p('forward_init_s'),
                                    'approach the Forest entrance (timed)')]
        if command.startswith('ROTATE_'):
            try:
                degrees = int(command.split('_', 1)[1])
            except ValueError:
                return None
            rate = float(self._p('rotate_rate'))
            duration = math.radians(degrees) / rate
            # planner degrees are CLOCKWISE -> negative wz
            return [_DriveTimedStep(0.0, 0.0, -rate, duration,
                                    f'rotate {degrees} deg clockwise (timed)')]
        if command == 'CLIMB_UP':
            return self._climb_up_steps(pick=False, end_on=True)
        if command == 'CLIMB_DOWN':
            return self._climb_down_steps(pick=False)
        if command == 'PICK_BLOCK_UP':
            return self._climb_up_steps(pick=True,
                                        end_on=bool(meta.get('end_on', True)))
        if command == 'PICK_BLOCK_DOWN':
            if not meta.get('end_on', True):
                raise _StepError(
                    'downward pick with reverse has no choreography; the '
                    'planner only emits end_on=True for downward picks')
            return self._climb_down_steps(pick=True)
        return None

    def _pick_steps(self, where):
        """The provisional pick firing (UNTESTED — OPEN item 5). Runs only
        when enable_pick_at_011 is set; otherwise loudly skips so bench runs
        can exercise the climb without the arm."""
        if not self._p('enable_pick_at_011'):
            return [_WarnStep(
                f"PICK REQUESTED at {where} but enable_pick_at_011 is OFF "
                f"— climb continues WITHOUT collecting the KFS")]
        steps = [_WarnStep(f"PROVISIONAL pick at {where} (untested)")]
        codes = [int(c) for c in self._p('pick_sequence_cmds') if int(c) != 0]
        if not codes:
            steps.append(_WarnStep(
                "pick_sequence_cmds is empty — no Teensy pick sequence is "
                "defined yet; nothing fired"))
        for code in codes:
            steps.append(_MechStep(code, self._p('pick_step_dwell_s'),
                                   f'pick sequence code {code}'))
        return steps

    def _climb_up_steps(self, pick, end_on):
        """B1: tested IR progression 000 -> 001 -> 011 -> 111. Driving
        forward onto a higher block; the edge sweeps back under the chassis,
        adding support before removing it (no tipping window).

        CAVEAT: the bit_set gates assume the reading is back at 000 when the
        climb starts (as in the tested ground approach). If the sensors
        still read the PREVIOUS block during block-to-block transit, these
        gates fire instantly and must be changed to re-trigger variants —
        bench-check the transit reading."""
        creep = float(self._p('creep_speed'))
        timeout = float(self._p('ir_timeout_s'))
        dwell = float(self._p('mech_dwell_s'))

        steps = [
            # 1. extend both pneumatics (climb mechanism) to lift the front
            _MechStep(self._p('climb_extend_both_cmd'), dwell,
                      'extend both climb pneumatics (OPEN: verify code)'),
            # 2. creep until bit 0: front deadwheel landed on the higher block
            _DriveUntilIrStep(creep, _bit_set(0), timeout,
                              'climb-up: front deadwheel landed (bit0, 001)'),
            # 3. retract the FRONT lift wheel
            _MechStep(self._p('front_lift_retract_cmd'), dwell,
                      'retract front lift wheel'),
            # 4. creep until bit 1: edge reached middle deadwheel front (011)
            _DriveUntilIrStep(creep, _bit_set(1), timeout,
                              'climb-up: edge at middle deadwheel front (bit1, 011)'),
        ]
        if pick:
            steps += self._pick_steps('011 (half on the target block)')
        if pick and not end_on:
            # reverse back to the previous block (PROVISIONAL choreography)
            steps += [
                _WarnStep('PROVISIONAL pick-reverse: backing off the target '
                          'block (untested)'),
                # re-extend the front lift wheel so it takes the front's
                # weight on the way back down
                _MechStep(self._p('front_lift_extend_cmd'), dwell,
                          'pick-reverse: re-extend front lift wheel'),
                _DriveUntilIrStep(-creep, lambda ir, state: ir == 0, timeout,
                                  'pick-reverse: all IR clear (back off edge)'),
                _DriveTimedStep(-creep, 0.0, 0.0,
                                float(self._p('pick_reverse_settle_s')),
                                'pick-reverse: timed settle back onto origin block'),
                _MechStep(self._p('front_lift_retract_cmd'), dwell,
                          'pick-reverse: retract front lift wheel'),
            ]
            return steps
        steps += [
            # 5. creep until bit 2: middle deadwheel landed (111)
            _DriveUntilIrStep(creep, _bit_set(2), timeout,
                              'climb-up: middle deadwheel landed (bit2, 111)'),
            # 6. retract the BACK lift wheel
            _MechStep(self._p('back_lift_retract_cmd'), dwell,
                      'retract back lift wheel'),
            # 7. TIMED seat of the back deadwheel (no sensor there)
            _DriveTimedStep(creep, 0.0, 0.0, float(self._p('climb_up_seat_s')),
                            'climb-up: timed back-deadwheel seat (tune!)'),
        ]
        return steps

    def _climb_down_steps(self, pick):
        """B2 (provisional): deploy front support the instant the front
        deadwheel leaves the edge, to avoid the blind tipping window (no
        sensor between bit 0 and bit 1).

        KEY UNTESTED ASSUMPTION: safety depends on the front lift wheel
        reaching the lower block BEFORE the CoM (~middle deadwheel) crosses
        the pillar edge — front-lift extension speed vs creep speed. MUST be
        bench-verified; keep creep_speed low. Fallbacks if it can't be
        guaranteed: (a) dedicated front-lift-wheel grounded sensor, or
        (b) gate the descent creep by odometry distance instead of bit 1.
        """
        creep = float(self._p('creep_speed'))
        timeout = float(self._p('ir_timeout_s'))
        dwell = float(self._p('mech_dwell_s'))

        steps = [
            # 1. must start fully seated on the upper block (111)
            _CheckIrStep(lambda ir, state: ir == 0b111,
                         'climb-down start: fully on upper block (111)',
                         strict=bool(self._p('strict_climb_down_precheck'))),
            # 2a. creep until bit 0 LOSES ground (front deadwheel in air, 110)
            _DriveUntilIrStep(creep, _bit_cleared(0), timeout,
                              'climb-down: front deadwheel over the edge '
                              '(bit0 low, 110)', stop_on_done=False),
            # 2b. IMMEDIATELY extend the front lift wheel toward the lower
            #     block (dwell 0: keep creeping while it extends)
            _MechStep(self._p('front_lift_extend_cmd'), 0.0,
                      'extend front lift wheel down (OPEN: verify code)'),
            # 3. creep until bit 1 re-triggers on the lower block, then dwell
            #    (post-110 IR progression PROVISIONAL — OPEN item 2)
            _DriveUntilIrStep(creep, _bit_retriggered(1), timeout,
                              'climb-down: bit1 re-triggered on lower block'),
            _WaitStep(float(self._p('climb_down_dwell_s')),
                      'climb-down: settle/level dwell (MAGIC NUMBER, tune!)'),
        ]
        if pick:
            steps += self._pick_steps('descent dwell (front on lower block)')
        steps += [
            # 4. creep until bit 2 re-triggers, then extend the BACK lift
            _DriveUntilIrStep(creep, _bit_retriggered(2), timeout,
                              'climb-down: bit2 re-triggered on lower block'),
            _MechStep(self._p('back_lift_extend_cmd'), 0.0,
                      'extend back lift wheel down (OPEN: verify code)'),
            # 5. TIMED clear of the upper block (no sensors remain behind bit2)
            _DriveTimedStep(creep, 0.0, 0.0,
                            float(self._p('climb_down_clear_s')),
                            'climb-down: timed clear of the upper block (tune!)'),
            # 6. retract BOTH lift wheels
            _MechStep(self._p('both_lift_retract_cmd'), dwell,
                      'retract both lift wheels'),
        ]
        return steps


def main(args=None):
    rclpy.init(args=args)
    node = TeensyCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
