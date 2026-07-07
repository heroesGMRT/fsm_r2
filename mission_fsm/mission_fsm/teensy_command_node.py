"""Teensy command bridge — real motion choreography (UPDATE.md Layer B,
motion primitive switched to /relative_move per spec section 3, 2026-07-07).

Translates the forest executor's high-level primitives from
``/teensy/command`` into:

* odometry-closed relative moves on ``/relative_move``
  (``geometry_msgs/Vector3``: x = forward metres, y = lateral metres) — the
  robot's own node drives until its deadwheel odometry covers the distance.
  Used for every known-distance move (spec 3.3). The node gives NO
  completion feedback, so each move is followed by a COMPUTED settle wait
  (distance / ``relmove_speed_est`` + ``relmove_settle_s``);
* IR-gated creeps ("go until a bit trips") as a NUDGE LOOP (spec 3.3):
  repeat { one ``/relative_move`` of ``nudge_distance_m`` (floor 0.1 m,
  NEVER below); wait for it to settle; check ``/ir_sensors`` } until the
  milestone predicate fires. Overshoot past a milestone is bounded by one
  nudge. ``_NudgeUntilIrStep`` is the abstraction point for the spec-3.4
  preemption upgrade (single long move + cancel on IR) if ``/relative_move``
  turns out to support mid-move cancel — swap only that class;
* timed ``geometry_msgs/Twist`` on ``/cmd_vel`` for rotations, unless
  ``rotate_via_relative_move`` is set (whether Vector3.z is yaw on
  ``/relative_move`` is UNCONFIRMED — check the node's source);
* the ORIGINAL open-loop timed cmd_vel choreography is fully retained:
  set ``motion_backend:=cmd_vel`` to switch every move back to it (kept
  in case cmd_vel is fixed);
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
1. Lift-wheel codes CONFIRMED (operator, 2026-07-07): timed variants
   110-115. The lift wheels ARE the climb mechanism: climb-up step 1
   extends both (114), then 112/113 retract front/back one at a time as
   the deadwheels seat. Still unverified: whether the Teensy's TIMED
   lift duration matches ``mech_dwell_s`` here.
2. Climb-down post-110 IR progression + front/back firing order (the
   re-trigger predicates encode the agreed intent, unverified).
3. Descent tipping safety: the front lift wheel must reach the lower block
   BEFORE the CoM crosses the pillar edge. In nudge mode the lift now
   extends while STATIONARY (bounded overshoot = one nudge), but the
   assumption still needs a bench check.
4. /relative_move behaviour: actual travel speed (``relmove_speed_est``
   feeds the settle wait — too small just wastes time, too big starts the
   next step mid-move), whether a new command mid-move preempts or queues,
   and whether Vector3.z is yaw (``rotate_via_relative_move``).
5. All ``*_m`` distances (BENCH-TUNE) and the 1 s descent dwell.
6. Whether pick-at-011 works (``enable_pick_at_011``, default off).
"""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
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
    """cmd_vel backend: creep until an IR predicate fires; error on timeout."""

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


class _RelativeMoveStep:
    """One odometry-closed /relative_move (known distance), then a COMPUTED
    settle wait — the relative_move node gives no completion feedback, so
    we wait travel/relmove_speed_est + relmove_settle_s before the next
    step (spec 3.1/3.3)."""

    def __init__(self, dx, dy, label, dz=0.0):
        self.dx, self.dy, self.dz = dx, dy, dz
        self.label = label
        self._deadline = None

    def start(self, node, now):
        node.publish_relative_move(self.dx, self.dy, self.dz, self.label)
        travel = max(abs(self.dx), abs(self.dy)) \
            / float(node._p('relmove_speed_est'))
        if self.dz:
            travel += abs(self.dz) / float(node._p('rotate_rate'))
        self._deadline = now + travel + float(node._p('relmove_settle_s'))

    def tick(self, node, now, ir):
        return now >= self._deadline


class _NudgeUntilIrStep:
    """IR-gated creep as a NUDGE LOOP (spec 3.3): issue one forward (or
    backward) /relative_move of nudge_distance_m (floor 0.1 m — NEVER
    below), wait for it to settle, check the IR predicate; repeat.
    Overshoot past the milestone is bounded by one nudge; runaway is
    bounded by max_nudges.

    The predicate is fed EVERY tick (also mid-nudge) and its success is
    latched, so stateful predicates (_bit_retriggered) can't miss a
    transient low while the robot is moving.

    ABSTRACTION POINT for the spec-3.4 preemption upgrade: if
    /relative_move accepts cancel/override mid-move, replace this class
    with one long move + cancel on IR — no FSM logic changes needed."""

    def __init__(self, direction, predicate, label):
        self.direction = 1 if direction >= 0 else -1
        self.predicate, self.label = predicate, label
        self._state = {}
        self._satisfied = False
        self._nudges = 0
        self._settle_deadline = None

    def start(self, node, now):
        node.get_logger().info(f"nudge loop until IR: {self.label}")
        self._state = {}
        self._satisfied = False
        self._nudges = 0
        self._settle_deadline = None

    def tick(self, node, now, ir):
        if self.predicate(ir, self._state):
            self._satisfied = True
        if self._settle_deadline is not None and now < self._settle_deadline:
            return False  # nudge still settling; keep feeding the predicate
        if self._satisfied:
            return True
        max_nudges = int(node._p('max_nudges'))
        if self._nudges >= max_nudges:
            raise _StepError(
                f"IR milestone not reached after {max_nudges} nudges: "
                f"{self.label} (last /ir_sensors={ir:03b})")
        # spec 3.3: nudge floor 0.1 m — tune UP if needed, never below
        dist = max(0.1, float(node._p('nudge_distance_m'))) * self.direction
        self._nudges += 1
        node.publish_relative_move(
            dist, 0.0, 0.0,
            f"nudge {self._nudges}/{max_nudges} toward: {self.label}")
        self._settle_deadline = now \
            + abs(dist) / float(node._p('relmove_speed_est')) \
            + float(node._p('relmove_settle_s'))
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

        # --- motion backend ---------------------------------------------------
        # 'relative_move' (spec 3, odometry-closed distances) or 'cmd_vel'
        # (the original open-loop timed backend, kept in case cmd_vel is fixed)
        self.declare_parameter('motion_backend', 'relative_move')
        # --- /relative_move motion (spec 3: distances, not durations) --------
        self.declare_parameter('nudge_distance_m', 0.1)    # FLOOR 0.1, never below
        self.declare_parameter('max_nudges', 8)            # runaway bound per creep
        self.declare_parameter('relmove_speed_est', 0.1)   # m/s, for settle wait
        self.declare_parameter('relmove_settle_s', 1.0)    # margin after each move
        self.declare_parameter('forward_init_m', 1.0)      # entrance approach (BENCH-TUNE)
        self.declare_parameter('climb_up_seat_m', 0.15)    # B1 step 7 (BENCH-TUNE)
        self.declare_parameter('climb_down_clear_m', 0.3)  # B2 step 5 (BENCH-TUNE)
        self.declare_parameter('pick_reverse_settle_m', 0.15)  # (BENCH-TUNE)
        # --- rotation (cmd_vel timed unless z-as-yaw is confirmed) -----------
        self.declare_parameter('rotate_rate', 0.5)         # rad/s
        self.declare_parameter('rotate_via_relative_move', False)  # z==yaw UNCONFIRMED
        # --- cmd_vel backend (only used when motion_backend == 'cmd_vel') ----
        self.declare_parameter('creep_speed', 0.05)        # m/s, climb creeps
        self.declare_parameter('transit_speed', 0.15)      # m/s, FORWARD_INIT
        self.declare_parameter('forward_init_s', 6.0)      # ~1 m approach
        self.declare_parameter('ir_timeout_s', 15.0)       # creep safety cap
        self.declare_parameter('climb_up_seat_s', 2.0)     # B1 step 7 (timed)
        self.declare_parameter('climb_down_clear_s', 3.0)  # B2 step 5 (timed)
        self.declare_parameter('pick_reverse_settle_s', 1.5)
        # --- mechanism timing -------------------------------------------------
        self.declare_parameter('mech_dwell_s', 1.0)        # after each code
        self.declare_parameter('climb_down_dwell_s', 1.0)  # B2 step 3 MAGIC NUMBER
        # --- mechanism codes (lift wheels: timed variants 110-115) -----------
        self.declare_parameter('climb_extend_both_cmd', 114)   # Both lifts up timed
        self.declare_parameter('front_lift_retract_cmd', 112)  # Front lift down timed
        self.declare_parameter('back_lift_retract_cmd', 113)   # Back lift down timed
        self.declare_parameter('front_lift_extend_cmd', 110)   # Front lift up timed
        self.declare_parameter('back_lift_extend_cmd', 111)    # Back lift up timed
        self.declare_parameter('both_lift_retract_cmd', 115)   # Both lifts down timed
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
        self._relmove_pub = self.create_publisher(Vector3, "/relative_move", 10)
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
        # NOTE: zero Twist stops cmd_vel motion, but an in-flight
        # /relative_move CANNOT be cancelled from here (preemption support
        # unknown, spec 3.4) — on abort the robot may finish its last nudge.
        self._cmd_vel_pub.publish(Twist())

    def publish_relative_move(self, dx, dy, dz, label):
        msg = Vector3()
        msg.x = float(dx)
        msg.y = float(dy)
        msg.z = float(dz)
        self._relmove_pub.publish(msg)
        self.get_logger().info(
            f"/relative_move x={dx:+.3f} y={dy:+.3f}"
            + (f" z={dz:+.3f}" if dz else "") + f": {label}")

    def publish_mech(self, code, label):
        msg = Int32()
        msg.data = int(code)
        self._mech_pub.publish(msg)
        self.get_logger().info(f"/fsm_command {code}: {label}")

    def _ack(self, sequence, status):
        ack = String()
        ack.data = json.dumps({"sequence": sequence, "status": status})
        self._ack_pub.publish(ack)

    # ── backend-agnostic motion builders ─────────────────────────────────
    # motion_backend == 'relative_move': odometry-closed distances (spec 3).
    # motion_backend == 'cmd_vel': the original open-loop timed backend,
    # kept selectable in case cmd_vel is fixed.

    def _use_relmove(self):
        return str(self._p('motion_backend')) != 'cmd_vel'

    def _known_move_step(self, meters, seconds, speed, label):
        """Known-distance forward/backward move. ``meters`` (signed) feeds
        the relative_move backend; ``speed``+``seconds`` (signed vx) feed
        the cmd_vel backend."""
        if self._use_relmove():
            return _RelativeMoveStep(float(meters), 0.0, label)
        return _DriveTimedStep(float(speed), 0.0, 0.0, float(seconds), label)

    def _creep_until_step(self, direction, predicate, label,
                          stop_on_done=True):
        """IR-gated creep: nudge loop (relative_move) or continuous creep
        (cmd_vel). ``stop_on_done`` only matters for cmd_vel — nudges always
        stop between moves."""
        if self._use_relmove():
            return _NudgeUntilIrStep(direction, predicate, label)
        return _DriveUntilIrStep(
            direction * float(self._p('creep_speed')), predicate,
            float(self._p('ir_timeout_s')), label, stop_on_done)

    # ── command -> step list ─────────────────────────────────────────────

    def _build_steps(self, command, meta):
        if command == 'FORWARD_INIT':
            return [self._known_move_step(
                self._p('forward_init_m'), self._p('forward_init_s'),
                self._p('transit_speed'), 'approach the Forest entrance')]
        if command.startswith('ROTATE_'):
            try:
                degrees = int(command.split('_', 1)[1])
            except ValueError:
                return None
            # planner degrees are CLOCKWISE -> negative yaw/wz
            if self._use_relmove() and self._p('rotate_via_relative_move'):
                return [
                    _WarnStep('rotate via /relative_move z: z==yaw is '
                              'UNCONFIRMED — verify against the node source'),
                    _RelativeMoveStep(0.0, 0.0,
                                      f'rotate {degrees} deg clockwise',
                                      dz=-math.radians(degrees)),
                ]
            rate = float(self._p('rotate_rate'))
            duration = math.radians(degrees) / rate
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
        dwell = float(self._p('mech_dwell_s'))

        steps = [
            # 1. extend both lift wheels to raise the chassis for the climb
            _MechStep(self._p('climb_extend_both_cmd'), dwell,
                      'extend both lift wheels (114)'),
            # 2. creep until bit 0: front deadwheel landed on the higher block
            self._creep_until_step(
                +1, _bit_set(0),
                'climb-up: front deadwheel landed (bit0, 001)'),
            # 3. retract the FRONT lift wheel
            _MechStep(self._p('front_lift_retract_cmd'), dwell,
                      'retract front lift wheel'),
            # 4. creep until bit 1: edge reached middle deadwheel front (011)
            self._creep_until_step(
                +1, _bit_set(1),
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
                self._creep_until_step(
                    -1, lambda ir, state: ir == 0,
                    'pick-reverse: all IR clear (back off edge)'),
                self._known_move_step(
                    -float(self._p('pick_reverse_settle_m')),
                    self._p('pick_reverse_settle_s'), -creep,
                    'pick-reverse: settle back onto origin block (tune!)'),
                _MechStep(self._p('front_lift_retract_cmd'), dwell,
                          'pick-reverse: retract front lift wheel'),
            ]
            return steps
        steps += [
            # 5. creep until bit 2: middle deadwheel landed (111)
            self._creep_until_step(
                +1, _bit_set(2),
                'climb-up: middle deadwheel landed (bit2, 111)'),
            # 6. retract the BACK lift wheel
            _MechStep(self._p('back_lift_retract_cmd'), dwell,
                      'retract back lift wheel'),
            # 7. seat the back deadwheel — no sensor there, so known-distance
            #    move (relative_move) or timed creep (cmd_vel)
            self._known_move_step(
                self._p('climb_up_seat_m'), self._p('climb_up_seat_s'),
                creep, 'climb-up: back-deadwheel seat (tune!)'),
        ]
        return steps

    def _climb_down_steps(self, pick):
        """B2 (provisional): deploy front support the instant the front
        deadwheel leaves the edge, to avoid the blind tipping window (no
        sensor between bit 0 and bit 1).

        KEY UNTESTED ASSUMPTION: safety depends on the front lift wheel
        reaching the lower block BEFORE the CoM (~middle deadwheel) crosses
        the pillar edge. In the relative_move backend the lift extends while
        the robot is STATIONARY between nudges (overshoot past the edge is
        bounded by one nudge) — safer than the cmd_vel race, but the bound
        still needs a bench check. In the cmd_vel backend the original race
        applies: keep creep_speed low. Fallbacks if it can't be guaranteed:
        (a) dedicated front-lift-wheel grounded sensor, or (b) gate the
        descent by odometry distance instead of bit 1.
        """
        creep = float(self._p('creep_speed'))
        dwell = float(self._p('mech_dwell_s'))
        # cmd_vel backend keeps creeping while the lifts extend (dwell 0,
        # the original race); nudge backend is stationary here, so give the
        # extension a full dwell before the next nudge.
        lift_deploy_dwell = dwell if self._use_relmove() else 0.0

        steps = [
            # 1. must start fully seated on the upper block (111)
            _CheckIrStep(lambda ir, state: ir == 0b111,
                         'climb-down start: fully on upper block (111)',
                         strict=bool(self._p('strict_climb_down_precheck'))),
            # 2a. creep until bit 0 LOSES ground (front deadwheel in air, 110)
            self._creep_until_step(
                +1, _bit_cleared(0),
                'climb-down: front deadwheel over the edge (bit0 low, 110)',
                stop_on_done=False),
            # 2b. extend the front lift wheel toward the lower block
            _MechStep(self._p('front_lift_extend_cmd'), lift_deploy_dwell,
                      'extend front lift wheel down'),
            # 3. creep until bit 1 re-triggers on the lower block, then dwell
            #    (post-110 IR progression PROVISIONAL — OPEN item 2)
            self._creep_until_step(
                +1, _bit_retriggered(1),
                'climb-down: bit1 re-triggered on lower block'),
            _WaitStep(float(self._p('climb_down_dwell_s')),
                      'climb-down: settle/level dwell (MAGIC NUMBER, tune!)'),
        ]
        if pick:
            steps += self._pick_steps('descent dwell (front on lower block)')
        steps += [
            # 4. creep until bit 2 re-triggers, then extend the BACK lift
            self._creep_until_step(
                +1, _bit_retriggered(2),
                'climb-down: bit2 re-triggered on lower block'),
            _MechStep(self._p('back_lift_extend_cmd'), lift_deploy_dwell,
                      'extend back lift wheel down'),
            # 5. clear the upper block (no sensors remain behind bit2):
            #    known-distance move (relative_move) or timed creep (cmd_vel)
            self._known_move_step(
                self._p('climb_down_clear_m'), self._p('climb_down_clear_s'),
                creep, 'climb-down: clear the upper block (tune!)'),
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
