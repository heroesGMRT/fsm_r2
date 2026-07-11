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
* IR-gated creeps ("go until a bit trips"). Two backends (``creep_backend``):
  - ``ceiling`` (DEFAULT, handoff v3 §1): issue ONE ``/relative_move`` with a
    generous forward CEILING (``creep_ceiling_m`` ~0.9 m, or the SHORT
    ``creep_ceiling_short_m`` at a tipping edge), watch ``/ir_sensors`` and
    publish ``{0,0,0}`` on ``/relative_move`` the instant the milestone
    predicate fires to cut the move short. RELIES on ``/relative_move`` being
    PRE-EMPTIBLE (a mid-move ``{0,0,0}`` halts it) — OPEN item 2, bench-check.
    If the ceiling is reached first the step errors (milestone missed).
  - ``nudge`` (fallback): repeat { one ``/relative_move`` of
    ``nudge_distance_m`` (floor 0.1 m); settle; check IR } until the predicate
    fires. Overshoot bounded by one nudge, runaway by ``max_nudges``. Use this
    if preemption turns out unreliable;
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

IR sensor model (``/ir_sensors``, Int32 0..15, 4 bits — handoff v3 §2)
---------------------------------------------------------------------
Four sensors, symmetric around the middle deadwheel; each detects the ground
close beneath it. Wheel order front->back: front deadwheel, front lift wheel,
middle deadwheel, back lift wheel, back deadwheel. Sensor mounting:

* bit 0 (value 1): back of the FRONT deadwheel      [front-most]
* bit 1 (value 2): FRONT of the MIDDLE deadwheel
* bit 2 (value 4): BACK of the MIDDLE deadwheel      (middle deadwheel landed)
* bit 3 (value 8): FRONT of the BACK deadwheel       [back-most]

bit 3 senses the back deadwheel, so BOTH climbs are now fully sensor-closed —
the old TIMED back-settle is gone (climb-up creeps to reading 15; climb-down
gates the retract on reading 0000). The same integer means different things in
climb-up vs climb-down, so each maneuver decodes it in its OWN context
(per-maneuver predicates below) — there is deliberately NO global
"IR value -> action" table. Dark tier-A block tops previously caused misreads;
if that recurs it is the first suspect.

OPEN ITEMS to confirm on hardware (all mapped to parameters):
1. Axis/sign convention (§0): +y = strafe LEFT, +z = CCW yaw (REP-103
   assumed). Two bench checks before any real run.
2. ``/relative_move`` pre-emptibility: a mid-move ``{0,0,0}`` halts it
   promptly (the ``ceiling`` creep backend depends on this).
3. Descent IR trace is clean/monotonic 1111 -> 1110 -> 1100 -> 1000 -> 0000
   with no re-set / repeated values.
4. Descent front-lift tipping window (bit0->bit1 unsupported): CoM behind
   the middle deadwheel.
5. All ``*_m`` distances and creep ceilings (BENCH-TUNE via the calibration
   script / ``climb_test --set``).
6. Whether the pick at reading 7 works and the KFS is in gripper reach there
   (``enable_pick``, fires ``pick_grab_cmd`` = 51 "Grab Put Up").
"""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
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
                f"(last /ir_sensors={ir:04b})")
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
                f"{self.label} (last /ir_sensors={ir:04b})")
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


class _CeilingUntilIrStep:
    """IR-gated creep, handoff v3 §1: issue ONE /relative_move with a
    generous forward (or backward) CEILING, then watch /ir_sensors and
    publish {0,0,0} the instant the milestone predicate fires to cut the
    move short. The ceiling = max safe travel if the sensor never fires
    (SHORT at a tipping edge). Reaching the ceiling without the milestone
    is an error.

    Depends on /relative_move being PRE-EMPTIBLE (a mid-move {0,0,0}
    halts it) — OPEN item 2. If preemption is unreliable, switch
    ``creep_backend`` to ``nudge``."""

    def __init__(self, direction, predicate, label, ceiling_m):
        self.direction = 1 if direction >= 0 else -1
        self.predicate, self.label = predicate, label
        self.ceiling_m = abs(float(ceiling_m))
        self._state = {}
        self._issued = False
        self._deadline = None

    def start(self, node, now):
        self._state = {}
        self._issued = False
        self._deadline = None

    def tick(self, node, now, ir):
        # Check the milestone with IR in hand BEFORE issuing the ceiling move,
        # so a reading already at the milestone (e.g. block-to-block transit)
        # doesn't fire a full-ceiling move we must instantly cancel.
        if not self._issued:
            if self.predicate(ir, self._state):
                return True
            dist = self.ceiling_m * self.direction
            node.publish_relative_move(
                dist, 0.0, 0.0,
                f"ceiling creep (<= {self.ceiling_m:.2f} m) toward: "
                f"{self.label}")
            speed = max(float(node._p('relmove_speed_est')), 1e-3)
            # allow the FULL ceiling to travel plus a settle margin before we
            # call it a miss (the node closes on odometry, no feedback)
            self._deadline = now + self.ceiling_m / speed \
                + float(node._p('relmove_settle_s'))
            self._issued = True
            return False
        if self.predicate(ir, self._state):
            node.publish_relmove_stop()  # {0,0,0}: cut the move short
            return True
        if now >= self._deadline:
            node.publish_relmove_stop()
            raise _StepError(
                f"IR milestone not reached within ceiling "
                f"{self.ceiling_m:.2f} m: {self.label} "
                f"(last /ir_sensors={ir:04b})")
        return False


class _LateralRecenterStep:
    """Null the odom-reported lateral (y) offset with a single /relative_move
    y: nudge (handoff v3 §6, +y = LEFT per REP-103). Matters most before a
    rotation, so it is prepended to ROTATE_* when ``recenter_before_rotate``
    is on. Needs an odom source (``odom_topic``); with none it warns and
    skips (offset unknown), so it is safe to leave wired."""

    def __init__(self, label):
        self.label = label
        self._issued = False
        self._deadline = None

    def start(self, node, now):
        self._issued = False
        self._deadline = None

    def tick(self, node, now, ir):
        if self._issued:
            return now >= self._deadline
        if node._odom_y is None:
            node.get_logger().warn(
                f"lateral recenter skipped ({self.label}): no odom on "
                f"'{node._p('odom_topic')}' — set odom_topic to enable")
            return True
        offset = float(node._odom_y)
        max_lat = float(node._p('max_lateral_m'))
        dy = max(-max_lat, min(max_lat, -offset))  # +y = left; drive toward 0
        if abs(dy) < 1e-3:
            return True
        node.publish_relative_move(
            0.0, dy, 0.0,
            f"lateral recenter (odom y={offset:+.3f}): {self.label}")
        self._deadline = now + abs(dy) / max(
            float(node._p('relmove_speed_est')), 1e-3) \
            + float(node._p('relmove_settle_s'))
        self._issued = True
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
                             f"(/ir_sensors={ir:04b})")
        node.get_logger().warn(
            f"precondition NOT met (continuing, strict check off): "
            f"{self.label} (/ir_sensors={ir:04b})")
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
        # creep_backend: 'ceiling' (handoff §1, one long move + {0,0,0} cut) or
        # 'nudge' (0.1 m nudge loop, if preemption proves unreliable).
        self.declare_parameter('creep_backend', 'ceiling')
        self.declare_parameter('creep_ceiling_m', 0.9)     # generous creep ceiling
        self.declare_parameter('creep_ceiling_short_m', 0.25)  # tipping-edge steps
        self.declare_parameter('nudge_distance_m', 0.1)    # FLOOR 0.1, never below
        self.declare_parameter('max_nudges', 12)           # runaway bound per creep
        self.declare_parameter('relmove_speed_est', 0.1)   # m/s, for settle wait
        self.declare_parameter('relmove_settle_s', 1.0)    # margin after each move
        self.declare_parameter('forward_init_m', 1.0)      # entrance approach (BENCH-TUNE)
        self.declare_parameter('d_center_up_m', 0.2)       # §3.8 centering (BENCH-TUNE)
        self.declare_parameter('d_center_down_m', 0.2)     # §4.7 centering (BENCH-TUNE)
        self.declare_parameter('pick_reverse_settle_m', 0.15)  # (BENCH-TUNE)
        # --- rotation (cmd_vel timed unless z-as-yaw is confirmed) -----------
        self.declare_parameter('rotate_rate', 0.5)         # rad/s
        self.declare_parameter('rotate_via_relative_move', False)  # z==yaw UNCONFIRMED
        # --- lateral centering / odom (handoff §6) ---------------------------
        # center-then-rotate: null the odom y offset before a turn. Needs an
        # odom source; with odom_topic empty the step self-skips (safe).
        self.declare_parameter('odom_topic', '')           # e.g. '/odom' to enable
        self.declare_parameter('recenter_before_rotate', False)  # needs odom_topic
        self.declare_parameter('max_lateral_m', 0.15)      # clamp on a recenter nudge
        self.declare_parameter('rezero_odom_after_climb', True)  # re-zero at good pose
        self.declare_parameter('odom_rezero_cmd', 20)      # /fsm_command re-zero code
        # --- cmd_vel backend (only used when motion_backend == 'cmd_vel') ----
        self.declare_parameter('creep_speed', 0.05)        # m/s, climb creeps
        self.declare_parameter('transit_speed', 0.15)      # m/s, FORWARD_INIT
        self.declare_parameter('forward_init_s', 6.0)      # ~1 m approach
        self.declare_parameter('ir_timeout_s', 15.0)       # creep safety cap
        self.declare_parameter('d_center_up_s', 2.0)       # §3.8 centering (timed)
        self.declare_parameter('d_center_down_s', 2.0)     # §4.7 centering (timed)
        self.declare_parameter('pick_reverse_settle_s', 1.5)
        # --- mechanism timing -------------------------------------------------
        self.declare_parameter('mech_dwell_s', 1.5)        # after each code (§ all)
        # --- mechanism codes (lift wheels: timed variants 110-115) -----------
        self.declare_parameter('climb_extend_both_cmd', 114)   # Both lifts up timed
        self.declare_parameter('front_lift_retract_cmd', 112)  # Front lift down timed
        self.declare_parameter('back_lift_retract_cmd', 113)   # Back lift down timed
        self.declare_parameter('front_lift_extend_cmd', 110)   # Front lift up timed
        self.declare_parameter('back_lift_extend_cmd', 111)    # Back lift up timed
        self.declare_parameter('both_lift_retract_cmd', 115)   # Both lifts down timed
        # --- pick (handoff §5: single fire of S1 = "Grab Put Up") ------------
        self.declare_parameter('enable_pick', True)        # fire the pick on PICK_*
        self.declare_parameter('pick_grab_cmd', 51)        # S1 grab-and-stow
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

        # Optional odom feed for lateral centering (handoff §6). Only
        # subscribed when odom_topic is set, so nothing is assumed by default.
        self._odom_y = None
        odom_topic = str(self._p('odom_topic'))
        if odom_topic:
            self.create_subscription(
                Odometry, odom_topic, self._odom_callback, 10)
            self.get_logger().info(f"lateral centering: tracking {odom_topic}")

        self._ir = 0
        self._steps = []
        self._step_idx = 0
        self._step_started = False
        self._sequence = None
        self._command_name = None

        self._tick_timer = self.create_timer(0.05, self._tick)  # 20 Hz

        backend = str(self._p('motion_backend'))
        creep = str(self._p('creep_backend'))
        self.get_logger().info(
            f"TeensyCommand bridge ready (4-bit IR): /teensy/command -> "
            f"motion_backend={backend}, creep_backend={creep}, "
            f"/fsm_command choreography, acks on /teensy/ack")
        if backend != 'cmd_vel' and creep == 'ceiling':
            self.get_logger().warn(
                "creep_backend=ceiling relies on /relative_move being "
                "PRE-EMPTIBLE ({0,0,0} cuts a move short — OPEN item 2). "
                "If mid-move stop is unreliable, set creep_backend:=nudge.")

    # ── param shorthand ──────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ── inbound ──────────────────────────────────────────────────────────

    def _ir_callback(self, msg: Int32):
        self._ir = msg.data & 0b1111

    def _odom_callback(self, msg: Odometry):
        self._odom_y = msg.pose.pose.position.y

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
        # Zero Twist stops cmd_vel motion; {0,0,0} on /relative_move is the
        # documented STOP (handoff §0) — send both so an abort halts either
        # backend. /relative_move preemption is OPEN item 2 (bench-check).
        self._cmd_vel_pub.publish(Twist())
        self._relmove_pub.publish(Vector3())

    def publish_relmove_stop(self):
        # {0,0,0} on /relative_move: STOP / cut an in-flight move short (§1).
        self._relmove_pub.publish(Vector3())

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
                          ceiling=None, stop_on_done=True):
        """IR-gated creep. relative_move backend: 'ceiling' (one long move +
        {0,0,0} cut, with a per-step ``ceiling`` override for tipping edges)
        or 'nudge' (0.1 m loop). cmd_vel backend: continuous timed creep.
        ``stop_on_done`` only matters for cmd_vel — the relative_move backends
        always stop between milestones."""
        if not self._use_relmove():
            return _DriveUntilIrStep(
                direction * float(self._p('creep_speed')), predicate,
                float(self._p('ir_timeout_s')), label, stop_on_done)
        if str(self._p('creep_backend')) == 'nudge':
            return _NudgeUntilIrStep(direction, predicate, label)
        ceil = float(self._p('creep_ceiling_m')) if ceiling is None \
            else float(ceiling)
        return _CeilingUntilIrStep(direction, predicate, label, ceil)

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
            steps = []
            # handoff §6: center laterally, THEN turn (a y offset compounds
            # through a rotation). Self-skips without an odom source.
            if self._use_relmove() and self._p('recenter_before_rotate'):
                steps.append(_LateralRecenterStep(f'before rotate {degrees}'))
            # planner degrees are CLOCKWISE -> negative yaw/wz
            if self._use_relmove() and self._p('rotate_via_relative_move'):
                steps += [
                    _WarnStep('rotate via /relative_move z: z==yaw is '
                              'UNCONFIRMED — verify against the node source'),
                    _RelativeMoveStep(0.0, 0.0,
                                      f'rotate {degrees} deg clockwise',
                                      dz=-math.radians(degrees)),
                ]
                return steps
            rate = float(self._p('rotate_rate'))
            duration = math.radians(degrees) / rate
            steps.append(_DriveTimedStep(
                0.0, 0.0, -rate, duration,
                f'rotate {degrees} deg clockwise (timed)'))
            return steps
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
        """Fire the pick (handoff §5): S1 (/fsm_command = pick_grab_cmd,
        default 51 "Grab Put Up") is the COMPLETE grab-and-stow routine, so a
        pick is a SINGLE code fire. Skipped with a loud warning when
        enable_pick is off, so a dry climb can run without the arm.
        PROVISIONAL: firing mid-climb and gripper reach at this pose are
        untested (OPEN item 6)."""
        if not self._p('enable_pick'):
            return [_WarnStep(
                f"PICK REQUESTED at {where} but enable_pick is OFF "
                f"— climb continues WITHOUT collecting the KFS")]
        code = int(self._p('pick_grab_cmd'))
        return [
            _WarnStep(f"pick at {where}: firing S1 grab-and-stow ({code}) "
                      f"— PROVISIONAL, untested reach"),
            _MechStep(code, float(self._p('pick_step_dwell_s')),
                      f'pick: grab-and-stow S1 ({code})'),
        ]

    def _rezero_steps(self, where):
        """Re-zero odometry at a confirmed-good pose (handoff §6) to stop
        drift accumulating across the route. Off if rezero_odom_after_climb
        is cleared."""
        if not self._p('rezero_odom_after_climb'):
            return []
        code = int(self._p('odom_rezero_cmd'))
        return [_MechStep(code, float(self._p('mech_dwell_s')),
                          f're-zero odometry ({code}) — {where}')]

    def _climb_up_steps(self, pick, end_on):
        """B1 (handoff §3): readings 0 -> 1 -> 3 -> 7 -> 15. Driving forward
        onto a higher block; the edge sweeps back under the chassis, adding
        support before removing it (no tipping window). Fully sensor-closed
        now that bit 3 senses the back deadwheel (no timed back-settle).

        CAVEAT: the bit_set gates assume the reading starts at 0 (as in the
        tested ground approach). For block-to-block transit the sensors may
        still read the previous block — bench-check the transit reading and
        switch to re-trigger variants if the gates fire instantly."""
        creep = float(self._p('creep_speed'))
        dwell = float(self._p('mech_dwell_s'))

        steps = [
            # 1. extend both lift wheels -> deadwheels off the ground (0)
            _MechStep(self._p('climb_extend_both_cmd'), dwell,
                      'extend both lift wheels (114)'),
            # 2. creep until reading 1 (0001): front deadwheel landed
            self._creep_until_step(
                +1, _bit_set(0),
                'climb-up: front deadwheel landed (reading 1 / 0001)'),
            # 3. retract the FRONT lift wheel (112)
            _MechStep(self._p('front_lift_retract_cmd'), dwell,
                      'retract front lift wheel (112)'),
            # 4. creep until reading 3 (0011)
            self._creep_until_step(
                +1, _bit_set(1),
                'climb-up: edge at middle deadwheel front (reading 3 / 0011)'),
            # 5. creep until reading 7 (0111): middle deadwheel landed
            self._creep_until_step(
                +1, _bit_set(2),
                'climb-up: middle deadwheel landed (reading 7 / 0111)'),
        ]
        # PICK POINT (§3.5): settled platform at reading 7.
        if pick:
            steps += self._pick_steps('reading 7 (middle deadwheel landed)')
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
            # 6. retract the BACK lift wheel (113)
            _MechStep(self._p('back_lift_retract_cmd'), dwell,
                      'retract back lift wheel (113)'),
            # 7. creep until reading 15 (1111): back deadwheel landed — SENSED
            #    now (bit 3), no timer
            self._creep_until_step(
                +1, _bit_set(3),
                'climb-up: back deadwheel landed (reading 15 / 1111)'),
            # 8. centering: forward D_center_up to center on the 120 cm block
            self._known_move_step(
                self._p('d_center_up_m'), self._p('d_center_up_s'),
                creep, 'climb-up: center on block (D_center_up)'),
        ]
        steps += self._rezero_steps('climb-up complete, centered')
        return steps

    def _climb_down_steps(self, pick):
        """B2 (handoff §4): readings 1111 -> 1110 -> 1100 -> 1000 -> 0000.
        Sensors clear front-to-back as each point goes over the upper edge;
        0000 = all deadwheels off the upper block, riding on both deployed
        lift wheels (airborne) = SAFETY GATE to retract.

        Front-lift deploy waits for bit 1 to clear: the front lift wheel sits
        between the front and middle deadwheels, so it only clears the upper
        edge around when bit 1 clears — deploying earlier presses it against
        the upper block. TIPPING NOTE (§4.3): in the bit0->bit1 window the
        front is unsupported (held by middle+back deadwheels), safe only if
        the CoM is behind the middle deadwheel — bench-check on first descent.
        The bit0/bit1 creeps use the SHORT ceiling so a missed IR read at the
        tipping edge cannot run the robot off the block.
        """
        creep = float(self._p('creep_speed'))
        dwell = float(self._p('mech_dwell_s'))
        short = float(self._p('creep_ceiling_short_m'))

        steps = [
            # 1. must start fully seated on the upper block (1111)
            _CheckIrStep(lambda ir, state: ir == 0b1111,
                         'climb-down start: fully on upper block (1111)',
                         strict=bool(self._p('strict_climb_down_precheck'))),
            # 2. creep SLOWLY until bit 0 clears (1110): front deadwheel over
            #    the edge. SHORT ceiling at the tipping edge.
            self._creep_until_step(
                +1, _bit_cleared(0),
                'climb-down: front deadwheel over the edge (1110)',
                ceiling=short, stop_on_done=False),
            # 3. creep until bit 1 clears (1100), THEN deploy the FRONT lift
            self._creep_until_step(
                +1, _bit_cleared(1),
                'climb-down: middle-front over the edge (1100)',
                ceiling=short, stop_on_done=False),
            _MechStep(self._p('front_lift_extend_cmd'), dwell,
                      'deploy front lift wheel down (110)'),
        ]
        # descent pick point: front now supported on the lower block.
        if pick:
            steps += self._pick_steps('descent (front on lower block)')
        steps += [
            # 4. creep until bit 2 clears (1000): middle deadwheel over edge
            self._creep_until_step(
                +1, _bit_cleared(2),
                'climb-down: middle deadwheel over the edge (1000)'),
            # 5. deploy the BACK lift wheel down (111)
            _MechStep(self._p('back_lift_extend_cmd'), dwell,
                      'deploy back lift wheel down (111)'),
            # 6. creep until reading 0000: back deadwheel off the upper block,
            #    airborne on both lift wheels = SAFETY GATE
            self._creep_until_step(
                +1, lambda ir, state: ir == 0,
                'climb-down: airborne on both lifts (0000 SAFETY GATE)'),
            # 7. centering (still RAISED at 0000): forward D_center_down to
            #    center over the lower block. Forward-FIRST.
            self._known_move_step(
                self._p('d_center_down_m'), self._p('d_center_down_s'),
                creep, 'climb-down: center over lower block (D_center_down)'),
            # 8. retract BOTH lift wheels (115): lower straight down. Second.
            _MechStep(self._p('both_lift_retract_cmd'), dwell,
                      'retract both lift wheels (115)'),
        ]
        steps += self._rezero_steps('climb-down complete, centered')
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
