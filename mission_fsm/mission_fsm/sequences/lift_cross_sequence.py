"""Lift-cross obstacle sequence for mission_fsm.

This module provides ``LiftCrossSequence``, a reusable, **non-blocking**
step-based helper that orchestrates the six-phase wheel-lift crossing
manoeuvre:

    Phase 1 — LIFT UP   : raise both front + back lifts              (cmd 104)
    Phase 2 — DRIVE FWD : move forward (autonomous drive cmd)         (cmd 40)
    Phase 3 — FRONT DOWN: lower front lifts while still moving        (cmd 102)
    Phase 4 — DRIVE FWD : continue forward                            (cmd 40)
    Phase 5 — BACK DOWN : lower back lifts, robot fully across        (cmd 103)
    Phase 6 — DRIVE FWD : move forward again                          (cmd 40)

Usage
-----
Instantiate once and then call ``tick(node)`` from your FSM state's
``execute()`` method every loop iteration.  ``tick()`` returns ``True``
when the sequence finishes so the caller can trigger a state transition.

Example (inside an area state)::

    from ...sequences.lift_cross_sequence import LiftCrossSequence

    class MyState(BaseState):
        def __init__(self):
            self._seq = LiftCrossSequence(
                drive_duration_1=3.0,   # seconds to drive before front-down
                drive_duration_2=2.0,   # seconds to drive before back-down
                drive_duration_3=2.0,   # seconds to drive after back-down
                lift_settle_time=1.5,   # seconds to wait after each lift cmd
            )

        def execute(self, node):
            done = self._seq.tick(node)
            if done:
                node.get_logger().info("Lift-cross complete!")

        def check_transition(self, node):
            if self._seq.is_done():
                return "NEXT_STATE"
            return None

You can also reset and replay the sequence at any time::

    self._seq.reset()

FSM Command reference (from keyboard_teleop_node.py)
----------------------------------------------------
    104  — BOTH lifts UP    (encoder/limit-switch)
    102  — Front lift DOWN  (encoder/limit-switch)
    103  — Back lift DOWN   (encoder/limit-switch)
     40  — Start autonomous drive sequence
     99  — Emergency STOP (not used here, caller's responsibility)
"""

from __future__ import annotations

from enum import IntEnum, auto

from std_msgs.msg import Int32


# ── FSM command integers ───────────────────────────────────────────────────────
_CMD_BOTH_LIFT_UP    = 104
_CMD_FRONT_LIFT_DOWN = 102
_CMD_BACK_LIFT_DOWN  = 103
_CMD_DRIVE_FORWARD   = 40


class _Phase(IntEnum):
    """Internal phases of the lift-cross sequence."""
    IDLE        = auto()  # not started yet
    LIFT_UP     = auto()  # (1) raise both lifts, wait for settle
    DRIVE_1     = auto()  # (2) drive forward for drive_duration_1
    FRONT_DOWN  = auto()  # (3) lower front lift, wait for settle
    DRIVE_2     = auto()  # (4) drive forward for drive_duration_2
    BACK_DOWN   = auto()  # (5) lower back lift, wait for settle
    DRIVE_3     = auto()  # (6) drive forward for drive_duration_3
    DONE        = auto()  # sequence complete


class LiftCrossSequence:
    """Non-blocking five-phase lift-cross-obstacle sequence.

    Designed to be called repeatedly from a ROS 2 timer callback
    (e.g. at 10 Hz) via ``tick(node)``.

    Args:
        drive_duration_1 (float):
            Seconds to drive forward between lift-up and front-down.
        drive_duration_2 (float):
            Seconds to drive forward between front-down and back-down.
        drive_duration_3 (float):
            Seconds to drive forward after back-down.
        lift_settle_time (float):
            Seconds to wait after each lift command to allow mechanical
            movement to complete before issuing the next command.
        fsm_topic (str):
            ROS 2 topic name for FSM integer commands.  Defaults to
            ``/fsm_command``.
    """

    FSM_TOPIC = "/fsm_command"

    def __init__(
        self,
        drive_duration_1: float = 3.0,
        drive_duration_2: float = 2.0,
        drive_duration_3: float = 2.0,
        lift_settle_time: float = 1.5,
        fsm_topic: str = FSM_TOPIC,
    ):
        self._drive_dur_1    = float(drive_duration_1)
        self._drive_dur_2    = float(drive_duration_2)
        self._drive_dur_3    = float(drive_duration_3)
        self._lift_settle    = float(lift_settle_time)
        self._fsm_topic      = fsm_topic

        self._phase          = _Phase.IDLE
        self._phase_start_t  = None   # rclpy.Time when current phase began
        self._publisher      = None   # lazy-initialised on first tick
        self._cmd_sent       = False  # True after the phase's command is sent

    # ── Public API ─────────────────────────────────────────────────────────────

    def start(self, node) -> None:
        """Explicitly start (or re-start) the sequence from Phase 1.

        Calling ``tick()`` without ``start()`` is also fine — the sequence
        begins automatically on the first tick if it is in the IDLE phase.
        """
        self.reset()
        self._ensure_publisher(node)
        self._enter_phase(_Phase.LIFT_UP, node)

    def reset(self) -> None:
        """Reset the sequence back to IDLE so it can be replayed."""
        self._phase         = _Phase.IDLE
        self._phase_start_t = None
        self._cmd_sent      = False

    def is_done(self) -> bool:
        """Return ``True`` when the full sequence has completed."""
        return self._phase == _Phase.DONE

    def is_running(self) -> bool:
        """Return ``True`` while the sequence is in progress."""
        return self._phase not in (_Phase.IDLE, _Phase.DONE)

    def tick(self, node) -> bool:
        """Advance the sequence by one step.

        Call this every loop iteration (e.g. from ``BaseState.execute()``).

        Args:
            node: The ROS 2 node used for logging, time, and publishing.

        Returns:
            ``True`` when the sequence has just finished (transitions from
            the last phase to DONE), ``False`` otherwise.
        """
        self._ensure_publisher(node)

        if self._phase == _Phase.IDLE:
            self._enter_phase(_Phase.LIFT_UP, node)
            return False

        if self._phase == _Phase.DONE:
            return False

        now = node.get_clock().now()
        elapsed = (now - self._phase_start_t).nanoseconds * 1e-9  # seconds

        # ── Phase dispatch ────────────────────────────────────────────────────
        if self._phase == _Phase.LIFT_UP:
            return self._run_lift_phase(
                node, elapsed,
                cmd=_CMD_BOTH_LIFT_UP,
                settle=self._lift_settle,
                label="BOTH lifts UP",
                next_phase=_Phase.DRIVE_1,
            )

        if self._phase == _Phase.DRIVE_1:
            return self._run_drive_phase(
                node, elapsed,
                duration=self._drive_dur_1,
                label="Drive forward (phase 1)",
                next_phase=_Phase.FRONT_DOWN,
            )

        if self._phase == _Phase.FRONT_DOWN:
            return self._run_lift_phase(
                node, elapsed,
                cmd=_CMD_FRONT_LIFT_DOWN,
                settle=self._lift_settle,
                label="Front lift DOWN",
                next_phase=_Phase.DRIVE_2,
            )

        if self._phase == _Phase.DRIVE_2:
            return self._run_drive_phase(
                node, elapsed,
                duration=self._drive_dur_2,
                label="Drive forward (phase 2)",
                next_phase=_Phase.BACK_DOWN,
            )

        if self._phase == _Phase.BACK_DOWN:
            return self._run_lift_phase(
                node, elapsed,
                cmd=_CMD_BACK_LIFT_DOWN,
                settle=self._lift_settle,
                label="Back lift DOWN",
                next_phase=_Phase.DRIVE_3,
            )

        if self._phase == _Phase.DRIVE_3:
            return self._run_drive_phase(
                node, elapsed,
                duration=self._drive_dur_3,
                label="Drive forward (phase 3)",
                next_phase=_Phase.DONE,
            )

        return False

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _ensure_publisher(self, node) -> None:
        """Lazily create the /fsm_command publisher on first use."""
        if self._publisher is None:
            self._publisher = node.create_publisher(Int32, self._fsm_topic, 10)

    def _enter_phase(self, phase: _Phase, node) -> None:
        """Transition to a new phase and record the start time."""
        self._phase         = phase
        self._phase_start_t = node.get_clock().now()
        self._cmd_sent      = False
        node.get_logger().info(
            f"[LiftCrossSequence] → Phase {phase.name}"
        )

    def _send_fsm_cmd(self, node, cmd: int, label: str) -> None:
        """Publish a single FSM integer command."""
        msg      = Int32()
        msg.data = cmd
        self._publisher.publish(msg)
        node.get_logger().info(
            f"[LiftCrossSequence] FSM cmd {cmd}  ({label})"
        )
        self._cmd_sent = True

    def _run_lift_phase(
        self,
        node,
        elapsed: float,
        cmd: int,
        settle: float,
        label: str,
        next_phase: _Phase,
    ) -> bool:
        """Generic handler for a 'send command, then wait settle seconds' phase.

        Returns ``True`` only when transitioning to ``_Phase.DONE``.
        """
        if not self._cmd_sent:
            self._send_fsm_cmd(node, cmd, label)

        if elapsed >= settle:
            if next_phase == _Phase.DONE:
                self._phase = _Phase.DONE
                node.get_logger().info(
                    "[LiftCrossSequence] ✓ Sequence COMPLETE"
                )
                return True
            self._enter_phase(next_phase, node)
        return False

    def _run_drive_phase(
        self,
        node,
        elapsed: float,
        duration: float,
        label: str,
        next_phase: _Phase,
    ) -> bool:
        """Generic handler for a 'send drive command, wait duration seconds' phase."""
        if not self._cmd_sent:
            self._send_fsm_cmd(node, _CMD_DRIVE_FORWARD, label)

        if elapsed >= duration:
            if next_phase == _Phase.DONE:
                self._phase = _Phase.DONE
                node.get_logger().info(
                    "[LiftCrossSequence] ✓ Sequence COMPLETE"
                )
                return True
            self._enter_phase(next_phase, node)
        return False
