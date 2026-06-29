"""CustomTaskManager — fully automatic mission with retry & recovery.

Usage in fsm_node.py:
    from .task.custom_task_manager import CustomTaskManager as TaskManager
"""

from .states.idle import IdleState
from .states.done import DoneState
from .states.failed_state import FailedState
from .states.sequence_state import SequenceState
from .states.navigate_state import NavigateState
from .states.cmd_vel_state import (
    DriveForwardState,
    DriveBackwardState,
    RotateLeftState,
    RotateRightState,
    CurveState,
    StopState,
)
from .states.wait_state import WaitState
from .states.wait_signal_state import WaitForSignalState
from .states.retry_state import RetryState
from .states.recovery_state import RecoveryState
from .states.test_cmd_vel_state import TestCmdVelState


class CustomTaskManager:
    """Fully automatic mission composer with retry & recovery support."""

    def __init__(self, node):
        self.node = node
        self.current_state = "IDLE"

        # =================================================================
        # BUILD YOUR MISSION SEQUENCE HERE
        # =================================================================

        # ── Recovery template (used by RetryState) ───────────────────────
        # Robot mundur sedikit sebelum mengulang sebuah gerakan.
        default_recovery = RecoveryState(back_duration=1.0, back_speed=0.12, wait_after=0.5)

        # ── AREA 1 SEQUENCE ──────────────────────────────────────────────
        # Navigasi ke area_1 → koreksi presisi.
        # Retry example: maju ke objek dengan verifikasi sensor.
        # Jika node.apriltag_visible == False setelah maju, retry 2x.
        # -----------------------------------------------------------------
        area_1_seq = SequenceState([
            # 1. Navigasi ke area_1 (Nav2 retry 3x built-in)
            NavigateState.from_areas_yaml("area_1", max_retries=3, retry_delay_sec=2.0),

            WaitState(1.0),

            # 2. Putar 90° kanan (tune 3.2s sesuai robotmu)
            RotateRightState(duration_sec=3.2, angular_speed=0.5),

            # 3. Maju pelan 40 cm — dengan RETRY + RECOVERY.
            #    Kalau setelah maju node.apriltag_visible masih False,
            #    robot mundur (recovery) lalu maju lagi, max 2x retry.
            RetryState(
                child=DriveForwardState(
                    duration_sec=2.7,
                    speed=0.15,
                    verify_attr="apriltag_visible",   # set node.apriltag_visible = True/False
                ),
                max_retries=2,
                retry_delay_sec=1.5,
                recovery=default_recovery,
                name="FwdAlign",
            ),

            StopState(),
            WaitState(2.0),   # simulasi: gripper ambil objek

            # 4. Mundur menjauh
            DriveBackwardState(duration_sec=1.5, speed=0.12),
            StopState(),
        ], next_state="AREA_2", name="Area1Pickup")

        # ── AREA 2 SEQUENCE ──────────────────────────────────────────────
        area_2_seq = SequenceState([
            NavigateState.from_areas_yaml("area_2", max_retries=3, retry_delay_sec=2.0),
            WaitState(0.5),

            DriveBackwardState(duration_sec=1.3, speed=0.15),

            # Putar 180° dengan retry: kalau robot belum tepat,
            # cek node.imu_aligned (dummy flag) dan retry.
            RetryState(
                child=RotateRightState(
                    duration_sec=6.4,
                    angular_speed=0.5,
                    # verify_attr="imu_aligned",  # uncomment kalau punya IMU check
                ),
                max_retries=1,
                retry_delay_sec=1.0,
                recovery=RecoveryState(back_duration=0.5, back_speed=0.1),
                name="Turn180",
            ),

            DriveForwardState(duration_sec=2.0, speed=0.15),
            StopState(),

            # Tunggu sinyal eksternal (executor)
            WaitForSignalState("area_complete"),

            DriveBackwardState(duration_sec=1.0, speed=0.12),
            StopState(),
        ], next_state="AREA_3", name="Area2Forest")

        # ── AREA 3 SEQUENCE ──────────────────────────────────────────────
        area_3_seq = SequenceState([
            NavigateState.from_areas_yaml("area_3", max_retries=3, retry_delay_sec=2.0),
            WaitState(0.8),

            RotateLeftState(duration_sec=1.6, angular_speed=0.5),
            DriveForwardState(duration_sec=1.5, speed=0.12),
            StopState(),

            WaitState(3.0),   # drop / scan akhir
            StopState(),
        ], next_state="DONE", name="Area3Finish")

        # ── TEST SEQUENCE (bypass Nav2 — langsung test /cmd_vel) ─────────
        test_seq = SequenceState([
            TestCmdVelState(duration_sec=3.0, speed=0.15),
            WaitState(1.0),
            TestCmdVelState(duration_sec=2.0, speed=-0.10),  # mundur
            WaitState(1.0),
            StopState(),
        ], next_state="DONE", name="TestCmdVel")

        # ── REGISTER STATES ──────────────────────────────────────────────
        self.states = {
            "IDLE":        IdleState(),
            "TEST_CMDVEL": test_seq,   # ← ganti entry-point untuk test
            "AREA_1":      area_1_seq,
            "AREA_2":      area_2_seq,
            "AREA_3":      area_3_seq,
            "DONE":        DoneState(),
            "FAILED":      FailedState(),
        }

    def update(self):
        state = self.states[self.current_state]
        state.execute(self.node)

        next_state = state.check_transition(self.node)
        if next_state:
            self.node.get_logger().info(
                f"FSM transition: {self.current_state} -> {next_state}"
            )
            self.current_state = next_state

    def reset(self):
        for s in self.states.values():
            if hasattr(s, "reset"):
                s.reset()
