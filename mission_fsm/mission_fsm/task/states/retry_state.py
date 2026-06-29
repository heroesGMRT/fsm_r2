"""RetryState — automatic retry wrapper for any child state.

Example::

    RetryState(
        child=DriveForwardState(2.0, 0.15, verify_attr="apriltag_visible"),
        max_retries=3,
        retry_delay_sec=1.5,
        recovery=RecoveryState(back_duration=1.0, back_speed=0.1),
    )

Flow:
1. Run child state until it returns non-None.
2. If child returns ``"done"`` → return ``"done"``.
3. If child returns ``"failed"`` or ``getattr(node, "step_failed", False)``:
   - execute ``recovery`` state once (if provided)
   - wait ``retry_delay_sec``
   - reset child and try again
   - after ``max_retries`` exhausted → return ``"failed"``
"""

from ..base_state import BaseState


class RetryState(BaseState):
    """Wrap a child state with automatic retry + optional recovery motion."""

    def __init__(
        self,
        child: BaseState,
        max_retries: int = 3,
        retry_delay_sec: float = 1.5,
        recovery: BaseState | None = None,
        name: str = "Retry",
    ):
        self.child = child
        self.max_retries = max_retries
        self.retry_delay_sec = retry_delay_sec
        self.recovery = recovery
        self.name = name

        # runtime
        self._retry_count = 0
        self._phase = "run"          # "run" | "recover" | "wait"
        self._wait_start = None

    def execute(self, node):
        # Update dashboard / node status so UI can show it
        node.retry_status = f"{self.name} {self._retry_count}/{self.max_retries}"

        if self._phase == "run":
            self.child.execute(node)

        elif self._phase == "recover" and self.recovery is not None:
            self.recovery.execute(node)

        elif self._phase == "wait":
            # Just idle during retry cooldown
            node.stop_cmd_vel()

    def check_transition(self, node):
        # ── WAIT phase ──────────────────────────────────────────────
        if self._phase == "wait":
            elapsed = (node.get_clock().now() - self._wait_start).nanoseconds / 1e9
            if elapsed < self.retry_delay_sec:
                return None
            # Cooldown finished → re-arm for another attempt
            self._phase = "run"
            self._retry_count += 1
            if hasattr(self.child, "reset"):
                self.child.reset()
            node.get_logger().info(
                f"{self.name}: attempt {self._retry_count + 1}/"
                f"{self.max_retries + 1}"
            )
            return None

        # ── RECOVER phase ───────────────────────────────────────────
        if self._phase == "recover":
            if self.recovery is None:
                self._phase = "wait"
                self._wait_start = node.get_clock().now()
                return None
            result = self.recovery.check_transition(node)
            if result is not None:
                # recovery done → go to wait
                if hasattr(self.recovery, "reset"):
                    self.recovery.reset()
                self._phase = "wait"
                self._wait_start = node.get_clock().now()
                node.get_logger().info(
                    f"{self.name}: recovery done, cooling down "
                    f"{self.retry_delay_sec}s..."
                )
            return None

        # ── RUN phase ───────────────────────────────────────────────
        child_result = self.child.check_transition(node)
        if child_result is None:
            return None

        if child_result == "done":
            node.retry_status = ""
            self.reset()
            return "done"

        # child reported failure (or node.step_failed was set)
        if child_result == "failed" or getattr(node, "step_failed", False):
            # clear global flag if it was used
            if getattr(node, "step_failed", False):
                node.step_failed = False

            if self._retry_count >= self.max_retries:
                node.get_logger().error(
                    f"{self.name}: child failed after {self.max_retries} retries."
                )
                node.retry_status = f"{self.name}: FAILED"
                self.reset()
                return "failed"

            # trigger recovery (if any) then wait
            node.get_logger().warn(
                f"{self.name}: child failed, retry "
                f"{self._retry_count + 1}/{self.max_retries} "
                f"after recovery+cooldown..."
            )
            if self.recovery is not None:
                self._phase = "recover"
            else:
                self._phase = "wait"
                self._wait_start = node.get_clock().now()
            return None

        # Any other result (e.g. "next") pass through
        node.retry_status = ""
        self.reset()
        return child_result

    def reset(self):
        self._retry_count = 0
        self._phase = "run"
        self._wait_start = None
        if hasattr(self.child, "reset"):
            self.child.reset()
        if self.recovery is not None and hasattr(self.recovery, "reset"):
            self.recovery.reset()
