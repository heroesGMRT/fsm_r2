"""Area 1 state for mission_fsm."""

import json
import os
import signal
import subprocess
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

from ..base_state import BaseState
from ...config.loader import AREA_GOALS

# ── Exit move after proximity task (hardcoded, tune as needed) ──────────────
_EXIT_MOVE_X   = 0.5   # metres (negative = reverse)
_EXIT_MOVE_Y   =  0.0   # metres (positive = left strafe)
_EXIT_WAIT_SEC =  3.0   # seconds to wait before transitioning to AREA_2

# ── Post-exit rotate + green-flash check (NEW) ───────────────────────────────
# /relative_move z = yaw in RADIANS (REP-103). 3.14159 = 180°, a half turn.
_ROTATE_Z        = 3.14159
_ROTATE_WAIT_SEC =   2.0   # seconds to hold the rotate command
_FLASH_AREA_NAME = "AREA_1_FLASH"  # informational only — green_light_node filters on "task"

# Seconds to hold after the proximity node's pickup-done signal before the
# exit (backward) move, giving the gripper time to settle.
_PICKUP_SETTLE_SEC = 5.0

# Proximity workspace to source before `ros2 launch`. Overridable per-machine
# via the PROXYMITY_WS env var (the two dev machines used different layouts).
_PROXYMITY_WS = os.environ.get(
    "PROXYMITY_WS", "/home/wafdan/workshop/proxymity_ws"
)


def kill_proxymity_process(node):
    """Kill the proxymity_launch.py process group if it's still running.

    Uses a process-group kill (not just the immediate bash PID) since
    `ros2 launch` spawns proxymity_node / proxymity_controller_node /
    green_light_node / the RealSense driver as its own children — killing
    only the shell would leave them orphaned and still holding GPIO/camera
    resources (this is what caused the "3 stale proximity_sensor nodes"
    GPIO-busy issue).

    Called both when Area 1 finishes (check_transition) and on FSM
    reset/retry (fsm_node.py), so a fresh instance can always launch cleanly.
    """
    proc = getattr(node, "proxymity_process", None)
    if proc is None:
        return

    if proc.poll() is not None:
        # Already exited on its own.
        node.proxymity_process = None
        return

    try:
        pgid = os.getpgid(proc.pid)
        node.get_logger().info(f"Killing proxymity_launch.py process group (pgid={pgid})...")
        os.killpg(pgid, signal.SIGINT)
        try:
            proc.wait(timeout=5.0)
            node.get_logger().info("proxymity_launch.py exited cleanly after SIGINT.")
        except subprocess.TimeoutExpired:
            node.get_logger().warn(
                "proxymity_launch.py didn't exit after SIGINT within 5s — sending SIGKILL."
            )
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=2.0)
    except ProcessLookupError:
        pass  # process group already gone
    except Exception as e:
        node.get_logger().error(f"Error killing proxymity_launch.py: {e}")
    finally:
        node.proxymity_process = None


class Area1State(BaseState):
    """Handles robot behaviour for Area 1.

    Phase 1: publish /relative_move (values from areas.yaml config)
    Phase 2: monitor move via system clock
    Phase 3: spawn proxymity_launch.py subprocess
    Phase 4: when proximity signals done (signal '31') → publish exit
             /relative_move, wait, then transition to AREA_2
    Phase 5: rotate 180 degrees via /relative_move
    Phase 6: trigger green_light_node via /fsm/area_command, wait for
             area_complete, then transition to AREA_2
    """

    def execute(self, node):
        # ── PHASE 1: Initial Movement Trigger ──────────────────────────────
        if not getattr(node, "post_align_nav_triggered", False):
            node.post_align_nav_triggered = True
            node.post_align_move_complete = False
            node.proximity_started = False
            node.settle_started = False
            node.exit_move_triggered = False
            node.exit_move_complete = False
            # NEW guards — reset alongside the rest on (re)entry to Area 1
            node.rotate_triggered = False
            node.rotate_complete = False
            node.flash_triggered = False
            node.flash_complete = False

            cfg = AREA_GOALS.get("area_1", {})
            move_x        = float(cfg.get("move_x",        -0.7))
            move_y        = float(cfg.get("move_y",         0.4))
            move_wait_sec = float(cfg.get("move_wait_sec",  0.5))
            # Captured now so Phase 3's launch picks up the UI-set value.
            node.prox_forward_x = float(cfg.get("prox_forward_x", -1.5))

            msg = Vector3()
            msg.x = move_x
            msg.y = move_y
            msg.z = 0.0
            node.relative_move_pub.publish(msg)
            node.get_logger().info(
                f"Area 1: /relative_move x={move_x} y={move_y}, "
                f"waiting {move_wait_sec}s."
            )
            node.move_finish_timestamp = (
                node.get_clock().now().nanoseconds + int(move_wait_sec * 1e9)
            )

        # ── PHASE 2: Monitor Movement Progress ─────────────────────────────
        if (
            getattr(node, "post_align_nav_triggered", False)
            and hasattr(node, "move_finish_timestamp")
            and not getattr(node, "post_align_move_complete", False)
        ):
            if node.get_clock().now().nanoseconds >= node.move_finish_timestamp:
                node.post_align_move_complete = True
                node.get_logger().info("Area 1: initial move complete.")

        # ── PHASE 3: Spawn Proximity Package ───────────────────────────────
        if (
            getattr(node, "post_align_move_complete", False)
            and not getattr(node, "proximity_started", False)
        ):
            node.proximity_started = True
            node.get_logger().info("Area 1: spawning proxymity_launch.py...")
            try:
                prox_forward_x = float(getattr(node, "prox_forward_x", -1.5))
                cmd = (
                    "source /opt/ros/$ROS_DISTRO/setup.bash && "
                    f"source {_PROXYMITY_WS}/install/setup.bash && "
                    "ros2 launch proxymity proxymity_launch.py "
                    f"forward_relative_x:={prox_forward_x}"
                )
                log_path = "/tmp/proxymity_launch.log"
                log_file = open(log_path, "w")
                node.proxymity_process = subprocess.Popen(
                    cmd, shell=True, executable="/bin/bash",
                    stdout=log_file, stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,  # new process group so we can kill the whole tree later
                )
                node.get_logger().info(
                    f"Area 1: proxymity_launch.py spawned (pid={node.proxymity_process.pid}). "
                    f"Output logging to {log_path}"
                )
            except Exception as e:
                node.get_logger().error(f"Area 1: failed to launch proximity: {e}")

        # ── PHASE 4a: Proximity done → start settle timer ──────────────────
        if (
            getattr(node, "proximity_started", False)
            and getattr(node, "proximity_done", False)
            and not getattr(node, "settle_started", False)
        ):
            node.settle_started = True
            node.settle_finish_timestamp = (
                node.get_clock().now().nanoseconds + int(_PICKUP_SETTLE_SEC * 1e9)
            )
            node.get_logger().info(
                f"Area 1: proximity done — settling {_PICKUP_SETTLE_SEC}s "
                "before exit move."
            )

        # ── PHASE 4b: Settle elapsed → exit move ───────────────────────────
        if (
            getattr(node, "settle_started", False)
            and node.get_clock().now().nanoseconds
                >= getattr(node, "settle_finish_timestamp", 0)
            and not getattr(node, "exit_move_triggered", False)
        ):
            node.exit_move_triggered = True
            node.exit_move_complete = False

            msg = Vector3()
            msg.x = _EXIT_MOVE_X
            msg.y = _EXIT_MOVE_Y
            msg.z = 0.0
            node.relative_move_pub.publish(msg)
            node.get_logger().info(
                f"Area 1: proximity done → exit /relative_move "
                f"x={_EXIT_MOVE_X} y={_EXIT_MOVE_Y}, waiting {_EXIT_WAIT_SEC}s."
            )
            node.exit_finish_timestamp = (
                node.get_clock().now().nanoseconds + int(_EXIT_WAIT_SEC * 1e9)
            )

        # Monitor exit move timer (same clock approach as Phase 2)
        if (
            getattr(node, "exit_move_triggered", False)
            and hasattr(node, "exit_finish_timestamp")
            and not getattr(node, "exit_move_complete", False)
        ):
            if node.get_clock().now().nanoseconds >= node.exit_finish_timestamp:
                node.exit_move_complete = True
                node.get_logger().info("Area 1: exit move complete.")

        # ── PHASE 5: Rotate 180 degrees (NEW) ───────────────────────────────
        if (
            getattr(node, "exit_move_complete", False)
            and not getattr(node, "rotate_triggered", False)
        ):
            node.rotate_triggered = True
            node.rotate_complete = False

            msg = Vector3()
            msg.x = 0.0
            msg.y = 0.0
            msg.z = _ROTATE_Z
            node.relative_move_pub.publish(msg)
            node.get_logger().info(
                f"Area 1: rotating — /relative_move z={_ROTATE_Z}, "
                f"waiting {_ROTATE_WAIT_SEC}s."
            )
            node.rotate_finish_timestamp = (
                node.get_clock().now().nanoseconds + int(_ROTATE_WAIT_SEC * 1e9)
            )

        # Monitor rotate timer
        if (
            getattr(node, "rotate_triggered", False)
            and hasattr(node, "rotate_finish_timestamp")
            and not getattr(node, "rotate_complete", False)
        ):
            if node.get_clock().now().nanoseconds >= node.rotate_finish_timestamp:
                node.rotate_complete = True
                node.get_logger().info("Area 1: rotation complete.")

        # ── PHASE 6: Trigger green-flash detection (NEW) ────────────────────
        if (
            getattr(node, "rotate_complete", False)
            and not getattr(node, "flash_triggered", False)
        ):
            node.flash_triggered = True
            # Reset the shared area_complete flag so we only react to THIS
            # green_light_node completion, not a stale one from elsewhere.
            node.area_complete = False

            area_cmd = String()
            area_cmd.data = json.dumps({
                "command": "start",
                "task": "green_detection",
                "area": _FLASH_AREA_NAME,
            })
            node.area_cmd_pub.publish(area_cmd)
            node.get_logger().info(
                f"Area 1: triggered green-flash detection for '{_FLASH_AREA_NAME}' "
                "via /fsm/area_command. Waiting for area_complete..."
            )

        # Monitor for green_light_node's completion signal (sets node.area_complete
        # via fsm_node.py's existing _signal_callback — no new subscription needed)
        if (
            getattr(node, "flash_triggered", False)
            and not getattr(node, "flash_complete", False)
            and getattr(node, "area_complete", False)
        ):
            node.flash_complete = True
            node.get_logger().info("Area 1: green flash confirmed.")

    def check_transition(self, node):
        if getattr(node, "flash_complete", False):
            node.get_logger().info("Area 1 complete. Transitioning to AREA_2.")
            kill_proxymity_process(node)
            return "AREA_2"
        return None