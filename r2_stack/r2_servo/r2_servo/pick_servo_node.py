"""AlignAndPick action server: PBVS lateral alignment before a KFS pick.

State machine (per handoff spec)::

    IDLE -> WAITING_FOR_SETTLE -> MEASURE -> TRANSFORM -> EVALUATE
         -> COMMAND_STRAFE -> RE_MEASURE -> (PICK) -> DONE

Settle-waiting and frame averaging live inside the kfs_localizer (its
LocateKfs action), so MEASURE here is one action call; the localizer's
feedback states are forwarded into AlignAndPick feedback.

Strafe backends (parameter ``strafe_backend``):
  * ``nav2``    — primary: a short relative NavigateToPose goal, lateral
                  offset applied in the base frame. Completion feedback for
                  free, but bounded by Nav2's goal tolerance.
  * ``cmd_vel`` — fallback: open-loop timed Twist on ``cmd_vel_topic``
                  (bridged to the Teensy). Used when Nav2 is not running
                  or its tolerance proves too coarse on the blocks.

TF discipline: the measured centroid is transformed camera->gripper at the
*measurement timestamp* (never ``Time(0)``); the only latest-available
lookup is "where is the robot now" when composing the Nav2 goal, where
latest is the correct semantics.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped support)

from r2_interfaces.action import AlignAndPick, LocateKfs


class ServoAbort(Exception):
    """Raised inside the goal execution to abort with a message."""


class PickServoNode(Node):

    def __init__(self):
        super().__init__('pick_servo')
        self._cb_group = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('gripper_frame', 'gripper_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('align_tolerance_mm', 10.0)
        self.declare_parameter('re_measure_threshold_mm', 15.0)
        self.declare_parameter('max_strafe_attempts', 2)
        self.declare_parameter('max_strafe_m', 0.30)
        self.declare_parameter('strafe_backend', 'nav2')  # 'nav2' | 'cmd_vel'
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('strafe_speed_m_s', 0.05)
        self.declare_parameter('locate_action', 'locate_kfs')
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('server_wait_s', 5.0)
        self.declare_parameter('measure_timeout_s', 30.0)
        self.declare_parameter('nav_timeout_s', 20.0)
        self.declare_parameter('tf_timeout_s', 1.0)
        # Pick sequence is not implemented on the Teensy yet; the executor
        # dispatches the PICK_BLOCK_* primitive after this action succeeds.
        self.declare_parameter('enable_pick_command', False)

        # ── TF ──────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Clients / publishers ────────────────────────────────────
        self._locate_client = ActionClient(
            self, LocateKfs,
            self.get_parameter('locate_action').value,
            callback_group=self._cb_group)
        self._nav_client = ActionClient(
            self, NavigateToPose,
            self.get_parameter('nav2_action').value,
            callback_group=self._cb_group)
        self._cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self._busy = False
        self._busy_lock = threading.Lock()

        self._server = ActionServer(
            self, AlignAndPick, 'align_and_pick',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group)

        self.get_logger().info(
            "pick_servo ready on 'align_and_pick' "
            f"(strafe backend: {self.get_parameter('strafe_backend').value}). "
            'Idles gracefully while localizer/Nav2 are absent.')

    # ── Goal gatekeeping ────────────────────────────────────────────
    def _on_goal(self, goal_request):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Rejecting AlignAndPick goal: busy.')
                return GoalResponse.REJECT
            if goal_request.block_height > AlignAndPick.Goal.HEIGHT_C:
                self.get_logger().error(
                    f'Rejecting AlignAndPick goal: bad height '
                    f'{goal_request.block_height}')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    # ── Helpers ─────────────────────────────────────────────────────
    def _param(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _wait_future(future, timeout_s):
        """Block the executing thread on a future without spinning."""
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_s):
            return None
        return future.result()

    def _feedback(self, goal_handle, state, offset_mm=float('nan')):
        fb = AlignAndPick.Feedback()
        fb.state = state
        fb.current_offset_mm = float(offset_mm)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(
            f'AlignAndPick: {state}'
            + ('' if math.isnan(offset_mm) else f' (offset {offset_mm:.1f} mm)'))

    def _check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            raise ServoAbort('canceled by client')

    # ── MEASURE: call the localizer ─────────────────────────────────
    def _measure(self, goal_handle, block_id, block_height):
        self._feedback(goal_handle, 'MEASURE')
        if not self._locate_client.wait_for_server(
                timeout_sec=self._param('server_wait_s')):
            raise ServoAbort(
                'kfs_localizer action server not available '
                f"('{self._param('locate_action')}')")

        goal = LocateKfs.Goal()
        goal.block_id = int(block_id)
        goal.block_height = int(block_height)

        def forward_feedback(fb_msg):
            self._feedback(goal_handle, fb_msg.feedback.state)

        handle = self._wait_future(
            self._locate_client.send_goal_async(
                goal, feedback_callback=forward_feedback),
            self._param('server_wait_s'))
        if handle is None or not handle.accepted:
            raise ServoAbort('localizer rejected or ignored the measure goal')

        result = self._wait_future(
            handle.get_result_async(), self._param('measure_timeout_s'))
        if result is None:
            handle.cancel_goal_async()
            raise ServoAbort('localizer measurement timed out')
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            msg = result.result.message if result.result else 'unknown'
            raise ServoAbort(f'localizer measurement failed: {msg}')
        return result.result.centroid

    # ── TRANSFORM + EVALUATE ────────────────────────────────────────
    def _lateral_offset_m(self, goal_handle, centroid):
        """Transform the centroid into the gripper frame; the gripper-frame
        lateral (Y) coordinate IS the strafe error. Positive = target left."""
        self._feedback(goal_handle, 'TRANSFORM')
        gripper_frame = self._param('gripper_frame')
        try:
            # Transform at the measurement stamp — NOT Time(0). The camera
            # extrinsic is static, so this resolves as long as TF is up.
            pt = self._tf_buffer.transform(
                centroid, gripper_frame,
                timeout=Duration(seconds=self._param('tf_timeout_s')))
        except tf2_ros.TransformException as exc:
            raise ServoAbort(
                f'TF {centroid.header.frame_id} -> {gripper_frame} failed: {exc}')
        offset_m = pt.point.y
        self._feedback(goal_handle, 'EVALUATE', offset_m * 1000.0)
        return offset_m

    # ── COMMAND_STRAFE backends ─────────────────────────────────────
    def _strafe(self, goal_handle, offset_m):
        self._feedback(goal_handle, 'COMMAND_STRAFE', offset_m * 1000.0)
        if abs(offset_m) > self._param('max_strafe_m'):
            raise ServoAbort(
                f'strafe {offset_m * 1000.0:.0f} mm exceeds max_strafe_m — '
                'measurement or extrinsics are suspect')
        backend = self._param('strafe_backend')
        if backend == 'nav2':
            self._strafe_nav2(goal_handle, offset_m)
        elif backend == 'cmd_vel':
            self._strafe_cmd_vel(goal_handle, offset_m)
        else:
            raise ServoAbort(f"unknown strafe_backend '{backend}'")

    def _strafe_nav2(self, goal_handle, offset_m):
        if not self._nav_client.wait_for_server(
                timeout_sec=self._param('server_wait_s')):
            raise ServoAbort('Nav2 navigate_to_pose server not available; '
                             "consider strafe_backend:=cmd_vel")

        global_frame = self._param('global_frame')
        base_frame = self._param('base_frame')
        try:
            # Latest-available is correct here: we want the robot's CURRENT
            # pose to compose a relative goal.
            tf = self._tf_buffer.lookup_transform(
                global_frame, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self._param('tf_timeout_s')))
        except tf2_ros.TransformException as exc:
            raise ServoAbort(f'TF {global_frame} -> {base_frame} failed: {exc}')

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        # Lateral offset in the base frame, rotated into the global frame.
        pose.pose.position.x = t.x + offset_m * -math.sin(yaw)
        pose.pose.position.y = t.y + offset_m * math.cos(yaw)
        pose.pose.position.z = 0.0
        pose.pose.orientation = q
        goal.pose = pose

        handle = self._wait_future(
            self._nav_client.send_goal_async(goal), self._param('server_wait_s'))
        if handle is None or not handle.accepted:
            raise ServoAbort('Nav2 rejected the strafe goal')
        result = self._wait_future(
            handle.get_result_async(), self._param('nav_timeout_s'))
        if result is None:
            handle.cancel_goal_async()
            raise ServoAbort('Nav2 strafe timed out')
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise ServoAbort(f'Nav2 strafe failed (status {result.status})')

    def _strafe_cmd_vel(self, goal_handle, offset_m):
        """Open-loop timed strafe. Positive offset = target is to the robot's
        left = strafe left = +Y twist (REP-103)."""
        speed = abs(self._param('strafe_speed_m_s'))
        if speed <= 0.0:
            raise ServoAbort('strafe_speed_m_s must be > 0')
        duration_s = abs(offset_m) / speed
        twist = Twist()
        twist.linear.y = math.copysign(speed, offset_m)

        period = 0.05  # 20 Hz command stream
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            self._cmd_vel_pub.publish(twist)
            time.sleep(period)
        stop = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(stop)
            time.sleep(period)

    # ── Main execution ──────────────────────────────────────────────
    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = AlignAndPick.Result()
        align_tol_m = self._param('align_tolerance_mm') / 1000.0
        re_measure_tol_m = self._param('re_measure_threshold_mm') / 1000.0
        max_attempts = int(self._param('max_strafe_attempts'))

        self.get_logger().info(
            f'AlignAndPick goal: block {goal.block_id}, '
            f'height {goal.block_height}')
        try:
            # Settle wait happens inside the localizer; state is forwarded
            # from its feedback. First measurement:
            self._check_cancel(goal_handle)
            centroid = self._measure(goal_handle, goal.block_id, goal.block_height)
            offset_m = self._lateral_offset_m(goal_handle, centroid)

            attempts = 0
            while abs(offset_m) > align_tol_m and attempts < max_attempts:
                self._check_cancel(goal_handle)
                self._strafe(goal_handle, offset_m)
                attempts += 1
                self._feedback(goal_handle, 'RE_MEASURE')
                centroid = self._measure(
                    goal_handle, goal.block_id, goal.block_height)
                offset_m = self._lateral_offset_m(goal_handle, centroid)

            if abs(offset_m) > re_measure_tol_m:
                raise ServoAbort(
                    f'residual offset {offset_m * 1000.0:.1f} mm still above '
                    f"{self._param('re_measure_threshold_mm')} mm after "
                    f'{attempts} strafe attempt(s)')

            if self._param('enable_pick_command'):
                # Placeholder until the pick sequence exists on the Teensy;
                # today the executor dispatches PICK_BLOCK_* itself.
                self._feedback(goal_handle, 'PICK', offset_m * 1000.0)
                self.get_logger().warn(
                    'enable_pick_command is true but the pick sequence is '
                    'not implemented; skipping.')

            self._feedback(goal_handle, 'DONE', offset_m * 1000.0)
            result.success = True
            result.final_offset_mm = float(offset_m * 1000.0)
            result.message = f'aligned in {attempts} strafe attempt(s)'
            goal_handle.succeed()
        except ServoAbort as exc:
            result.success = False
            result.final_offset_mm = float('nan')
            result.message = str(exc)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f'AlignAndPick canceled: {exc}')
            else:
                goal_handle.abort()
                self.get_logger().error(f'AlignAndPick aborted: {exc}')
        finally:
            with self._busy_lock:
                self._busy = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickServoNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
