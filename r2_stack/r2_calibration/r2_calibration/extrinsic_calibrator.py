"""Offline extrinsic calibration: base_link -> camera_link for the D435i.

Run OFFLINE with the robot stationary — never during a match. Warm the
camera up 5+ minutes first (thermal drift affects depth and intrinsics).

Three modes::

    # 1. Capture N ChArUco detections (board on a jig at a known base_link
    #    pose). Also records camera_link->color-optical from the live driver.
    ros2 run r2_calibration extrinsic_calibrator capture \
        --session /tmp/calib_session.yaml --samples 30

    # 2. Solve base_link->camera_link from the session + the known board
    #    pose, and write the result into r2_bringup's extrinsics file.
    ros2 run r2_calibration extrinsic_calibrator solve \
        --session /tmp/calib_session.yaml \
        --board-pose 0.50 0.0 0.20 0 0 180 \
        --output <repo>/r2_bringup/config/camera_extrinsics.yaml

    # 3. Verify: place a real KFS cube at a known base_link position, run
    #    the localizer, and report residual in mm. Repeat at 0.35, 0.5 and
    #    0.8 m — stereo disparity error is non-linear, check all three.
    ros2 run r2_calibration extrinsic_calibrator verify \
        --extrinsics <repo>/r2_bringup/config/camera_extrinsics.yaml \
        --cube-pos 0.50 0.0 0.375 --samples 10

Recalibrate whenever verification residual exceeds 10 mm or the camera
mount has been physically disturbed.
"""

import argparse
import sys
import time

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from .transforms import (
    average_se3,
    matrix_to_rpy_deg,
    quat_to_matrix,
    rpy_deg_to_matrix,
    se3,
    se3_inverse,
)

PASS_RESIDUAL_MM = 10.0


# ── capture ──────────────────────────────────────────────────────────────
class CaptureNode(Node):

    def __init__(self, args):
        super().__init__('extrinsic_capture')
        self._args = args
        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._optical_frame = None
        self._samples = []  # list of 4x4 camera_optical -> board

        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, args.dictionary))
        board = cv2.aruco.CharucoBoard(
            (args.squares_x, args.squares_y),
            args.square_len, args.marker_len, dictionary)
        self._board = board
        self._detector = cv2.aruco.CharucoDetector(board)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(CameraInfo, args.info_topic, self._on_info, 10)
        self.create_subscription(Image, args.image_topic, self._on_image, 10)
        self.get_logger().info(
            f'Capturing {args.samples} ChArUco detections from '
            f'{args.image_topic} — keep robot and board perfectly still.')

    def _on_info(self, msg):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64)
            self._optical_frame = msg.header.frame_id

    def _on_image(self, msg):
        if self._camera_matrix is None or self.done():
            return
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        corners, ids, _, _ = self._detector.detectBoard(image)
        if ids is None or len(ids) < self._args.min_corners:
            self.get_logger().warn(
                'Board not (fully) visible — '
                f'{0 if ids is None else len(ids)} corners', throttle_duration_sec=2.0)
            return
        obj_pts, img_pts = self._board.matchImagePoints(corners, ids)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self._camera_matrix, self._dist_coeffs)
        if not ok:
            return
        rot, _ = cv2.Rodrigues(rvec)
        self._samples.append(se3(rot, tvec.reshape(3)))
        self.get_logger().info(
            f'sample {len(self._samples)}/{self._args.samples}')

    def done(self):
        return len(self._samples) >= self._args.samples

    def optical_offset(self):
        """camera_link -> color optical frame, from the live driver's TF."""
        tf = self._tf_buffer.lookup_transform(
            'camera_link', self._optical_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=5.0))
        t = tf.transform.translation
        q = tf.transform.rotation
        return se3(quat_to_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])

    def write_session(self, path):
        cam_t_optical = self.optical_offset()
        data = {
            'optical_frame': self._optical_frame,
            'camera_link_to_optical': cam_t_optical.tolist(),
            'camera_optical_to_board_samples': [s.tolist() for s in self._samples],
            'board': {
                'squares_x': self._args.squares_x,
                'squares_y': self._args.squares_y,
                'square_len': self._args.square_len,
                'marker_len': self._args.marker_len,
                'dictionary': self._args.dictionary,
            },
        }
        with open(path, 'w') as f:
            yaml.safe_dump(data, f)
        self.get_logger().info(f'Session written: {path}')


def run_capture(args):
    rclpy.init()
    node = CaptureNode(args)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() > deadline:
                node.get_logger().error(
                    f'Timed out with {len(node._samples)}/{args.samples} '
                    'samples. Is the camera up and the board in view?')
                return 1
        node.write_session(args.session)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── solve ────────────────────────────────────────────────────────────────
def run_solve(args):
    with open(args.session) as f:
        session = yaml.safe_load(f)

    samples = [np.array(s) for s in session['camera_optical_to_board_samples']]
    if len(samples) < 5:
        print(f'ERROR: only {len(samples)} samples in session; need >= 5.')
        return 1

    optical_t_board = average_se3(samples)
    spread_mm = 1000.0 * max(
        np.linalg.norm(s[:3, 3] - optical_t_board[:3, 3]) for s in samples)
    print(f'{len(samples)} samples, translation spread {spread_mm:.1f} mm')
    if spread_mm > 5.0:
        print('WARNING: spread > 5 mm — vibration or lighting flicker during '
              'capture. Consider recapturing.')

    bx, by, bz, br, bp, byaw = args.board_pose
    base_t_board = se3(rpy_deg_to_matrix(br, bp, byaw), [bx, by, bz])
    camlink_t_optical = np.array(session['camera_link_to_optical'])

    base_t_optical = base_t_board @ se3_inverse(optical_t_board)
    base_t_camlink = base_t_optical @ se3_inverse(camlink_t_optical)

    trans = base_t_camlink[:3, 3]
    roll, pitch, yaw = matrix_to_rpy_deg(base_t_camlink[:3, :3])
    print(f'base_link -> camera_link: t=({trans[0]:.4f}, {trans[1]:.4f}, '
          f'{trans[2]:.4f}) m  rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f}) deg')

    # Preserve the gripper transform already stored in the output file.
    try:
        with open(args.output) as f:
            existing = yaml.safe_load(f) or {}
    except FileNotFoundError:
        existing = {}
    gripper = existing.get('base_link_to_gripper_link', {
        'parent': 'base_link', 'child': 'gripper_link',
        'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'rotation_rpy_deg': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    })

    out = {
        'calibrated': True,
        'base_link_to_camera_link': {
            'parent': 'base_link',
            'child': 'camera_link',
            'translation': {
                'x': float(trans[0]), 'y': float(trans[1]), 'z': float(trans[2])},
            'rotation_rpy_deg': {
                'roll': float(roll), 'pitch': float(pitch), 'yaw': float(yaw)},
        },
        'base_link_to_gripper_link': gripper,
        # Kept so `verify` can chain base -> optical without a live driver.
        'camera_link_to_optical': camlink_t_optical.tolist(),
        'optical_frame': session['optical_frame'],
    }
    with open(args.output, 'w') as f:
        f.write('# Written by r2_calibration extrinsic_calibrator (solve).\n'
                '# Do not hand-edit base_link_to_camera_link.\n')
        yaml.safe_dump(out, f)
    print(f'Extrinsics written: {args.output}')
    print('Rebuild/relaunch r2_bringup, then run verify at 0.35, 0.5, 0.8 m.')
    return 0


# ── verify ───────────────────────────────────────────────────────────────
class VerifyNode(Node):

    def __init__(self, args, base_t_optical):
        super().__init__('extrinsic_verify')
        self._known = np.array(args.cube_pos)
        self._base_t_optical = base_t_optical
        self._residuals_mm = []
        self._target = args.samples
        self.create_subscription(
            PointStamped, args.centroid_topic, self._on_centroid, 10)
        self.get_logger().info(
            f'Verifying against cube at base_link ({self._known}) using '
            f'{args.centroid_topic}. Trigger measurements via the locate_kfs '
            'action or the localizer test path.')

    def _on_centroid(self, msg):
        if self.done():
            return
        p_opt = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0])
        p_base = self._base_t_optical @ p_opt
        residual = np.linalg.norm(p_base[:3] - self._known) * 1000.0
        self._residuals_mm.append(residual)
        self.get_logger().info(
            f'sample {len(self._residuals_mm)}/{self._target}: '
            f'residual {residual:.1f} mm')

    def done(self):
        return len(self._residuals_mm) >= self._target

    def report(self):
        res = np.array(self._residuals_mm)
        print(f'\nresiduals over {len(res)} samples: '
              f'mean {res.mean():.1f} mm, max {res.max():.1f} mm')
        if res.max() <= PASS_RESIDUAL_MM:
            print(f'PASS (max <= {PASS_RESIDUAL_MM} mm). '
                  'Repeat at the other distances (0.35 / 0.5 / 0.8 m).')
            return 0
        print(f'FAIL (max > {PASS_RESIDUAL_MM} mm) — recalibrate.')
        return 1


def run_verify(args):
    with open(args.extrinsics) as f:
        ext = yaml.safe_load(f)
    if not ext.get('calibrated', False):
        print('ERROR: extrinsics file is not calibrated (placeholder values).')
        return 1
    if 'camera_link_to_optical' not in ext:
        print('ERROR: extrinsics file predates solve-mode output; re-run solve.')
        return 1

    cam = ext['base_link_to_camera_link']
    t = cam['translation']
    r = cam['rotation_rpy_deg']
    base_t_camlink = se3(
        rpy_deg_to_matrix(r['roll'], r['pitch'], r['yaw']),
        [t['x'], t['y'], t['z']])
    base_t_optical = base_t_camlink @ np.array(ext['camera_link_to_optical'])

    rclpy.init()
    node = VerifyNode(args, base_t_optical)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() > deadline:
                node.get_logger().error(
                    f'Timed out with {len(node._residuals_mm)}/{args.samples} '
                    'centroids. Is the localizer publishing?')
                return 1
        return node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── CLI ──────────────────────────────────────────────────────────────────
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)

    cap = sub.add_parser('capture', help='capture ChArUco detections')
    cap.add_argument('--session', required=True)
    cap.add_argument('--samples', type=int, default=30)
    cap.add_argument('--timeout', type=float, default=120.0)
    cap.add_argument('--image-topic', default='/camera/color/image_raw')
    cap.add_argument('--info-topic', default='/camera/color/camera_info')
    cap.add_argument('--squares-x', type=int, default=7)
    cap.add_argument('--squares-y', type=int, default=5)
    cap.add_argument('--square-len', type=float, default=0.04,
                     help='chessboard square edge, metres')
    cap.add_argument('--marker-len', type=float, default=0.03,
                     help='ArUco marker edge, metres')
    cap.add_argument('--dictionary', default='DICT_5X5_100')
    cap.add_argument('--min-corners', type=int, default=12)

    sol = sub.add_parser('solve', help='solve and write extrinsics yaml')
    sol.add_argument('--session', required=True)
    sol.add_argument('--board-pose', type=float, nargs=6, required=True,
                     metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                     help='board origin in base_link: metres + degrees')
    sol.add_argument('--output', required=True)

    ver = sub.add_parser('verify', help='residual check with a real cube')
    ver.add_argument('--extrinsics', required=True)
    ver.add_argument('--cube-pos', type=float, nargs=3, required=True,
                     metavar=('X', 'Y', 'Z'),
                     help='known cube centroid in base_link, metres')
    ver.add_argument('--samples', type=int, default=10)
    ver.add_argument('--timeout', type=float, default=120.0)
    ver.add_argument('--centroid-topic', default='/r2_perception/kfs_centroid')

    args = parser.parse_args(argv)
    if args.mode == 'capture':
        return run_capture(args)
    if args.mode == 'solve':
        return run_solve(args)
    return run_verify(args)


if __name__ == '__main__':
    sys.exit(main())
