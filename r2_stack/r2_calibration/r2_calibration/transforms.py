"""Minimal SE(3) helpers for the extrinsic calibrator (numpy only)."""

import math

import numpy as np


def rpy_deg_to_matrix(roll, pitch, yaw):
    """XYZ-fixed-axis (ROS convention) RPY in degrees -> 3x3 rotation."""
    r, p, y = (math.radians(a) for a in (roll, pitch, yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy_deg(rot):
    """3x3 rotation -> (roll, pitch, yaw) degrees, ROS fixed-axis."""
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        yaw = 0.0
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def se3(rot, trans):
    """3x3 rotation + 3-vector -> 4x4 homogeneous transform."""
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = np.asarray(trans).reshape(3)
    return mat


def se3_inverse(mat):
    rot = mat[:3, :3]
    inv = np.eye(4)
    inv[:3, :3] = rot.T
    inv[:3, 3] = -rot.T @ mat[:3, 3]
    return inv


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def average_se3(mats):
    """Average a list of nearby 4x4 transforms.

    Translations: arithmetic mean. Rotations: chordal mean via SVD
    projection of the averaged matrix back onto SO(3) — accurate for the
    tightly clustered samples produced by a static calibration capture.
    """
    trans = np.mean([m[:3, 3] for m in mats], axis=0)
    rot_sum = np.sum([m[:3, :3] for m in mats], axis=0)
    u, _, vt = np.linalg.svd(rot_sum)
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        u[:, -1] *= -1
        rot = u @ vt
    return se3(rot, trans)
