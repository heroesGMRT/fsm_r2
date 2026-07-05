"""Bring up the R2 visual servo stack.

    ros2 launch r2_bringup servo_stack.launch.py                # no camera (dev)
    ros2 launch r2_bringup servo_stack.launch.py use_camera:=true

Starts:
  * static TF base_link->camera_link and base_link->gripper_link, values
    read from config/camera_extrinsics.yaml (written by r2_calibration)
  * kfs_localizer (r2_perception)
  * pick_servo (r2_servo)
  * realsense2_camera driver, only when use_camera:=true

No sleep() hacks: the localizer and servo idle gracefully until camera
topics appear, so start order does not matter (D435i takes 2-4 s on USB).
"""

import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf_node(entry, name):
    t = entry['translation']
    r = entry['rotation_rpy_deg']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(t['x']), '--y', str(t['y']), '--z', str(t['z']),
            '--roll', str(math.radians(r['roll'])),
            '--pitch', str(math.radians(r['pitch'])),
            '--yaw', str(math.radians(r['yaw'])),
            '--frame-id', entry['parent'],
            '--child-frame-id', entry['child'],
        ],
    )


def generate_launch_description():
    bringup_share = get_package_share_directory('r2_bringup')
    perception_share = get_package_share_directory('r2_perception')

    extrinsics_path = os.path.join(bringup_share, 'config', 'camera_extrinsics.yaml')
    with open(extrinsics_path, 'r') as f:
        extrinsics = yaml.safe_load(f)

    actions = [
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Start the RealSense D435i driver (robot only).'),
    ]

    if not extrinsics.get('calibrated', False):
        actions.append(LogInfo(msg=(
            '[r2_bringup] WARNING: camera_extrinsics.yaml holds PLACEHOLDER '
            'values (calibrated: false). Run r2_calibration before trusting '
            'any servo output.')))

    actions += [
        _static_tf_node(
            extrinsics['base_link_to_camera_link'], 'static_tf_camera'),
        _static_tf_node(
            extrinsics['base_link_to_gripper_link'], 'static_tf_gripper'),
        Node(
            package='r2_perception',
            executable='kfs_localizer_node',
            name='kfs_localizer',
            output='screen',
            parameters=[os.path.join(
                perception_share, 'config', 'kfs_localizer_params.yaml')],
        ),
        Node(
            package='r2_servo',
            executable='pick_servo_node',
            name='pick_servo',
            output='screen',
            parameters=[os.path.join(
                bringup_share, 'config', 'pick_servo_params.yaml')],
        ),
        # D435i driver — robot only. Emitter forced to max power against
        # venue IR washout; IMU united so /camera/imu exists for the
        # settle check; aligned depth is the only depth stream consumed.
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_camera')),
            parameters=[{
                'align_depth.enable': True,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 2,  # linear interpolation -> /camera/imu
                'depth_module.emitter_enabled': 1,
                'depth_module.laser_power': 360.0,
                'rgb_camera.profile': '640x480x30',
                'depth_module.profile': '640x480x30',
            }],
        ),
    ]

    return LaunchDescription(actions)
