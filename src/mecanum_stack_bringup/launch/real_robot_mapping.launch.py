#!/usr/bin/env python3
"""Real robot mapping: hardware + sensors + EKF + SLAM. No AMCL, no Nav2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('mecanum_stack_bringup')
    full = os.path.join(pkg, 'launch', 'mecanum_full_stack.launch.py')

    arduino_port = LaunchConfiguration('arduino_serial_port')
    lidar_port = LaunchConfiguration('lidar_serial_port')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')
    use_imu_ekf = LaunchConfiguration('use_imu_ekf')

    return LaunchDescription(
        [
            DeclareLaunchArgument('arduino_serial_port', default_value='/dev/ttyACM0'),
            DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('lidar_scan_mode', default_value='Standard'),
            DeclareLaunchArgument(
                'use_imu_ekf',
                default_value='true',
                description='Toggle IMU + EKF together (true/false).',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(full),
                launch_arguments=[
                    ('use_sim_time', 'false'),
                    ('arduino_serial_port', arduino_port),
                    ('lidar_serial_port', lidar_port),
                    ('lidar_scan_mode', lidar_scan_mode),
                    ('enable_sim_kinematic', 'false'),
                    ('enable_mecanum_hw', 'true'),
                    ('enable_lidar', 'true'),
                    ('enable_imu', 'false'),#because the imu is on a different device
                    ('enable_ultrasonic', 'false'),
                    ('enable_ekf', use_imu_ekf),
                    ('enable_slam', 'true'),
                    ('enable_localization', 'false'),
                    ('enable_navigation', 'false'),
                    ('enable_follow', 'false'),
                    ('enable_centroid_publisher', 'false'),
                    ('enable_teleop', 'false'),
                ],
            ),
        ]
    )
