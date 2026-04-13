#!/usr/bin/env python3
"""Real robot navigation: hardware + sensors + EKF + AMCL + Nav2."""
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
    map_yaml = LaunchConfiguration('map_yaml')
    nav_delay = LaunchConfiguration('navigation_startup_delay')

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
            DeclareLaunchArgument(
                'map_yaml',
                default_value=os.path.join(pkg, 'maps', 'my_lab.yaml'),
            ),
            DeclareLaunchArgument('navigation_startup_delay', default_value='2.0'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(full),
                launch_arguments=[
                    ('use_sim_time', 'false'),
                    ('arduino_serial_port', arduino_port),
                    ('lidar_serial_port', lidar_port),
                    ('lidar_scan_mode', lidar_scan_mode),
                    ('map_yaml', map_yaml),
                    ('navigation_startup_delay', nav_delay),
                    ('enable_sim_kinematic', 'false'),
                    ('enable_mecanum_hw', 'true'),
                    ('enable_lidar', 'true'),
                    ('enable_imu', use_imu_ekf),
                    ('enable_ultrasonic', 'false'),
                    ('enable_ekf', use_imu_ekf),
                    ('enable_slam', 'false'),
                    ('enable_localization', 'true'),
                    ('enable_navigation', 'true'),
                    ('enable_follow', 'false'),
                    ('enable_centroid_publisher', 'false'),
                    ('enable_teleop', 'false'),
                ],
            ),
        ]
    )
