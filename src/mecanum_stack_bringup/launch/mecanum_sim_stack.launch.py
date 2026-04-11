#!/usr/bin/env python3
"""
Sim-friendly full stack: kinematic sim + odom-only EKF + fake scan (no serial / lidar / IMU).

Example:
  ros2 launch mecanum_stack_bringup mecanum_sim_stack.launch.py enable_navigation:=false
  ros2 launch mecanum_stack_bringup mecanum_sim_stack.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('mecanum_stack_bringup')
    full = os.path.join(pkg, 'launch', 'mecanum_full_stack.launch.py')

    ekf_odom = os.path.join(pkg, 'config', 'ekf', 'stack_ekf_odom_only.yaml')
    sim_yaml = os.path.join(pkg, 'config', 'sim', 'kinematic_sim_nav.yaml')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(full),
                launch_arguments=[
                    ('use_sim_time', 'false'),
                    ('enable_mecanum_hw', 'false'),
                    ('enable_sim_kinematic', 'true'),
                    ('enable_lidar', 'false'),
                    ('enable_imu', 'false'),
                    ('enable_ultrasonic', 'false'),
                    ('enable_ekf', 'true'),
                    ('ekf_params_file', ekf_odom),
                    ('sim_params_file', sim_yaml),
                    ('enable_slam', 'false'),
                    ('enable_localization', 'true'),
                    ('enable_navigation', 'true'),
                    ('enable_follow', 'false'),
                    ('enable_centroid_publisher', 'false'),
                    ('navigation_startup_delay', '1.0'),
                ],
            )
        ]
    )
