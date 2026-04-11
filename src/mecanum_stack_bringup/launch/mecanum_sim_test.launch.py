#!/usr/bin/env python3
"""Phase B sim: URDF + kinematic sim + optional odom-only EKF.

Run teleop in another terminal, e.g.:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('mecanum_stack_bringup')
    urdf_path = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'urdf',
        'mecanum_small.urdf',
    )
    with open(urdf_path, encoding='utf-8') as f:
        robot_description = f.read()

    sim_yaml = os.path.join(pkg, 'config', 'sim', 'kinematic_sim.yaml')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf', 'stack_ekf_odom_only.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_ekf = LaunchConfiguration('enable_ekf')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('enable_ekf', default_value='true'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            Node(
                package='mecanum_stack_bringup',
                executable='mecanum_kinematic_sim',
                name='mecanum_kinematic_sim',
                output='screen',
                parameters=[
                    sim_yaml,
                    {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            GroupAction(
                condition=IfCondition(enable_ekf),
                actions=[
                    Node(
                        package='robot_localization',
                        executable='ekf_node',
                        name='ekf_filter_node',
                        output='screen',
                        parameters=[
                            ekf_yaml,
                            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                        ],
                    )
                ],
            ),
        ]
    )
