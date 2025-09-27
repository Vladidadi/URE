#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    robot_description_dir = get_package_share_directory('robot_description')
    robot_hardware_dir = get_package_share_directory('robot_hardware')
    
    # Configuration files
    urdf_file = os.path.join(robot_description_dir, 'urdf', 'robot.urdf.xacro')
    robot_controllers_file = os.path.join(robot_hardware_dir, 'config', 'robot_hardware.yaml')
    rviz_config_file = os.path.join(robot_description_dir, 'config', 'view_robot.rviz')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz if true'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]),
                    value_type=str
                )
            }]
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_controllers_file],
            output='screen',
            remappings=[
                ('~/robot_description', '/robot_description'),
            ],
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Differential Drive Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen'
        ),

        # Sensor Nodes
        # LiDAR
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            ])
        ),

        # IMU
        Node(
            package='mpu_6050_driver',  # or your IMU package
            executable='mpu_6050_node',
            name='imu_sensor',
            output='screen'
        ),

        # Ultrasonic Sensor
        Node(
            package='hc_sr04_pkg',
            executable='hc_sr04_node',
            name='ultrasonic_sensor',
            output='screen'
        ),

        # Encoder/Odometry Node (your upcoming package)
        Node(
            package='encoder_pkg',  # Your encoder package
            executable='encoder_node',
            name='encoder_odometry',
            output='screen'
        ),

        # Transform publishers for sensors
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.1', '0', '0.12', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # RViz2 (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            condition=LaunchConfiguration('use_rviz'),
            output='screen'
        ),

        # Robot Localization (EKF) for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(robot_hardware_dir, 'config', 'ekf.yaml')],
            remappings=[('odometry/filtered', 'odom')]
        ),
    ])
