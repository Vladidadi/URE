#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_pwm_pkg',
            executable='dual_motor_node',
            name='dual_motor_node',
            output='screen',
            parameters=[{
                # Add any parameters your motor node needs
                # 'max_speed': 1.0,
                # 'wheel_separation': 0.24,
                # etc.
            }],
            remappings=[
                # If you need to remap topics
                # ('/cmd_vel', '/robot/cmd_vel'),
            ]
        ),
    ])
