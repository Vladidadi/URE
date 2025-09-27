from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = '/home/pi4/ros2_ws/src/robot_navigation/config/nav2_params.yaml'

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            parameters=[params_file],
        )
    ])


