import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_bringup')
    default_ekf = os.path.join(pkg, 'config', 'ekf.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ekf_config',
                default_value=default_ekf,
                description='EKF YAML (use ekf_odom_only.yaml if IMU is not running).',
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[LaunchConfiguration('ekf_config')],
            ),
        ]
    )
