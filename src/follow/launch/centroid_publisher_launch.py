from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow',
            executable='centroid_publisher',
            name='centroid_publisher',
            output='screen'
        )
    ])
