from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encoder_pkg',
            executable='encoder_node',
            name='wheel_encoder_odometry',
            output='screen',
            parameters=[
                {'wheel_radius': 0.05},
                {'wheel_separation': 0.24},
                {'ticks_per_revolution': 20},
                {'left_encoder_pin': 20},
                {'right_encoder_pin': 21}
            ]
        ),
    ])
