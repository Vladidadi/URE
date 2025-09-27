from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hc_sr04_pkg',
            executable='hc_sr04_node',
            name='ultrasonic_sensor',
            output='screen',
            parameters=[
                {'trigger_pin': 9},  # GPIO pin numbers
                {'echo_pin': 10},
                {'frame_id': 'ultrasonic_link'}
            ]
        ),
    ])
