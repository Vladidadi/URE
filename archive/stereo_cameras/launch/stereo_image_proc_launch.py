from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_cameras',
            executable='stereo_image_proc',
            name='stereo_image_proc',
            remappings=[
                ('left/image_raw', '/stereo/left/image_raw'),
                ('right/image_raw', '/stereo/right/image_raw'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info')
            ],
            output='screen'
        )
    ])
