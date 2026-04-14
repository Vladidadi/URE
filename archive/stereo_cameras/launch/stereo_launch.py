from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your USB stereo camera node
        Node(
            package='stereo_cameras',
            executable='camera_nodes',
            name='camera_nodes',
            output='screen',
        ),

        # stereo_image_proc node: rectifies images
        # Node(
        #     package='stereo_image_proc',
        #     executable='stereo_image_proc_node',
        #     name='stereo_image_proc',
        #     remappings=[
        #         ('left/image_raw', 'left/image_raw'),
        #         ('right/image_raw', 'right/image_raw'),
        #         ('left/camera_info', 'left/camera_info'),
        #         ('right/camera_info', 'right/camera_info'),
        #     ],
        #     output='screen',
        # ),

        # disparity_node: computes disparity from rectified images
        # Node(
        #     package='stereo_image_proc',
        #     executable='disparity_node',
        #     name='disparity_node',
        #     remappings=[
        #         ('left/image_rect', 'left/image_rect'),
        #         ('right/image_rect', 'right/image_rect'),
        #         ('left/camera_info', 'left/camera_info'),
        #         ('right/camera_info', 'right/camera_info'),
        #     ],
        #     output='screen',
        # ),
    ])
