from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follower',
            executable='centroid_follower',
            name='centroid_follower',
            parameters=[{
                'image_width': 640,
                'lookahead_distance': 1.0,
                'angle_scale': 0.5
            }],
            output='screen'
        ),
        #    Node(
        # package='nav2_bt_navigator',
        # executable='bt_blackboard_translator',
        # name='follow_point_translator',
        # output='screen',
        # parameters=[{
        #     'topic': '/follow_point',
        #     'blackboard_key': 'goal',
        #     'msg_type': 'geometry_msgs/PoseStamped'
        # }]
        # )
    ])
