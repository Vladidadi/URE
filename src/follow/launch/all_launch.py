from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Centroid follower node
        Node(
            package='follower',
            executable='centroid_follower',
            name='centroid_follower',
            output='screen'
        ),

        # Bridge node to write /follow_point into BT blackboar
        

        # Nav2 BT navigator node (follow_point behavior tree)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='follow_point',
            output='screen',
            parameters=[{
                'bt_xml_filename': '/home/vlad/ros2_ws/src/follower/config/follow_point_bt.xml'
            }]
        ),
    ])
