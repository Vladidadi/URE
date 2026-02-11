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

    
    from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bt_xml_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'behavior_trees',
        'follow_point.xml'
    )

    return LaunchDescription([
        # Centroid follower node publishes /follow_point
        Node(
            package='follower',
            executable='centroid_follower',
            name='centroid_follower',
            output='screen'
        ),

        # Nav2 BT Navigator running your follow_point behavior tree
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='follow_point',
            output='screen',
            parameters=[{
                'bt_xml_filename': bt_xml_file
            }]
        ),

        # Blackboard Translator: takes /follow_point and writes it to BT key 'goal'
    
    ])

