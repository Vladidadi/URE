from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bt_xml_file = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'follow_point.xml'
    )

    return LaunchDescription([
       

	 Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='follow_point',
            output='screen',
            parameters=[{
                'bt_xml_filename': "~/ros2_ws/src/follow/config/follow_point_bt.xml"
            }]
        ),
    ])
