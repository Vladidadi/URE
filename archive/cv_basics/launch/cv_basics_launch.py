from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cv_basics')
#    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_current2.urdf')
     
    return LaunchDescription([
        Node(
            package='cv_basics',
            executable='img_subscriber',
            name='img_subscriber',
            output='screen',
            parameters=[]
        ),
    ])


