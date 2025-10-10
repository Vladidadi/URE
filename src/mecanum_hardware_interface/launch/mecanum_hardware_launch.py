from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_hardware_interface')
    config_file = os.path.join(pkg_share, 'config', 'mecanum_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM1')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM1',
            description='Serial port for Arduino connection'
        ),
        
        Node(
            package='mecanum_hardware_interface',
            executable='mecanum_hardware_interface',
            name='mecanum_hardware_interface',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time},
                {'serial_port': serial_port}
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
            ]
        ),
    ])
