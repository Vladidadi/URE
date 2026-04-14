from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_hardware_interface')
    config_file = os.path.join(pkg_share, 'config', 'mecanum_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    publish_tf = LaunchConfiguration('publish_tf', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino connection'
        ),

        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish odom->base_link TF from this node. Use false when ekf_node publishes it.',
        ),
        
        Node(
            package='mecanum_hardware_interface',
            executable='mecanum_hardware_interface',
            name='mecanum_hardware_interface',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'serial_port': serial_port},
                {'publish_tf': ParameterValue(publish_tf, value_type=bool)},
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
            ]
        ),
    ])
