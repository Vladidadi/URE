#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    robot_desc_pkg = get_package_share_directory('robot_description_pkg')
    robot_bringup_pkg = get_package_share_directory('robot_bringup')
    
    urdf_file = os.path.join(robot_desc_pkg, 'urdf', 'robot_current2.urdf')
    ekf_config = os.path.join(robot_bringup_pkg, 'config', 'ekf.yaml')
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str)
            }],
            output='screen'
        ),
        
        # Wheel Encoder Odometry
        Node(
            package='encoder_pkg',
            executable='encoder_node',
            name='encoder_node',
            output='screen'
        ),
        
        # Motor Driver
       # Node(
      #      package='motor_pwm_pkg',
     #       executable='dual_motor_l298n_node',
    #        name='dual_motor_l298n_node',
   #         output='screen'
  #      ),
        
        # IMU
        Node(
            package='ros2_mpu6050',
            executable='ros2_mpu6050',
            name='mpu6050_sensor',
            output='screen'
        ),
        
        # Ultrasonic Sensor
#        Node(
 #           package='hc_sr04_pkg',
  #          executable='hc_sr04_node',
   #         name='ultrasonic_sensor',
    #        output='screen'
     #   ),
        
        # LIDAR
      #  Node(
      #      package='sllidar_ros2',
      #      executable='sllidar_node',
      #      name='sllidar_node',
      #      output='screen'
      #  ),
        
        # EKF (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
    ])
