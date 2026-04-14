#!/usr/bin/env python3
"""
Single entry point for the mecanum stack (replaces ad-hoc shell aliases).

Components map roughly to:
  lb  -> robot_bringup mecanum_bringup (robot_state_publisher)
  lm* -> mecanum_hardware_interface
  li  -> ros2_mpu6050
  ll* -> sllidar_ros2
  lu  -> hc_sr04_pkg (optional)
  EKF -> config/ekf/stack_ekf.yaml
  lst -> robot_bringup slam_launch
  acml + lnav -> nav2_bringup localization + navigation with config/nav2/*.yaml
  lf / lfp -> follow package nodes
  lt  -> teleop_twist_keyboard (optional)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_stack_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    robot_bringup_share = get_package_share_directory('robot_bringup')

    ekf_yaml_default = os.path.join(pkg_share, 'config', 'ekf', 'stack_ekf.yaml')
    sim_yaml_default = os.path.join(pkg_share, 'config', 'sim', 'kinematic_sim.yaml')
    nav2_nav_yaml = os.path.join(pkg_share, 'config', 'nav2', 'navigation.yaml')
    nav2_loc_yaml = os.path.join(pkg_share, 'config', 'nav2', 'localization.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'my_lab.yaml')
    slam_params = os.path.join(robot_bringup_share, 'config', 'slam_toolbox_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arduino_port = LaunchConfiguration('arduino_serial_port')
    lidar_port = LaunchConfiguration('lidar_serial_port')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')

    enable_mecanum_hw = LaunchConfiguration('enable_mecanum_hw')
    enable_sim_kinematic = LaunchConfiguration('enable_sim_kinematic')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    sim_params_file = LaunchConfiguration('sim_params_file')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_ultrasonic = LaunchConfiguration('enable_ultrasonic')
    enable_ekf = LaunchConfiguration('enable_ekf')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_follow = LaunchConfiguration('enable_follow')
    enable_centroid_publisher = LaunchConfiguration('enable_centroid_publisher')
    enable_teleop = LaunchConfiguration('enable_teleop')
    navigation_startup_delay = LaunchConfiguration('navigation_startup_delay')

    map_yaml = LaunchConfiguration('map_yaml')

    localization_ready = PythonExpression(
        [
            '"',
            enable_localization,
            '" == "true" and "',
            enable_slam,
            '" != "true"',
        ]
    )

    def mecanum_hw_include(context, *args, **kwargs):
        hw = LaunchConfiguration('enable_mecanum_hw').perform(context)
        sim_k = LaunchConfiguration('enable_sim_kinematic').perform(context)
        if hw != 'true' or sim_k == 'true':
            return []
        ekf_on = LaunchConfiguration('enable_ekf').perform(context) == 'true'
        publish_tf_str = 'false' if ekf_on else 'true'
        pkg_mec = get_package_share_directory('mecanum_hardware_interface')
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_mec, 'launch', 'mecanum_hardware_launch.py')
                ),
                launch_arguments=[
                    ('use_sim_time', LaunchConfiguration('use_sim_time')),
                    ('serial_port', LaunchConfiguration('arduino_serial_port')),
                    ('publish_tf', TextSubstitution(text=publish_tf_str)),
                ],
            )
        ]

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Must match Gazebo / bag clock if using simulation.',
            ),
            DeclareLaunchArgument(
                'arduino_serial_port',
                default_value='/dev/ttyACM0',
                description='Serial device for mecanum_hardware_interface (Arduino).',
            ),
            DeclareLaunchArgument(
                'lidar_serial_port',
                default_value='/dev/ttyUSB0',
                description='Serial device for RPLidar A1.',
            ),
            DeclareLaunchArgument(
                'lidar_scan_mode',
                default_value='Standard',
                description='RPLidar scan mode (e.g. Standard, Sensitivity).',
            ),
            DeclareLaunchArgument(
                'map_yaml',
                default_value=default_map,
                description='Full path to occupancy map YAML for AMCL / map_server.',
            ),
            DeclareLaunchArgument(
                'enable_mecanum_hw',
                default_value='true',
                description='Start mecanum_hardware_interface (set false when using sim).',
            ),
            DeclareLaunchArgument(
                'enable_sim_kinematic',
                default_value='false',
                description='Start built-in kinematic sim (cmd_vel -> /odom + /joint_states).',
            ),
            DeclareLaunchArgument(
                'ekf_params_file',
                default_value=ekf_yaml_default,
                description='EKF YAML (use stack_ekf_odom_only.yaml with kinematic sim, no IMU).',
            ),
            DeclareLaunchArgument(
                'sim_params_file',
                default_value=sim_yaml_default,
                description='Parameters for mecanum_kinematic_sim.',
            ),
            DeclareLaunchArgument(
                'enable_lidar',
                default_value='true',
                description='Start sllidar_ros2.',
            ),
            DeclareLaunchArgument(
                'enable_imu',
                default_value='true',
                description='Start ros2_mpu6050.',
            ),
            DeclareLaunchArgument(
                'enable_ultrasonic',
                default_value='false',
                description='Start hc_sr04_pkg.',
            ),
            DeclareLaunchArgument(
                'enable_ekf',
                default_value='true',
                description='Start ekf_node with config/ekf/stack_ekf.yaml.',
            ),
            DeclareLaunchArgument(
                'enable_slam',
                default_value='false',
                description='Start slam_toolbox (mapping). Do not enable with enable_localization.',
            ),
            DeclareLaunchArgument(
                'enable_localization',
                default_value='true',
                description='Start map_server + AMCL via nav2 localization_launch.',
            ),
            DeclareLaunchArgument(
                'enable_navigation',
                default_value='true',
                description='Start Nav2 navigation stack (controller, planner, BT, ...).',
            ),
            DeclareLaunchArgument(
                'enable_follow',
                default_value='false',
                description='Start follow/centroid_follower node.',
            ),
            DeclareLaunchArgument(
                'enable_centroid_publisher',
                default_value='false',
                description='Start follow/centroid_publisher via its launch file.',
            ),
            DeclareLaunchArgument(
                'enable_teleop',
                default_value='false',
                description='Start teleop_twist_keyboard.',
            ),
            DeclareLaunchArgument(
                'navigation_startup_delay',
                default_value='2.0',
                description='Seconds before starting Nav2 so /odom + EKF can publish odom->base_link.',
            ),
            # --- robot_state_publisher (URDF) ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_bringup_share, 'launch', 'mecanum_bringup.launch.py')
                )
            ),
            # --- mecanum serial driver (publish_tf false when EKF provides odom->base_link) ---
            OpaqueFunction(function=mecanum_hw_include),
            # --- kinematic simulator ---
            GroupAction(
                condition=IfCondition(enable_sim_kinematic),
                actions=[
                    Node(
                        package='mecanum_stack_bringup',
                        executable='mecanum_kinematic_sim',
                        name='mecanum_kinematic_sim',
                        output='screen',
                        parameters=[
                            sim_params_file,
                            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                        ],
                    )
                ],
            ),
            # --- LiDAR ---
            # Delay startup slightly so USB/serial settle before first scan command.
            TimerAction(
                period=2.5,
                actions=[
                    GroupAction(
                        condition=IfCondition(enable_lidar),
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(
                                        get_package_share_directory('sllidar_ros2'),
                                        'launch',
                                        'sllidar_a1_launch.py',
                                    )
                                ),
                                launch_arguments={
                                    'serial_port': lidar_port,
                                    'scan_mode': lidar_scan_mode,
                                }.items(),
                            )
                        ],
                    )
                ],
            ),
            # --- IMU ---
            GroupAction(
                condition=IfCondition(enable_imu),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('ros2_mpu6050'),
                                'launch',
                                'ros2_mpu6050.launch.py',
                            )
                        )
                    )
                ],
            ),
            # --- Ultrasonic ---
            GroupAction(
                condition=IfCondition(enable_ultrasonic),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('hc_sr04_pkg'),
                                'launch',
                                'hc_sr04_launch.py',
                            )
                        )
                    )
                ],
            ),
            # --- EKF ---
            GroupAction(
                condition=IfCondition(enable_ekf),
                actions=[
                    Node(
                        package='robot_localization',
                        executable='ekf_node',
                        name='ekf_filter_node',
                        output='screen',
                        parameters=[
                            ekf_params_file,
                            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                        ],
                    )
                ],
            ),
            # --- SLAM Toolbox ---
            GroupAction(
                condition=IfCondition(enable_slam),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(robot_bringup_share, 'launch', 'slam_launch.py')
                        ),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'slam_params_file': slam_params,
                        }.items(),
                    )
                ],
            ),
            # --- Map server + AMCL ---
            GroupAction(
                condition=IfCondition(localization_ready),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_share, 'launch', 'localization_launch.py')
                        ),
                        launch_arguments={
                            'map': map_yaml,
                            'use_sim_time': use_sim_time,
                            'params_file': nav2_loc_yaml,
                        }.items(),
                    )
                ],
            ),
            # --- Nav2 navigation (delayed so EKF + /odom establish odom frame before lifecycle) ---
            TimerAction(
                period=navigation_startup_delay,
                actions=[
                    GroupAction(
                        condition=IfCondition(enable_navigation),
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(nav2_share, 'launch', 'navigation_launch.py')
                                ),
                                launch_arguments={
                                    'use_sim_time': use_sim_time,
                                    'params_file': nav2_nav_yaml,
                                }.items(),
                            )
                        ],
                    )
                ],
            ),
            # --- Follow stack ---
            GroupAction(
                condition=IfCondition(enable_follow),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('follow'),
                                'launch',
                                'follower_launch.py',
                            )
                        )
                    )
                ],
            ),
            GroupAction(
                condition=IfCondition(enable_centroid_publisher),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('follow'),
                                'launch',
                                'centroid_publisher_launch.py',
                            )
                        )
                    )
                ],
            ),
            # --- Teleop ---
            # GroupAction(
            #     condition=IfCondition(enable_teleop),
            #     actions=[
            #         Node(
            #             package='teleop_twist_keyboard',
            #             executable='teleop_twist_keyboard',
            #             name='teleop_twist_keyboard',
            #             output='screen',
            #             parameters=[
            #                 {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
            #             ],
            #         )
            #     ],
            # ),
        ]
    )

    return ld
