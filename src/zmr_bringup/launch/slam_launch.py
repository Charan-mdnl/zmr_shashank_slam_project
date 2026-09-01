"""
ZMR SLAM Mapping Launch File
Launches the full SLAM pipeline:
  1. sllidar_ros2 driver (A3M1)
  2. Static TF: base_link → laser
  3. rf2o_laser_odometry (computes odom → base_link from laser scans)
  4. SLAM Toolbox (online async)
  5. RViz2 with pre-configured display

Usage:
  ros2 launch zmr_bringup slam_launch.py
  ros2 launch zmr_bringup slam_launch.py serial_port:=/dev/ttyUSB1

To save the map once you're done mapping:
  ros2 run nav2_map_server map_saver_cli -f ~/ZMR_project/src/zmr_bringup/maps/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('zmr_bringup')

    # ── Paths ──
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_slam.rviz')

    # ── Declare launch arguments ──
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the RPLidar A3M1 (use /dev/rplidar after udev rule installed)'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='256000',
        description='Baud rate for A3M1'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='RPLidar scan mode'
    )

    # ── 1. sllidar_ros2 Node (latest Slamtec driver) ──
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
    )

    # ── 2. Static Transform: base_link → laser ──
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.1',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    # ── 3. rf2o Laser Odometry ──
    # Computes odom → base_link from consecutive laser scans.
    # This replaces the static odom transform and gives SLAM Toolbox
    # real movement data so the map updates as you move.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    # ── 4. SLAM Toolbox — Online Async Mode ──
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
        ],
    )

    # ── 5. RViz2 ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        scan_mode_arg,
        sllidar_node,
        static_tf_node,
        rf2o_node,
        slam_toolbox_node,
        rviz_node,
    ])
