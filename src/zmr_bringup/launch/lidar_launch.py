"""
RPLidar A3M1 — LiDAR Only Launch File
Launches the sllidar_ros2 driver node to publish /scan topic.

Usage:
  ros2 launch zmr_bringup lidar_launch.py
  ros2 launch zmr_bringup lidar_launch.py serial_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declare launch arguments ──
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the RPLidar A3M1 (use /dev/rplidar after udev rule installed)'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='256000',
        description='Baud rate for A3M1 (256000 standard)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='TF frame ID for laser scans'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='Scan mode: Sensitivity (default for A3), Standard, Express, Boost'
    )

    # ── sllidar Node (latest Slamtec driver) ──
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        scan_mode_arg,
        sllidar_node,
    ])
