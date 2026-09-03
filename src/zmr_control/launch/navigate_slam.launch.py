"""Autonomous point-to-point navigation over SLAM. Fully headless.

Pipeline:
    waypoint_navigator -> /cmd_vel_auto -> obstacle_avoidance -> /cmd_vel -> robot
    slam_toolbox       -> map -> odom (pose correction the navigator steers by)

    ros2 launch zmr_control navigate_slam.launch.py
    ros2 launch zmr_control navigate_slam.launch.py \
        waypoints:="[9.0, 2.5, 0.0, 15.0, 9.0, 1.57, 21.0, 16.0, 0.0]"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('zmr_sim')
    ctrl_share = get_package_share_directory('zmr_control')

    args = [
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('x', default_value='2.5'),
        DeclareLaunchArgument('y', default_value='2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=os.path.join(ctrl_share, 'config', 'waypoints.yaml')),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('record_path', default_value='/tmp/zmr_path.csv'),
    ]

    sim_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim_slam.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    avoidance = Node(
        package='zmr_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    navigator = Node(
        package='zmr_control',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'loop': LaunchConfiguration('loop'),
        }],
    )

    recorder = Node(
        package='zmr_tools',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output': LaunchConfiguration('record_path'),
        }],
    )

    return LaunchDescription(args + [sim_slam, avoidance, navigator, recorder])
