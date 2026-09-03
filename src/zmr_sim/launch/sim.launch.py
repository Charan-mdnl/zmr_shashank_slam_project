"""Headless ZMR simulator + robot_state_publisher.

No Gazebo, no RViz, no display of any kind.

    ros2 launch zmr_sim sim.launch.py
    ros2 launch zmr_sim sim.launch.py world:=corridor x:=1.0 y:=1.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('zmr_description'), 'urdf', 'zmr_robot.urdf')
    with open(urdf, 'r') as fh:
        robot_description = fh.read()

    args = [
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='warehouse | empty | corridor'),
        DeclareLaunchArgument('world_yaml', default_value='',
                              description='load a ROS map pair as the world instead'),
        DeclareLaunchArgument('x', default_value='2.5'),
        DeclareLaunchArgument('y', default_value='2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    sim = Node(
        package='zmr_sim',
        executable='sim_node',
        name='zmr_sim',
        output='screen',
        parameters=[{
            'world': LaunchConfiguration('world'),
            'world_yaml': LaunchConfiguration('world_yaml'),
            'initial_x': LaunchConfiguration('x'),
            'initial_y': LaunchConfiguration('y'),
            'initial_yaw': LaunchConfiguration('yaw'),
            'publish_clock': use_sim_time,
        }],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription(args + [sim, rsp])
