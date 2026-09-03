"""Headless ZMR simulator + SLAM Toolbox (online async mapping).

Brings up the simulator, the robot description and slam_toolbox. Nothing here
opens a window; inspect the result with `ros2 run zmr_tools render_map`.

    ros2 launch zmr_sim sim_slam.launch.py
    ros2 launch zmr_sim sim_slam.launch.py world:=corridor
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
    bringup_share = get_package_share_directory('zmr_bringup')
    slam_params = os.path.join(bringup_share, 'config', 'slam_toolbox_sim.yaml')

    args = [
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('x', default_value='2.5'),
        DeclareLaunchArgument('y', default_value='2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': 'true',
        }.items(),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': True}],
    )


    # slam_toolbox is a lifecycle node: without this it stays 'unconfigured',
    # silently publishing nothing. nav2_lifecycle_manager normally does this.
    lifecycle_manager = Node(
        package='zmr_tools',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'node_names': ['slam_toolbox'],
            'autostart': True,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription(args + [sim, slam])
