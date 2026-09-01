"""
View a previously saved map in RViz2.

Usage:
  ros2 launch zmr_bringup view_map_launch.py map:=my_map
  ros2 launch zmr_bringup view_map_launch.py map:=/full/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('zmr_bringup')
    default_map_dir = os.path.join(pkg_share, 'maps')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_slam.rviz')

    # ── Launch Arguments ──
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(default_map_dir, 'my_map.yaml'),
        description='Full path to the map YAML file, or just the map name (without extension)'
    )

    # ── Map Server ──
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False,
        }],
    )

    # ── Lifecycle Manager to activate the map server ──
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ── RViz2 ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        map_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz_node,
    ])
