"""
ZMR Gazebo SLAM Simulation — Complete Pipeline Launch (v2)
==========================================================
Uses simulation-only URDF (primitives, no STL meshes) for reliability.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    zmr_gazebo_dir = get_package_share_directory('zmr_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # File paths — simulation URDF lives inside zmr_gazebo package
    urdf_path = os.path.join(zmr_gazebo_dir, 'urdf', 'zmr_robot_sim.urdf')
    world_path = os.path.join(zmr_gazebo_dir, 'worlds', 'zmr_test_world.world')
    slam_params_path = os.path.join(zmr_gazebo_dir, 'config', 'slam_toolbox_sim.yaml')
    rviz_config_path = os.path.join(zmr_gazebo_dir, 'config', 'rviz_slam_sim.rviz')

    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 1. Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # 2. Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # 3. Spawn robot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'zmr_robot',
            '-topic', '/robot_description',
            '-x', '2.0', '-y', '2.0', '-z', '0.0',
        ],
    )

    # 4. SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_path, {'use_sim_time': True}],
    )

    # 5. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_pub,
        spawn_robot,
        slam_toolbox,
        rviz,
    ])
