"""Drive the ZMR by keyboard while SLAM builds a map. Fully headless.

Pipeline:  teleop_key -> /cmd_vel_teleop -> obstacle_avoidance -> /cmd_vel -> robot

Run the teleop node in its own terminal (it needs a TTY to read keys):

    ros2 launch zmr_control teleop_slam.launch.py     # sim + slam + avoidance
    ros2 run zmr_control teleop_key                   # separate terminal
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('zmr_sim')

    args = [
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('x', default_value='2.5'),
        DeclareLaunchArgument('y', default_value='2.5'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_teleop', default_value='false',
                              description='teleop needs a TTY; usually run it '
                                          'yourself in a second terminal'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim_slam.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
        }.items(),
    )

    avoidance = Node(
        package='zmr_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    teleop = Node(
        package='zmr_control',
        executable='teleop_key',
        name='teleop_key',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_teleop')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(args + [sim_slam, avoidance, teleop])
