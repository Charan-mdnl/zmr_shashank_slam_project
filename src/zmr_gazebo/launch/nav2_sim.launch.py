import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_zmr_gazebo = get_package_share_directory('zmr_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Include the SLAM and Simulation launch file (Gazebo + Robot + SLAM Toolbox + RViz)
    slam_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zmr_gazebo, 'launch', 'slam_sim.launch.py')
        )
    )

    # Path to our custom Nav2 parameters
    nav2_params_file = os.path.join(pkg_zmr_gazebo, 'config', 'zmr_nav2.yaml')

    # Launch Nav2 Navigation Stack (Planner, Controller, Recoveries, BT Navigator)
    # We do NOT launch AMCL or Map Server because SLAM Toolbox is handling that.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    return LaunchDescription([
        slam_sim_launch,
        nav2_navigation_launch
    ])
