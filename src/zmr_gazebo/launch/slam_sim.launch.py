"""ZMR SLAM in Gazebo Sim (Harmonic), headless.

REQUIRES ros_gz, which is NOT part of a base ROS 2 Jazzy desktop install:

    sudo apt install ros-jazzy-ros-gz

If you cannot install it, use the dependency-free simulator instead - it needs
nothing beyond this workspace and exposes the same topics:

    ros2 launch zmr_sim sim_slam.launch.py

WHAT CHANGED FROM THE PREVIOUS VERSION
  The old file launched `gazebo_ros`, which is Gazebo Classic. Classic was
  removed from ROS 2 well before Jazzy, so `gazebo.launch.py` does not exist
  and this launch file could never start. It now uses ros_gz_sim, runs the
  server headless (-s, no GUI), and bridges the gz topics into ROS.

    ros2 launch zmr_gazebo slam_sim.launch.py
    ros2 launch zmr_gazebo slam_sim.launch.py gui:=true   # needs a display
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gz_share = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('zmr_gazebo')
    bringup_share = get_package_share_directory('zmr_bringup')

    urdf_path = os.path.join(pkg_share, 'urdf', 'zmr_robot_sim.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'zmr_test_world.world')
    slam_params = os.path.join(bringup_share, 'config', 'slam_toolbox_sim.yaml')
    with open(urdf_path, 'r') as fh:
        robot_description = fh.read()

    args = [
        DeclareLaunchArgument('gui', default_value='false',
                              description='false = headless server only'),
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params),
    ]
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    # -s = server only (headless), -r = run immediately, -v 2 = warnings
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v 2 ', world]}.items(),
        condition=UnlessCondition(gui),
    )
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 2 ', world]}.items(),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'zmr_robot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.1',
        ],
    )

    # gz <-> ROS bridge. The gz topic names come from the <topic> tags set on
    # the plugins and the sensor in urdf/zmr_robot_sim.urdf.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
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

    return LaunchDescription(args + [
        gz_headless, gz_gui, robot_state_publisher, spawn, bridge, slam,
        lifecycle_manager,
    ])
