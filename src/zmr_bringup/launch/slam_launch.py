"""ZMR SLAM mapping on the REAL robot. Headless by default.

Pipeline:
  1. sllidar_ros2 driver  -> /scan in frame lidar_link
  2. robot_state_publisher -> the URDF's TF tree, including base_link -> lidar_link
  3. rf2o_laser_odometry   -> odom -> base_link from consecutive scans
  4. slam_toolbox (async)  -> map -> odom

Usage:
  ros2 launch zmr_bringup slam_launch.py
  ros2 launch zmr_bringup slam_launch.py serial_port:=/dev/rplidar

Save the map (no nav2 needed):
  ros2 run zmr_tools map_saver --ros-args -p filename:=<pkg>/maps/my_map
  ros2 service call /map_saver/save std_srvs/srv/Trigger

FIXED in this revision:
  * The old version published a hand-written static transform
    base_link -> laser at (0, 0, 0.1) and never started
    robot_state_publisher. That discarded the CAD lidar pose entirely - the
    URDF puts lidar_link 0.425 m from base_link, not at the centre - so every
    scan was registered from the wrong place. The URDF is now the single
    source of truth for that transform and the driver publishes into
    'lidar_link' to match it.
  * RViz is no longer launched: this stack is expected to run headless on the
    robot. Set start_rviz:=true if you are on a machine with a display.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('zmr_bringup')
    desc_share = get_package_share_directory('zmr_description')

    slam_params_file = os.path.join(bringup_share, 'config',
                                    'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(bringup_share, 'config', 'rviz_slam.rviz')
    urdf_path = os.path.join(desc_share, 'urdf', 'zmr_robot.urdf')
    with open(urdf_path, 'r') as fh:
        robot_description = fh.read()

    args = [
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/rplidar',
            description='RPLidar A3M1 port; /dev/rplidar comes from '
                        'config/rplidar.rules'),
        DeclareLaunchArgument('serial_baudrate', default_value='256000'),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity'),
        DeclareLaunchArgument(
            'lidar_frame', default_value='lidar_link',
            description='must match the URDF link the lidar is mounted on'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file),
        DeclareLaunchArgument('start_rviz', default_value='false',
                              description='headless by default'),
    ]

    # 1. LiDAR driver - publishes into the URDF's lidar_link frame
    sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
    )

    # 2. The URDF is the single source of truth for base_link -> lidar_link
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # The real ZMR has no wheel encoders, so there is no wheel odometry to
    # publish joint states from. Publish zeros so the wheel links still have a
    # transform and the model stays complete.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # 3. Laser odometry: this robot's substitute for wheel encoders
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    # 4. SLAM
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
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
            'use_sim_time': False,
        }],
    )

    return LaunchDescription(args + [
        sllidar,
        robot_state_publisher,
        joint_state_publisher,
        rf2o,
        slam_toolbox,
        lifecycle_manager,
        rviz,
    ])
