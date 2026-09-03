"""Render a saved map to PNG - the headless replacement for opening RViz.

The previous version started nav2_map_server, nav2_lifecycle_manager and RViz.
None of those are needed just to look at a map, and the nav2 packages are not
part of this workspace, so the launch file could never run. This renders the
map (and optionally a recorded path) to an image instead.

  ros2 launch zmr_bringup view_map_launch.py map:=my_map
  ros2 launch zmr_bringup view_map_launch.py \
      map:=/abs/path/my_map.yaml path:=/tmp/zmr_path.csv out:=/tmp/map.png
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('zmr_bringup')
    map_arg = LaunchConfiguration('map').perform(context)
    path_arg = LaunchConfiguration('path').perform(context)
    out_arg = LaunchConfiguration('out').perform(context)

    map_yaml = map_arg
    if not map_yaml.endswith('.yaml'):
        map_yaml += '.yaml'
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.join(pkg_share, 'maps', map_yaml)

    if not out_arg:
        out_arg = os.path.splitext(map_yaml)[0] + '.png'

    argv = [map_yaml, out_arg]
    if path_arg:
        argv.append(path_arg)

    return [Node(
        package='zmr_tools',
        executable='render_map',
        name='render_map',
        output='screen',
        arguments=argv,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='my_map',
                              description='map name in zmr_bringup/maps, or an '
                                          'absolute path to a .yaml'),
        DeclareLaunchArgument('path', default_value='',
                              description='optional path CSV from path_recorder'),
        DeclareLaunchArgument('out', default_value='',
                              description='output PNG (defaults next to the map)'),
        OpaqueFunction(function=_resolve),
    ])
