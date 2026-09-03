#!/bin/bash
# Keyboard teleoperation with the LiDAR safety layer, building a map as you go.
#
# Terminal 1:  ./scripts/demo_teleop.sh
# Terminal 2:  source scripts/setup_env.sh && ros2 run zmr_control teleop_key
#
# teleop_key needs a real TTY to read keypresses, so it is deliberately NOT
# started here. Drive with w/a/s/d; the safety layer will refuse any command
# that would put the body into something the LiDAR can see.
#
# When you are done mapping, in a third terminal:
#   ros2 run zmr_tools map_saver --ros-args -p filename:=$HOME/my_map \
#        -p use_sim_time:=true
#   ros2 service call /map_saver/save std_srvs/srv/Trigger
set -e
ZMR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ZMR/scripts/setup_env.sh"
bash "$ZMR/scripts/stop_all.sh"
echo "== ZMR teleop + SLAM (headless) =="
echo "   drive from a second terminal:  ros2 run zmr_control teleop_key"
exec ros2 launch zmr_control teleop_slam.launch.py "$@"
