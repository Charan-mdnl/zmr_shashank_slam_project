#!/bin/bash
# Source this before running anything in the ZMR workspace.
#   source scripts/setup_env.sh
#
# deps_ws carries slam_toolbox (+ bond_core), built from source because
# ros-jazzy-slam-toolbox is not installed on this machine and apt needs sudo.
ZMR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$ZMR_ROOT/install/setup.bash" ] && source "$ZMR_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset DISPLAY            # headless: nothing may open a window
