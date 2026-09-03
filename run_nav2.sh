#!/bin/bash
# Autonomous Navigation with official ROS 2 Nav2 stack!

SAVED_DISPLAY="$DISPLAY"
eval "$(conda shell.bash hook 2>/dev/null)" && conda deactivate 2>/dev/null

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

export ROS_DOMAIN_ID=91
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export DISPLAY="$SAVED_DISPLAY"

echo "========================================="
echo "Starting ZMR Simulator & SLAM Toolbox (Domain $ROS_DOMAIN_ID)..."
bash ./scripts/demo_teleop.sh > /tmp/zmr_sim.log 2>&1 &
SIM_PID=$!

echo "Waiting 3 seconds for simulator and TF tree..."
sleep 3

echo "Starting official ROS 2 Nav2 Stack..."
ros2 launch zmr_bringup nav2.launch.py > /tmp/zmr_nav2.log 2>&1 &
NAV2_PID=$!

echo "Starting RViz GUI..."
/opt/ros/humble/bin/rviz2 -d src/zmr_bringup/config/rviz_slam.rviz --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
RVIZ_PID=$!

echo "Waiting 5 seconds for Nav2 stack to initialize..."
sleep 5

echo "========================================="
echo "Nav2 Stack is ACTIVE!"
echo "Use '2D Nav Goal' in RViz to navigate autonomously."
echo "Press Ctrl-C to quit."
echo "========================================="

# Clean up on exit
trap "kill $RVIZ_PID $NAV2_PID $SIM_PID 2>/dev/null; bash ./scripts/stop_all.sh 2>/dev/null; exit 0" EXIT INT TERM

while true; do
    sleep 1
done
