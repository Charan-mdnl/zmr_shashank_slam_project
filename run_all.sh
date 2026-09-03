#!/bin/bash
# A single script to run the simulator, RViz, and Teleop Keyboard all at once!

# Save DISPLAY variable before setup_env might unset it
SAVED_DISPLAY="$DISPLAY"

# Deactivate Conda
eval "$(conda shell.bash hook 2>/dev/null)" && conda deactivate 2>/dev/null

# Source setup_env.sh to get exact ROS_DOMAIN_ID=91 and environment settings
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts/setup_env.sh"

# Restore DISPLAY so GUI applications (like RViz) can open windows
export DISPLAY="$SAVED_DISPLAY"

echo "========================================="
echo "Starting Simulator & SLAM on ROS_DOMAIN_ID=$ROS_DOMAIN_ID..."
bash ./scripts/demo_teleop.sh > /tmp/zmr_sim_output.log 2>&1 &
SIM_PID=$!

echo "Starting RViz GUI in background..."
/opt/ros/humble/bin/rviz2 -d src/zmr_bringup/config/rviz_slam.rviz --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
RVIZ_PID=$!

echo "Waiting 3 seconds for systems to initialize..."
sleep 3
echo "========================================="
echo "Starting Teleop Keyboard..."
echo "Use W/A/S/D to drive. Press Ctrl-C to quit."
echo "========================================="

# Run teleop in foreground
python3 install/zmr_control/lib/zmr_control/teleop_key

# Clean up
echo "Cleaning up processes..."
kill $RVIZ_PID 2>/dev/null
kill $SIM_PID 2>/dev/null
bash ./scripts/stop_all.sh 2>/dev/null
echo "All done!"
