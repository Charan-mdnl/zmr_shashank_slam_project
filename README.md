# ZMR Robot SLAM & Simulation Project

This repository contains the simulation and SLAM setup for the ZMR differential drive robot. It uses ROS 2 Humble and Gazebo Classic.

## Features
- **Accurate Physical Dimensions**: The URDF reflects the true physical topology of the ZMR robot, with accurately placed wheels and casters.
- **Robust Physics Configuration**: Tuned for Gazebo's ODE engine to prevent jitter and time-jump errors. 
- **Lidar & Odometry Integration**: `gazebo_ros_diff_drive` and `gazebo_ros_ray_sensor` are configured to provide flawless `/odom` and `/scan` data.
- **Pre-configured RViz**: Orbit 3D camera views and TF trees perfectly structured.
- **SLAM Toolbox Ready**: The launch file brings up `async_slam_toolbox_node` out-of-the-box for instant mapping.

## Prerequisites
- ROS 2 Humble
- Gazebo Classic (`gazebo_ros_pkgs`)
- `slam_toolbox`
- `teleop_twist_keyboard`

## Running the Simulation & SLAM

1. **Build and Source:**
   ```bash
   colcon build
   source install/setup.bash
   ```

2. **Launch the Simulation:**
   ```bash
   ros2 launch zmr_gazebo slam_sim.launch.py
   ```
   *(This launches Gazebo headlessly, spawns the robot, and opens RViz 2 with SLAM active.)*

3. **Drive the Robot (Teleop):**
   Open a new terminal, bypass Conda, and run:
   ```bash
   source /opt/ros/humble/setup.bash
   /usr/bin/python3 /opt/ros/humble/lib/teleop_twist_keyboard/teleop_twist_keyboard
   ```
   Use `i`, `,`, `j`, `l` to move and map your environment.
