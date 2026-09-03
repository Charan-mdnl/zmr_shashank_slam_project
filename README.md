# ZMR SLAM Project

Headless SLAM, teleoperation, obstacle avoidance and point-to-point navigation
for the ZMR differential-drive AMR — on the real robot, or in a simulator that
needs nothing but this workspace.

**Nothing here opens a window.** No Gazebo GUI, no RViz, no X server. Results
are inspected as saved maps, CSV paths and rendered PNGs.

## The robot

CAD-verified, from `src/zmr_description/config/robot_geometry.yaml`:

| | |
|---|---|
| Body | 1.05 m long × 0.71 m wide |
| Drive wheels | radius 0.080 m, separation 0.526 m |
| Castors | 4, at the corners |
| LiDAR | RPLIDAR A3M1 — `lidar_link` at base_link `(-0.425, 0.000, 0.210)`, yaw −90° |
| Odometry (real robot) | `rf2o_laser_odometry` — there are no wheel encoders |

## Quick start

```bash
source scripts/setup_env.sh          # ROS 2 Jazzy + deps_ws + this workspace
colcon build --symlink-install
python3 test/test_zmr.py             # 24 offline tests, no ROS graph needed
```

### Autonomous navigation demo

```bash
./scripts/demo_navigate.sh           # ~7 min, writes to results/
```

Drives six waypoints through the simulated warehouse, saves the map it built,
and renders map + driven path to `results/result.png`. A previously recorded
run is already committed in [`results/`](results/) — see `results/README.md`.

### Teleoperation demo

```bash
# terminal 1
./scripts/demo_teleop.sh
# terminal 2 — teleop_key needs a real TTY to read keys
source scripts/setup_env.sh && ros2 run zmr_control teleop_key
```

`w`/`s` forward/reverse, `a`/`d` turn, `q`/`e`/`z`/`c` arcs, space stop,
`-`/`=` speed. The safety layer sits downstream and will refuse any command
that would drive the body into something the LiDAR can see.

## The three control nodes

```
teleop_key ──────────► /cmd_vel_teleop ─┐
                                        ├─► obstacle_avoidance ─► /cmd_vel ─► robot
waypoint_navigator ──► /cmd_vel_auto ───┘         ▲
        ▲                                         │
        └── map->odom from slam_toolbox        /scan
```

| Node | Package | What it does |
|---|---|---|
| `teleop_key` | `zmr_control` | Raw-TTY keyboard driving. Works over SSH. Publishes `/cmd_vel_teleop`. |
| `obstacle_avoidance` | `zmr_control` | The safety layer and command arbiter. Teleop wins over autonomy while it is fresh. Forward-simulates candidate `(v, ω)` arcs against the live scan using the true rectangular footprint, and picks the nearest safe alternative to what was asked for. Publishes `/cmd_vel`, `/safety/state`, `/safety/min_clearance`. |
| `waypoint_navigator` | `zmr_control` | Goes to one place, then the next. A* over the SLAM map (inflated by the robot radius) plus pure-pursuit following, replanning as the map grows. Publishes `/cmd_vel_auto`, `/plan`, `/navigation/state`. |

`waypoint_navigator` never talks to the wheels directly — every command it
produces is collision-checked by `obstacle_avoidance` first.

## Packages

| Package | Purpose |
|---|---|
| `zmr_description` | URDF, CAD meshes, geometry constants |
| `zmr_sim` | Dependency-free headless 2D simulator — differential-drive physics, LiDAR ray casting, `/scan` `/odom` `/joint_states` `/ground_truth` + TF |
| `zmr_control` | `teleop_key`, `obstacle_avoidance`, `waypoint_navigator`, `grid_planner` |
| `zmr_tools` | `map_saver`, `lifecycle_manager`, `path_recorder`, `render_map` |
| `zmr_bringup` | Real-robot launch: RPLIDAR + rf2o + slam_toolbox |
| `results/` | Recorded verification output — map, path, PNG, three reliability runs |
| `zmr_gazebo` | Gazebo Sim (Harmonic) launch — needs `ros-jazzy-ros-gz` |
| `sllidar_ros2`, `rf2o_laser_odometry` | Vendored drivers |

## Measured results

From `./scripts/demo_navigate.sh` on the built-in warehouse world:

| Metric | Result |
|---|---|
| Waypoints reached | **6 / 6**, on three consecutive runs (tolerance 0.30 m) |
| Waypoint error | 0.228 – 0.248 m |
| Distance driven | 54.0 m, identical across runs |
| SLAM pose error vs ground truth | mean **0.009 – 0.022 m**, max 0.078 m |
| Map vs ground truth | mean **0.020 m**; 95 % within 5 cm, 100 % within 20 cm |
| Teleop into a wall at 0.4 m/s | stopped with the nose **0.254 m** clear |
| Offline test suite | 24 / 24 passing |

## Running on the real robot

```bash
sudo cp src/zmr_bringup/config/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

ros2 launch zmr_bringup slam_launch.py            # lidar + rf2o + slam_toolbox
ros2 run zmr_control obstacle_avoidance
ros2 run zmr_control teleop_key                   # or waypoint_navigator

ros2 run zmr_tools map_saver --ros-args -p filename:=$HOME/site
ros2 service call /map_saver/save std_srvs/srv/Trigger
```

The same three control nodes run unchanged against the simulator or the robot.

## Why slam_toolbox is built from source

`ros-jazzy-slam-toolbox` is not installed here and `apt` needs sudo, so it is
built into a sibling `deps_ws/` (with `bond_core`). Two patches were needed:

- `cmake_policy(SET CMP0167 OLD)` guarded — CMP0167 needs CMake ≥ 3.30, this
  host has 3.28.
- `nav2_map_server` dropped from `package.xml` — slam_toolbox only shells out
  to it at runtime for map saving, which `zmr_tools/map_saver` now does.

`slam_toolbox`'s nodes are **lifecycle nodes**: their `main()` spins them but
never transitions them, so on their own they sit `unconfigured`, publishing
nothing and logging no error. `zmr_tools/lifecycle_manager` drives them to
`active` — it replaces `nav2_lifecycle_manager`, which is not in this
workspace.

## Notes and limits

- `zmr_gazebo` was written for Gazebo Classic, which does not exist on ROS 2
  Jazzy, so it could never start. It has been ported to Gazebo Sim (Harmonic)
  and made headless, but it **cannot be tested here** — `ros-jazzy-ros-gz` is
  not installed. `zmr_sim` is the path that is verified working.
- The simulator's odom frame starts at the robot's true pose, so map, odom and
  world coordinates coincide and waypoints are plain world positions. Pass
  `odom_at_origin:=true` for the real-robot convention.
- Test-world doorways are 2.2 m. A 1.05 × 0.71 m robot has a 0.634 m
  circumscribed radius, so a planner keeping that clearance needs ~1.3 m of any
  opening; 1.6 m doors left only a 0.3 m ribbon to steer down.
- `robot_radius` (planner) must stay ≥ what `obstacle_avoidance` enforces
  fore/aft — `body_length/2 + safety_margin` = 0.645 m — or the planner emits
  routes the safety layer will refuse to execute.
