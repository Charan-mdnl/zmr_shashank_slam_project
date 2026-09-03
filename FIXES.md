# What was broken, and what changed

Everything below was found by reading the code, reproducing the failure in the
headless simulator, or measuring against ground truth. Each entry says how it
showed up, because several of these fail *silently*.

## Pre-existing bugs in the original workspace

### 1. `slam_launch.py` threw away the CAD LiDAR pose
It published a hand-written static transform `base_link → laser` at
`(0, 0, 0.1)` and never started `robot_state_publisher`. The URDF puts
`lidar_link` at `(-0.425, 0.000, 0.210)` with yaw −90°, so every scan was
registered from a point 0.425 m and 90° away from where the sensor actually is.
The URDF is now the single source of truth and the driver publishes into
`lidar_link`.

### 2. The right wheel joint axis was mirrored
`base_to_right_wheel` resolved to **−Y** in `base_link` while the left wheel
resolved to **+Y**. `diff_drive_controller` requires both wheels to share a
positive-is-forward convention; as written, commanding both wheels forward
would have spun the robot. Axis negated, with the reasoning in the URDF.

### 3. `base_footprint` was 70 mm below the floor
The `base_footprint → base_link` joint was `xyz="0 0 0"`. The drive-wheel
centre sits at `base_link` z = +0.010 with radius 0.080, so the ground plane is
z = −0.070. Now `xyz="0 0 0.070"`, and a test asserts the wheel bottom lands
exactly on the footprint plane.

### 4. `zmr_gazebo` targeted a simulator that does not exist on Jazzy
`slam_sim.launch.py` launched `gazebo_ros` (Gazebo Classic), removed from ROS 2
well before Jazzy, so it could never start. The URDF used
`libgazebo_ros_{diff_drive,ray_sensor,joint_state_publisher}.so`, none of which
exist either. Ported to Gazebo Sim (Harmonic): `gz-sim-diff-drive-system`,
`gpu_lidar`, `gz-sim-joint-state-publisher-system`, world system plugins, a
`ros_gz_bridge`, and headless `-s` operation. **Not verifiable here** —
`ros-jazzy-ros-gz` is not installed.

### 5. `view_map_launch.py` depended on packages not in the workspace
It started `nav2_map_server`, `nav2_lifecycle_manager` and RViz. None are
present, and RViz is a GUI. Replaced with headless PNG rendering.

### 6. `package.xml` files declared absent dependencies
`nav2_map_server`, `nav2_lifecycle_manager`, `gazebo_ros`, `gazebo_plugins`.
Corrected to what is actually used.

## Bugs found while bringing the system up

### 7. `slam_toolbox` never left the `unconfigured` state
Its nodes are `rclcpp_lifecycle::LifecycleNode` and `main()` only spins them.
With no lifecycle manager they sit inactive: no subscriptions, no `/map`, no
`map → odom`, **and no error message**. `zmr_tools/lifecycle_manager` now
drives them to `active`.

### 8. Blocked motion desynchronised ground truth from odometry
When the simulated robot hit something, it still applied rotation to ground
truth but recorded zero rotation in odometry. That injects a heading error no
filter can observe, and it corrupted every map built after a collision.

### 9. `np.clip` turned every no-return beam into a real 25 m hit
`np.clip(out, range_min, range_max, out=out)` ran over the *whole* ranges
array, so beams that hit nothing (`inf`) became exactly `range_max` — which
every consumer accepts as a genuine return. That painted phantom walls on a
25 m arc through open floor. Now only the finite returns are clipped.

### 10. `base_link` had two TF parents
`sim_node` published `odom → base_link` while `robot_state_publisher` published
`base_footprint → base_link` from the URDF. A frame may have exactly one
parent; tf2 caches by *child* frame, so the two publishers fought over one slot
and lookups went stale or failed depending on which arrived first. Odometry now
goes to `base_footprint`, the convention the Gazebo diff-drive plugin uses.

### 11. The safety layer could freeze itself permanently
`arc_is_clear()` tested every candidate against the 0.12 m comfort margin.
Once the robot was already *inside* that margin, no arc could pass — including
reversing and rotating away — so it sat frozen at ~0.10 m clearance until the
watchdog gave up. **This was the direct cause of the observed stalls.** Escape
manoeuvres now use a hard collision bound and take whichever legal move buys
the most clearance.

### 12. Clearance outvoted the planned heading
The candidate score weighted clearance heavily enough to beat tracking, so the
robot abandoned its route and drifted into open space, then stranded itself.
Rebalanced so clearance only breaks ties.

## Bugs found by the adversarial review fleet

### 13. `simplify()` undid the inflation gradient
Its line-of-sight test only rejected chords through `INSCRIBED` cells, so it
replaced cost-shaped A* paths with straight chords grazing the inscribed
contour — measured dropping clearance from 0.90 m to 0.60 m, below the 0.634 m
the body needs to rotate. Now cost-aware, and it samples both cells at a
diagonal step so it cannot slip between two blocked corners.
*Measured after the fix: minimum clearance along the simplified path 0.95 m.*

### 14. Unknown cells were graded cheaper than known-clear floor
With `plan_unknown_as_obstacle: False`, unmapped cells got cost 0 — the same as
verified-open floor, and cheaper than anything near an obstacle. A* therefore
preferred unmapped space to a doorway it could actually see. Unknown cells now
carry an explicit cost.

### 15. Failed plans were retried at 20 Hz with no backoff
`last_plan_t` was only advanced on success, so a ~0.9 s A* exhaustion re-ran
every tick, starving the single-threaded executor until `/cmd_vel_auto` fell
past the safety layer's 1.0 s input timeout and the robot stopped — while the
logs still showed `FOLLOW`. Failures are now rate-limited, and `max_nodes`
actually bounds the search (it was 4,000,000 against a 240,000-cell grid).

### 16. `nearest_free` returned the first ring cell, not the nearest
A Chebyshev ring spans *r* to *r*√2, and the scan took the top-left corner —
up to 41 % farther than the true nearest. Now sorted by squared distance.

### 17. Arrival was judged against the *snapped* goal
`dist` measured to the point the goal had been snapped to, so the node could
log `reached waypoint … error 0.05 m` while standing metres from the commanded
waypoint. Progress, the watchdog and the log now all reference the commanded
waypoint, and any snap offset is reported explicitly.

### 18. The path endpoint was restored to a possibly-lethal cell
The raw waypoint was written back as the final path point whenever the snap was
merely *close*, which says nothing about the goal being free. Now conditioned on
the goal cell actually being traversable.

### 19. `world_to_cell` truncated instead of flooring
`int()` truncates toward zero, so a point in the cell just outside the map's
low edge folded back to index 0 — defeating the frontier test. SLAM map origins
are routinely negative.

### 20. "Frontier" tested array bounds, not whether the goal was unknown
`slam_toolbox` pads its grid with −1 long before that space is scanned, so
goals in unmapped-but-in-array space were treated as ordinary — getting the
slow replan cadence and the short watchdog exactly when the plan was least
trustworthy. Now tests the cell value too.

### 21. Planner radius was below what the safety layer enforces
`obstacle_avoidance` needs `body_length/2 + safety_margin` = 0.645 m fore/aft,
and the body's circumscribed radius is 0.634 m — but the planner used 0.40, then
0.60. Every plan was unexecutable by the node that owns the wheels. Now 0.65,
with the derivation in a comment so the two cannot drift apart again.

### 22. The inflation band ended in a cost cliff
Cost decayed to ~102 at `inflation_radius` and then dropped to 0, so the
planner tracked that contour instead of open floor. Now normalised to decay
continuously to zero.

### 23. `lidar_yaw_fallback` disagreed with the URDF
Defaulted to 0.0 where the URDF resolves −90°. On any transient TF timeout the
whole point cloud was placed unrotated — a wall ahead written as a wall to the
right. Now −π/2, matching the URDF and the simulator.

### 24. The escape gate was unsatisfiable when the obstacle was abeam
The `best is None` branch seeded `best_gain` with the robot's *current*
clearance and then required a candidate to strictly beat it. `min_d` is the
minimum over the whole arc, so when the closest return is **abeam** — a
doorjamb the robot is sliding past, or a pillar beside it — reversing leaves
that lateral distance exactly unchanged, `min_d == here`, and every legal
escape was discarded. The node emitted `(0, 0)`, the scan therefore never
changed, and `BLOCKED` became a permanent fixed point with no timeout.

An adversarial reviewer reproduced it beside the pillar at (15, 9): pose
(15.000, 9.635, 0°), `here = +0.0676`, reverse collision-free with
`min_d = +0.0676` — identical, so rejected; all six rotations blocked. A full
pose sweep found **25,642 poses (28 % of those reaching the escape branch)**
where nothing could pass the gate. Head-on approaches worked, which is why the
branch looked fine in the common case.

Fixed by seeding `best_gain = -inf` so `min_d` only *ranks* candidates —
legality is already decided by `hard_margin` inside `arc_is_clear` — plus a
`blocked_unlatch_time` that forces a slow reverse rather than staying wedged.
(`stuck_for` was being computed and never read.)

### 25. `render_map` mis-parsed launch-injected arguments
It filtered the literal `--ros-args` token but left `-r` and `__node:=…`
behind, which were then read as `path_csv` and `title`. Now truncates at
`--ros-args`.

## Not a code bug: the test world

Doorways were 1.6 m. A 1.05 × 0.71 m robot has a 0.634 m circumscribed radius,
so a planner keeping that clearance consumes ~1.3 m of any opening — leaving a
0.3 m ribbon. Widened to 2.2 m, the realistic industrial width for a truck this
size. This is why waypoint 4 was unreachable even after the code was correct.


## Not a code bug: the test harness

### 26. Stale nodes accumulated between runs and fought over `/cmd_vel`
`stop_all.sh` used `pkill -x` with names truncated to 14 characters, but Linux
truncates a process's `comm` to **15**, so `obstacle_avoidance`,
`waypoint_navigator`, `lifecycle_manager` and `async_slam_toolbox_node` were
never matched and survived every "cleanup". `ros2 node list` showed duplicate
`/obstacle_avoidance` and `/waypoint_navigator` nodes from earlier runs, all
publishing to the same topics — so results depended on which stale node
happened to win.

**This, not the robot, was the source of the run-to-run flakiness.** The script
now resolves PIDs with `pgrep -f`, excludes its own process and every ancestor
(so it can never kill the shell that called it), escalates TERM→KILL, and
*verifies* nothing survived instead of exiting silently.

After the fix: three consecutive runs, 6/6 waypoints each, identical 54.0 m
route, SLAM error 0.009–0.022 m mean.
