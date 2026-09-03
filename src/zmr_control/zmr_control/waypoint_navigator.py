#!/usr/bin/env python3
"""Waypoint navigation for the ZMR AMR: go to one place, then the next.

Architecture (the same split nav2 uses, at a fraction of the size):

    global planner   A* over the SLAM map, inflated by the robot radius
    local controller pure pursuit along the planned path
    safety layer     obstacle_avoidance, downstream of this node

A purely reactive controller cannot get a robot around a wall, so this node
plans a route over the live map from slam_toolbox, follows it, and replans
periodically and whenever it is blocked. Pose comes from TF: it prefers
``map -> base_link`` (SLAM-corrected) and falls back to ``odom -> base_link``
before the first map arrives.

Velocity is published on /cmd_vel_auto, which is an *input* to
obstacle_avoidance - this node never talks to the wheels directly, so every
command it produces is collision-checked before execution.

    subscribes  /map  /goal_pose
    publishes   /cmd_vel_auto  /navigation/state  /navigation/progress  /plan
    services    /navigation/start  /navigation/stop  (std_srvs/Trigger)
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from zmr_control.grid_planner import astar, build_costmap, nearest_free, simplify


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class WaypointNavigator(Node):

    PLAN, FOLLOW, ALIGN, ARRIVED, DONE, IDLE, BLOCKED = (
        'PLAN', 'FOLLOW', 'ALIGN', 'ARRIVED', 'DONE', 'IDLE', 'BLOCKED')

    def __init__(self):
        super().__init__('zmr_waypoint_navigator')

        self.declare_parameter('waypoints', [2.5, 2.5, 0.0])
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('loop', False)
        self.declare_parameter('autostart', True)

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('fallback_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('max_linear_vel', 0.45)
        self.declare_parameter('max_angular_vel', 0.90)
        self.declare_parameter('approach_vel', 0.15)

        self.declare_parameter('xy_tolerance', 0.30)
        self.declare_parameter('yaw_tolerance', 0.15)
        self.declare_parameter('turn_threshold', 0.70)
        self.declare_parameter('slow_radius', 1.20)
        self.declare_parameter('lookahead', 0.90)
        self.declare_parameter('k_heading', 1.5)

        # Planner clearance. This MUST NOT be below what the downstream safety
        # layer enforces, or every plan is unexecutable by the node that owns
        # the wheels. obstacle_avoidance collision-checks the true rectangle
        # plus safety_margin, so it needs body_length/2 + margin
        # = 0.525 + 0.12 = 0.645 m fore/aft. The body's circumscribed radius is
        # sqrt(0.525^2 + 0.355^2) = 0.634 m, so 0.65 covers both.
        self.declare_parameter('robot_radius', 0.65)
        self.declare_parameter('inflation_radius', 0.95)
        self.declare_parameter('cost_scaling', 3.0)
        self.declare_parameter('replan_period', 3.0)
        self.declare_parameter('min_plan_interval', 0.5)
        self.declare_parameter('plan_unknown_as_obstacle', False)

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('arrive_hold', 0.5)
        self.declare_parameter('goal_timeout', 240.0)

        p = self.get_parameter
        self.global_frame = str(p('global_frame').value)
        self.fallback_frame = str(p('fallback_frame').value)
        self.base_frame = str(p('base_frame').value)
        self.v_max = float(p('max_linear_vel').value)
        self.w_max = float(p('max_angular_vel').value)
        self.v_app = float(p('approach_vel').value)
        self.xy_tol = float(p('xy_tolerance').value)
        self.yaw_tol = float(p('yaw_tolerance').value)
        self.turn_thresh = float(p('turn_threshold').value)
        self.slow_r = float(p('slow_radius').value)
        self.lookahead = float(p('lookahead').value)
        self.k_head = float(p('k_heading').value)
        self.robot_radius = float(p('robot_radius').value)
        self.infl_radius = float(p('inflation_radius').value)
        self.cost_scaling = float(p('cost_scaling').value)
        self.replan_period = float(p('replan_period').value)
        self.min_plan_interval = float(p('min_plan_interval').value)
        self.unknown_block = bool(p('plan_unknown_as_obstacle').value)
        self.arrive_hold = float(p('arrive_hold').value)
        self.goal_timeout = float(p('goal_timeout').value)
        self.loop = bool(p('loop').value)

        self.waypoints = self.load_waypoints()
        self.idx = 0
        self.state = self.IDLE
        self.active = bool(p('autostart').value)
        self.arrived_t = 0.0
        self.goal_start_t = 0.0
        self.last_plan_t = -1e9
        self.best_dist = float('inf')
        self.best_dist_t = 0.0
        self.no_progress_timeout = 25.0
        self.frame_in_use = None
        self._warned_tf = False
        self._warned_frame = False
        self.plan_fail = 0

        self.goal_eff = None      # goal snapped out of obstacles, in world m
        self.goal_is_frontier = False
        self.frontier_xy = None
        self.warned_unreachable = False
        self.grid = None          # raw OccupancyGrid
        self.cost = None          # inflated cost array
        self.path_world = []      # list of (x, y)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.state_pub = self.create_publisher(String, 'navigation/state', 10)
        self.prog_pub = self.create_publisher(Float32, 'navigation/progress', 10)
        self.path_pub = self.create_publisher(Path, 'plan', 10)

        self.create_service(Trigger, 'navigation/start', self.srv_start)
        self.create_service(Trigger, 'navigation/stop', self.srv_stop)

        self.create_timer(1.0 / float(p('control_rate').value), self.step)

        self.get_logger().info(
            f'waypoint navigator up | {len(self.waypoints)} waypoint(s) | '
            f'A* planner, robot_radius {self.robot_radius:.2f} m | '
            f'{"autostart" if self.active else "waiting for navigation/start"}')
        for i, (x, y, yaw) in enumerate(self.waypoints):
            yaw_s = 'any' if math.isnan(yaw) else f'{math.degrees(yaw):.0f} deg'
            self.get_logger().info(f'  {i + 1}. ({x:+.2f}, {y:+.2f})  heading {yaw_s}')

    # ------------------------------------------------------------- waypoints
    def load_waypoints(self):
        path = str(self.get_parameter('waypoints_file').value)
        if path:
            with open(path, 'r') as fh:
                data = yaml.safe_load(fh)
            raw = data['waypoints'] if isinstance(data, dict) else data
            out = []
            for wp in raw:
                if isinstance(wp, dict):
                    out.append((float(wp['x']), float(wp['y']),
                                float(wp.get('yaw', float('nan')))))
                else:
                    yaw = float(wp[2]) if len(wp) > 2 else float('nan')
                    out.append((float(wp[0]), float(wp[1]), yaw))
            return out
        flat = list(self.get_parameter('waypoints').value)
        if len(flat) % 3 != 0:
            raise ValueError('waypoints must be flat [x, y, yaw] triples; '
                             f'got {len(flat)} values')
        return [(float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
                for i in range(0, len(flat), 3)]

    def on_goal(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.waypoints.append((msg.pose.position.x, msg.pose.position.y, yaw))
        self.get_logger().info(
            f'goal appended: ({msg.pose.position.x:+.2f}, '
            f'{msg.pose.position.y:+.2f}) -> {len(self.waypoints)} waypoints')
        if self.state == self.DONE:
            self.idx = len(self.waypoints) - 1
            self.state = self.IDLE
        self.active = True

    # ----------------------------------------------------------------- map
    def on_map(self, msg: OccupancyGrid):
        self.grid = msg
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        try:
            self.cost = build_costmap(
                data, msg.info.resolution, self.robot_radius,
                self.infl_radius, self.cost_scaling,
                unknown_is_obstacle=self.unknown_block)
        except Exception as exc:                     # scipy missing, etc.
            self.get_logger().error(f'costmap build failed: {exc}')
            self.cost = None

    def world_to_cell(self, x, y):
        # floor, not int(): int() truncates toward zero, so a point in the
        # cell just outside the map's low edge would fold back to index 0.
        info = self.grid.info
        return (int(math.floor((y - info.origin.position.y) / info.resolution)),
                int(math.floor((x - info.origin.position.x) / info.resolution)))

    def cell_to_world(self, r, c):
        info = self.grid.info
        return (info.origin.position.x + (c + 0.5) * info.resolution,
                info.origin.position.y + (r + 0.5) * info.resolution)

    # -------------------------------------------------------------- services
    def srv_start(self, _req, resp):
        self.active = True
        if self.state == self.DONE:
            self.idx = 0
        self.state = self.IDLE
        resp.success = True
        resp.message = f'navigating to waypoint {self.idx + 1}/{len(self.waypoints)}'
        self.get_logger().info(resp.message)
        return resp

    def srv_stop(self, _req, resp):
        self.active = False
        self.publish_cmd(0.0, 0.0)
        resp.success = True
        resp.message = 'navigation stopped'
        self.get_logger().info(resp.message)
        return resp

    # ------------------------------------------------------------------- tf
    def robot_pose(self):
        for frame in (self.global_frame, self.fallback_frame):
            try:
                tr = self.tf_buffer.lookup_transform(
                    frame, self.base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            q = tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if self.frame_in_use != frame:
                self.frame_in_use = frame
                self.get_logger().info(f'localising against "{frame}"')
            return (tr.transform.translation.x, tr.transform.translation.y,
                    yaw, frame)
        if not self._warned_tf:
            self.get_logger().warn(
                f'no transform to {self.base_frame} from '
                f'"{self.global_frame}" or "{self.fallback_frame}" yet')
            self._warned_tf = True
        return None

    # -------------------------------------------------------------- planning
    def plan(self, x, y, gx, gy) -> bool:
        """Plan a route to (gx, gy). Falls back to a straight line with no map."""
        if self.grid is None or self.cost is None:
            self.path_world = [(gx, gy)]
            self.goal_eff = (gx, gy)
            self.goal_is_frontier = False
            return True

        h, w = self.cost.shape
        start = nearest_free(self.cost, self.world_to_cell(x, y))
        gr, gc = self.world_to_cell(gx, gy)

        # Two different situations must not be confused:
        #
        #   frontier    the goal is outside the mapped area. It is not
        #               unreachable, just not seen yet. Head for the edge of
        #               the map along the line to it and keep replanning as
        #               the map grows; arrival is still judged on the real goal.
        #   unreachable the goal is inside the map but lands in an obstacle or
        #               its inflation. Aim for the closest free cell and say so.
        raw = np.asarray(self.grid.data, dtype=np.int16).reshape(h, w)
        in_array = (0 <= gr < h and 0 <= gc < w)
        # A goal is a frontier if it is outside the array OR still unknown.
        # slam_toolbox pads its grid with -1 well before that space is scanned.
        self.goal_is_frontier = (not in_array) or bool(raw[gr, gc] < 0)
        if self.goal_is_frontier:
            sr, sc = self.world_to_cell(x, y)
            best = None
            for t in np.linspace(0.02, 1.0, 200):
                rr = int(round(sr + (gr - sr) * t))
                cc = int(round(sc + (gc - sc) * t))
                if 0 <= rr < h and 0 <= cc < w:
                    best = (rr, cc)
                else:
                    break
            if best is None:
                return False
            gr, gc = best

        goal = nearest_free(self.cost, (gr, gc))
        if start is None or goal is None:
            return False

        snapped = self.cell_to_world(*goal)
        if self.goal_is_frontier:
            # aim at the frontier, but the waypoint itself is still the goal
            self.goal_eff = None
        else:
            self.goal_eff = snapped
            if math.dist(snapped, (gx, gy)) > self.xy_tol and not self.warned_unreachable:
                self.warned_unreachable = True
                self.get_logger().warn(
                    f'waypoint {self.idx + 1} ({gx:+.2f}, {gy:+.2f}) is not '
                    f'reachable - it is inside an obstacle or its inflation; '
                    f'aiming for the closest free point '
                    f'({snapped[0]:+.2f}, {snapped[1]:+.2f}) instead')
        self.frontier_xy = snapped

        cells = astar(self.cost, start, goal)
        if not cells:
            return False
        cells = simplify(cells, self.cost)
        self.path_world = [self.cell_to_world(r, c) for r, c in cells]
        # End the path on the true goal only when that cell is itself
        # traversable - closeness of the snap says nothing about that.
        if in_array and self.cost[gr, gc] < 253:
            self.path_world[-1] = (gx, gy)
        self.publish_path()
        return True

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = ((self.grid.header.frame_id if self.grid
                                else None) or self.frame_in_use
                               or self.global_frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path_world:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def carrot(self, x, y):
        """Pure-pursuit target: the point on the path `lookahead` ahead."""
        if not self.path_world:
            return None
        # drop path points already passed
        while len(self.path_world) > 1 and \
                math.dist((x, y), self.path_world[0]) < self.lookahead:
            self.path_world.pop(0)
        for px, py in self.path_world:
            if math.dist((x, y), (px, py)) >= self.lookahead * 0.6:
                return (px, py)
        return self.path_world[-1]

    # ------------------------------------------------------------- main step
    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.active or self.state == self.DONE or not self.waypoints:
            self.publish_cmd(0.0, 0.0)
            self.publish_status()
            return

        pose = self.robot_pose()
        if pose is None:
            self.publish_cmd(0.0, 0.0)
            self.publish_status()
            self.goal_start_t = now          # do not burn the watchdog waiting
            self.best_dist_t = now
            return
        x, y, th, pose_frame = pose

        # The costmap cells are anchored to the grid's own frame. Planning with
        # a pose taken from a DIFFERENT frame silently offsets the whole plan by
        # the map->odom correction, which is exactly what happens during the
        # first seconds while slam_toolbox is still coming up. Wait it out
        # rather than planning against mismatched frames.
        if self.grid is not None:
            grid_frame = self.grid.header.frame_id or self.global_frame
            if pose_frame != grid_frame:
                if not self._warned_frame:
                    self._warned_frame = True
                    self.get_logger().info(
                        f'holding: pose is in "{pose_frame}" but the map is in '
                        f'"{grid_frame}"; waiting for localisation')
                self.publish_cmd(0.0, 0.0)
                self.publish_status()
                self.goal_start_t = now
                self.best_dist_t = now
                return
            if self._warned_frame:
                self._warned_frame = False
                self.get_logger().info(f'localised in "{grid_frame}", navigating')
        gx, gy, gyaw = self.waypoints[self.idx]
        # Always measured to the COMMANDED waypoint. goal_eff is only where
        # the path is allowed to end; reporting progress against it would let
        # the node claim success while standing metres away.
        dist = math.hypot(gx - x, gy - y)
        reach_dist = (math.hypot(self.goal_eff[0] - x, self.goal_eff[1] - y)
                      if self.goal_eff is not None else dist)

        if self.state == self.IDLE:
            self.state = self.PLAN
            self.goal_start_t = now
            self.plan_fail = 0

        if self.state == self.PLAN:
            if now - self.last_plan_t < self.min_plan_interval:
                self.publish_cmd(0.0, 0.0)
                self.publish_status(dist)
                return
            self.last_plan_t = now          # advance even when planning fails
            if self.plan(x, y, gx, gy):
                self.state = self.FOLLOW
                self.last_plan_t = now
                self.plan_fail = 0
                self.get_logger().info(
                    f'planned {len(self.path_world)} points to waypoint '
                    f'{self.idx + 1} ({gx:+.2f}, {gy:+.2f})')
            else:
                self.plan_fail += 1
                self.publish_cmd(0.0, 0.0)
                if self.plan_fail == 1 or self.plan_fail % 40 == 0:
                    self.get_logger().warn(
                        f'no path to waypoint {self.idx + 1} '
                        f'({gx:+.2f}, {gy:+.2f}) yet - map may be incomplete')
                if now - self.goal_start_t > self.goal_timeout:
                    self.get_logger().error(
                        f'waypoint {self.idx + 1} unreachable - skipping')
                    self.next_waypoint(now)
                self.publish_status(dist)
                return

        if self.state == self.FOLLOW:
            if dist <= self.xy_tol or reach_dist <= self.xy_tol:
                self.state = self.ALIGN if not math.isnan(gyaw) else self.ARRIVED
                self.arrived_t = now
            else:
                period = (1.0 if self.goal_is_frontier else self.replan_period)
                if now - self.last_plan_t > period:
                    self.last_plan_t = now   # advance even when planning fails
                    self.plan(x, y, gx, gy)
                tgt = self.carrot(x, y)
                if tgt is None:
                    self.state = self.PLAN
                    self.publish_status(dist)
                    return
                heading_err = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - th)

                if abs(heading_err) > self.turn_thresh:
                    # too far off course to drive: rotate in place first
                    self.publish_cmd(0.0, self.clamp_w(self.k_head * heading_err))
                else:
                    v = self.v_max
                    if dist < self.slow_r:
                        v = self.v_app + (self.v_max - self.v_app) * (dist / self.slow_r)
                    v *= max(0.25, math.cos(min(abs(heading_err), math.pi / 2)))
                    self.publish_cmd(min(v, self.v_max),
                                     self.clamp_w(self.k_head * heading_err))
                # progress watchdog: the safety layer may legitimately
                # refuse to close the last few centimetres.
                if dist < self.best_dist - 0.10:
                    self.best_dist = dist
                    self.best_dist_t = now
                elif now - self.best_dist_t > (self.no_progress_timeout *
                                               (3.0 if self.goal_is_frontier else 1.0)):
                    self.get_logger().warn(
                        f'waypoint {self.idx + 1}: no progress for '
                        f'{self.no_progress_timeout:.0f} s at {dist:.2f} m - '
                        f'accepting and moving on')
                    self.next_waypoint(now)
                    self.publish_status(dist)
                    return
                if now - self.goal_start_t > self.goal_timeout:
                    self.get_logger().warn(
                        f'waypoint {self.idx + 1} timed out - skipping')
                    self.next_waypoint(now)
                self.publish_status(dist)
                return

        if self.state == self.ALIGN:
            yaw_err = wrap(gyaw - th)
            if abs(yaw_err) < self.yaw_tol:
                self.state = self.ARRIVED
                self.arrived_t = now
            else:
                self.publish_cmd(0.0, self.clamp_w(self.k_head * yaw_err))
                self.publish_status(dist)
                return

        if self.state == self.ARRIVED:
            self.publish_cmd(0.0, 0.0)
            if now - self.arrived_t >= self.arrive_hold:
                note = ''
                if self.goal_eff is not None and \
                        math.dist(self.goal_eff, (gx, gy)) > self.xy_tol:
                    note = (f'  [goal was snapped '
                            f'{math.dist(self.goal_eff, (gx, gy)):.2f} m out of '
                            f'an obstacle]')
                self.get_logger().info(
                    f'reached waypoint {self.idx + 1}/{len(self.waypoints)} '
                    f'({gx:+.2f}, {gy:+.2f})  error {dist:.3f} m{note}')
                self.next_waypoint(now)
            self.publish_status(dist)
            return

        self.publish_status(dist)

    def next_waypoint(self, now: float):
        self.idx += 1
        self.goal_start_t = now
        self.path_world = []
        self.goal_eff = None
        self.goal_is_frontier = False
        self.warned_unreachable = False
        self.best_dist = float('inf')
        self.best_dist_t = now
        if self.idx >= len(self.waypoints):
            if self.loop:
                self.idx = 0
                self.state = self.IDLE
                self.get_logger().info('looping back to the first waypoint')
            else:
                self.idx = len(self.waypoints) - 1
                self.state = self.DONE
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info('all waypoints reached')
        else:
            self.state = self.IDLE

    # ------------------------------------------------------------ publishing
    def clamp_w(self, w: float) -> float:
        return max(-self.w_max, min(self.w_max, w))

    def publish_cmd(self, v: float, w: float):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.cmd_pub.publish(m)

    def publish_status(self, dist: float = float('nan')):
        n = max(1, len(self.waypoints))
        done = self.idx + (1 if self.state == self.DONE else 0)
        self.state_pub.publish(String(
            data=f'{self.state} wp={min(self.idx + 1, n)}/{n} dist={dist:.2f}'))
        self.prog_pub.publish(Float32(data=float(done) / float(n)))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
