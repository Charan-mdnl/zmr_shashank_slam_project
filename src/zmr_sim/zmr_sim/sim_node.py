#!/usr/bin/env python3
"""Headless 2-D simulator for the ZMR AMR.

Publishes exactly the interfaces the real robot exposes, so every downstream
node (SLAM, obstacle avoidance, waypoint navigation) runs unchanged against
either the simulator or the physical machine:

    subscribes   /cmd_vel            geometry_msgs/Twist
    publishes    /scan               sensor_msgs/LaserScan     (frame lidar_link)
                 /odom               nav_msgs/Odometry         (odom -> base_link)
                 /joint_states       sensor_msgs/JointState
                 /ground_truth       nav_msgs/Odometry         (world -> base_link, exact)
                 /clock              rosgraph_msgs/Clock       (when use_sim_time)
    broadcasts   TF odom -> base_link

Geometry defaults are the CAD-verified ZMR values from
zmr_description/config/robot_geometry.yaml. No rendering, no GUI, no Gazebo.
"""

from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from zmr_sim.worlds import World, build


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ZmrSim(Node):

    def __init__(self):
        super().__init__('zmr_sim')

        # ------------------------------------------------------- parameters
        self.declare_parameter('world', 'warehouse')
        self.declare_parameter('world_yaml', '')
        self.declare_parameter('resolution', 0.05)

        # CAD-verified ZMR geometry
        self.declare_parameter('wheel_radius', 0.080)
        self.declare_parameter('wheel_separation', 0.526)
        self.declare_parameter('body_length', 1.05)
        self.declare_parameter('body_width', 0.71)
        # lidar_link resolved in base_link. These are only a fallback: at
        # startup the pose is taken from TF (robot_state_publisher / the URDF)
        # so the simulated sensor can never disagree with the robot model.
        # The URDF puts lidar_link behind base_link and rotated -90 deg,
        # because it hangs off the rotated cad_base_link frame.
        self.declare_parameter('lidar_x', -0.425)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', -math.pi / 2.0)
        self.declare_parameter('lidar_pose_from_tf', True)

        # RPLIDAR A3M1
        self.declare_parameter('scan_samples', 1440)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 25.0)
        self.declare_parameter('range_noise_std', 0.012)

        self.declare_parameter('physics_rate', 100.0)
        self.declare_parameter('max_linear_vel', 0.8)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('max_linear_accel', 0.6)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('cmd_timeout', 0.6)

        # odometry error model: this is a hall/encoder-free robot, so odom drifts
        self.declare_parameter('odom_linear_error', 0.02)   # fraction of distance
        self.declare_parameter('odom_angular_error', 0.03)  # fraction of rotation
        self.declare_parameter('odom_seed', 42)

        self.declare_parameter('initial_x', 2.5)
        self.declare_parameter('initial_y', 2.5)
        self.declare_parameter('initial_yaw', 0.0)

        self.declare_parameter('odom_at_origin', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # The URDF already parents base_link to base_footprint, and a
        # frame may have exactly one parent. Publish odometry onto
        # base_footprint - the convention the Gazebo diff_drive plugin
        # uses - and let robot_state_publisher supply the rest.
        self.declare_parameter('odom_child_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('publish_clock', True)

        p = self.get_parameter
        self.R = float(p('wheel_radius').value)
        self.L = float(p('wheel_separation').value)
        self.body_l = float(p('body_length').value)
        self.body_w = float(p('body_width').value)
        self.lidar_xy = (float(p('lidar_x').value), float(p('lidar_y').value))
        self.lidar_yaw = float(p('lidar_yaw').value)
        self.n_beams = int(p('scan_samples').value)
        self.range_min = float(p('range_min').value)
        self.range_max = float(p('range_max').value)
        self.range_noise = float(p('range_noise_std').value)
        self.v_max = float(p('max_linear_vel').value)
        self.w_max = float(p('max_angular_vel').value)
        self.a_lin = float(p('max_linear_accel').value)
        self.a_ang = float(p('max_angular_accel').value)
        self.cmd_timeout = float(p('cmd_timeout').value)
        self.odom_lin_err = float(p('odom_linear_error').value)
        self.odom_ang_err = float(p('odom_angular_error').value)
        self.odom_frame = str(p('odom_frame').value)
        self.base_frame = str(p('base_frame').value)
        self.odom_child = str(p('odom_child_frame').value)
        self.lidar_frame = str(p('lidar_frame').value)
        self.publish_clock = bool(p('publish_clock').value)

        # ------------------------------------------------------------ world
        world_yaml = str(p('world_yaml').value)
        if world_yaml:
            if not os.path.exists(world_yaml):
                raise FileNotFoundError(f'world_yaml not found: {world_yaml}')
            self.world: World = World.from_map_yaml(world_yaml)
            src = world_yaml
        else:
            self.world = build(str(p('world').value), float(p('resolution').value))
            src = f'built-in "{p("world").value}"'

        # --------------------------------------------------- state (ground truth)
        self.x = float(p('initial_x').value)
        self.y = float(p('initial_y').value)
        self.th = float(p('initial_yaw').value)
        self.v = 0.0
        self.w = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.collided = False

        # Odometry estimate. A real robot starts its odom frame at zero
        # wherever it happens to be; here it is initialised to the true start
        # pose so that odom - and therefore the SLAM map frame - coincides with
        # the world frame. That makes waypoints, world files and the ground
        # truth all share one coordinate system, which is far easier to reason
        # about. Set odom_at_origin:=true for the real-robot convention.
        if bool(p('odom_at_origin').value):
            self.ox, self.oy, self.oth = 0.0, 0.0, 0.0
        else:
            self.ox, self.oy, self.oth = self.x, self.y, self.th
        self.rng = np.random.default_rng(int(p('odom_seed').value))

        if self.world.is_occupied(self.x, self.y):
            self.get_logger().warn(
                f'initial pose ({self.x:.2f}, {self.y:.2f}) is inside an obstacle')

        # ---------------------------------------------- footprint (base_link)
        hl, hw = self.body_l / 2.0, self.body_w / 2.0
        edge = np.linspace(-1.0, 1.0, 9)
        pts = []
        for e in edge:
            pts += [(hl, e * hw), (-hl, e * hw), (e * hl, hw), (e * hl, -hw)]
        self.footprint = np.unique(np.array(pts), axis=0)

        # ------------------------------------------------------------ ros io
        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.gt_pub = self.create_publisher(Odometry, 'ground_truth', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 20)
        self.clock_pub = (self.create_publisher(Clock, '/clock', 10)
                          if self.publish_clock else None)
        self.tf_bc = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_pose_resolved = not bool(p('lidar_pose_from_tf').value)

        self.wl = 0.0
        self.wr = 0.0
        self.last_cmd_t = 0.0
        self.sim_t = 0.0

        self.dt = 1.0 / float(p('physics_rate').value)
        self.create_timer(self.dt, self.step)
        self.scan_period = 1.0 / float(p('scan_rate').value)
        self.next_scan_t = 0.0

        # precomputed beam angles
        self.beam_ang = np.linspace(-math.pi, math.pi, self.n_beams,
                                    endpoint=False, dtype=np.float64)

        self.get_logger().info(
            f'ZMR sim up | world: {src} '
            f'({self.world.width}x{self.world.height} @ {self.world.resolution} m) | '
            f'R={self.R} L={self.L} | lidar at base_link '
            f'({self.lidar_xy[0]:+.3f}, {self.lidar_xy[1]:+.3f})')

    # ------------------------------------------------------------------ time
    def now_msg(self):
        if self.publish_clock:
            return Time(seconds=self.sim_t).to_msg()
        return self.get_clock().now().to_msg()

    # --------------------------------------------------------------- cmd_vel
    def on_cmd(self, msg: Twist):
        self.cmd_v = float(np.clip(msg.linear.x, -self.v_max, self.v_max))
        self.cmd_w = float(np.clip(msg.angular.z, -self.w_max, self.w_max))
        self.last_cmd_t = self.sim_t

    # ------------------------------------------------------------- collision
    def collides(self, x, y, th) -> bool:
        c, s = math.cos(th), math.sin(th)
        xs = x + self.footprint[:, 0] * c - self.footprint[:, 1] * s
        ys = y + self.footprint[:, 0] * s + self.footprint[:, 1] * c
        return self.world.any_occupied(xs, ys)

    # ------------------------------------------------------------ main step
    def step(self):
        self.sim_t += self.dt
        if self.clock_pub is not None:
            self.clock_pub.publish(Clock(clock=Time(seconds=self.sim_t).to_msg()))

        # stop if the command has gone stale, as the real base driver does
        tgt_v, tgt_w = self.cmd_v, self.cmd_w
        if self.sim_t - self.last_cmd_t > self.cmd_timeout:
            tgt_v, tgt_w = 0.0, 0.0

        # acceleration limits
        dv = float(np.clip(tgt_v - self.v, -self.a_lin * self.dt, self.a_lin * self.dt))
        dw = float(np.clip(tgt_w - self.w, -self.a_ang * self.dt, self.a_ang * self.dt))
        self.v += dv
        self.w += dw

        # integrate ground truth (midpoint / second-order form)
        ds = self.v * self.dt
        dth = self.w * self.dt
        nx = self.x + ds * math.cos(self.th + dth * 0.5)
        ny = self.y + ds * math.sin(self.th + dth * 0.5)
        nth = wrap(self.th + dth)

        if self.collides(nx, ny, nth):
            # Translation is blocked. Rotating on the spot may still be legal;
            # if it is, the wheels really did turn, so that rotation must also
            # appear in the odometry. Reporting the ground-truth rotation but
            # zero odometry rotation would inject an error no filter could see.
            self.collided = True
            self.v = 0.0
            if not self.collides(self.x, self.y, nth):
                self.th = nth
                ds = 0.0
            else:
                self.w = 0.0
                ds = 0.0
                dth = 0.0
        else:
            self.collided = False
            self.x, self.y, self.th = nx, ny, nth

        # wheel angles for joint_states
        if abs(self.R) > 1e-9:
            self.wl += (ds - dth * self.L / 2.0) / self.R
            self.wr += (ds + dth * self.L / 2.0) / self.R

        # odometry with drift: scale error grows with distance travelled
        if ds != 0.0 or dth != 0.0:
            eds = ds * (1.0 + self.rng.normal(0.0, self.odom_lin_err))
            edth = dth * (1.0 + self.rng.normal(0.0, self.odom_ang_err))
            edth += abs(ds) * self.rng.normal(0.0, self.odom_ang_err * 0.5)
            self.ox += eds * math.cos(self.oth + edth * 0.5)
            self.oy += eds * math.sin(self.oth + edth * 0.5)
            self.oth = wrap(self.oth + edth)

        stamp = self.now_msg()
        self.publish_odom(stamp)
        self.publish_joints(stamp)
        if self.sim_t >= self.next_scan_t:
            self.next_scan_t = self.sim_t + self.scan_period
            self.publish_scan(stamp)

    # ------------------------------------------------------------ publishing
    def publish_odom(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.odom_child
        t.transform.translation.x = self.ox
        t.transform.translation.y = self.oy
        t.transform.rotation = yaw_to_quat(self.oth)
        self.tf_bc.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.odom_child
        o.pose.pose.position.x = self.ox
        o.pose.pose.position.y = self.oy
        o.pose.pose.orientation = yaw_to_quat(self.oth)
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        stationary = abs(self.v) < 1e-4 and abs(self.w) < 1e-4
        pxy = 1e-6 if stationary else 1e-3
        pyaw = 1e-6 if stationary else 1e-2
        o.pose.covariance[0] = pxy
        o.pose.covariance[7] = pxy
        o.pose.covariance[14] = 1e6
        o.pose.covariance[21] = 1e6
        o.pose.covariance[28] = 1e6
        o.pose.covariance[35] = pyaw
        o.twist.covariance[0] = pxy
        o.twist.covariance[7] = 1e6
        o.twist.covariance[14] = 1e6
        o.twist.covariance[21] = 1e6
        o.twist.covariance[28] = 1e6
        o.twist.covariance[35] = pyaw
        self.odom_pub.publish(o)

        g = Odometry()
        g.header.stamp = stamp
        g.header.frame_id = 'world'
        g.child_frame_id = self.base_frame
        g.pose.pose.position.x = self.x
        g.pose.pose.position.y = self.y
        g.pose.pose.orientation = yaw_to_quat(self.th)
        g.twist.twist.linear.x = self.v
        g.twist.twist.angular.z = self.w
        self.gt_pub.publish(g)

    def publish_joints(self, stamp):
        js = JointState()
        js.header.stamp = stamp
        js.name = ['base_to_left_wheel', 'base_to_right_wheel']
        js.position = [self.wl, self.wr]
        self.js_pub.publish(js)

    def resolve_lidar_pose(self):
        """Take base_link -> lidar_frame from TF so the simulated sensor sits
        exactly where the URDF says it does."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.base_frame, self.lidar_frame, rclpy.time.Time())
        except Exception:
            return
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.lidar_xy = (tr.transform.translation.x, tr.transform.translation.y)
        self.lidar_yaw = yaw
        self.lidar_pose_resolved = True
        self.get_logger().info(
            f'lidar pose taken from TF: base_link -> {self.lidar_frame} = '
            f'({self.lidar_xy[0]:+.3f}, {self.lidar_xy[1]:+.3f}) '
            f'yaw {math.degrees(yaw):+.1f} deg')

    def publish_scan(self, stamp):
        if not self.lidar_pose_resolved:
            self.resolve_lidar_pose()
        c, s = math.cos(self.th), math.sin(self.th)
        lx = self.x + self.lidar_xy[0] * c - self.lidar_xy[1] * s
        ly = self.y + self.lidar_xy[0] * s + self.lidar_xy[1] * c
        ranges = self.raycast(lx, ly, wrap(self.th + self.lidar_yaw))

        m = LaserScan()
        m.header.stamp = stamp
        m.header.frame_id = self.lidar_frame
        m.angle_min = float(self.beam_ang[0])
        m.angle_max = float(self.beam_ang[-1])
        m.angle_increment = float(self.beam_ang[1] - self.beam_ang[0])
        m.time_increment = 0.0
        m.scan_time = self.scan_period
        m.range_min = self.range_min
        m.range_max = self.range_max
        m.ranges = ranges.tolist()
        self.scan_pub.publish(m)

    def raycast(self, lx: float, ly: float, lth: float) -> np.ndarray:
        """Vectorised DDA ray cast of all beams against the occupancy grid."""
        res = self.world.resolution
        occ = self.world.occupied
        H, W = occ.shape
        n = self.n_beams
        ang = self.beam_ang + lth
        ca, sa = np.cos(ang), np.sin(ang)

        cx = (lx - self.world.origin[0]) / res
        cy = (ly - self.world.origin[1]) / res

        out = np.full(n, np.inf, dtype=np.float64)
        first = max(1, int(self.range_min / res))
        last = int(self.range_max / res)
        active = np.ones(n, dtype=bool)

        # march all beams together, one cell at a time
        for k in range(first, last + 1):
            if not active.any():
                break
            idx = np.nonzero(active)[0]
            xs = (cx + ca[idx] * k).astype(np.int64)
            ys = (cy + sa[idx] * k).astype(np.int64)
            inside = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)
            hit = np.zeros(idx.size, dtype=bool)
            hit[inside] = occ[ys[inside], xs[inside]]
            # a beam leaving the grid terminates with no return
            done_out = ~inside
            if hit.any():
                out[idx[hit]] = k * res
                active[idx[hit]] = False
            if done_out.any():
                active[idx[done_out]] = False

        finite = np.isfinite(out)
        if self.range_noise > 0.0 and finite.any():
            noisy = out[finite] + self.rng.normal(0.0, self.range_noise,
                                                  int(finite.sum()))
            # Clip only the real returns. Clipping the whole array would turn
            # every no-return (inf) beam into exactly range_max, which every
            # consumer then accepts as a genuine hit - painting a phantom wall
            # at 25 m through open floor.
            out[finite] = np.clip(noisy, self.range_min, self.range_max)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ZmrSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
