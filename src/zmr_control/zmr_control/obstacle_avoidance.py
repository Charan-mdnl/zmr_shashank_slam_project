#!/usr/bin/env python3
"""LiDAR obstacle avoidance and command arbitration for the ZMR AMR.

This node is the last thing between any velocity command and the wheels. It

  1. arbitrates between a teleop command and an autonomous command
     (teleop always wins while it is fresh),
  2. rejects any command that would drive the robot's footprint into something
     the LiDAR can see, and
  3. picks the nearest safe alternative instead of simply refusing.

The search is a small dynamic-window: candidate (v, w) pairs reachable within
one acceleration step are forward-simulated as arcs, each arc is checked
against the live scan using the real rectangular footprint, and the surviving
candidate closest to what was asked for wins. If nothing survives, the robot
stops.

Scan points are transformed from the LiDAR frame into base_link via TF, which
matters on this robot: lidar_link sits 0.425 m from base_link, so an obstacle
"1 m ahead of the sensor" is not 1 m ahead of the robot.

    subscribes  /scan  /cmd_vel_teleop  /cmd_vel_auto
    publishes   /cmd_vel  /safety/state  /safety/min_clearance
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('zmr_obstacle_avoidance')

        # ---- footprint (CAD-verified ZMR body) --------------------------
        self.declare_parameter('body_length', 1.05)
        self.declare_parameter('body_width', 0.71)
        self.declare_parameter('safety_margin', 0.12)
        self.declare_parameter('hard_margin', 0.02)
        self.declare_parameter('blocked_unlatch_time', 3.0)
        self.declare_parameter('slow_clearance', 0.55)

        # ---- limits ------------------------------------------------------
        self.declare_parameter('max_linear_vel', 0.60)
        self.declare_parameter('max_angular_vel', 1.00)
        self.declare_parameter('max_linear_accel', 0.50)
        self.declare_parameter('max_angular_accel', 1.50)

        # ---- dynamic window ----------------------------------------------
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('horizon', 1.6)          # s of look-ahead
        self.declare_parameter('horizon_steps', 8)
        self.declare_parameter('v_samples', 7)
        self.declare_parameter('w_samples', 21)
        self.declare_parameter('max_scan_points', 360)

        # ---- behaviour ----------------------------------------------------
        self.declare_parameter('allow_reverse_escape', True)
        self.declare_parameter('teleop_priority_timeout', 0.5)
        self.declare_parameter('input_timeout', 1.0)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('base_frame', 'base_link')
        # fallback LiDAR offset used only if TF is unavailable
        self.declare_parameter('lidar_x_fallback', -0.425)
        self.declare_parameter('lidar_y_fallback', 0.0)
        # Must match the URDF: lidar_link hangs off the rotated
        # cad_base_link, so it resolves to yaw -90 deg in base_link.
        self.declare_parameter('lidar_yaw_fallback', -math.pi / 2.0)

        p = self.get_parameter
        self.hl = float(p('body_length').value) / 2.0
        self.hw = float(p('body_width').value) / 2.0
        self.margin = float(p('safety_margin').value)
        self.hard_margin = float(p('hard_margin').value)
        self.blocked_unlatch = float(p('blocked_unlatch_time').value)
        self.slow_clearance = float(p('slow_clearance').value)
        self.v_max = float(p('max_linear_vel').value)
        self.w_max = float(p('max_angular_vel').value)
        self.a_lin = float(p('max_linear_accel').value)
        self.a_ang = float(p('max_angular_accel').value)
        self.horizon = float(p('horizon').value)
        self.n_steps = int(p('horizon_steps').value)
        self.n_v = int(p('v_samples').value)
        self.n_w = int(p('w_samples').value)
        self.max_pts = int(p('max_scan_points').value)
        self.allow_reverse = bool(p('allow_reverse_escape').value)
        self.teleop_timeout = float(p('teleop_priority_timeout').value)
        self.input_timeout = float(p('input_timeout').value)
        self.scan_timeout = float(p('scan_timeout').value)
        self.base_frame = str(p('base_frame').value)

        # ---- state --------------------------------------------------------
        self.pts = np.zeros((0, 2))          # obstacles in base_link
        self.scan_stamp = None
        self.teleop = (0.0, 0.0)
        self.teleop_t = -1e9
        self.auto = (0.0, 0.0)
        self.auto_t = -1e9
        self.v_cur = 0.0
        self.w_cur = 0.0
        self._warned_tf = False
        self.blocked_since = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, 'scan', self.on_scan, sensor_qos)
        self.create_subscription(Twist, 'cmd_vel_teleop', self.on_teleop, 10)
        self.create_subscription(Twist, 'cmd_vel_auto', self.on_auto, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'safety/state', 10)
        self.clear_pub = self.create_publisher(Float32, 'safety/min_clearance', 10)

        self.dt = 1.0 / float(p('control_rate').value)
        self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f'obstacle avoidance up | footprint {2*self.hl:.2f}x{2*self.hw:.2f} m '
            f'+{self.margin:.2f} m margin | horizon {self.horizon:.1f} s')

    # ------------------------------------------------------------- callbacks
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_teleop(self, m: Twist):
        self.teleop = (float(m.linear.x), float(m.angular.z))
        self.teleop_t = self.now()

    def on_auto(self, m: Twist):
        self.auto = (float(m.linear.x), float(m.angular.z))
        self.auto_t = self.now()

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        ang = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        good = np.isfinite(r) & (r >= max(msg.range_min, 1e-3)) & (r <= msg.range_max)
        if not np.any(good):
            self.pts = np.zeros((0, 2))
            self.scan_stamp = self.now()
            return
        r, ang = r[good], ang[good]

        # points in the LiDAR frame
        lx = r * np.cos(ang)
        ly = r * np.sin(ang)

        # LiDAR frame -> base_link. TF is authoritative; the parameters are a
        # fallback so the node still runs if the URDF is not being published.
        tx, ty, tyaw = self.lidar_to_base(msg.header.frame_id)
        c, s = math.cos(tyaw), math.sin(tyaw)
        bx = tx + lx * c - ly * s
        by = ty + lx * s + ly * c

        pts = np.column_stack((bx, by))
        if pts.shape[0] > self.max_pts:
            step = int(math.ceil(pts.shape[0] / self.max_pts))
            pts = pts[::step]
        self.pts = pts
        self.scan_stamp = self.now()

    def lidar_to_base(self, frame_id: str):
        """Return (x, y, yaw) of the LiDAR frame expressed in base_link."""
        if frame_id and frame_id != self.base_frame:
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.base_frame, frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
                q = tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return (tr.transform.translation.x, tr.transform.translation.y, yaw)
            except Exception:
                if not self._warned_tf:
                    self.get_logger().warn(
                        f'no TF {self.base_frame} <- {frame_id}; using fallback '
                        'lidar offset parameters')
                    self._warned_tf = True
        return (float(self.get_parameter('lidar_x_fallback').value),
                float(self.get_parameter('lidar_y_fallback').value),
                float(self.get_parameter('lidar_yaw_fallback').value))

    # ------------------------------------------------------------ collision
    def arc_is_clear(self, v: float, w: float, margin: float = None):
        """Forward-simulate an arc; return (clear, min_clearance).

        `margin` defaults to the comfort buffer. Escape manoeuvres pass a hard
        bound instead: once the robot is already inside the comfort buffer, a
        buffer-based test rejects every arc - including the ones that would
        get it out - and the robot freezes.
        """
        if margin is None:
            margin = self.margin
        if self.pts.shape[0] == 0:
            return True, float('inf')

        ts = np.linspace(self.horizon / self.n_steps, self.horizon, self.n_steps)
        if abs(w) < 1e-6:
            xs = v * ts
            ys = np.zeros_like(ts)
            ths = np.zeros_like(ts)
        else:
            ths = w * ts
            xs = (v / w) * np.sin(ths)
            ys = (v / w) * (1.0 - np.cos(ths))

        P = self.pts                                   # (N, 2)
        # obstacle points expressed in the robot frame at each future pose
        dx = P[None, :, 0] - xs[:, None]
        dy = P[None, :, 1] - ys[:, None]
        c = np.cos(-ths)[:, None]
        s = np.sin(-ths)[:, None]
        lx = dx * c - dy * s
        ly = dx * s + dy * c

        # signed distance outside the rectangle, per point per step
        ox = np.abs(lx) - self.hl
        oy = np.abs(ly) - self.hw
        outside = np.hypot(np.maximum(ox, 0.0), np.maximum(oy, 0.0))
        inside = np.minimum(np.maximum(ox, oy), 0.0)
        dist = outside + inside                        # <=0 means inside body

        min_d = float(dist.min())
        return (min_d > margin), min_d

    # ------------------------------------------------------------- main step
    def step(self):
        now = self.now()

        scan_ok = (self.scan_stamp is not None
                   and (now - self.scan_stamp) < self.scan_timeout)

        # ---- arbitrate ---------------------------------------------------
        if now - self.teleop_t < self.teleop_timeout:
            req_v, req_w = self.teleop
            source = 'teleop'
        elif now - self.auto_t < self.input_timeout:
            req_v, req_w = self.auto
            source = 'auto'
        else:
            req_v, req_w = 0.0, 0.0
            source = 'idle'

        req_v = float(np.clip(req_v, -self.v_max, self.v_max))
        req_w = float(np.clip(req_w, -self.w_max, self.w_max))

        if not scan_ok:
            # No usable scan: refuse to drive. Rotation in place is still
            # allowed so the operator can recover, but translation is not.
            out_v, out_w = 0.0, float(np.clip(req_w, -0.4, 0.4))
            self.blocked_since = None
            self.emit(out_v, out_w, 'NO_SCAN', float('nan'))
            return

        if abs(req_v) < 1e-3 and abs(req_w) < 1e-3:
            self.blocked_since = None
            self.emit(0.0, 0.0, 'IDLE' if source == 'idle' else 'STOP',
                      self.clearance_ahead())
            return

        # ---- dynamic window around the current velocity -------------------
        dv = self.a_lin * self.dt * 4.0
        dw = self.a_ang * self.dt * 4.0
        v_lo = max(-self.v_max, min(req_v, self.v_cur - dv))
        v_hi = min(self.v_max, max(req_v, self.v_cur + dv))
        w_lo = max(-self.w_max, min(req_w, self.w_cur - dw))
        w_hi = min(self.w_max, max(req_w, self.w_cur + dw))

        v_cands = np.unique(np.concatenate((
            np.linspace(v_lo, v_hi, self.n_v), [req_v, 0.0])))
        w_cands = np.unique(np.concatenate((
            np.linspace(w_lo, w_hi, self.n_w), [req_w, 0.0])))
        # never propose driving the opposite way to the request
        if req_v > 0:
            v_cands = v_cands[v_cands >= 0.0]
        elif req_v < 0:
            v_cands = v_cands[v_cands <= 0.0]

        best = None
        best_score = -1e18
        best_clear = -1e9
        for v in v_cands:
            for w in w_cands:
                clear, min_d = self.arc_is_clear(float(v), float(w))
                if not clear:
                    continue
                # Follow the request; clearance only breaks ties. Weighting
                # clearance heavily makes the robot abandon the planned route
                # and drift into open space, which then strands it.
                score = (-6.0 * abs(v - req_v)
                         - 3.0 * abs(w - req_w)
                         + 0.25 * min(min_d, 1.0))
                if score > best_score:
                    best_score = score
                    best = (float(v), float(w))
                    best_clear = min_d

        if best is None:
            if self.blocked_since is None:
                self.blocked_since = now
            stuck_for = now - self.blocked_since

            # Escapes are judged against actual collision (hard_margin), not
            # the comfort buffer, and we take whichever legal manoeuvre buys
            # the most clearance rather than the first one that merely fits.
            here = self.clearance_ahead()
            # Rank legal escapes on merit - do NOT use `here` as an admission
            # threshold. Legality is decided by hard_margin inside
            # arc_is_clear; min_d only chooses between candidates. Seeding with
            # `here` made the gate unsatisfiable whenever the closest obstacle
            # is abeam, because reversing leaves its lateral distance exactly
            # unchanged, so min_d == here and every legal escape was discarded.
            # The node then emitted (0,0), the scan never changed, and BLOCKED
            # became a permanent fixed point.
            best_escape, best_gain = None, -float('inf')
            candidates = []
            if self.allow_reverse:
                candidates += [(-0.12, 0.0), (-0.20, 0.0),
                               (-0.12, 0.4), (-0.12, -0.4)]
            candidates += [(0.0, 0.6), (0.0, -0.6), (0.0, 0.9), (0.0, -0.9)]
            for ev, ew in candidates:
                ok, min_d = self.arc_is_clear(ev, ew, margin=self.hard_margin)
                if ok and min_d > best_gain + 1e-3:
                    best_gain, best_escape = min_d, (ev, ew)
            if best_escape is not None:
                tag = 'ESCAPE_REVERSE' if best_escape[0] < 0.0 else 'ESCAPE_ROTATE'
                self.emit(best_escape[0], best_escape[1], tag, best_gain)
                return
            # Nothing is legal even at hard_margin. Emitting (0,0) forever
            # would leave the scan unchanged and latch this branch, so after a
            # grace period force a slow reverse: being wedged is worse than
            # nudging backwards against a conservative margin.
            if stuck_for > self.blocked_unlatch:
                self.emit(-0.08, 0.0, 'ESCAPE_FORCED', here)
                return
            self.emit(0.0, 0.0, 'BLOCKED', here)
            return

        # Taper speed as clearance shrinks. The robot's footprint half-
        # diagonal is ~0.63 m, so it needs roughly that much room to rotate on
        # the spot; approaching a wall at full speed leaves it unable to turn.
        v_out, w_out = best
        if best_clear < self.slow_clearance:
            scale = max(0.25, best_clear / self.slow_clearance)
            v_out *= scale
        state = 'CLEAR'
        if abs(v_out - req_v) > 0.02 or abs(w_out - req_w) > 0.05:
            state = 'AVOIDING'
        self.blocked_since = None
        self.emit(v_out, w_out, state, best_clear)

    def clearance_ahead(self) -> float:
        _, d = self.arc_is_clear(0.0, 0.0)
        return d

    def emit(self, v: float, w: float, state: str, clearance: float):
        # respect acceleration limits on the way out
        v = float(np.clip(v, self.v_cur - self.a_lin * self.dt,
                          self.v_cur + self.a_lin * self.dt))
        w = float(np.clip(w, self.w_cur - self.a_ang * self.dt,
                          self.w_cur + self.a_ang * self.dt))
        self.v_cur, self.w_cur = v, w

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)
        self.state_pub.publish(String(data=state))
        self.clear_pub.publish(Float32(data=float(clearance)))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
