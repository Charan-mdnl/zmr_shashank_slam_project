#!/usr/bin/env python3
"""Record the robot's driven path and report progress, headlessly.

Logs the pose from TF (and the exact pose from /ground_truth when running in
simulation) to a CSV, and prints a periodic one-line summary. This is how you
verify a run finished correctly on a machine with no display.

    ros2 run zmr_tools path_recorder --ros-args -p output:=/tmp/run.csv
"""

from __future__ import annotations

import csv
import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class PathRecorder(Node):

    def __init__(self):
        super().__init__('zmr_path_recorder')
        self.declare_parameter('output', '/tmp/zmr_path.csv')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('fallback_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('report_period', 5.0)

        self.global_frame = str(self.get_parameter('global_frame').value)
        self.fallback_frame = str(self.get_parameter('fallback_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gt = None
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, 10)

        path = str(self.get_parameter('output').value)
        d = os.path.dirname(os.path.abspath(path))
        if d:
            os.makedirs(d, exist_ok=True)
        self.fh = open(path, 'w', newline='')
        self.csv = csv.writer(self.fh)
        self.csv.writerow(['t', 'frame', 'x', 'y', 'yaw', 'gt_x', 'gt_y', 'gt_yaw'])

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.dist = 0.0
        self.last_xy = None
        self.n = 0
        self.last_report = 0.0
        self.report_period = float(self.get_parameter('report_period').value)

        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.step)
        self.get_logger().info(f'recording path to {path}')

    def on_gt(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.gt = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                   math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        pose, frame = None, ''
        for f in (self.global_frame, self.fallback_frame):
            try:
                tr = self.tf_buffer.lookup_transform(
                    f, self.base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            q = tr.transform.rotation
            pose = (tr.transform.translation.x, tr.transform.translation.y,
                    math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            frame = f
            break
        if pose is None:
            return

        g = self.gt if self.gt else (float('nan'),) * 3
        self.csv.writerow([f'{now - self.t0:.3f}', frame,
                           f'{pose[0]:.4f}', f'{pose[1]:.4f}', f'{pose[2]:.4f}',
                           f'{g[0]:.4f}', f'{g[1]:.4f}', f'{g[2]:.4f}'])
        self.fh.flush()
        self.n += 1

        track = self.gt if self.gt else pose
        if self.last_xy is not None:
            self.dist += math.dist(track[:2], self.last_xy)
        self.last_xy = track[:2]

        if now - self.last_report >= self.report_period:
            self.last_report = now
            err = ''
            if self.gt:
                err = (f'  slam_err={math.dist(pose[:2], self.gt[:2]):.3f} m')
            self.get_logger().info(
                f't={now - self.t0:6.1f}s  frame={frame}  '
                f'pose=({pose[0]:+.2f}, {pose[1]:+.2f}, {math.degrees(pose[2]):+6.1f} deg)  '
                f'travelled={self.dist:.2f} m{err}')

    def destroy_node(self):
        try:
            self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'recorded {node.n} samples, {node.dist:.2f} m travelled')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
