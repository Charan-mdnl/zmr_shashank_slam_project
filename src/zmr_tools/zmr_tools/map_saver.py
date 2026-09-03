#!/usr/bin/env python3
"""Save the /map occupancy grid as a ROS map pair (.pgm + .yaml).

A drop-in replacement for `ros2 run nav2_map_server map_saver_cli` for sites
where the nav2 stack is not installed. Subscribes with TRANSIENT_LOCAL
durability so it picks up the latched map that slam_toolbox publishes.

    ros2 run zmr_tools map_saver --ros-args -p filename:=/path/to/my_map
    ros2 service call /map_saver/save std_srvs/srv/Trigger
"""

from __future__ import annotations

import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger


def write_pgm(path: str, img: np.ndarray) -> None:
    h, w = img.shape
    with open(path, 'wb') as fh:
        fh.write(b'P5\n%d %d\n255\n' % (w, h))
        fh.write(img.astype(np.uint8).tobytes())


class MapSaver(Node):

    def __init__(self):
        super().__init__('zmr_map_saver')
        self.declare_parameter('filename', 'map')
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.196)
        self.declare_parameter('save_on_receive', False)
        self.declare_parameter('exit_after_save', False)

        self.grid: OccupancyGrid | None = None
        self.saved = False

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'map', self.on_map, qos)
        self.create_service(Trigger, 'map_saver/save', self.on_save)
        self.get_logger().info('map_saver ready; waiting for /map')

    def on_map(self, msg: OccupancyGrid):
        self.grid = msg
        if bool(self.get_parameter('save_on_receive').value) and not self.saved:
            ok, detail = self.save()
            self.get_logger().info(detail)
            if ok and bool(self.get_parameter('exit_after_save').value):
                raise SystemExit(0)

    def on_save(self, _req, resp):
        ok, detail = self.save()
        resp.success = ok
        resp.message = detail
        self.get_logger().info(detail)
        return resp

    def save(self):
        if self.grid is None:
            return False, 'no map received yet'
        g = self.grid
        base = str(self.get_parameter('filename').value)
        d = os.path.dirname(os.path.abspath(base))
        if d:
            os.makedirs(d, exist_ok=True)

        data = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        occ_t = float(self.get_parameter('occupied_thresh').value) * 100.0
        free_t = float(self.get_parameter('free_thresh').value) * 100.0

        img = np.full(data.shape, 205, dtype=np.uint8)   # unknown
        img[(data >= 0) & (data <= free_t)] = 254        # free
        img[data >= occ_t] = 0                           # occupied
        img = np.flipud(img)                             # ROS maps are y-up

        write_pgm(base + '.pgm', img)
        meta = {
            'image': os.path.basename(base) + '.pgm',
            'mode': 'trinary',
            'resolution': float(g.info.resolution),
            'origin': [float(g.info.origin.position.x),
                       float(g.info.origin.position.y), 0.0],
            'negate': 0,
            'occupied_thresh': float(self.get_parameter('occupied_thresh').value),
            'free_thresh': float(self.get_parameter('free_thresh').value),
        }
        with open(base + '.yaml', 'w') as fh:
            yaml.safe_dump(meta, fh, default_flow_style=False, sort_keys=False)

        self.saved = True
        known = int(((data >= 0)).sum())
        return True, (f'saved {base}.pgm + {base}.yaml  '
                      f'({g.info.width}x{g.info.height} @ {g.info.resolution:.3f} m, '
                      f'{known} known cells)')


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
