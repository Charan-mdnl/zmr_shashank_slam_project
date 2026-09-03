#!/usr/bin/env python3
"""Keyboard teleoperation for the ZMR AMR.

Reads single keypresses from the terminal in raw mode and publishes a Twist.
No GUI and no X server: it works over SSH and in a plain TTY.

    w / s      forward / reverse
    a / d      rotate left / right
    q / e      forward-left / forward-right arc
    z / c      reverse-left / reverse-right arc
    space      stop immediately
    - / =      slower / faster (linear)
    [ / ]      slower / faster (angular)
    k          hold last command (latch) on/off
    Ctrl-C     quit

Output goes to /cmd_vel_teleop by default. Run obstacle_avoidance downstream to
turn that into a collision-checked /cmd_vel, or remap this node's output
straight to /cmd_vel to drive without the safety layer.
"""

from __future__ import annotations

import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# key -> (linear scale, angular scale)
BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 1.0),
    'e': (1.0, -1.0),
    'z': (-1.0, 1.0),
    'c': (-1.0, -1.0),
}

BANNER = """
ZMR keyboard teleop  ->  publishing on {topic}

     q  w  e        w/s  forward / reverse
     a  s  d        a/d  rotate left / right
     z     c        q/e/z/c  arcs
                    space  stop
  -/= linear speed  [/] angular speed
  k   latch on/off  Ctrl-C  quit
"""


class TeleopKey(Node):

    def __init__(self):
        super().__init__('zmr_teleop_key')
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('linear_max', 0.80)
        self.declare_parameter('angular_max', 1.50)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('latch', False)
        self.declare_parameter('key_timeout', 0.4)

        p = self.get_parameter
        self.lin = float(p('linear_speed').value)
        self.ang = float(p('angular_speed').value)
        self.lin_max = float(p('linear_max').value)
        self.ang_max = float(p('angular_max').value)
        self.latch = bool(p('latch').value)
        self.key_timeout = float(p('key_timeout').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.topic = self.pub.topic_name

        self._v = 0.0
        self._w = 0.0
        self._last_key_t = 0.0
        self._lock = threading.Lock()
        self._running = True

        self.create_timer(1.0 / float(p('publish_rate').value), self.publish)

    # ------------------------------------------------------------- publishing
    def publish(self):
        with self._lock:
            v, w = self._v, self._w
            # Without latch, a key must be held down; release stops the robot.
            if not self.latch and self.key_timeout > 0.0:
                age = self.get_clock().now().nanoseconds * 1e-9 - self._last_key_t
                if age > self.key_timeout:
                    v, w = 0.0, 0.0
                    self._v, self._w = 0.0, 0.0
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    # ------------------------------------------------------------ key handling
    def handle_key(self, key: str) -> bool:
        """Apply a key. Returns False if the node should quit."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if key == '\x03':  # Ctrl-C
            return False
        with self._lock:
            if key in BINDINGS:
                fv, fw = BINDINGS[key]
                self._v = fv * self.lin
                self._w = fw * self.ang
                self._last_key_t = now
            elif key == ' ':
                self._v = 0.0
                self._w = 0.0
                self._last_key_t = now
            elif key == 'k':
                self.latch = not self.latch
                self.get_logger().info(f'latch {"ON" if self.latch else "OFF"}')
            elif key in ('-', '_'):
                self.lin = max(0.05, self.lin - 0.05)
                self.report()
            elif key in ('=', '+'):
                self.lin = min(self.lin_max, self.lin + 0.05)
                self.report()
            elif key == '[':
                self.ang = max(0.1, self.ang - 0.1)
                self.report()
            elif key == ']':
                self.ang = min(self.ang_max, self.ang + 0.1)
                self.report()
        return True

    def report(self):
        self.get_logger().info(
            f'linear {self.lin:.2f} m/s   angular {self.ang:.2f} rad/s')

    def stop(self):
        self.pub.publish(Twist())


def read_key(timeout: float):
    """Non-blocking single-character read from stdin."""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if r:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()

    interactive = sys.stdin.isatty()
    if not interactive:
        node.get_logger().warn(
            'stdin is not a TTY - no keys will be read. This node needs an '
            'interactive terminal; for scripted motion publish to '
            f'{node.topic} directly.')

    print(BANNER.format(topic=node.topic), flush=True)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    old = None
    try:
        if interactive:
            old = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            if not interactive:
                # nothing to read; just keep the (zero) command publishing
                rclpy.spin_once(node, timeout_sec=0.2)
                continue
            key = read_key(0.1)
            if key is not None and not node.handle_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nteleop stopped', flush=True)


if __name__ == '__main__':
    main()
