#!/usr/bin/env python3
"""Configure and activate managed (lifecycle) nodes.

slam_toolbox's nodes are rclcpp_lifecycle::LifecycleNode. Their main() only
spins them - it never transitions them - so on their own they sit in the
`unconfigured` state: no subscriptions, no map, and no error message either.
Normally nav2_lifecycle_manager does the transitions; this is a small
stand-alone replacement for workspaces that do not carry the nav2 stack.

    ros2 run zmr_tools lifecycle_manager --ros-args \
        -p node_names:="['slam_toolbox']"
"""

from __future__ import annotations

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


TRANSITIONS = {
    'configure': Transition.TRANSITION_CONFIGURE,
    'activate': Transition.TRANSITION_ACTIVATE,
    'deactivate': Transition.TRANSITION_DEACTIVATE,
    'cleanup': Transition.TRANSITION_CLEANUP,
    'shutdown': Transition.TRANSITION_ACTIVE_SHUTDOWN,
}

STATE_NAMES = {
    1: 'unconfigured', 2: 'inactive', 3: 'active', 4: 'finalized',
}


class LifecycleManager(Node):

    def __init__(self):
        super().__init__('zmr_lifecycle_manager')
        self.declare_parameter('node_names', ['slam_toolbox'])
        self.declare_parameter('autostart', True)
        self.declare_parameter('service_timeout', 30.0)
        self.declare_parameter('attempt_period', 1.0)

        self.node_names = list(self.get_parameter('node_names').value)
        self.timeout = float(self.get_parameter('service_timeout').value)
        self.pending = list(self.node_names)
        self.elapsed = 0.0

        self.period = float(self.get_parameter('attempt_period').value)
        self.autostart = bool(self.get_parameter('autostart').value)

    # ------------------------------------------------------------- helpers
    def call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def state_of(self, name: str):
        cli = self.create_client(GetState, f'/{name}/get_state')
        if not cli.wait_for_service(timeout_sec=0.5):
            self.destroy_client(cli)
            return None
        res = self.call(cli, GetState.Request())
        self.destroy_client(cli)
        return res.current_state.id if res else None

    def transition(self, name: str, label: str) -> bool:
        cli = self.create_client(ChangeState, f'/{name}/change_state')
        if not cli.wait_for_service(timeout_sec=2.0):
            self.destroy_client(cli)
            return False
        req = ChangeState.Request()
        req.transition.id = TRANSITIONS[label]
        res = self.call(cli, req)
        self.destroy_client(cli)
        return bool(res and res.success)

    # ------------------------------------------------------------- bringup
    def bringup(self) -> bool:
        """Drive every managed node to `active`. Blocking; call before spin()."""
        if not self.autostart:
            return True
        self.get_logger().info(
            f'bringing up managed nodes: {", ".join(self.node_names)}')
        while rclpy.ok() and self.pending and self.elapsed < self.timeout:
            still = []
            for name in self.pending:
                state = self.state_of(name)
                if state is None:
                    still.append(name)          # node not up yet
                    continue
                if state == 3:                  # already active
                    self.get_logger().info(f'{name}: active')
                    continue
                if state == 1:                  # unconfigured -> configure
                    if self.transition(name, 'configure'):
                        self.get_logger().info(f'{name}: configured')
                        state = 2
                    else:
                        self.get_logger().warn(f'{name}: configure failed, retrying')
                        still.append(name)
                        continue
                if state == 2:                  # inactive -> activate
                    if self.transition(name, 'activate'):
                        self.get_logger().info(f'{name}: active')
                        continue
                    self.get_logger().warn(f'{name}: activate failed, retrying')
                still.append(name)

            self.pending = still
            if not self.pending:
                self.get_logger().info('all managed nodes are active')
                return True
            self.elapsed += self.period
            rclpy.spin_once(self, timeout_sec=self.period)

        self.get_logger().error(
            f'gave up after {self.elapsed:.0f} s; still inactive: '
            f'{", ".join(self.pending)}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManager()
    try:
        node.bringup()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
