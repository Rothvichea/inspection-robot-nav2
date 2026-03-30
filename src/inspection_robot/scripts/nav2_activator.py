#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time
import threading


class Nav2Activator(Node):

    def __init__(self):
        super().__init__('nav2_activator')

        # Correct order — dependencies first
        self.nodes_to_activate = [
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'waypoint_follower',
            'velocity_smoother',
            'bt_navigator',
        ]

        self.timer = self.create_timer(2.0, self.activate_all)
        self.done  = False
        self.get_logger().info('Nav2 activator started.')

    def _change_state(self, node_name, transition_id):
        client = self.create_client(
            ChangeState,
            f'/{node_name}/change_state'
        )
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(f'Service not available: {node_name}')
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)

        # Poll the future from this thread — the main spin() processes the response
        timeout = 30.0
        start   = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self.get_logger().warn(f'{node_name} transition {transition_id} timed out')
                return False
            time.sleep(0.05)

        if future.result() and future.result().success:
            self.get_logger().info(f'{node_name} → transition {transition_id} OK')
            return True
        else:
            self.get_logger().warn(f'{node_name} → transition {transition_id} FAILED (may already be active)')
            return False

    def _do_activate(self):
        self.get_logger().info('Activating Nav2 nodes...')
        failed = []
        for node_name in self.nodes_to_activate:
            self._change_state(node_name, Transition.TRANSITION_CONFIGURE)
            time.sleep(0.5)
            ok = self._change_state(node_name, Transition.TRANSITION_ACTIVATE)
            time.sleep(0.5)
            if not ok:
                failed.append(node_name)

        # Retry any nodes that failed (bt_navigator needs controller_server active first)
        if failed:
            self.get_logger().info(f'Retrying failed nodes: {failed}')
            time.sleep(2.0)
            for node_name in failed:
                self._change_state(node_name, Transition.TRANSITION_CONFIGURE)
                time.sleep(0.5)
                self._change_state(node_name, Transition.TRANSITION_ACTIVATE)
                time.sleep(0.5)

        self.get_logger().info('Nav2 activation complete!')

    def activate_all(self):
        if self.done:
            return
        self.done = True
        self.timer.cancel()
        # Run in a separate thread so we don't block the executor
        threading.Thread(target=self._do_activate, daemon=True).start()


def main():
    rclpy.init()
    node = Nav2Activator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
