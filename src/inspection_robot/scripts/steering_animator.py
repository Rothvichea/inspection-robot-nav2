#!/usr/bin/env python3
"""
steering_animator.py
Subscribes to /cmd_vel and publishes the 4WS steering joint positions
to /joint_states so robot_state_publisher renders the wheels steering
correctly in RViz.

Actual body motion is handled by libgazebo_ros_planar_move.
This node only provides the visual steering animation.

Low-pass filter (alpha=0.15) smooths rapid cmd_vel changes so wheels
don't flick left-right during MPPI micro-corrections.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

WHEELBASE   = 0.50   # front-to-rear wheel centre distance [m]
TRACK_WIDTH = 0.53   # left-to-right wheel centre distance [m]
MAX_STEER   = math.pi / 2.0  # 90° hard limit

# Low-pass filter strength: 0.0 = no update, 1.0 = instant snap
# 0.35 balances smooth Nav2 anti-oscillation with responsive teleop steering
ALPHA = 0.35


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class SteeringAnimator(Node):

    def __init__(self):
        super().__init__('steering_animator')

        self._js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Empty, 'steering_home', self._on_home, 10)

        # Filtered current steering angles
        self._fl = self._fr = self._rl = self._rr = 0.0

        self._publish(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('Steering animator started.')

    def _on_cmd_vel(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        speed = math.hypot(vx, vy)

        # ── Pure in-place rotation ──────────────────────────────────────
        if speed < 0.01 and abs(wz) > 0.01:
            angle = math.pi / 4.0
            target_fl, target_fr = angle, -angle
            target_rl, target_rr = -angle, angle
        # ── Translation (with optional rotation) ────────────────────────
        elif speed > 0.01:
            heading = math.atan2(vy, vx)
            steer = clamp(heading, -MAX_STEER, MAX_STEER)

            if abs(wz) > 0.05:
                bias = clamp(wz * 0.15, -0.3, 0.3)
                target_fl = target_fr = clamp(steer + bias, -MAX_STEER, MAX_STEER)
                target_rl = target_rr = clamp(steer - bias, -MAX_STEER, MAX_STEER)
            else:
                target_fl = target_fr = target_rl = target_rr = steer
        else:
            target_fl = target_fr = target_rl = target_rr = 0.0

        # Low-pass filter — smooth out rapid MPPI micro-corrections
        self._fl += ALPHA * (target_fl - self._fl)
        self._fr += ALPHA * (target_fr - self._fr)
        self._rl += ALPHA * (target_rl - self._rl)
        self._rr += ALPHA * (target_rr - self._rr)

        self._publish(self._fl, self._fr, self._rl, self._rr)

    def _on_home(self, _: Empty):
        """Reset all steering joints to 0° (straight) without stopping the body."""
        self._fl = self._fr = self._rl = self._rr = 0.0
        self._publish(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('Wheels homed to straight.')

    def _publish(self, fl: float, fr: float, rl: float, rr: float):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['steering_fl_joint', 'steering_fr_joint',
                       'steering_rl_joint', 'steering_rr_joint']
        js.position = [fl, fr, rl, rr]
        self._js_pub.publish(js)


def main():
    rclpy.init()
    node = SteeringAnimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
