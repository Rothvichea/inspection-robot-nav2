#!/usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


# ── Spawn position must match gazebo.launch.py arguments ──
SPAWN_X   = 0.0
SPAWN_Y   = 0.0
SPAWN_YAW = 0.0   # radians — 0 = facing +X (forward)


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.attempts = 0
        self.max_attempts = 10
        # Try every 2 s; AMCL needs to be active to receive the pose
        self.timer = self.create_timer(2.0, self.publish_initial_pose)
        self.get_logger().info(
            f'Initial pose publisher started — target ({SPAWN_X}, {SPAWN_Y})'
        )

    def publish_initial_pose(self):
        if self.attempts >= self.max_attempts:
            self.get_logger().info('Initial pose published successfully, shutting down.')
            self.timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # t=0 → AMCL uses latest TF
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x    = SPAWN_X
        msg.pose.pose.position.y    = SPAWN_Y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0  # yaw = 0

        # Standard AMCL covariance for a known starting position
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.06853891909122467

        self.publisher.publish(msg)
        self.attempts += 1
        self.get_logger().info(
            f'Initial pose sent (attempt {self.attempts}/{self.max_attempts}) '
            f'→ x={SPAWN_X}, y={SPAWN_Y}, yaw={SPAWN_YAW}'
        )


def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
