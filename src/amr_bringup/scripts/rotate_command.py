#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

class RotationController(Node):
    def __init__(self):
        super().__init__('rotation_controller')

        # Parameters
        self.target_yaw = 1.57  # rad
        self.tolerance = 0.01   # rad
        self.max_angular_z = 0.5  # rad/s
        self.Kp = 1.0  # Proportional gain

        # Publisher & Subscriber
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)

        self.get_logger().info("P-Control RotationController initialized.")

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(q)

        # Calculate error and normalize
        error = self.normalize_angle(self.target_yaw - yaw)

        self.get_logger().info(f'Current Yaw: {yaw:.3f} | Error: {error:.3f}')

        if abs(error) < self.tolerance:
            self.publish_velocity(0.0)
            self.get_logger().info("Yaw target reached. Stopping.")
        else:
            angular_z = self.Kp * error
            # Clamp angular velocity
            angular_z = max(min(angular_z, self.max_angular_z), -self.max_angular_z)
            self.publish_velocity(angular_z)

    def publish_velocity(self, angular_z):
        twist = Twist()
        twist.angular.z = angular_z
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RotationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
