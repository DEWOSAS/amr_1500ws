#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImuAccelTiltTF(Node):
    def __init__(self):
        super().__init__('imu_accel_tilt_tf_node')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_parent = 'dummy_link'  # หรือ camera_link แล้วแต่ setup
        self.frame_child = 'accel_tilt'

        self.create_subscription(
            Imu,
            '/camera/camera/accel/sample',
            self.imu_callback,
            qos
        )
        self.get_logger().info("IMU Accel Tilt TF node started.")

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # สร้างเวกเตอร์จาก accel
        v = np.array([ax, ay, az])
        norm = np.linalg.norm(v)
        if norm < 1e-6:
            self.get_logger().warn("Accel magnitude too small, skipping frame.")
            return
        v /= norm

        # target direction (ขึ้นบนตาม Y-axis)
        target = np.array([0.0, 1.0, 0.0])
        axis = np.cross(v, target)
        angle = math.acos(np.clip(np.dot(v, target), -1.0, 1.0))

        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-6:
            axis = np.array([1.0, 0.0, 0.0])
            angle = 0.0
        else:
            axis /= axis_norm

        # Quaternion
        qx = axis[0] * math.sin(angle / 2)
        qy = axis[1] * math.sin(angle / 2)
        qz = axis[2] * math.sin(angle / 2)
        qw = math.cos(angle / 2)

        # Broadcast TF
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.frame_parent
        tf.child_frame_id = self.frame_child
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = ImuAccelTiltTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
