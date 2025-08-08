#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImuTiltCalibrator(Node):
    def __init__(self):
        super().__init__('imu_tilt_calibrator')
        # QoS ให้เป็น BEST_EFFORT
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # เก็บผลรวมและนับตัวอย่าง
        self.sum_ax = self.sum_ay = self.sum_az = 0.0
        self.count = 0
        self.max_samples = 1000

        print(f">> Started, waiting for {self.max_samples} IMU samples...")
        self.create_subscription(
            Imu,
            '/camera/camera/accel/sample',
            self.imu_callback,
            qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg: Imu):
        if self.count < self.max_samples:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            self.sum_ax += ax
            self.sum_ay += ay
            self.sum_az += az
            self.count += 1

            # ปริ้นต์สถานะการเก็บตัวอย่าง
            percent = (self.count / self.max_samples) * 100.0
            print(f">> Collected sample {self.count}/{self.max_samples} ({percent:.1f}%)")
            
            if self.count == self.max_samples:
                print(f">> Collected {self.max_samples} samples, computing average...")
                self.compute_and_broadcast()

    def compute_and_broadcast(self):
        # 1) ค่าเฉลี่ย
        avg_ax = self.sum_ax / self.max_samples
        avg_ay = self.sum_ay / self.max_samples
        avg_az = self.sum_az / self.max_samples

        # 2) normalize vector
        v = np.array([avg_ax, avg_ay, avg_az])
        v /= np.linalg.norm(v)

        # 3) target = Y+
        t = np.array([0.0, 1.0, 0.0])
        axis = np.cross(v, t)
        axis_norm = np.linalg.norm(axis)

        if axis_norm < 1e-6:
            # แกน aligned แล้ว
            angle = 0.0
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis /= axis_norm
            angle = math.acos(np.clip(np.dot(v, t), -1.0, 1.0))

        # 4) quaternion (x,y,z,w)
        qx = axis[0] * math.sin(angle/2)
        qy = axis[1] * math.sin(angle/2)
        qz = axis[2] * math.sin(angle/2)
        qw = math.cos(angle/2)

        # 5) แปลง quaternion → roll, pitch, yaw
        #   (Z–Y–X convention)
        sinr_cosp = 2*(qw*qx + qy*qz)
        cosr_cosp = 1 - 2*(qx*qx + qy*qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2*(qw*qy - qz*qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2*(qw*qz + qx*qy)
        cosy_cosp = 1 - 2*(qy*qy + qz*qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        print(f">> Computed RPY (rad): roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}")

        # 6) สร้างและส่ง TransformStamped ครั้งเดียว
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'camera_imu_frame'
        tf.child_frame_id = 'camera_imu_corrected'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)
        print(">> Broadcasted corrected IMU TF once. Shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImuTiltCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
