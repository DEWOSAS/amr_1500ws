#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

class ImuFrameIdRemapper(Node):
    def __init__(self):
        super().__init__('imu_frame_id_remapper')
        self.declare_parameter('input_topic', '/camera/camera/imu')
        self.declare_parameter('output_topic', '/camera/camera/imu_repub')
        self.declare_parameter('new_frame_id', 'camera_link')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.new_frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(Imu, output_topic, qos)
        self.get_logger().info(f'Publishing remapped IMU on "{output_topic}" with BEST_EFFORT QoS')

        self.sub = self.create_subscription(
            Imu,
            input_topic,
            self.cb_imu,
            qos)
        self.get_logger().info(f'Subscribed to "{input_topic}" with BEST_EFFORT QoS')

    def cb_imu(self, msg: Imu):
        msg.header.frame_id = self.new_frame_id
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFrameIdRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
