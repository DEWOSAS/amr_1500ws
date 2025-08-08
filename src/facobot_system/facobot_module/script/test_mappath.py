#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MapPathSubscriber(Node):
    def __init__(self):
        super().__init__('map_path_subscriber')
        self.create_subscription(String, 'latest_map_path', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"Latest map path: {msg.data}")

def main():
    rclpy.init()
    node = MapPathSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
