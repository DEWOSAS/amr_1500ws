#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class joy_control(Node):
	def __init__(self):
		self.init_node()
		self.init_param()
		self.init_pub()
		self.init_sub()
		self.start_loop()

	def init_node(self):
		super().__init__('joy_control')
		self.get_logger().info('Node : joy_control')

	def init_param(self):
		self.joy = Joy()
		self.joy.axes = [0.0] * 8
		self.joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]
		self.joy_vel = Twist()
		self.gain = 0.5

	def init_pub(self):
		self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
		self.pub_vel_protect = self.create_publisher(Twist, '/cmd_vel_protect', 10)
		self.pub_module_command = self.create_publisher(String, '/module_command', 10)
		self.pub_alignment = self.create_publisher(String, '/alignment_command', 10)
		self.pub_disable_safety = self.create_publisher(Int8, '/disable_safety', 10)

	def init_sub(self):
		self.create_subscription(Joy, '/joy', self.joy_callback, 10)

	def joy_callback(self,msg):
		self.joy = msg

	def add_vel_gain(self,add_gain):
		self.gain += add_gain
		if self.gain > 1.5:
			self.gain = 1.5
		elif self.gain < 0.2:
			self.gain = 0.2

	def filter_max_value(self,value):
		if abs(value) > 0.7071:
			if value < 0:
				return -1
			else:
				return 1
		else:
			return value/0.7071

	def send_velocity(self):
		if self.joy.buttons[4] == 1:
			self.add_vel_gain(0.05)
		if self.joy.buttons[5] == 1:
			self.add_vel_gain(-0.05)

		lin_gain = self.filter_max_value(self.joy.axes[1])
		ang_gain = self.filter_max_value(self.joy.axes[0])
		if self.joy.buttons[3] == 1: #ปุ่ม Y
			self.pub_disable_safety.publish(Int8(data=3))
			if self.joy.axes[5] < 0:
				if abs(lin_gain) > abs(ang_gain):
					self.joy_vel.linear.x = lin_gain * 0.35
					self.joy_vel.angular.z = 0
				else:
					self.joy_vel.linear.x = 0
					self.joy_vel.angular.z = ang_gain * 0.35
			else:
				self.joy_vel.linear.x = lin_gain*0.35
				self.joy_vel.angular.z = ang_gain*0.35
			self.pub_vel.publish(self.joy_vel)
		elif self.joy.buttons[2] == 1: # ปุ่ม X
			if self.joy.axes[5] < 0:
				if abs(lin_gain) > abs(ang_gain):
					self.joy_vel.linear.x = lin_gain * self.gain
					self.joy_vel.angular.z = 0
				else:
					self.joy_vel.linear.x = 0
					self.joy_vel.angular.z = ang_gain * self.gain
			else:
				self.joy_vel.linear.x = lin_gain * self.gain
				self.joy_vel.angular.z = ang_gain * self.gain
			self.pub_vel.publish(self.joy_vel)
		elif self.joy.buttons[1] == 1:
			self.pub_alignment.publish(String(data="ALIGNMENT"))
		elif self.joy.buttons[0] == 1:
			self.pub_alignment.publish(String(data="CHARGER"))
   
   

	def send_module(self):
		if self.joy.axes[7] == 1:
			self.pub_module_command.publish(String(data="up"))
		elif self.joy.axes[7] == -1:
			self.pub_module_command.publish(String(data="down"))
		elif self.joy.axes[6] == 1:
			self.pub_module_command.publish(String(data="left"))
		elif self.joy.axes[6] == -1:
			self.pub_module_command.publish(String(data="right"))

	def node_handle(self):
		self.send_velocity()
		self.send_module()

	def start_loop(self):
		self.create_timer(0.05, self.node_handle)

def main(args=None):
    rclpy.init(args=args)
    node = joy_control()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

