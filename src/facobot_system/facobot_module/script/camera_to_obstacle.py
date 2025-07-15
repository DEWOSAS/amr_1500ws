#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import struct
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from facobot_msg.msg import GroupPointList
import time


class camera_to_obstacle(Node):
	def __init__(self):
		super().__init__("camera_to_obstacle")
		self.get_logger().info("Node : Camera to Obstacle Started!")
		self.init_param()
		self.init_pub()
		self.init_sub()
		self.start_loop()
		self.last_image_time = time.time()
		self.count = 0
		
		
	def init_param(self):
		self.body = GroupPointList()
		self.image = []
		self.frame_size = 500
		self.half_frame_size = int(self.frame_size/2)
		self.cam_recv_timeout = 4
		self.cam_recv_count = self.cam_recv_timeout
		self.camera_lost = False

		# declear params
		self.declare_parameter('pixel_ratio', 0.0002344)
		self.declare_parameter('sampling_pixel_size', 4)
		self.declare_parameter('group_point_threshold', 18)
		self.declare_parameter('camera_offset', -2)
		self.declare_parameter('reverse_camera', True)
		self.declare_parameter('camera_obs_show', True)

		#get params
		self.pixel_ratio = float(self.get_parameter('pixel_ratio').value)
		self.sampling_pixel_size = int(self.get_parameter('sampling_pixel_size').value)
		self.group_point_threshold = int(self.get_parameter('group_point_threshold').value)
		self.camera_offset = int(self.get_parameter('camera_offset').value)
		self.reverse_camera = bool(self.get_parameter('reverse_camera').value)
		self.camera_obs_show = bool(self.get_parameter('camera_obs_show').value)
		
		# self.pixel_ratio = 0.0002344
		# self.camera_topic = str(rospy.get_param('~camera_topic', 'cam1/depth/image_rect_raw'))
		self.camera_width_limit = round(320*5000*self.pixel_ratio)

	def init_pub(self):
		self.pub_body = self.create_publisher(GroupPointList, "/body_detect", 1)
		self.pub_error_report = self.create_publisher(String, "/error_report", 3)
	

	def init_sub(self):
		self.create_subscription(
			Image,'/camera/camera/depth/image_rect_raw',
    		self.image_callback,10)
		
	def image_callback(self,msg):
		# now = time.time()
		# dt = now - self.last_image_time
		# self.last_image_time = now
		# fps = 1.0 / dt if dt > 0 else 0
		# self.get_logger().info(f"Image callback FPS: {fps:.2f}")
		self.get_logger().info(f"image_callback Recive Data{self.count}")
		if self.reverse_camera:
			self.image = list(reversed(msg.data))
		else:
			self.image = msg.data
		self.cam_recv_count = 0

	def handle_obstacle_check(self):
		if self.image != [] and self.cam_recv_count < self.cam_recv_timeout:
			self.cam_recv_count += 1
			self.body.positions = []
			body_position = Point()
			body_point = 0
			first_point = 0
			point_break = 0
			select_body_point = 0
			select_x = 0
			select_y = 0
			select_z = 0
			weight = 0
			pixel_y = 0
			last_weight = 0
			focus_range = []
			focus_list = []
			# self.scan_data = self.init_scan_data

			for num in range(0,1280,self.sampling_pixel_size*2): # 640*2 = 1280
				select_y = pixel_y
				pixel_y += 4
				first_point = 0
				body_point = 0
				body_position = Point()
				body_position.x = 10000.0
				body_position.y = 10000.0
				body_position.z = 10000.0
				for h in range(0,420,self.sampling_pixel_size):
					index = (h*1280)+num
					if self.reverse_camera:
						weight = struct.pack("B",self.image[index+1])+struct.pack("B",self.image[index])
						weight = struct.unpack("<H",weight)[0]
					else:
						weight = struct.pack("B",self.image[index])+struct.pack("B",self.image[index+1])
						weight = struct.unpack("<H",weight)[0]

					weight * 2
					reject = 0
					if weight >= 10:
						if first_point == 0:
							body_point=1
							first_point = 1
							select_x = weight
							last_weight = weight
							point_break = 0
							select_z = h
						if abs(last_weight-weight) <= 50:
							body_point+=1
							last_weight = weight
							if select_x > weight:
								select_x = weight
						else:
							reject = 1
					else:
						reject = 1
							
					if reject == 1:
						if first_point == 1:
							point_break+=1
							if point_break >= 6:
								first_point = 0
								x = select_x/10
								y = (select_y-320)*select_x*self.pixel_ratio + self.camera_offset
								z = (240-select_z)*select_x*self.pixel_ratio
								if body_point > self.group_point_threshold and body_position.x > x:# and z < 200:
									select_body_point = body_point
									body_position.x = x
									body_position.y = y
									body_position.z = z
				
				if first_point == 1:
					x = select_x/10
					y = (select_y-320)*select_x*self.pixel_ratio + self.camera_offset
					z = (240-select_z)*select_x*self.pixel_ratio
					if body_point > self.group_point_threshold and body_position.x > x:# and z < 200:
						select_body_point = body_point
						body_position.x = x
						body_position.y = y
						body_position.z = z

				if body_position.x != 10000: 
					focus_range.append(select_body_point)
					focus_list.append(body_position)
					self.add_circle(round(body_position.x),round(body_position.y),4,(0,0,255))
			
			self.body.number.data = len(focus_range)
			self.body.ranges.data = focus_range
			self.body.positions = focus_list
			self.pub_body.publish(self.body)
			if self.camera_lost:
				self.camera_lost = False
				self.pub_error_report.publish("CLEAR")
			self.get_logger().info("hadle_obstacle_check")
		else:
			if not self.camera_lost:
				self.camera_lost = True
				self.pub_error_report.publish(String(data="ER0002:Depth Camera Lost!!"))  


	def add_circle(self,x,y,width=4,color=(0, 0, 255),fill=-1):
		if self.camera_obs_show: cv2.circle(self.frame, (self.half_frame_size+y,self.frame_size-x), width, color, fill)

	def add_rectangle(self,x1,y1,x2,y2,color=(0,255,0),width=2):
		if self.camera_obs_show: cv2.rectangle(self.frame, (self.half_frame_size+y1,self.frame_size-x1), (self.half_frame_size+y2,self.frame_size-x2), color, width)

	def add_line(self,x1,y1,x2,y2,color=(255,0,0),width=2):	
		if self.camera_obs_show: cv2.line(self.frame, (self.half_frame_size+y1,self.frame_size-x1), (self.half_frame_size+y2,self.frame_size-x2), color, width)

	def create_frame(self):
		if self.camera_obs_show:
			self.frame = np.zeros((self.frame_size, self.frame_size,3), np.uint8)
			self.add_line(0,0,500,self.camera_width_limit, (0,255,150), 2)
			self.add_line(0,0,500,-self.camera_width_limit, (0,255,150), 2)
			self.add_circle(0, 0, 8, (0, 200, 255))

	def show_frame(self):
		if self.camera_obs_show:
			cv2.imshow("Obstacle View",self.frame)
			cv2.waitKey(1)

	def node_handle(self):
		self.count +=1
		self.get_logger().info(f"node_handle Recive data{self.count}")
		self.create_frame()
		self.handle_obstacle_check()
		self.show_frame()

	
	def start_loop(self):
		self.create_timer(0.1,self.node_handle)
	

def main(args=None):
    rclpy.init(args=args)
    node = camera_to_obstacle()
    rclpy.spin(node)
    rclpy.shutdown()
	

if __name__ == '__main__':
	main()