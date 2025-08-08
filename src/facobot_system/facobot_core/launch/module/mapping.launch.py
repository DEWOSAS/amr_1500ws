from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='online_sync_launch.py',  # หรือ async_slam_toolbox_node
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'map_update_interval': 1.0,              # ความถี่ในการอัปเดต map
                'resolution': 0.03,                      # ความละเอียดของ map (เหมือน delta)
                'range_threshold': 30.0,                 # ระยะไกลสุดของ laser
                'max_laser_range': 30.0,                 # ใช้ร่วมกับ range_threshold
                'minimum_time_interval': 0.0,            # คล้าย lstep/astep = ถ้า > 0 จะรอระยะ
                'transform_publish_period': 0.05,
                'mode': 'mapping',                       # หรือ localization
                'scan_topic': '/sick_safetyscanners/scan1',
                'use_scan_matching': True,
                'use_pose_graph': True,
                'minimum_travel_distance': 0.1,          # ระยะทางขั้นต่ำที่จะรับ scan ใหม่
                'minimum_travel_heading': 0.1,           # มุมหมุนขั้นต่ำ
                'scan_buffer_maximum_scan_distance': 5.0,
                'scan_buffer_maximum_scan_angle': 1.57,
                'scan_buffer_minimum_time': 5.0,
                'link_match_minimum_response_fine': 0.1,
                'do_loop_closing': True                  # มี loop closure (ต่างจาก gmapping)
            }]
        )
    ])
