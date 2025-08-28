from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    map_to_control_params = os.path.join(get_package_share_directory('map_to_jpeg'), 'config', 'map_to_control_params.yaml')
    return LaunchDescription([
        Node(
            package='map_to_jpeg',               # ชื่อแพ็กเกจที่ build แล้วมี executable นี้
            executable='map_to_control',      # ชื่อ executable (อาจมาจากไฟล์ .cpp หรือ .py)
            name='map_to_control_node',             # ชื่อ Node ที่จะแสดงใน ROS graph
            output='screen',                     # แสดง log บนหน้าจอเทอร์มินัล
            parameters=[map_to_control_params]  # ใช้ไฟล์พารามิเตอร์ที่กำหนดไว้
        )
    ])
