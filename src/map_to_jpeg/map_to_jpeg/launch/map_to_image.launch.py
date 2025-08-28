from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_to_jpeg',               # ชื่อแพ็กเกจที่ build แล้วมี executable นี้
            executable='map_to_image_node',      # ชื่อ executable (อาจมาจากไฟล์ .cpp หรือ .py)
            name='map_image_stream',             # ชื่อ Node ที่จะแสดงใน ROS graph
            output='screen',                     # แสดง log บนหน้าจอเทอร์มินัล
            # parameters=[
            #     # คำสั่ง param ด้านล่างนี้เคยถูกคอมเมนต์ไว้ใน ROS 1 XML
            #     {"dev": "/dev/input/js0"},                   # อุปกรณ์ joystick (กรณีใช้ input device)
            #     {"autorepeat_rate": 15.0},                   # ความถี่การส่งค่าซ้ำเมื่อไม่มีการเคลื่อนไหว
            #     {"deadzone": 0.1},                           # พื้นที่เฉื่อย: ค่าที่จะถูกละเว้นหากต่ำกว่าค่านี้
            #     {"coalesce_interval": 0.001}                 # ค่าหน่วงเวลาสำหรับรวม input
            # ]
        )
    ])
