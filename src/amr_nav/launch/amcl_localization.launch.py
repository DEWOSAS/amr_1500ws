from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')  # 'false' สำหรับของจริง, 'true' ถ้า sim/bag       # path ไปยัง .yaml ของแผนที่
    params_file = '/home/babartos/amr_1500ws/src/amr_nav/include/amr_nav/amcl_params.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

#         # 1) map_server (ระบุไฟล์แผนที่ให้ชัด)
#         Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=['/home/babartos/amr_1500ws/src/amr_nav/include/amr_nav/amcl_params.yaml'],
# )
# ,

        # 2) amcl (มีแค่ตัวเดียว) — รวมพารามิเตอร์จากไฟล์ + override initial pose ใน launch ได้
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'set_initial_pose': True,
                    'initial_pose': [1.0, 2.0, 1.57],   # x, y, yaw(rad) — ปรับตามจริง
                    'initial_covariance': [0.25, 0.25, 0.45]
                }
            ],
            remappings=[
                ('scan', '/scan')   # ถ้า lidar คุณชื่ออื่น เช่น /scan_multi ให้ remap มาที่นี่
            ]
        ),

        # 3) lifecycle manager ดูแลทั้ง map_server และ amcl
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }]
        ),
    ])
