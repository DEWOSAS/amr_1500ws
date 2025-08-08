import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node                                # ← เพิ่มตรงนี้
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # หา path ของแพ็กเกจ bringup และ sick_safetyscanners2
    pkg_server = get_package_share_directory('facobot_core')
    pkg_map  = get_package_share_directory('map_to_jpeg')

    # 1) ประกาศ IncludeLaunchDescription สำหรับสอง launch เดิม
    server_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_server, 'launch', 'amr_server.launch.py')
        )
    )

    map_to_jpeg_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_map, 'launch', 'map_to_image.launch.py')
        )
    )
    web_video_server_node = Node(
    package='web_video_server',
    executable='web_video_server',
    name='web_video_server',
    output='screen'
    )

    ros_bridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen'
    )   


    # 3) สร้าง LaunchDescription แล้ว add action ตามลำดับ
    ld = LaunchDescription()
    ld.add_action(server_action)
    ld.add_action(web_video_server_node)
    ld.add_action(map_to_jpeg_action)
    ld.add_action(ros_bridge_node)

    return ld
