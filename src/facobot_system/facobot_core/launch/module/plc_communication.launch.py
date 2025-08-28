from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # หา path ของแพ็กเกจ facobot_module
    pkg_module = get_package_share_directory('facobot_module')
    params_file = os.path.join(pkg_module, 'config', 'plc_communication.yaml')
    
    return LaunchDescription([
        Node(
            package='facobot_module',
            executable='plc_communication.py',
            name='modbus_plc',
            output='screen',
            parameters=[params_file]
        )
    ])