from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='facobot_module',
            executable='amr_web_server.py',
            name='web_server',
            output='screen',
            parameters=[{'web_ip': '172.16.0.210'}]
        )
    ])