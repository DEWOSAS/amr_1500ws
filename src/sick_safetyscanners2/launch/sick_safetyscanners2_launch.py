from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First Sick Safety Scanner Node
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="sick_safetyscanners2_front",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "back_scan",
                 "sensor_ip": "192.168.50.14",
                 "host_ip": "192.168.50.107",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 6060,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 2,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ]
        ),
    ])
