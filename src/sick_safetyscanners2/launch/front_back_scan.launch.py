from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First Sick Safety Scanner Node
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="back_sick_safetyscanners2",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "back_scan",
                 "sensor_ip": "192.168.50.14",
                 "host_ip": "192.168.50.10",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": -2.181,
                 "angle_end": 2.094,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            remappings=[
                ('/scan', '/back_scan')   # จาก ต้นทาง → ปลายทาง
            ],
        ),
        



         Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="front_sick_safetyscanners2",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "front_scan",
                 "sensor_ip": "192.168.50.13",
                 "host_ip": "192.168.50.10",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": -1.919,
                 "angle_end": 1.972,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            remappings=[
                ('/scan', '/front_scan')   # จาก ต้นทาง → ปลายทาง
            ],
        )
    ])
