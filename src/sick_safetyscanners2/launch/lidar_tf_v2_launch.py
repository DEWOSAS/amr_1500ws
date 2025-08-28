from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node_start_delay = LaunchConfiguration('node_start_delay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'node_start_delay',
            default_value='10.0',
            description='Delay before starting the node'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser1_link',
            arguments=['0.628', '-0.401', '0.12', '-0.76794487088', '0.0', '3.14159', 'base_link', 'front_scan'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser2_link',
            arguments=['-0.628', '0.401', '0.12', '2.3561944902', '0.0', '3.14159', 'base_link', 'back_scan'],
        ),

        TimerAction(
            period=node_start_delay,
            actions=[
                Node(
                    package='ira_laser_tools',
                    executable='laserscan_multi_merger',
                    name='laserscan_multi_merger',
                    output='screen',
                    parameters=[{
                        'destination_frame': 'base_link',
                        'cloud_destination_topic': '/merged_cloud',
                        'scan_destination_topic': '/scan_multi',
                        'laserscan_topics': ['/sick_safetyscanners/scan1', '/sick_safetyscanners/scan2'],
                        'angle_min': -3.1415,
                        'angle_max': 3.1415,
                        'angle_increment': 0.005759587,
                        'scan_time': 0.66666,
                        'range_min': 0.6,
                        'range_max': 25.0
                    }]
                )
            ]
        )
    ])
