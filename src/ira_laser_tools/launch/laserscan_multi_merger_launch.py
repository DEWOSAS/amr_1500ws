from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

#         Node(
#             package='ira_laser_tools',
#             name='mir_laser_scan_merger',
#             executable='laserscan_multi_merger',
#             parameters=[{'laserscan_topics': "back_scan front_scan",
#                          'destination_frame': "base_link",
#                          'scan_destination_topic': "scan",
#                          'cloud_destination_topic': "scan_cloud",
#                          'angle_max': 3.1415,
#                          'max_completion_time': 0.05,
#                          'max_merge_time_diff': 0.005,
#                          'use_sim_time': LaunchConfiguration('use_sim_time'),
#                          'best_effort': True}],
#             namespace=namespace,    # adds namespace to topic names and frames
#             output='screen')
#     ])
# <param name="angle_min" value="-3.1415"/>
#                 <param name="angle_max" value="3.1415"/>
#                 <param name="angle_increment" value="0.005759587"/>
#                 <param name="scan_time" value="0.66666"/>
#                 <param name="range_min" value="0.6"/>
#                 <param name="range_max" value="25.0"/>
        Node(
            package='ira_laser_tools',
            name='mir_laser_scan_merger',
            executable='laserscan_multi_merger',
            parameters=[{
                'laserscan_topics': "back_scan front_scan",
                'destination_frame': "base_link",
                'scan_destination_topic': "scan",
                'cloud_destination_topic': "scan_cloud",
                'angle_max': 3.1415,
                'angle_min': -3.1415,                # ← เพิ่มจากตรงนี้
                'angle_increment': 0.005759587,
                'scan_time': 0.6666,
                'range_min': 0.6,
                'range_max': 40.0,                   # ← ถึงตรงนี้
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'best_effort': False,
                # 'max_merge_time_diff' : 0.05
            }],
            namespace=namespace,    # adds namespace to topic names and frames
            output='screen'
        )
    ])