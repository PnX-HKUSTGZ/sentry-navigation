import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'tf_publish_frequency': 20,
                'local_plan_timeout_sec': 0.5,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'fake_base_frame': 'base_link_fake',
                'cmd_vel_topic': '/cmd_vel',
                'cmd_vel_out_topic': '/cmd_vel_chassis',
                'local_plan_topic': '/local_plan'
            }
        ]
    )

    return LaunchDescription([fake_vel_transform_node])
