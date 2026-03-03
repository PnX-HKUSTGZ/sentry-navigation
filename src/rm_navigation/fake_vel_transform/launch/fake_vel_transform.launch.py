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
                'local_plan_topic': '/local_plan',
                'follow_mark_enable': False,
                'follow_mark_topic': '/chassis/follow_mark',
                'follow_mark_manual_topic': '/chassis/follow_mark_manual',
                'follow_mark_hint_topic': '/chassis/follow_mark_hint',
                'follow_mark_mode': 'off',
                'follow_mark_input_stale_timeout_sec': 0.3,
                'follow_mark_enter_margin_m': 0.35,
                'follow_mark_exit_margin_m': 0.55,
                'follow_mark_default_value': 1,
                'follow_mark_zone_value': 0,
                'follow_mark_zone_rects': [],
                'follow_mark_zone_polygon_points': [],
                'follow_mark_zone_polygon_sizes': [],
            }
        ]
    )

    return LaunchDescription([fake_vel_transform_node])
