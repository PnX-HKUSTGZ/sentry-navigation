#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('small_gicp_registration')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    pcd_path_arg = DeclareLaunchArgument(
        'pcd_path',
        default_value='',
        description='Path to the reference PCD map file'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'small_gicp_registration.yaml'),
        description='Path to the configuration file'
    )
    
    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_path = LaunchConfiguration('pcd_path')
    config_file = LaunchConfiguration('config_file')
    
    # Small GICP Registration Node
    small_gicp_node = Node(
        package='small_gicp_registration',
        executable='small_gicp_registration_node',
        name='small_gicp_registration',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'pcd_path': pcd_path
            }
        ],
        # Uncomment for debugging
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        pcd_path_arg,
        config_file_arg,
        small_gicp_node
    ])
