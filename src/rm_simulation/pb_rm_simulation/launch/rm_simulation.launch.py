#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.actions.append_environment_variable import AppendEnvironmentVariable

# Enum for world types
class WorldType:
    RMUC_24 = 'RMUC_24'
    RMUL_24 = 'RMUL_24'
    RMUC_25 = 'RMUC_25'
    RMUC_26 = 'RMUC_26'
    RMUL_26 = 'RMUL_26'

def get_world_config(world_type):
    world_configs = {
        WorldType.RMUC_24: {
            'x': '6.35',
            'y': '7.6',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUC2024_world/RMUC2024_world.world'
        },
        WorldType.RMUL_24: {
            'x': '4.3',
            'y': '3.35',
            'z': '1.16',
            'yaw': '0.0',
            'world_path': 'RMUL2024_world/RMUL2024_world.world'
            # 'world_path': 'RMUL2024_world/RMUL2024_world_dynamic_obstacles.world'
        },
        WorldType.RMUC_25: {
            'x': '1.0',
            'y': '11.0',
            'z': '0.2',
            'yaw': '1.57079632679',
            'world_path': 'RMUC2025_world/RMUC2025_world.world'
        }
        ,
        WorldType.RMUC_26: {
            # Default spawn pose for RMUC_26; adjust if you have a preferred spawn point.
            'x': '0.0',
            'y': '0.0',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUC2026_world/RMUC2026_world.world'
        }
        ,
        WorldType.RMUL_26: {
            # Spawn pose kept consistent with RMUL_24; adjust to your preferred start.
            # Left-upper spawn (map-derived) for easier team-side debugging.
            'x': '-5.375',
            'y': '3.425',
            'z': '1.16',
            'yaw': '-0.567331',
            'world_path': 'RMUL2026_world/RMUL2026_world.world'
        }
    }
    return world_configs.get(world_type, None)

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    default_robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('pb_rm_simulation'), 'urdf', 'simulation_waking_robot.xacro')])

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('gui', default='false')
    use_rviz = LaunchConfiguration('rviz', default='false')
    robot_description = LaunchConfiguration('robot_description')

    # Ensure Gazebo can discover all required plugins:
    # - gazebo_ros sensor plugins (e.g., libgazebo_ros_imu_sensor.so)
    # - Livox simulation plugin (libros2_livox.so)
    # - Obstacle plugins shipped with this simulation package
    gazebo_ros_plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(get_package_prefix('gazebo_ros'), 'lib')
    )
    livox_sim_plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(get_package_prefix('ros2_livox_simulation'), 'lib')
    )
    obstacle_plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(
            get_package_share_directory('pb_rm_simulation'),
            'meshes',
            'obstacles',
            'obstacle_plugin',
            'lib',
        )
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RMUC_24,
        description='Choose <RMUC_24> or <RMUL_24> or <RMUC_25> or <RMUC_26> or <RMUL_26>'
    )

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Whether to launch Gazebo GUI (gzclient). Set false for headless evaluation.'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(use_gui),
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': os.path.join(bringup_dir, 'world', world_config['world_path'])}.items(),
                )
                ,
                TimerAction(
                    period=2.0,
                    actions=[
                        Node(
                            package='gazebo_ros',
                            executable='spawn_entity.py',
                            arguments=[
                                '-entity', 'robot',
                                '-topic', 'robot_description',
                                '-x', world_config['x'],
                                '-y', world_config['y'],
                                '-z', world_config['z'],
                                '-Y', world_config['yaw']
                            ],
                        )
                    ]
                ),
            ]
        )

    bringup_RMUC_24_cmd_group = create_gazebo_launch_group(WorldType.RMUC_24)
    bringup_RMUL_24_cmd_group = create_gazebo_launch_group(WorldType.RMUL_24)
    bringup_RMUC_25_cmd_group = create_gazebo_launch_group(WorldType.RMUC_25)
    bringup_RMUC_26_cmd_group = create_gazebo_launch_group(WorldType.RMUC_26)
    bringup_RMUL_26_cmd_group = create_gazebo_launch_group(WorldType.RMUL_26)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(gazebo_ros_plugin_path)
    ld.add_action(livox_sim_plugin_path)
    ld.add_action(obstacle_plugin_path)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_RMUL_24_cmd_group) # type: ignore
    ld.add_action(bringup_RMUC_24_cmd_group) # type: ignore
    ld.add_action(bringup_RMUC_25_cmd_group) # type: ignore
    ld.add_action(bringup_RMUC_26_cmd_group) # type: ignore
    ld.add_action(bringup_RMUL_26_cmd_group) # type: ignore

    # Uncomment this line if you want to start RViz
    ld.add_action(start_rviz_cmd)

    return ld
