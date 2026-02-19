"""
哨兵机器人导航系统启动文件
=======================

该启动文件用于启动哨兵机器人的完整导航系统，支持仿真和真实环境：
- Gazebo仿真环境 / 真实硬件环境
- 激光雷达里程计(LIO)算法
- 定位系统（SLAM工具箱/AMCL/ICP）
- 建图功能
- Navigation2导航系统

所有配置参数从 launch_params.yaml 文件中读取，包括：
- 运行模式 (nav/mapping)
- 定位方法 (slam_toolbox/amcl/icp)
- LIO算法 (fastlio/pointlio)
- 环境类型 (仿真/真实)
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
sys.path.append(os.path.join(get_package_share_directory('rm_nav_bringup'), 'launch'))


def _launch_setup(context, *args, **kwargs):
    map_override = LaunchConfiguration('map').perform(context).strip()
    if map_override:
        os.environ['RM_NAV_MAP'] = map_override

    localization_override = LaunchConfiguration('localization').perform(context).strip()
    if localization_override:
        os.environ['RM_NAV_LOCALIZATION'] = localization_override

    controller_override = LaunchConfiguration('controller').perform(context).strip()
    if controller_override:
        os.environ['RM_NAV_CONTROLLER'] = controller_override

    nav_start_delay_override = LaunchConfiguration('nav_start_delay').perform(context).strip()

    # 从 common 模块导入所有必要的变量和节点定义（导入时会读取 RM_NAV_MAP / RM_NAV_LOCALIZATION 覆盖）
    from common import (
        mode,
        lio,
        localization,
        use_sim,
        use_lio_rviz,
        dual_lidar_enable,
        icp_map_exists,
        slam_map_exists,
        fastlio_rviz_cfg_dir,
        pointlio_rviz_cfg_dir,
        # 节点定义
        bringup_linefit_ground_segmentation_node,
        bringup_linefit_ground_segmentation_nodes,
        bringup_pointcloud_to_laserscan_node,
        start_imu_complementary_filter,
        start_navigation2,
        odom_tf,
        start_rm_simulation,
        bringup_fake_vel_transform_node,
        start_robot_state_publisher,
        start_livox_ros_driver2_node,
        fast_lio_node,
        point_lio_node,
        start_map_server,
        slam_toolbox_node,
        icp_node,
        small_gicp_node,
        start_amcl,
        start_mapping_node,
    )

    actions = []

    # 1. 仿真环境
    if use_sim:
        print("1. 启动仿真环境...")
        actions.append(start_rm_simulation)
    else:
        print("1. 启动真实硬件环境...")
        actions.append(start_livox_ros_driver2_node)
        actions.append(start_robot_state_publisher)

    # 2. 传感器处理节点
    print("2. 启动传感器处理节点...")
    actions.append(start_imu_complementary_filter)
    if dual_lidar_enable:
        print("   启用双雷达分割链路（primary + right）")
    for segmentation_node in bringup_linefit_ground_segmentation_nodes:
        actions.append(segmentation_node)
    actions.append(bringup_pointcloud_to_laserscan_node)
    # 3. LIO算法

    actions.append(odom_tf)
    print("3. 启动LIO算法...")
    if lio == "fastlio":
        print("   启动FAST-LIO算法...")
        actions.append(fast_lio_node)
        if use_lio_rviz:
            print("   启动FAST-LIO可视化...")
            fastlio_rviz = Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", fastlio_rviz_cfg_dir],
            )
            actions.append(fastlio_rviz)

    elif lio == "pointlio":
        print("   启动Point-LIO算法...")
        actions.append(point_lio_node)
        if use_lio_rviz:
            print("   启动Point-LIO可视化...")
            pointlio_rviz = Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", pointlio_rviz_cfg_dir],
            )
            actions.append(pointlio_rviz)

    # 4. 定位系统
    if mode == "nav":
        print("4. 启动定位系统...")
        print(f"   启动导航模式，使用{localization}定位...")
        
        # 根据定位方法启动相应节点
        if localization == "slam_toolbox":
            if not slam_map_exists:
                print("   [警告] slam_toolbox 缺少 posegraph，建议改用 amcl 或 icp/small_gicp")
            actions.append(slam_toolbox_node)

        elif localization == "amcl":
            # AMCL 启动文件已包含 map_server（并统一由一个 lifecycle_manager 管理），避免重复启动
            actions.append(start_amcl)

        elif localization == "icp":
            # ICP定位需要延迟启动，等待LIO稳定
            if icp_map_exists and icp_node is not None:
                print("   ICP定位将在7秒后启动...")
                icp_timer = TimerAction(period=7.0, actions=[icp_node, start_map_server])
                actions.append(icp_timer)
            else:
                print("   未找到ICP所需的PCD地图，跳过ICP，仅启动Map Server（如可用）...")
                actions.append(start_map_server)

        elif localization == "small_gicp":
            # Small GICP定位需要延迟启动，等待LIO稳定
            if small_gicp_node is not None:
                print("   Small GICP定位将在7秒后启动...")
                small_gicp_timer = TimerAction(period=7.0, actions=[small_gicp_node, start_map_server])
                actions.append(small_gicp_timer)
            else:
                print("   未找到Small-GICP所需的PCD地图或节点未创建，跳过Small-GICP，仅启动Map Server（如可用）...")
                actions.append(start_map_server)

        else:
            print(f"   警告：未知的定位方法 '{localization}'，将使用默认的slam_toolbox")
            actions.append(slam_toolbox_node)

    # 5. 辅助节点
    print("5. 启动辅助节点...")
    actions.append(bringup_fake_vel_transform_node)

    # 6. 建图功能
    if mode == "mapping":
        print("6. 启动建图功能...")
        actions.append(start_mapping_node)

    # 7. 导航系统
    print("7. 启动Navigation2导航系统...")
    if nav_start_delay_override:
        try:
            nav_start_delay = float(nav_start_delay_override)
        except ValueError as exc:
            raise ValueError(
                f"Invalid nav_start_delay: '{nav_start_delay_override}'. Must be a float in seconds."
            ) from exc
    else:
        # 仿真场景中 Gazebo 与机器人 spawn 通常慢于 Nav2，短延迟可避免 bt_navigator 配置超时。
        nav_start_delay = 8.0 if use_sim else 0.0

    if nav_start_delay > 0.0:
        print(f"   Navigation2 将在 {nav_start_delay:.1f}s 后启动（等待仿真/TF稳定）...")
        actions.append(TimerAction(period=nav_start_delay, actions=[start_navigation2]))
    else:
        actions.append(start_navigation2)

    print("哨兵导航系统启动完成！")
    return actions


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Map/PCD resource name override. Example: map:=RMUL2026. If empty, uses config/launch_params.yaml world.'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'localization',
            default_value='',
            description='Localization override: amcl|slam_toolbox|icp|small_gicp. If empty, uses config/launch_params.yaml.'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'controller',
            default_value='',
            description='Local controller override: teb|dwb. If empty, uses config/launch_params.yaml.'
        )
    )

    # 允许在命令行覆盖：ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false
    ld.add_action(
        DeclareLaunchArgument(
            'nav_rviz',
            default_value='true',
            description='Whether to launch Navigation2 RViz (rm_navigation/rviz_launch.py)'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'nav_start_delay',
            default_value='',
            description='Delay Navigation2 startup in seconds. Empty uses auto policy (sim=8.0, real=0.0).'
        )
    )

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
