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
from launch.actions import TimerAction
from launch_ros.actions import Node
sys.path.append(os.path.join(get_package_share_directory('rm_nav_bringup'), 'launch'))

def generate_launch_description():
    # 从 common 模块导入所有必要的变量和节点定义
    from common import (
        mode,
        lio,
        localization,
        use_sim,
        use_lio_rviz,
        fastlio_rviz_cfg_dir,
        pointlio_rviz_cfg_dir,
        # 节点定义
        bringup_linefit_ground_segmentation_node,
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
    ld = LaunchDescription()
    
    # 1. 仿真环境
    if use_sim:
        print("1. 启动仿真环境...")
        ld.add_action(start_rm_simulation)
    else:
        print("1. 启动真实硬件环境...")
        ld.add_action(start_livox_ros_driver2_node)
        ld.add_action(start_robot_state_publisher)

    # 2. 传感器处理节点
    print("2. 启动传感器处理节点...")
    ld.add_action(start_imu_complementary_filter)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    # 3. LIO算法

    ld.add_action(odom_tf)
    print("3. 启动LIO算法...")
    if lio == "fastlio":
        print("   启动FAST-LIO算法...")
        ld.add_action(fast_lio_node)
        if use_lio_rviz:
            print("   启动FAST-LIO可视化...")
            fastlio_rviz = Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", fastlio_rviz_cfg_dir],
            )
            ld.add_action(fastlio_rviz)

    elif lio == "pointlio":
        print("   启动Point-LIO算法...")
        ld.add_action(point_lio_node)
        if use_lio_rviz:
            print("   启动Point-LIO可视化...")
            pointlio_rviz = Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", pointlio_rviz_cfg_dir],
            )
            ld.add_action(pointlio_rviz)

    # 4. 定位系统
    if mode == "nav":
        print("4. 启动定位系统...")
        print(f"   启动导航模式，使用{localization}定位...")
        
        # 根据定位方法启动相应节点
        if localization == "slam_toolbox":
            ld.add_action(slam_toolbox_node)

        elif localization == "amcl":
            ld.add_action(start_amcl)
            ld.add_action(start_map_server)

        elif localization == "icp":
            # ICP定位需要延迟启动，等待LIO稳定
            print("   ICP定位将在7秒后启动...")
            icp_timer = TimerAction(period=7.0, actions=[icp_node, start_map_server])
            ld.add_action(icp_timer)

        elif localization == "small_gicp":
            # Small GICP定位需要延迟启动，等待LIO稳定
            print("   Small GICP定位将在7秒后启动...")
            small_gicp_timer = TimerAction(period=7.0, actions=[small_gicp_node, start_map_server])
            ld.add_action(small_gicp_timer)

        else:
            print(f"   警告：未知的定位方法 '{localization}'，将使用默认的slam_toolbox")
            ld.add_action(slam_toolbox_node)

    # 5. 辅助节点
    print("5. 启动辅助节点...")
    ld.add_action(bringup_fake_vel_transform_node)

    # 6. 建图功能
    if mode == "mapping":
        print("6. 启动建图功能...")
        ld.add_action(start_mapping_node)

    # 7. 导航系统
    print("7. 启动Navigation2导航系统...")
    ld.add_action(start_navigation2)

    print("哨兵导航系统启动完成！")
    return ld
