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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
sys.path.append(os.path.join(get_package_share_directory('rm_nav_bringup'), 'launch'))


def _parse_bool_str(value: str, *, key: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "on"}:
        return True
    if lowered in {"0", "false", "no", "off"}:
        return False
    raise ValueError(
        f"Invalid boolean launch arg {key}: {value!r}. "
        "Allowed: 1/0/true/false/yes/no/on/off"
    )


def _bool_to_launch_str(value: bool) -> str:
    return 'true' if value else 'false'


def _load_launch_params_yaml():
    config_path = os.path.join(
        get_package_share_directory('rm_nav_bringup'),
        'config',
        'launch_params.yaml',
    )
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        raise FileNotFoundError(f"Missing launch params file: {config_path}")
    except yaml.YAMLError as exc:
        raise yaml.YAMLError(f"Invalid YAML in {config_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"launch_params.yaml must be a mapping, got: {type(data)}")
    return data


def _bool_from_launch_params(params: dict, key: str, default: bool) -> bool:
    raw = params.get(key, default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        return _parse_bool_str(raw, key=f'launch_params.{key}')
    if raw is None:
        return default
    raise ValueError(
        f"launch_params.{key} must be bool or bool-like string, got: {raw!r}"
    )


def _launch_setup(context, *args, **kwargs):
    world_override = LaunchConfiguration('world').perform(context).strip()
    if world_override:
        os.environ['RM_NAV_WORLD'] = world_override

    map_override = LaunchConfiguration('map').perform(context).strip()
    if map_override:
        os.environ['RM_NAV_MAP'] = map_override

    localization_override = LaunchConfiguration('localization').perform(context).strip()
    if localization_override:
        os.environ['RM_NAV_LOCALIZATION'] = localization_override

    lio_override = LaunchConfiguration('lio').perform(context).strip()
    if lio_override:
        os.environ['RM_NAV_LIO'] = lio_override

    controller_override = LaunchConfiguration('controller').perform(context).strip()
    if controller_override:
        os.environ['RM_NAV_CONTROLLER'] = controller_override

    lidar_noise_override = LaunchConfiguration('lidar_noise_stddev').perform(context).strip()
    if lidar_noise_override:
        os.environ['RM_NAV_LIDAR_NOISE_STDDEV'] = lidar_noise_override

    use_stvl_override = LaunchConfiguration('use_stvl').perform(context).strip()
    if use_stvl_override:
        os.environ['RM_NAV_USE_STVL'] = use_stvl_override

    obstacle_profile_override = LaunchConfiguration('obstacle_profile').perform(context).strip()
    if obstacle_profile_override:
        os.environ['RM_NAV_OBSTACLE_PROFILE'] = obstacle_profile_override

    nav_start_delay_override = LaunchConfiguration('nav_start_delay').perform(context).strip()
    enable_nav2_override = LaunchConfiguration('enable_nav2').perform(context).strip()

    # 从 common 模块导入所有必要的变量和节点定义（导入时会读取 RM_NAV_MAP / RM_NAV_LOCALIZATION 覆盖）
    from common import (
        mode,
        lio,
        localization,
        use_sim,
        use_lio_rviz,
        dual_lidar_enable,
        dual_lidar_obstacle_fusion_mode,
        icp_map_exists,
        slam_map_exists,
        fastlio_rviz_cfg_dir,
        pointlio_rviz_cfg_dir,
        # 节点定义
        bringup_linefit_ground_segmentation_node,
        bringup_linefit_ground_segmentation_nodes,
        bringup_obstacle_merge_process,
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
    if dual_lidar_enable and dual_lidar_obstacle_fusion_mode == "merged":
        print("   启用后融合障碍点云（/segmentation/obstacle_merged）")
    for segmentation_node in bringup_linefit_ground_segmentation_nodes:
        actions.append(segmentation_node)
    if bringup_obstacle_merge_process is not None:
        actions.append(bringup_obstacle_merge_process)
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
    # 默认策略：导航模式启用 Nav2，建图模式关闭 Nav2（可通过 launch 参数覆盖）。
    enable_nav2 = mode == "nav"
    if enable_nav2_override:
        enable_nav2 = _parse_bool_str(enable_nav2_override, key="enable_nav2")

    if enable_nav2:
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
            # ICP/Small-GICP 需要等待首帧配准并发布 map->odom，给更保守的默认延迟。
            if use_sim and localization in ("icp", "small_gicp"):
                nav_start_delay = 12.0
            else:
                nav_start_delay = 8.0 if use_sim else 0.0

        if nav_start_delay > 0.0:
            print(f"   Navigation2 将在 {nav_start_delay:.1f}s 后启动（等待仿真/TF稳定）...")
            actions.append(TimerAction(period=nav_start_delay, actions=[start_navigation2]))
        else:
            actions.append(start_navigation2)
    else:
        print("7. 跳过Navigation2（当前模式默认不启动；可用 enable_nav2:=true 强制开启）")

    print("哨兵导航系统启动完成！")
    return actions


def generate_launch_description():
    launch_params = _load_launch_params_yaml()
    mode_default = str(launch_params.get('mode', 'nav')).strip().lower()
    enable_nav2_default = _bool_from_launch_params(
        launch_params, 'enable_nav2', mode_default == 'nav'
    )
    nav_rviz_default = _bool_from_launch_params(
        launch_params, 'nav_rviz', True
    )
    nav_rviz_config_default = str(
        launch_params.get('nav_rviz_config', 'nav2.rviz')
    ).strip() or 'nav2.rviz'

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Gazebo world override. Example: world:=RMUL_26_WAVE. If empty, uses config/launch_params.yaml.'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Map/PCD resource name override. Example: map:=RMUL2026. If empty, uses config/launch_params.yaml world.'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'lio',
            default_value='',
            description='LIO override: fastlio|pointlio. If empty, uses config/launch_params.yaml.'
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

    ld.add_action(
        DeclareLaunchArgument(
            'lidar_noise_stddev',
            default_value='',
            description=(
                'Gazebo lidar Gaussian noise stddev override. '
                'If empty, uses config/launch_params.yaml.'
            )
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'use_stvl',
            default_value='',
            description=(
                'Enable/disable STVL for global costmap '
                '(true|false). Empty uses config/launch_params.yaml.'
            )
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'obstacle_profile',
            default_value='',
            description=(
                'Obstacle filtering profile override '
                '(e.g. default|anti_self|anti_self_dualspot|anti_self_lowload|anti_self_wiring). '
                'Empty uses config/launch_params.yaml.'
            )
        )
    )

    # 允许在命令行覆盖：ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false
    ld.add_action(
        DeclareLaunchArgument(
            'nav_rviz',
            default_value=_bool_to_launch_str(nav_rviz_default),
            description=(
                'Whether to launch Navigation2 RViz (rm_navigation/rviz_launch.py). '
                'Default comes from config/launch_params.yaml nav_rviz.'
            )
        )
    )

    # 允许在命令行覆盖：ros2 launch rm_nav_bringup bringup.launch.py nav_rviz_config:=nav2_debug.rviz
    ld.add_action(
        DeclareLaunchArgument(
            'nav_rviz_config',
            default_value=nav_rviz_config_default,
            description=(
                'Navigation2 RViz config override. '
                'Supports absolute path or package-relative file name. '
                'Default comes from config/launch_params.yaml nav_rviz_config.'
            )
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'nav_start_delay',
            default_value='',
            description=(
                'Delay Navigation2 startup in seconds. Empty uses auto policy '
                '(sim=8.0, sim+icp/small_gicp=12.0, real=0.0).'
            )
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'enable_nav2',
            default_value=_bool_to_launch_str(enable_nav2_default),
            description=(
                'Enable/disable Navigation2 startup (true|false). '
                'Default comes from config/launch_params.yaml enable_nav2 '
                '(fallback: mode==nav).'
            )
        )
    )

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
