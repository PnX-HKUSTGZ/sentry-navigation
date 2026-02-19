import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 启动文件路径
rm_nav_bringup_dir = get_package_share_directory("rm_nav_bringup")  # 主启动包目录
navigation2_launch_dir = os.path.join(
    get_package_share_directory("rm_navigation"), "launch"
)  # 导航启动包目录
imu_complementary_filter_launch_dir = os.path.join(
    get_package_share_directory("imu_complementary_filter"), "launch"
)  # IMU滤波器启动文件目录
# =========================== launch parameters ==========================================
# 加载启动参数配置文件
try:
    launch_params = yaml.safe_load(
        open(os.path.join(rm_nav_bringup_dir, "config", "launch_params.yaml"))
    )
except FileNotFoundError:
    raise FileNotFoundError("launch_params.yaml 配置文件未找到！")
except yaml.YAMLError as e:
    raise yaml.YAMLError(f"launch_params.yaml 文件格式错误: {e}")

# 获取基本参数
# 注意：launch_params.yaml 里的 world 既被用作仿真 world，也被用作“地图资源名”。
# 为支持命令行选择地图（map:=XXX）且不影响仿真 world，这里拆分：
# - world: 仿真 world（仍来自配置文件）
# - map_name: 地图/PCD 资源名（可由环境变量覆盖）
world = launch_params.get("world", "RMUL")  # 仿真世界名称，默认为RMUL
_env_map_name = (os.environ.get("RM_NAV_MAP", "").strip() or os.environ.get("RM_NAV_WORLD", "").strip())
map_name = _env_map_name if _env_map_name else world

# 兼容：有些 world 名称含下划线（如 RMUL_26），但 map 资源文件可能不含下划线（如 RMUL26.yaml）。
# 仅当未显式指定 map override 时启用回退，避免掩盖用户输入错误。
if not _env_map_name:
    map_dir = os.path.join(rm_nav_bringup_dir, "map")
    preferred_map_yaml = os.path.join(map_dir, map_name + ".yaml")
    if not os.path.exists(preferred_map_yaml) and "_" in map_name:
        fallback = map_name.replace("_", "")
        fallback_map_yaml = os.path.join(map_dir, fallback + ".yaml")
        if os.path.exists(fallback_map_yaml):
            print(f"[信息] 未找到地图资源 {map_name}.yaml，回退使用 {fallback}.yaml")
            map_name = fallback

mode = launch_params.get("mode", "nav")  # 获取运行模式 (mapping/nav)，默认为nav
lio = launch_params.get("lio", "fastlio")  # 激光雷达惯性里程计 (pointlio/fastlio)，默认为fastlio
_env_localization = os.environ.get("RM_NAV_LOCALIZATION", "").strip()
localization = _env_localization if _env_localization else launch_params.get("localization", "slam_toolbox")  # 获取定位模式 (amcl/slam_toolbox/icp)
_env_controller = os.environ.get("RM_NAV_CONTROLLER", "").strip()
controller = _env_controller if _env_controller else launch_params.get("controller", "teb")  # 局部控制器 (teb/dwb)
use_sim = launch_params.get("use_sim", False)  # 是否使用仿真，默认为False
use_sim_time = LaunchConfiguration(
    'use_sim_time',
    default='true' if use_sim else 'false'
)  # 供ROS使用（避免使用 'True'/'False' 造成类型/解析歧义）

# 作为 Node 参数下发时强制为 bool，避免 /tf 与 /clock 时间基准不一致
use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
use_lio_rviz = launch_params.get("use_lio_rviz", False)  # 可视化 FAST_LIO 或 Point_LIO 的点云图
nav_rviz = LaunchConfiguration('nav_rviz', default='true')  # Navigation2 RViz（可由 launch 参数覆盖）
dual_lidar_cfg = launch_params.get("dual_lidar", {})
dual_lidar_enable = bool(dual_lidar_cfg.get("enable", False))
dual_lidar_nav2_consume_right = bool(dual_lidar_cfg.get("nav2_consume_right", True))
dual_lidar_obstacle_fusion_mode = str(
    dual_lidar_cfg.get("obstacle_fusion_mode", "separate")
).strip().lower()
if dual_lidar_obstacle_fusion_mode not in {"separate", "merged"}:
    raise ValueError(
        "dual_lidar.obstacle_fusion_mode 仅支持 'separate' 或 'merged'"
    )

# 主雷达沿用历史参数键，避免影响既有配置与定位链路。
primary_lidar_pose = launch_params["base_link2livox_frame"]
right_lidar_pose = launch_params.get(
    "base_link2livox_right_frame",
    {
        "xyz": '"0.12 -0.14 0.175"',
        "rpy": '"0.0 0.0 -0.610865"',
    },
)

# 参数验证
valid_modes = ["mapping", "nav"]
valid_lio_types = ["fastlio", "pointlio"]
valid_localization_types = ["amcl", "slam_toolbox", "icp", "small_gicp"]
valid_controller_types = ["teb", "dwb"]

if mode not in valid_modes:
    raise ValueError(f"无效的mode参数: {mode}. 有效值: {valid_modes}")
if lio not in valid_lio_types:
    raise ValueError(f"无效的lio参数: {lio}. 有效值: {valid_lio_types}")
if localization not in valid_localization_types:
    raise ValueError(f"无效的localization参数: {localization}. 有效值: {valid_localization_types}")
if controller not in valid_controller_types:
    raise ValueError(f"无效的controller参数: {controller}. 有效值: {valid_controller_types}")

print("启动参数配置:")
print(f"  世界环境: {world}")
print(f"  地图资源: {map_name}")
print(f"  运行模式: {mode}")
print(f"  LIO算法: {lio}")
print(f"  定位方法: {localization}")
print(f"  局部控制器: {controller}")
print(f"  仿真模式: {use_sim}")
print(f"  LIO可视化: {use_lio_rviz}")
print(f"  双雷达: {dual_lidar_enable}")
print(f"  Nav2消费右雷达: {dual_lidar_nav2_consume_right}")
print(f"  双雷达障碍融合模式: {dual_lidar_obstacle_fusion_mode}")

if use_sim:
    config_dir = os.path.join(rm_nav_bringup_dir, "config", "simulation")
else:
    config_dir = os.path.join(rm_nav_bringup_dir, "config", "reality")

# =========================== robot description parameters ================================
# 使用xacro生成机器人URDF描述，导入雷达坐标系参数
if use_sim:
    robot_description_sim = Command(
        [
            "xacro ",
            os.path.join(
                get_package_share_directory("rm_nav_bringup"),
                "urdf",
                "sentry_robot_sim.xacro",
            ),
            " xyz:=",
            primary_lidar_pose["xyz"],
            " rpy:=",
            primary_lidar_pose["rpy"],
            " dual_lidar:=",
            "true" if dual_lidar_enable else "false",
            " right_xyz:=",
            right_lidar_pose["xyz"],
            " right_rpy:=",
            right_lidar_pose["rpy"],
        ]
    )
else:
    robot_description_real = Command(
        [
            "xacro ",
            os.path.join(
                get_package_share_directory("rm_nav_bringup"),
                "urdf",
                "sentry_robot_real.xacro",
            ),
            " xyz:=",
            primary_lidar_pose["xyz"],
            " rpy:=",
            primary_lidar_pose["rpy"],
            " dual_lidar:=",
            "true" if dual_lidar_enable else "false",
            " right_xyz:=",
            right_lidar_pose["xyz"],
            " right_rpy:=",
            right_lidar_pose["rpy"],
        ]
    )

# =========================== pointcloud pretreatment parameters ==========================
segmentation_params = os.path.join(config_dir, "segmentation.yaml")
segmentation_right_params = os.path.join(config_dir, "segmentation_right.yaml")
pointcloud_to_laserscan_params = os.path.join(
    config_dir, "pointcloud_to_laserscan.yaml"
)
obstacle_merge_script = os.path.join(rm_nav_bringup_dir, "scripts", "merge_obstacle_clouds.py")

scan_cloud_input_topic = "/segmentation/obstacle"
if dual_lidar_enable and dual_lidar_obstacle_fusion_mode == "merged":
    scan_cloud_input_topic = "/segmentation/obstacle_merged"

# ================================== FAST_LIO parameters ==================================
fastlio_mid360_params = os.path.join(
    config_dir, "fastlio_mid360.yaml"
)
fastlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, "rviz", "fastlio.rviz")

# ================================= POINT_LIO parameters ==================================
pointlio_mid360_params = os.path.join(
    config_dir, "pointlio_mid360.yaml"
)
pointlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, "rviz", "pointlio.rviz")

# ================================ slam_toolbox parameters ================================
slam_toolbox_map_dir = os.path.join(rm_nav_bringup_dir, "map", map_name)
slam_toolbox_localization_file_dir = os.path.join(
    config_dir, "mapper_params_localization.yaml"
)
slam_toolbox_mapping_file_dir = os.path.join(
    config_dir, "mapper_params_online_async.yaml"
)

# ================================= navigation2 parameters =================================
nav2_map_dir = os.path.join(rm_nav_bringup_dir, "map", map_name + ".yaml")


def _load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Nav2 params yaml must be a mapping at top-level: {path}")
    return data


def _deep_merge(base: dict, overlay: dict, *, replace_keys: set[str]) -> dict:
    """Recursively merge overlay into base.

    For keys in replace_keys, overlay fully replaces base (no recursive merge).
    """
    result = dict(base)
    for key, overlay_value in overlay.items():
        if key in replace_keys:
            result[key] = overlay_value
            continue

        base_value = result.get(key)
        if isinstance(base_value, dict) and isinstance(overlay_value, dict):
            result[key] = _deep_merge(base_value, overlay_value, replace_keys=replace_keys)
        else:
            result[key] = overlay_value
    return result


def _drop_dual_lidar_sources(nav2_params: dict) -> dict:
    """Remove right-lidar obstacle sources from Nav2 params for single-lidar mode."""

    def _drop_tokens(token_str: str, denied: set[str]) -> str:
        tokens = [token for token in token_str.split() if token not in denied]
        return " ".join(tokens)

    # local/global obstacle_layer style
    for scope in ("local_costmap", "global_costmap"):
        ros_params = (
            nav2_params
            .get(scope, {})
            .get(scope, {})
            .get("ros__parameters", {})
        )
        obstacle_layer = ros_params.get("obstacle_layer")
        if isinstance(obstacle_layer, dict):
            observation_sources = obstacle_layer.get("observation_sources")
            if isinstance(observation_sources, str):
                obstacle_layer["observation_sources"] = _drop_tokens(
                    observation_sources, {"lidar_right"}
                )
            obstacle_layer.pop("lidar_right", None)

    # STVL style (real robot global costmap)
    real_global_ros_params = (
        nav2_params
        .get("global_costmap", {})
        .get("global_costmap", {})
        .get("ros__parameters", {})
    )
    stvl_layer = real_global_ros_params.get("stvl_layer")
    if isinstance(stvl_layer, dict):
        stvl_observation_sources = stvl_layer.get("observation_sources")
        if isinstance(stvl_observation_sources, str):
            stvl_layer["observation_sources"] = _drop_tokens(
                stvl_observation_sources,
                {"livox_right_mark", "livox_right_clear"},
            )
        stvl_layer.pop("livox_right_mark", None)
        stvl_layer.pop("livox_right_clear", None)

    return nav2_params


def _get_nav2_params_file(
    config_dir: str,
    controller_type: str,
    dual_lidar_enabled: bool,
    nav2_consume_right: bool,
    obstacle_fusion_mode: str,
) -> str:
    base_params = os.path.join(config_dir, "nav2_params.yaml")
    merged_params = _load_yaml(base_params)

    if controller_type != 'teb':
        overlay = os.path.join(config_dir, f"nav2_controller_{controller_type}.yaml")
        if not os.path.exists(overlay):
            raise FileNotFoundError(f"Nav2 controller overlay not found: {overlay}")

        merged_params = _deep_merge(
            merged_params,
            _load_yaml(overlay),
            replace_keys={'controller_server'},
        )

    # In single-lidar mode or merged-obstacle mode, Nav2 should not subscribe
    # the right source directly to avoid duplicate obstacle marking.
    if (
        (not dual_lidar_enabled)
        or (not nav2_consume_right)
        or obstacle_fusion_mode == "merged"
    ):
        merged_params = _drop_dual_lidar_sources(merged_params)

    if (
        controller_type == 'teb'
        and dual_lidar_enabled
        and nav2_consume_right
        and obstacle_fusion_mode != "merged"
    ):
        # Fast-path: base file already matches requested setup.
        return base_params

    out_dir = os.path.join('/tmp', 'rm_nav_bringup')
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(
        out_dir,
        (
            f"nav2_params_{'sim' if use_sim else 'real'}_{controller_type}_"
            f"{'dual' if dual_lidar_enabled else 'single'}_"
            f"{'consume_right' if nav2_consume_right else 'ignore_right'}_"
            f"{obstacle_fusion_mode}.yaml"
        )
    )
    with open(out_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(
            merged_params,
            f,
            default_flow_style=False,
            sort_keys=False,
            allow_unicode=True,
        )
    return out_file


nav2_params_file_dir = _get_nav2_params_file(
    config_dir,
    controller,
    dual_lidar_enable,
    dual_lidar_nav2_consume_right,
    dual_lidar_obstacle_fusion_mode,
)

# =============================== icp_registration parameters ==============================
icp_pcd_dir = os.path.join(rm_nav_bringup_dir, "PCD", map_name + ".pcd")
icp_registration_params_dir = os.path.join(
    config_dir, "icp_registration.yaml"
)

# ============================= small_gicp_registration parameters ========================
small_gicp_pcd_dir = os.path.join(rm_nav_bringup_dir, "PCD", map_name + ".pcd")
small_gicp_registration_params_dir = os.path.join(
    config_dir, "small_gicp_registration.yaml"
)

# =============================== 地图存在性检查（容错机制） ===================================
icp_map_exists = os.path.exists(icp_pcd_dir)
nav2_map_exists = os.path.exists(nav2_map_dir)
slam_map_exists = os.path.exists(slam_toolbox_map_dir) or os.path.exists(slam_toolbox_map_dir + ".posegraph")
small_gicp_map_exists = os.path.exists(small_gicp_pcd_dir)

if localization == "slam_toolbox" and not slam_map_exists:
    print(
        f"[警告] 选择了 slam_toolbox 定位，但未找到 posegraph/serialization 地图: {slam_toolbox_map_dir}(.posegraph)。"
        " slam_toolbox localization 需要对应的 posegraph；没有的话建议使用 amcl 或 icp/small_gicp。"
    )

if localization == "icp" and not icp_map_exists:
    print(f"[警告] 选择了 ICP 定位，但未找到 PCD 地图: {icp_pcd_dir}，将跳过 ICP 节点启动，仅启动 map_server（如有）。")
if localization == "small_gicp" and not small_gicp_map_exists:
    print(f"[警告] 选择了 Small-GICP 定位，但未找到 PCD 地图: {small_gicp_pcd_dir}，将跳过 Small-GICP 节点启动，仅启动 map_server（如有）。")
if (localization in ["amcl", "slam_toolbox"] or mode == "nav") and not nav2_map_exists:
    print(f"[警告] 未找到 Nav2 地图 YAML: {nav2_map_dir}，Map Server 可能启动失败。")

# =================================== 点云处理节点定义 =========================================

# 地面分割节点 - 使用线性拟合算法从点云中分离地面和障碍物
bringup_linefit_ground_segmentation_node = Node(
    package="linefit_ground_segmentation_ros",
    executable="ground_segmentation_node",
    output="screen",
    parameters=[segmentation_params],
)

bringup_linefit_ground_segmentation_right_node = None
if dual_lidar_enable:
    bringup_linefit_ground_segmentation_right_node = Node(
        package="linefit_ground_segmentation_ros",
        executable="ground_segmentation_node",
        namespace="right",
        name="ground_segmentation",
        output="screen",
        parameters=[segmentation_right_params],
    )

bringup_linefit_ground_segmentation_nodes = [
    bringup_linefit_ground_segmentation_node
]
if bringup_linefit_ground_segmentation_right_node is not None:
    bringup_linefit_ground_segmentation_nodes.append(
        bringup_linefit_ground_segmentation_right_node
    )

bringup_obstacle_merge_process = None
if dual_lidar_enable and dual_lidar_obstacle_fusion_mode == "merged":
    if not os.path.exists(obstacle_merge_script):
        raise FileNotFoundError(
            f"融合脚本不存在: {obstacle_merge_script}"
        )
    merge_cmd = [
        "/usr/bin/python3",
        obstacle_merge_script,
        "--target-frame",
        "livox_frame",
        "--left-topic",
        "/segmentation/obstacle",
        "--right-topic",
        "/segmentation/obstacle_right",
        "--output-topic",
        "/segmentation/obstacle_merged",
        "--publish-rate",
        "5.0",
        "--max-points-per-cloud",
        "4000",
    ]
    if use_sim:
        merge_cmd.append("--use-sim-time")
    bringup_obstacle_merge_process = ExecuteProcess(
        cmd=merge_cmd,
        output="screen",
    )

# 点云转激光扫描节点 - 将3D点云数据转换为2D激光扫描数据供导航使用
bringup_pointcloud_to_laserscan_node = Node(
    package="pointcloud_to_laserscan",
    executable="pointcloud_to_laserscan_node",
    name="pointcloud_to_laserscan",
    parameters=[pointcloud_to_laserscan_params, {"target_frame": "livox_frame"}],
    remappings=[("cloud_in", scan_cloud_input_topic), ("scan", "/scan")],
)

# IMU互补滤波器 - 融合加速度计和陀螺仪数据提供稳定的姿态估计
start_imu_complementary_filter = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            imu_complementary_filter_launch_dir, "complementary_filter.launch.py"
        )
    )
)

# ================================ LIO算法节点定义 ==========================================

# FAST-LIO节点 - 基于卡尔曼滤波器的快速激光雷达惯性里程计
fast_lio_node = Node(
    package="fast_lio",
    executable="fastlio_mapping",
    parameters=[fastlio_mid360_params, {"use_sim_time": use_sim_time_param}],
    output="screen",
)

# Point-LIO节点 - 基于点特征的激光雷达惯性里程计
point_lio_node = Node(
    package="point_lio",
    executable="pointlio_mapping",
    name="laserMapping",
    output="screen",
    parameters=[pointlio_mid360_params, {"use_sim_time": use_sim_time_param}],
)

# =================================== 定位算法节点定义 =======================================

# SLAM工具箱定位节点 - 使用预先构建的地图进行机器人定位
slam_toolbox_node = Node(
    package="slam_toolbox",
    executable="localization_slam_toolbox_node",
    name="slam_toolbox",
    parameters=[
        slam_toolbox_localization_file_dir,
        {
            "use_sim_time": use_sim_time_param,
            "map_file_name": slam_toolbox_map_dir,  # 预加载的地图文件
            "map_start_pose": [0.0, 0.0, 0.0],  # 地图起始位姿
        },
    ],
)

# AMCL定位启动 - 自适应蒙特卡罗定位算法
start_amcl = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(navigation2_launch_dir, "localization_amcl_launch.py")
    ),
    launch_arguments={
        "use_sim_time": use_sim_time,
        "map": nav2_map_dir,
        "params_file": nav2_params_file_dir,
    }.items(),
)

# ICP定位节点 - 基于迭代最近点算法的点云配准定位
icp_node = None
if icp_map_exists:
    icp_node = Node(
        package="icp_registration",
        executable="icp_registration_node",
        output="screen",
        parameters=[
            icp_registration_params_dir,
            {"use_sim_time": use_sim_time_param, "pcd_path": icp_pcd_dir},  # 点云地图路径
        ],
        # 可选的调试日志级别
        # arguments=['--ros-args', '--log-level', ['icp_registration:=', 'DEBUG']]
    )

# Small GICP定位节点 - 基于高性能小型GICP算法的点云配准定位
small_gicp_node = None
if small_gicp_map_exists:
    small_gicp_node = Node(
        package="small_gicp_registration",
        executable="small_gicp_registration_node",
        output="screen",
        parameters=[
            small_gicp_registration_params_dir,
            {"use_sim_time": use_sim_time_param, "pcd_path": small_gicp_pcd_dir},  # 点云地图路径
        ],
        # 可选的调试日志级别
        # arguments=['--ros-args', '--log-level', ['small_gicp_registration:=', 'DEBUG']]
    )

# 地图服务器启动 - 提供预构建的占用栅格地图
start_map_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(navigation2_launch_dir, "map_server_launch.py")
    ),
    launch_arguments={
        "use_sim_time": use_sim_time,
        "map": nav2_map_dir,
        "params_file": nav2_params_file_dir,
        "container_name": "nav2_container",
    }.items(),
)

# 虚拟速度变换节点 - 为非全向移动底盘提供速度变换
bringup_fake_vel_transform_node = Node(
    package='fake_vel_transform',
    executable='fake_vel_transform_node',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time_param,
        'spin_speed': 0.0  # 旋转速度 (rad/s)
    }]
)

# SLAM工具箱建图节点 - 在线异步SLAM建图
start_mapping_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    parameters=[
        slam_toolbox_mapping_file_dir,
        {'use_sim_time': use_sim_time_param}
    ],
)

# Navigation2导航系统启动 - 完整的路径规划和导航功能
start_navigation2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'map': nav2_map_dir,
        'params_file': nav2_params_file_dir,
    'nav_rviz': nav_rviz}.items()
)

# 里程计坐标系变换 - 建立odom和lidar_odom之间的等价静态坐标变换
odom_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        '--frame-id', 'odom',          # 父坐标系
        '--child-frame-id', 'lidar_odom'  # 子坐标系
    ],
)
# 预创建变量
start_rm_simulation = None
start_robot_state_publisher = None
start_livox_ros_driver2_node = None
# ========================= 仿真环境节点定义 =========================================
if use_sim:

    # 仿真启动文件目录
    pb_rm_simulation_launch_dir = os.path.join(
        get_package_share_directory("pb_rm_simulation"), "launch"
    )

    # RoboMaster仿真环境启动 - 包含Gazebo世界和机器人模型
    start_rm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pb_rm_simulation_launch_dir, "rm_simulation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "robot_description": robot_description_sim,
            "rviz": "False",  # 关闭仿真包自带的RViz
            "gui": "false",   # 关闭 Gazebo GUI（gzclient），避免评估时内存飙升
        }.items(),
    )
    
# ========================= 真实环境节点定义 =========================================
else:
    # ros_driver2参数文件路径
    livox_ros_driver2_params_dir = os.path.join(
        config_dir, "livox_ros_driver2.yaml"
    )
    user_config_path = os.path.join(config_dir, "MID360_config.json")
    # 机器人状态发布器 - 发布机器人的URDF模型和TF变换
    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description_real}],
        output="screen",
    )

    # Livox激光雷达驱动节点 - 驱动Livox MID-360激光雷达
    start_livox_ros_driver2_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[livox_ros_driver2_params_dir, {"user_config_path": user_config_path}],
    )
