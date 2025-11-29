import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
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
world = launch_params.get("world", "RMUL")  # 获取世界名称，默认为RMUL
mode = launch_params.get("mode", "nav")  # 获取运行模式 (mapping/nav)，默认为nav
lio = launch_params.get("lio", "fastlio")  # 激光雷达惯性里程计 (pointlio/fastlio)，默认为fastlio
localization = launch_params.get("localization", "slam_toolbox")  # 获取定位模式 (amcl/slam_toolbox/icp)
use_sim = launch_params.get("use_sim", False)  # 是否使用仿真，默认为False
use_sim_time = LaunchConfiguration('use_sim_time', default=str(use_sim))  # 供ROS使用
use_lio_rviz = launch_params.get("use_lio_rviz", False)  # 可视化 FAST_LIO 或 Point_LIO 的点云图

# 参数验证
valid_modes = ["mapping", "nav"]
valid_lio_types = ["fastlio", "pointlio"]
valid_localization_types = ["amcl", "slam_toolbox", "icp", "small_gicp"]

if mode not in valid_modes:
    raise ValueError(f"无效的mode参数: {mode}. 有效值: {valid_modes}")
if lio not in valid_lio_types:
    raise ValueError(f"无效的lio参数: {lio}. 有效值: {valid_lio_types}")
if localization not in valid_localization_types:
    raise ValueError(f"无效的localization参数: {localization}. 有效值: {valid_localization_types}")

print("启动参数配置:")
print(f"  世界环境: {world}")
print(f"  运行模式: {mode}")
print(f"  LIO算法: {lio}")
print(f"  定位方法: {localization}")
print(f"  仿真模式: {use_sim}")
print(f"  LIO可视化: {use_lio_rviz}")

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
            launch_params["base_link2livox_frame"]["xyz"],
            " rpy:=",
            launch_params["base_link2livox_frame"]["rpy"],
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
            launch_params["base_link2livox_frame"]["xyz"],
            " rpy:=",
            launch_params["base_link2livox_frame"]["rpy"],
        ]
    )

# =========================== pointcloud pretreatment parameters ==========================
segmentation_params = os.path.join(config_dir, "segmentation.yaml")
pointcloud_to_laserscan_params = os.path.join(
    config_dir, "pointcloud_to_laserscan.yaml"
)

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
slam_toolbox_map_dir = os.path.join(rm_nav_bringup_dir, "map", world)
slam_toolbox_localization_file_dir = os.path.join(
    config_dir, "mapper_params_localization.yaml"
)
slam_toolbox_mapping_file_dir = os.path.join(
    config_dir, "mapper_params_online_async.yaml"
)

# ================================= navigation2 parameters =================================
nav2_map_dir = os.path.join(rm_nav_bringup_dir, "map", world + ".yaml")
nav2_params_file_dir = os.path.join(config_dir, "nav2_params.yaml")

# =============================== icp_registration parameters ==============================
icp_pcd_dir = os.path.join(rm_nav_bringup_dir, "PCD", world + ".pcd")
icp_registration_params_dir = os.path.join(
    config_dir, "icp_registration.yaml"
)

# ============================= small_gicp_registration parameters ========================
small_gicp_pcd_dir = os.path.join(rm_nav_bringup_dir, "PCD", world + ".pcd")
small_gicp_registration_params_dir = os.path.join(
    config_dir, "small_gicp_registration.yaml"
)

# =============================== 地图存在性检查（容错机制） ===================================
icp_map_exists = os.path.exists(icp_pcd_dir)
nav2_map_exists = os.path.exists(nav2_map_dir)
slam_map_exists = os.path.exists(slam_toolbox_map_dir) or os.path.exists(slam_toolbox_map_dir + ".posegraph")
small_gicp_map_exists = os.path.exists(small_gicp_pcd_dir)

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

# 点云转激光扫描节点 - 将3D点云数据转换为2D激光扫描数据供导航使用
bringup_pointcloud_to_laserscan_node = Node(
    package="pointcloud_to_laserscan",
    executable="pointcloud_to_laserscan_node",
    name="pointcloud_to_laserscan",
    parameters=[pointcloud_to_laserscan_params],
    remappings=[("cloud_in", "/segmentation/obstacle"), ("scan", "/scan")],
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
    parameters=[fastlio_mid360_params, {use_sim_time: use_sim_time}],
    output="screen",
)

# Point-LIO节点 - 基于点特征的激光雷达惯性里程计
point_lio_node = Node(
    package="point_lio",
    executable="pointlio_mapping",
    name="laserMapping",
    output="screen",
    parameters=[pointlio_mid360_params, {"use_sim_time": use_sim_time}],
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
            "use_sim_time": use_sim_time,
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
            {"use_sim_time": use_sim_time, "pcd_path": icp_pcd_dir},  # 点云地图路径
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
            {"use_sim_time": use_sim_time, "pcd_path": small_gicp_pcd_dir},  # 点云地图路径
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
        'use_sim_time': use_sim_time,
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
        {'use_sim_time': use_sim_time}
    ],
)

# Navigation2导航系统启动 - 完整的路径规划和导航功能
start_navigation2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'map': nav2_map_dir,
        'params_file': nav2_params_file_dir,
        'nav_rviz': 'true'}.items()
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


