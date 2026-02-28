# Mid360 实车部署流程：连接 → LiDAR/IMU 标定 → 先建图后导航

适用仓库：`~/sentry/sentry-navigation`

本流程基于你们当前的启动架构：统一用 `rm_nav_bringup` 启动（实车 `use_sim: false` 时会自动启动 `livox_ros_driver2`、IMU 互补滤波、LIO、SLAM/定位、Nav2）。

---

## 0. 前置条件（一次性）

- 系统：Ubuntu 22.04 + ROS 2 Humble
- 已在 `~/sentry/sentry-navigation` 完成编译

```bash
cd ~/sentry/sentry-navigation
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install
```

每次新终端先 source：

```bash
source ~/sentry/sentry-navigation/install/setup.bash
```

---

## 1. Mid360 连接（网口）

### 1.1 物理连接

- Mid360 网口接入上位机网卡（直连或交换机）
- 确保上位机与雷达在同一网段

### 1.2 IP 配置（按你们当前配置）

你们仓库默认的 Mid360 配置文件为：

- Mid360 配置（主机 IP / 雷达 IP / 端口）：`src/rm_nav_bringup/config/reality/MID360_config.json`
	- 当前示例：主机 `192.168.1.50`，雷达 `192.168.1.147`

修改方法：直接编辑 `MID360_config.json` 内的：

- `host_net_info.*_ip`：上位机网卡 IP
- `lidar_configs[0].ip`：雷达 IP

连通性检查：

```bash
ping -c 3 192.168.1.147
```

### 1.3 驱动参数（发布频率 / frame）

驱动参数文件：`src/rm_nav_bringup/config/reality/livox_ros_driver2.yaml`

常用字段：
- `publish_freq`：点云发布频率（默认 10Hz）
- `frame_id`：默认 `livox_frame`
- `xfer_format: 4`：发布 AllMsg（包含 `CustomMsg` + `PointCloud2` + IMU）

---

## 2. 话题链路验收（标定/建图/导航前必做）

启动整套系统（实车）：

```bash
cd ~/sentry/sentry-navigation
source install/setup.bash
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false
```

验收 Mid360 与 IMU 相关话题：

```bash
ros2 topic list | grep -E "(/livox/lidar$|/livox/lidar/pointcloud$|/livox/imu$)"
```

### 2.1 关于 FastLIO 的 IMU 话题（很关键）

你们当前 `fastlio_mid360.yaml` 订阅的是 `/imu/data`，而 Mid360 驱动输出的是 `/livox/imu`。

系统会通过 IMU 互补滤波器完成转换：

- 输入：`/livox/imu`（被 remap 成 `/imu/data_raw`）
- 输出：`/imu/data`

因此验收时要确认 `/imu/data` 存在且有数据：

```bash
ros2 topic echo -n 1 /imu/data
```

---

## 3. LiDAR–IMU 标定（外参 + 时间偏移）

你们仓库已经预留了“写回点”，标定结果最终要填到 LIO 参数文件里。

### 3.1 需要产出的标定量

- LiDAR 相对 IMU 的外参（在 IMU 机体系下的 LiDAR 位姿）：
	- 平移：`extrinsic_T = [tx, ty, tz]`
	- 旋转：`extrinsic_R`（3×3 旋转矩阵）
- LiDAR–IMU 时间偏移（可选但强烈建议）：
	- FastLIO：`time_offset_lidar_to_imu`
	- PointLIO：`time_diff_lidar_to_imu`

### 3.2 写入位置

**FastLIO（实车）**：`src/rm_nav_bringup/config/reality/fastlio_mid360.yaml`

- `mapping.extrinsic_T`
- `mapping.extrinsic_R`
- `common.time_offset_lidar_to_imu`
- 建议外参确定后保持：`mapping.extrinsic_est_en: false`

**PointLIO（实车）**：`src/rm_nav_bringup/config/reality/pointlio_mid360.yaml`

- `mapping.extrinsic_T`
- `mapping.extrinsic_R`
- `common.time_diff_lidar_to_imu`

### 3.3 标定方法建议（不绑定具体算法）

- 录包（同时录 LiDAR 与 IMU，建议包含 `CustomMsg` 以保留点级时间戳）：
	- `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）
	- `/livox/imu`（或 `/imu/data`）
- 使用你们习惯的 LiDAR–IMU 初始化/同步工具求解外参与时间偏移
- 将结果写回上面的 yaml，重启系统验证：
	- 点云畸变明显下降
	- LIO 轨迹更平滑
	- 静止时漂移更小

### 3.4 另一个“安装位姿”（底盘↔雷达 TF）

`base_link` 到 `livox_frame` 的安装位置请在：

- `src/rm_nav_bringup/config/launch_params.yaml` 的 `base_link2livox_frame.xyz/rpy`

这影响 URDF/TF 与导航坐标系，不等同于 LiDAR–IMU 外参（LIO 内部使用的外参）。

---

## 4. 实车建图（SLAM Toolbox + LIO）

### 4.1 配置（mapping 模式）

编辑：`src/rm_nav_bringup/config/launch_params.yaml`

建议最小改动：
- `use_sim: false`
- `mode: mapping`
- `lio: fastlio`（或 `pointlio`）
- `world: YOUR_MAP_NAME`（建议直接用你希望保存的地图名）

> `mapping` 模式下会启动 `slam_toolbox` 的在线建图节点（见 `rm_nav_bringup/launch/bringup.launch.py`）。

### 4.2 启动建图

```bash
cd ~/sentry/sentry-navigation
source install/setup.bash
ros2 launch rm_nav_bringup bringup.launch.py
```

建图过程中建议检查：

```bash
ros2 topic list | grep -E "(/scan$|/map$|/odom$)"
# 等 60-90 秒让 LIO 稳定
sleep 60
# /map 是否在更新（频率不一定高）
ros2 topic echo -n 1 /map
```

---

## 5. 保存地图（建图完成后）

你们仓库区分两类“地图产物”：

1) **Nav2 栅格地图**：`YOUR_MAP_NAME.yaml` + `YOUR_MAP_NAME.pgm`（用于 Nav2 的 map_server / AMCL 等）
2) **（可选）点云/posegraph**：用于 slam_toolbox 定位模式或 ICP/Small-GICP

### 5.1 保存 Nav2 栅格地图（推荐必做）

你们提供了脚本：`tools/legacy/save_grid_map.sh`，实际命令为：

```bash
ros2 run nav2_map_server map_saver_cli -f src/rm_nav_bringup/map/YOUR_MAP_NAME
```

执行示例：

```bash
cd ~/sentry/sentry-navigation
# 直接运行脚本会提示你替换地图名（脚本本身就是一行命令）
./tools/legacy/save_grid_map.sh
```

产物位置：
- `src/rm_nav_bringup/map/YOUR_MAP_NAME.yaml`
- `src/rm_nav_bringup/map/YOUR_MAP_NAME.pgm`

### 5.2 （可选）保存 FastLIO 点云地图（给 ICP/Small-GICP 用）

你们提供了脚本：`tools/legacy/save_pcd.sh`，会调用 FastLIO 的服务：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

注意：这是 FastLIO 内部保存点云（不是 nav2 的栅格）。

### 5.3 （推荐）保存 slam_toolbox posegraph（给 slam_toolbox 定位用）

你们的地图目录里已有 `.posegraph` 文件（例如 `RMUC_24.posegraph`），因此建议把建图完成后的 posegraph 也保存到：

- `src/rm_nav_bringup/map/YOUR_MAP_NAME.posegraph`

不同版本 slam_toolbox 的保存服务名/类型可能略有差异，建议按如下方式发现并调用：

```bash
# 找 slam_toolbox 相关服务
ros2 service list | grep slam_toolbox

# 常见服务名包含 serialize
ros2 service list | grep -E "serialize|save" | grep slam_toolbox

# 查看服务类型（用你实际机器上的服务名替换）
ros2 service type /slam_toolbox/serialize_map

# 查看接口定义
ros2 interface show $(ros2 service type /slam_toolbox/serialize_map)
```

---

## 6. 切换到实车导航（Nav2）

### 6.1 配置（nav 模式）

编辑：`src/rm_nav_bringup/config/launch_params.yaml`

建议最小改动：
- `use_sim: false`
- `mode: nav`
- `world: YOUR_MAP_NAME`（必须与 `src/rm_nav_bringup/map/YOUR_MAP_NAME.yaml` 对应）
- `lio: fastlio` 或 `pointlio`
- `localization` 推荐顺序：
	1) `slam_toolbox`（需要 `.posegraph`，定位通常更稳）
	2) `amcl`（只需要 `.yaml/.pgm`，但通常需要 RViz 手动初始位姿）

### 6.2 启动导航

```bash
cd ~/sentry/sentry-navigation
source install/setup.bash
ros2 launch rm_nav_bringup bringup.launch.py
```

等待 60-90 秒后验收：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic list | grep -E "(odom|scan|map|cmd_vel)"
```

如果使用 AMCL：
- 在 RViz 使用 `2D Pose Estimate` 设置初始位姿

---

## 7. 常见问题速查

### 7.1 驱动有数据但 FastLIO 不动

- 检查 `/imu/data` 是否有数据（互补滤波输出）：
	- `ros2 topic echo -n 1 /imu/data`
- 检查 `/livox/lidar` 是否存在（CustomMsg）：
	- `ros2 topic list | grep /livox/lidar$`

### 7.2 只想验证“驱动 + 话题”

也可以单独启动驱动（不建议长期这样跑，推荐仍用 bringup）：

```bash
source ~/sentry/sentry-navigation/install/setup.bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
	--ros-args --params-file src/rm_nav_bringup/config/reality/livox_ros_driver2.yaml \
	-p user_config_path:=src/rm_nav_bringup/config/reality/MID360_config.json
```

### 7.3 FastDDS SHM 报错

通常仿真更常见；如遇到可用 wrapper：

```bash
cd ~/sentry/sentry-navigation
./tools/legacy/launch_with_fastdds_fix.sh nav_rviz:=false
```

---

## 8. 实车控制（底盘/电控）

这部分决定“导航给出速度指令后，车是否真的动起来”。

### 8.1 控制链路（你们当前架构的真实输出）

你们的 Nav2 输出并不是直接给电控用的 `/cmd_vel`，中间还有一层坐标系兼容：

1. Nav2（`velocity_smoother`）发布：`/cmd_vel`（基于 `base_link_fake` 的速度）
2. `fake_vel_transform_node` 订阅 `/cmd_vel`，结合 `/local_plan` 与 TF，把速度转换到 `base_link`
3. `fake_vel_transform_node` 发布最终给底盘/电控执行的速度：`/cmd_vel_chassis`

说明：`fake_vel_transform` 的设计目的是在“云台 yaw 与底盘 yaw 不一致/小陀螺”时仍能稳定跟踪轨迹。

### 8.2 电控/底盘驱动需要提供什么接口？

在本仓库 `sentry-navigation/src/rm_driver/` 下只有 `livox_ros_driver2`，**没有**底盘串口/CAN 驱动。

因此实车运行时，你需要另外启动（或由电控板/上位机桥接提供）一个“底盘驱动节点”，至少做到：

- **订阅** `geometry_msgs/msg/Twist`：`/cmd_vel_chassis`
- **发布** `nav_msgs/msg/Odometry`：默认参数里 Nav2 期待的是 `/Odometry`（注意大小写）
- **发布 TF**：`odom -> base_link`（实时变换）

如果你们底盘驱动只能发布 `/odom` 而不是 `/Odometry`，有两种解决方案：

1) 修改底盘驱动发布为 `/Odometry`（最直接）
2) 修改 `src/rm_nav_bringup/config/reality/nav2_params.yaml` 里的 `odom_topic` 以匹配你实际话题（需要全局一致）

### 8.3 最小验收（不跑导航也能测）

**A. 验证 Nav2 是否在出速度**（跑起来后才会有）：

```bash
ros2 topic echo /cmd_vel --once
```

**B. 验证转换后的底盘速度是否在出**：

```bash
ros2 topic echo /cmd_vel_chassis --once
```

**C. 直接下发测试速度（危险，务必顶起车/留安全距离/随时急停）**

```bash
ros2 topic pub -r 10 /cmd_vel_chassis geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果底盘不动：

- 先确认底盘驱动节点是否在订阅：`ros2 topic info /cmd_vel_chassis`
- 再确认底盘是否在发布里程计：`ros2 topic echo -n 1 /Odometry`
- 再确认 TF：`ros2 run tf2_ros tf2_echo odom base_link`

### 8.4 关于 `spin_speed`（小陀螺配合）

`fake_vel_transform_node` 有参数 `spin_speed`（默认 -6.0，且在 `rm_nav_bringup/launch/common.py` 里当前被设为 `0.0`）。

- 如果你们电控端有“固定小陀螺角速度”，且希望移动时减缓/抵消，可以把 `spin_speed` 设为负值
- 否则保持 `0.0` 更安全

---

## 9. 最小“照抄”指令（总结）

1) 配好 `MID360_config.json` 的主机 IP/雷达 IP
2) `launch_params.yaml` 设 `use_sim:false + mode:mapping + world:YOUR_MAP_NAME`
3) 启动：`ros2 launch rm_nav_bringup bringup.launch.py`
4) 保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f src/rm_nav_bringup/map/YOUR_MAP_NAME`
5) 切换 `launch_params.yaml` 为 `mode:nav + world:YOUR_MAP_NAME`，选择 `localization`
6) 再启动：`ros2 launch rm_nav_bringup bringup.launch.py`
