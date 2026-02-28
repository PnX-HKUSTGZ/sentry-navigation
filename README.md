# 哨兵导航系统 (sentry-navigation)

深圳北理莫斯科大学北极熊战队哨兵导航仿真/实车工作区（ROS 2 Humble）。

本仓库当前主线用于以下目标：
- 单入口启动导航栈（`bringup.launch.py`）
- 支持 `FAST-LIO2 / Point-LIO + Nav2 + (AMCL / ICP / small-gicp / slam_toolbox)`
- 支持 RMUL_26 及波浪路段回归（含动态障碍回归工具）

## 文档导航

- 文档总索引：`docs/INDEX.md`
- RMUL_26 联调最小可复现：`docs/RMUL26_JOINT_REPRO.md`
- 固定发布说明（跨仓库版本钉住）：`docs/RMUL26_REPRO_RELEASE.md`
- 波浪路段稳定性测试：`docs/WAVE_ROAD_STABILITY_TEST.md`
- Fast-LIO2 + DWB + small-gicp 收敛报告：`docs/SMALL_GICP_NAV_CLOSURE.md`
- 自动评估模块说明：`src/rm_nav_bringup/evaluation/README.md`
- 历史文档归档：`docs/legacy/`
- 历史兼容脚本归档：`tools/legacy/`

## 1. 当前默认配置（以代码为准）

配置文件：`src/rm_nav_bringup/config/launch_params.yaml`

当前默认值（2026-02）：
- `mode: nav`
- `use_sim: true`
- `world: RMUL_26`
- `lio: fastlio`
- `localization: slam_toolbox`
- `controller: dwb`
- `use_stvl: true`（全局代价地图 STVL 开关）
- `obstacle_profile: default`（障碍过滤模板）

注意：README 中所有示例都按以上默认主线编写；如你本地修改了 `launch_params.yaml`，以你的本地配置为准。

## 2. 环境与依赖

基线：Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic。

### 2.1 方案 A：RoboStack（推荐）

```bash
cd sentry-navigation

# 1) 创建/更新 robostack 环境并编译
bash tools/setup_robostack_env.sh

# 2) 每个新终端激活
source tools/activate_robostack.sh

# 3) 环境自检
bash tools/check_robostack_env.sh
```

可选组件：
```bash
# 安全启动包装（防 conda 污染 + FastDDS SHM 问题）
bash tools/launch_nav_safe.sh

# Foxglove bridge
bash tools/run_foxglove_bridge.sh
```

### 2.2 方案 B：系统 ROS（兼容）

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

## 3. 启动（单入口约束）

正式联调时只使用：
- `ros2 launch rm_nav_bringup bringup.launch.py`

不要额外再起第二套 `map_server / lifecycle_manager / amcl / nav2`，否则 ROS 图会重复并导致行为不确定。

### 3.1 最小启动命令

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup.launch.py
```

### 3.2 常用覆盖参数

```bash
ros2 launch rm_nav_bringup bringup.launch.py \
  world:=RMUL_26_WAVE \
  map:=RMUL26_WAVE \
  lio:=fastlio \
  localization:=small_gicp \
  controller:=dwb \
  use_stvl:=true \
  obstacle_profile:=default \
  nav_rviz:=false
```

支持的覆盖参数：
- `world:=...`（仿真 world）
- `map:=...`（地图/PCD 资源名）
- `lio:=fastlio|pointlio`
- `localization:=amcl|slam_toolbox|icp|small_gicp`
- `controller:=teb|dwb`
- `use_stvl:=true|false`（全局 costmap STVL 开关，仿真/实车均可）
- `obstacle_profile:=default|anti_self|anti_self_strict|anti_self_lift_test|anti_self_dualspot|anti_self_lowload|anti_self_wiring`（障碍过滤模板，按 `config/<mode>` 下 profile 文件加载）
- `nav_rviz:=true|false`
- `nav_start_delay:=<seconds>`
- `lidar_noise_stddev:=<float>`

常用 profile 对应文件（实车）：
- `anti_self`：`src/rm_nav_bringup/config/reality/nav2_obstacle_profile_anti_self.yaml` + `src/rm_nav_bringup/config/reality/pointcloud_to_laserscan_anti_self.yaml`
- `anti_self_dualspot`：`src/rm_nav_bringup/config/reality/nav2_obstacle_profile_anti_self_dualspot.yaml` + `src/rm_nav_bringup/config/reality/pointcloud_to_laserscan_anti_self_dualspot.yaml`
- `anti_self_lowload`：`src/rm_nav_bringup/config/reality/nav2_obstacle_profile_anti_self_lowload.yaml` + `src/rm_nav_bringup/config/reality/pointcloud_to_laserscan_anti_self_lowload.yaml`
- `anti_self_wiring`：`src/rm_nav_bringup/config/reality/nav2_obstacle_profile_anti_self_wiring.yaml` + `src/rm_nav_bringup/config/reality/pointcloud_to_laserscan_anti_self_wiring.yaml`

## 4. 关键运行约定

### 4.1 map/world 资源名

- world 名和 map 资源名可以不同。
- 例如：`world:=RMUL_26`，`map:=RMUL26`。
- map 资源由 `src/rm_nav_bringup/map/<name>.yaml` 指向 `.pgm`。
- ICP/small-gicp 资源在 `src/rm_nav_bringup/PCD/<name>.pcd`。

### 4.2 base_link_fake 机制

默认 Nav2 控制参考系为 `base_link_fake`（配套 `fake_vel_transform`）：
- 真实姿态仍来自 `base_link`
- `base_link_fake` 用于控制层解耦（尤其是 yaw）

快速自检：
```bash
bash tools/check_frame_contract.sh
ros2 run tf2_ros tf2_echo base_link base_link_fake
```

### 4.3 AMCL 初始位姿 QoS

在仿真里发 `/initialpose` 时请使用：
- `reliability=BEST_EFFORT`
- `durability=VOLATILE`

否则可能出现 AMCL 未吃到初始位姿。

## 5. 回归测试入口

### 5.1 单目标复现

```bash
bash tools/repro_nav2_sim_goal.sh
```

### 5.2 多目标压力与动态障碍

```bash
ART=artifacts/nav_stress_$(date +%Y%m%d_%H%M%S)
ROUNDS=10 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=small_gicp \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
BRINGUP_LIO=fastlio \
NAV_USE_STVL=1 \
DYNAMIC_OBS_ENABLE=1 \
DYNAMIC_OBS_MODE=cloud \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

### 5.3 自动评估

```bash
./tools/legacy/run_evaluation.sh --single-test
```

### 5.4 波浪路段“轨迹契约”一键回归（推荐）

目标：避免“Action 成功但没真正经过 8 个起伏”的误判。  
脚本会自动完成：
- 运行单入口导航回归（`stress_dynamic_nav.sh`）
- 生成 `analysis_drift/loc_vs_ground_truth.csv`
- 判定是否真实覆盖波浪段（按 `estimated_waves_crossed`）
- 计算刚体对齐后的定位误差（过滤固定坐标偏置）
- 输出 `contract_pass=1|0`

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

bash tools/run_wave8_contract_regression.sh
```

默认契约门限（可通过环境变量覆盖）：
- `estimated_waves_crossed >= 7.5`
- `inside_time_span_s >= 2.0`
- `aligned_err_wave_p95_m <= 0.35`
- `aligned_err_wave_max_m <= 0.55`
- `summary_succeeded >= 1`（可通过 `REQUIRE_ACTION_SUCCESS=0` 关闭）

结果文件：
- 主报告：`artifacts/wave8_contract_<timestamp>/contract_report.txt`
- Nav 过程：`artifacts/wave8_contract_<timestamp>/wave8_contract_stress.log`
- 原始压力摘要：`artifacts/wave8_contract_<timestamp>/summary.txt`
- 漂移图表：`artifacts/wave8_contract_<timestamp>/analysis_drift/`

## 6. 快速验收清单

```bash
# Action server
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose

# 定位与 TF
ros2 topic echo -n 1 /amcl_pose
ros2 topic hz /tf

# 节点重复检查（应避免重复 nav2/amcl/map_server）
ros2 node list
```

## 7. 常见问题

1. `navigate_to_pose` 不出现
- 检查是否只用单入口 bringup。
- 检查 `bringup.log` 是否卡在 lifecycle。
- 适当增大 `nav_start_delay` 和 `ACTION_WAIT_TIMEOUT`（压力脚本中可配）。

2. AMCL 没收到初始位姿
- 使用正确 QoS 重发 `/initialpose`。
- 等待 `map->odom` 稳定后再发 goal。

3. small-gicp 启动但漂移或长时间不收敛
- 确认 map 对应 PCD 存在。
- 查看 `small_gicp_registration` 日志中的 `score` 与 `recovery mode`。
- 优先参考 `docs/SMALL_GICP_NAV_CLOSURE.md` 的参数与命令矩阵。

4. 动态避障不明显
- 确认 `DYNAMIC_OBS_ENABLE=1` 且 `dynamic_obstacle.log` 有持续发布记录。
- 检查 local costmap 是否包含 `obstacle_layer`，并确认障碍源 topic 有数据。

5. 多 RViz 运行一段时间后地图停止更新（`queue is full` / `timestamp earlier than all data`）
- 先只保留 1 个 RViz，关闭不必要显示（PointCloud2 / Path / Costmap）。
- 启动时切换低负载模板：
  `ros2 launch rm_nav_bringup bringup.launch.py obstacle_profile:=anti_self_lowload nav_rviz:=false`
- 若仍有掉包，检查 CPU 占用与 TF 链时延：
  `ros2 topic hz /scan`、`ros2 topic hz /tf`、`ros2 run tf2_ros tf2_monitor`

## 8. 实车迁移提示

真实环境请重点校准：
- `src/rm_nav_bringup/config/reality/MID360_config.json`（雷达网络与外参）
- `src/rm_nav_bringup/config/reality/segmentation.yaml` / `segmentation_right.yaml`（地面分割）
- `src/rm_nav_bringup/config/reality/nav2_params.yaml`（半径、速度、代价地图）
- `src/rm_nav_bringup/config/reality/nav2_obstacle_profile_anti_self.yaml`（抗自体伪障碍模板）
- `src/rm_nav_bringup/config/reality/pointcloud_to_laserscan_anti_self.yaml`（近场自体过滤模板）

## 9. 致谢

- FAST-LIO、Point-LIO、Nav2、small_gicp、slam_toolbox、Livox SDK2 及相关开源社区。
- 历史背景、开发心得和详细致谢沿用仓库历史版本说明。
