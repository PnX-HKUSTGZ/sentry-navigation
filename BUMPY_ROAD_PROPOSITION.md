# 起伏路段导航与分段限速 Proposition（RM 哨兵）

日期：2026-01-12

## 0. 目标

赛场存在起伏路段（明显的 roll/pitch/竖直冲击）。本提案目标是：

- 降低起伏路段对定位与导航稳定性的影响
- 支持“特定路段自动降速 / 出段恢复速度”
- 方案尽量复用现有仓库能力，改动可控、可回滚

---

## 1. 现状证据（来自仓库）

### 1.1 你们已经支持 `pointlio + small_gicp`

在 `rm_nav_bringup` 的启动参数校验中：

- `lio` 支持 `fastlio` / `pointlio`
- `localization` 支持 `amcl` / `slam_toolbox` / `icp` / `small_gicp`

见：
- sentry-navigation/src/rm_nav_bringup/launch/common.py

`bringup.launch.py` 中明确：

- 当 `lio == "pointlio"` 时启动 `point_lio_node`
- 当 `localization == "small_gicp"` 时延迟 7 秒启动 `small_gicp_node`，并启动 `map_server`

见：
- sentry-navigation/src/rm_nav_bringup/launch/bringup.launch.py

`small_gicp_node` 的来源：

- 若 PCD 地图存在（`rm_nav_bringup/PCD/<world>.pcd`），则创建 `small_gicp_registration_node`

见：
- sentry-navigation/src/rm_nav_bringup/launch/common.py

Small-GICP 参数文件示例（含 `range_odom_frame_id: "lidar_odom"` 等）：

- sentry-navigation/src/rm_nav_bringup/config/reality/small_gicp_registration.yaml

### 1.2 你们的速度链路已有“统一限速入口”

Nav2 输出速度经 `velocity_smoother` 再到最终 `cmd_vel`（launch 中做了 remap），因此你们至少可以“全局限速”并得到平滑。

- sentry-navigation/src/rm_navigation/rm_navigation/launch/navigation_launch.py

速度上限配置入口（真实机）：

- `velocity_smoother.ros__parameters.max_velocity: [2.0, 2.0, 3.0]`

见：
- sentry-navigation/src/rm_nav_bringup/config/reality/nav2_params.yaml

另外，TEB 控制器本身也有 `max_vel_x/max_vel_theta` 等上限（同一个 nav2 params 文件内）。

---

## 2. 问题分析：起伏路段对定位/导航的典型影响

> 这里按“你们当前架构”分析：LIO（fastlio/pointlio）提供里程计相关坐标，定位模块（slam_toolbox/amcl/icp/small_gicp）输出 `map->odom`，Nav2 在 `map` 下规划并在 `odom` 下控制。

### 2.1 定位层（LIO + 定位融合）的影响

起伏路段带来：

- IMU 角速度/加速度冲击：预积分误差短时增大；若有饱和会出现突变
- 点云畸变：在抖动与速度叠加下，点云帧内一致性下降
- 2D 假设破坏：如果 2D 激光或 2D costmap 依赖“水平扫描”，起伏会导致地面/坡面被当成障碍或空洞

对不同定位模式的风险排序（经验性结论，需结合实测确认）：

- `amcl`（2D scan 匹配）通常对 pitch/roll 更敏感
- `icp/small_gicp`（点云配准）对点云质量、时间同步与初始位姿更敏感
- `slam_toolbox` 介于两者之间，仍受点云/里程计质量影响

### 2.2 导航控制层的影响

- `odom` 波动会导致局部控制器在短时间内“追着跳” → 表现为抖动、急加减、路径贴边
- 起伏段附着力/轮地接触变化会让底盘实际速度跟不上 `cmd_vel`，导致 progress checker / controller failure（导航失败）

因此起伏段的最有效手段通常是：

- 降低速度/加速度（最直接）
- 给定位留余量（例如提高 transform 容差、适当增大匹配/搜索范围，但要控制误匹配风险）

---

## 3. Proposition A：使用 `pointlio + small_gicp` 作为起伏段优先方案

### 3.1 结论

**可以使用**，仓库启动链路已支持该组合。

### 3.2 使用条件（必须满足）

- `rm_nav_bringup/PCD/<world>.pcd` 必须存在，否则 `small_gicp_node` 不会创建
- `launch_params.yaml` 中设置：
  - `lio: pointlio`
  - `localization: small_gicp`
  - `mode: nav`

配置入口：

- sentry-navigation/src/rm_nav_bringup/config/launch_params.yaml

### 3.3 建议的 Small-GICP 参数关注点（优先按实测调）

参数文件：

- sentry-navigation/src/rm_nav_bringup/config/reality/small_gicp_registration.yaml

优先关注：

- `downsampling_resolution`：起伏段噪声更大时，适当增大可提升稳健性但降低细节
- `max_correspondence_distance`：太小会配不上，太大易误配
- `xy_search_range/yaw_search_range`：起伏冲击造成初值抖动时，适当放宽可提升捕获率（但有误配风险）
- `num_threads/max_iterations/convergence_threshold`：保证实时性与收敛的平衡

---

## 4. Proposition B：特定路段自动降速（两种落地路线）

> 你们当前配置中**未看到**已启用 Nav2 costmap speed filter/keepout filter（未在 nav2_params.yaml 中检索到相关字段），所以本提案给两条可落地路线。

### 4.1 路线 B1（优先）：按区域/路段做“自动限速”

思路：

- 用“区域 mask / 路段标注”驱动速度上限
- 进入起伏区域自动把速度限到较低值，出区域恢复

优点：

- 行为确定、与规划器/控制器解耦
- 不依赖临时手动切参

代价：

- 需要引入/启用 Nav2 costmap filter 相关配置与 mask 资源（工程化工作量中等）

### 4.2 路线 B2（最小代码）：按位置动态改参数

思路：

- 写一个轻量节点：订阅 TF（`map->base_link`），判断是否进入起伏多边形
- 进入起伏区：下调两处上限（推荐“先保守再微调”）
  - `velocity_smoother.max_velocity`
  - TEB 的 `max_vel_x/max_vel_theta`（同 nav2 params 中）
- 出区域：恢复默认上限

优点：

- 不需要改地图资源
- 回滚简单（停掉节点即可）

风险：

- 参数切换需要处理滞后与抖动（建议加 hysteresis：进入/退出边界做缓冲）

已知速度入口（真实机）：

- sentry-navigation/src/rm_nav_bringup/config/reality/nav2_params.yaml
  - `velocity_smoother.ros__parameters.max_velocity: [2.0, 2.0, 3.0]`
  - `controller_server.FollowPath.max_vel_x/max_vel_theta/...`

---

## 5. 推荐的赛场策略（组合拳）

- 起伏段：强制低速通过（先做 B2 最小可行，再考虑 B1 工程化）
- 定位：优先测试 `pointlio + small_gicp`；若对 PCD/初值依赖太强，再对比 `pointlio + slam_toolbox`
- 控制：以 `velocity_smoother` 做最终兜底限速与加速度限制，TEB 上限作为次级约束

---

## 6. 验证清单（上场前必做）

- 确认启动参数：`launch_params.yaml` 生效（启动日志会打印 world/mode/lio/localization）
- 确认 Small-GICP 节点已启动：
  - `ros2 node list | grep -i gicp`
- 确认 `map->odom` 与 `odom->base_link` 连续：
  - `ros2 run tf2_ros tf2_echo map odom`
  - `ros2 run tf2_ros tf2_echo odom base_link`
- 确认速度链路：
  - `ros2 topic echo /cmd_vel`
  - `ros2 topic echo /cmd_vel_nav`（若存在）

---

## 7. 已做的小修复（避免缺地图时 launch 崩溃）

为避免 `small_gicp_node` 在缺 PCD 地图时为 `None` 导致 launch 异常，已在 bringup 中加了防护：

- sentry-navigation/src/rm_nav_bringup/launch/bringup.launch.py

行为：

- 若 `small_gicp_node` 未创建，则跳过 Small-GICP，仅启动 `map_server`。
