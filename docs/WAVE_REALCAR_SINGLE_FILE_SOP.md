# 起伏路段实车单文件测试 SOP（当天可执行）

适用仓库：`sentry-navigation`  
ROS：`ROS 2 Humble`  
目标：先验证“能过”（A），再验证“导航误差小”（B），并按统一验收口径（C）出结论。  

---

## 0. 测试目标与输出

本 SOP 一次性回答三个问题：

1. 底盘+感知链路是否能稳定通过起伏路段（无地图预验）。
2. 在最小局部地图下，导航 A->B->A 是否满足小误差与成功率。
3. 是否达到统一门槛（>=95% 成功率、<=0.25m 终点误差、无致命故障）。

本次测试结束后至少输出：
- 1 个 rosbag（A 阶段）
- 1 个 rosbag（B 阶段）
- 1 份结果记录表（见文末模板）

---

## 1. 前置准备（统一）

### 1.1 环境

```bash
cd /data/home/sim6g/sentry/sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 1.2 安全与急停

建议单独开一个终端常驻急停命令：

```bash
ros2 topic pub --once /cmd_vel_chassis geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

### 1.3 推荐启动参数（实车 anti-self）

今天实车建议用 `anti_self` 抗自体伪障碍模板：

```bash
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false obstacle_profile:=anti_self
```

> 注：本 SOP 所有步骤都遵循“单入口 bringup”，不要手工另起第二套 map_server/lifecycle/nav2。

---

## 2. A 阶段：无地图通过性预验（建图模式）

目标：验证“底盘+感知链路能不能过起伏段”，不考核导航到点误差。

### 2.1 配置切换（`launch_params.yaml`）

文件：`src/rm_nav_bringup/config/launch_params.yaml`

```yaml
use_sim: false
mode: mapping
lio: fastlio
localization: slam_toolbox
controller: dwb
```

### 2.2 启动

```bash
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false obstacle_profile:=anti_self
```

### 2.3 30 秒预检查（避免无效测试）

```bash
ros2 topic hz /imu/data
ros2 topic hz /Odometry
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link livox_frame
```

通过标准（建议）：
- `/imu/data`、`/Odometry`、`/scan` 频率稳定，无明显掉频到 0。
- `base_link -> livox_frame` 可持续查询，不中断。

### 2.4 录包（A 阶段）

> 实车通常没有 `/ground_truth/odom`，有就录；没有也不影响 A 阶段结论。

```bash
ros2 bag record \
  /Odometry /imu/data /tf /tf_static /scan /map /cmd_vel_chassis \
  -o /tmp/wave_mapping_test
```

如确认有 `/ground_truth/odom`，可加上：

```bash
ros2 bag record \
  /Odometry /ground_truth/odom /imu/data /tf /tf_static /scan /map /cmd_vel_chassis \
  -o /tmp/wave_mapping_test
```

### 2.5 固定速度跨越 + 倒回（建议用 timeout 防误操作）

```bash
# 前进跨越（示例 8 秒，约 4m）
timeout 8 ros2 topic pub --rate 20 /cmd_vel_chassis geometry_msgs/msg/Twist \
"{linear: {x: 0.50, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel_chassis geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# 倒回（示例 10 秒，约 4m）
timeout 10 ros2 topic pub --rate 20 /cmd_vel_chassis geometry_msgs/msg/Twist \
"{linear: {x: -0.40, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel_chassis geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

### 2.6 A 阶段判定（通过性）

通过条件：
- 无人工接管；
- 无卡死超过 2 秒；
- 轨迹最大位移覆盖完整起伏段长度；
- IMU 无异常突跳、姿态不发散。

说明：A 阶段只回答“能不能过”，不评价导航点位精度。

---

## 3. B 阶段：最小局部地图导航小误差验证（推荐）

目标：验证 A->B->A 重复导航的成功率与终点误差。

### 3.1 场地布置（最小局部地图）

- 测试区建议：约 `4m x 6m`。
- 在起伏段周边放 `3~4` 个竖直特征物（纸箱/反光板）提升约束。
- 标定地标：起点 A、终点 B（卷尺可测）。

### 3.2 建图并保存

先保持 A 阶段配置（`mode: mapping`）跑一圈后保存栅格：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/wave_local_map
```

把以下文件放到地图目录：

```bash
cp /tmp/wave_local_map.yaml src/rm_nav_bringup/map/WAVE_LOCAL.yaml
cp /tmp/wave_local_map.pgm  src/rm_nav_bringup/map/WAVE_LOCAL.pgm
```

### 3.3 切换导航模式

修改 `src/rm_nav_bringup/config/launch_params.yaml`：

```yaml
use_sim: false
mode: nav
localization: amcl
world: WAVE_LOCAL
controller: dwb
```

启动：

```bash
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false obstacle_profile:=anti_self
```

### 3.4 AMCL 初始化（必做）

每次正式发导航目标前，发布一次 `/initialpose`（BEST_EFFORT + VOLATILE）：

```bash
ros2 topic pub --rate 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: AX, y: AY, z: 0.0}, orientation: {z: AQZ, w: AQW}}}}" \
--qos-reliability best_effort --qos-durability volatile
```

> 把 `AX/AY/AQZ/AQW` 替换为 A 点初始位姿。

### 3.5 B 阶段录包（导航）

```bash
ros2 bag record \
  /Odometry /amcl_pose /tf /tf_static /scan /map /cmd_vel_chassis /plan \
  -o /tmp/wave_nav_test
```

### 3.6 A->B->A 重复 20 goals

单次发送示例（A/B 两点替换坐标）：

```bash
# A -> B
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: BX, y: BY, z: 0.0}, orientation: {z: BQZ, w: BQW}}}}"

# B -> A
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: AX, y: AY, z: 0.0}, orientation: {z: AQZ, w: AQW}}}}"
```

建议总计执行 20 次 goal（A/B 往返）。

### 3.7 B 阶段判定（导航小误差）

按当前参数口径：
- `xy_goal_tolerance = 0.25m`
- 终点误差验收：`<= 0.25m`

---

## 4. C 阶段：统一验收门槛

### 4.1 通过性（A 阶段）

- 单向跨越成功率 `>= 95%`（20 次至少 19 次）。

### 4.2 导航到点（B 阶段）

- A/B 往返 20 goals 成功率 `>= 95%`。
- 终点误差 `<= 0.25m`。

### 4.3 致命故障

- 无长期 TF 断链。
- 无 planner 崩溃。
- 无持续不可恢复卡死。

---

## 5. 结果记录模板（可直接复制）

```text
[测试批次]
日期：
场地：
车辆：
操作者：

[配置]
use_sim=false
mode=mapping/nav
lio=fastlio
localization=slam_toolbox/amcl
controller=dwb
obstacle_profile=anti_self

[A阶段 通过性]
尝试次数：
成功次数：
成功率：
是否有人工接管：
最长卡死时长：
备注：

[B阶段 导航]
总goals：
成功goals：
成功率：
终点误差(平均/p95/最大)：
是否出现TF断链：
是否出现planner崩溃：
备注：

[结论]
是否通过A阶段：
是否通过B阶段：
是否达到C门槛：
```

---

## 6. 常见失败与快速处理

1. `/navigate_to_pose` 不可用  
- 检查是否只用了单入口 `bringup.launch.py`。  
- 重启并先发 `/initialpose` 后再发 goal。

2. AMCL 不稳定或不收敛  
- 确保 `/initialpose` 使用 `BEST_EFFORT + VOLATILE`。  
- 增加场地竖直特征物，减少纯平/纯重复纹理区域。

3. 伪障碍导致诡异绕行  
- 确保 `obstacle_profile:=anti_self`。  
- 检查雷达安装角度、线束遮挡、近场反射。

4. 多 RViz 导致高负载  
- 测试时建议 `nav_rviz:=false`，只保留必要可视化。

---

## 7. 结束收尾（必做）

```bash
# 停车
ros2 topic pub --once /cmd_vel_chassis geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# 停止录包（Ctrl+C）
# 保存本次测试记录与 bag 路径
```

