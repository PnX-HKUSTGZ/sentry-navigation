# RMUL_26_WAVE 导航稳定性测试（波峰间距 240mm，峰谷差 70mm）

## 1. 测试对象

- 仿真 world：`RMUL_26_WAVE`
- 栅格地图：`RMUL26_WAVE`（别名，引用 `RMUL2026.pgm`）
- 路面参数：
  - 波峰间距（波长）`0.24 m`
  - 峰谷差 `0.07 m`
- 目标点往返：
  - `(-3.8, 3.3)` 与 `(-5.6, 3.3)`（覆盖波浪带）

## 2. 运行命令（可复现）

```bash
cd /data/home/sim6g/sentry/sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

# 构建（首次或改动后）
colcon build --packages-select pb_rm_simulation rm_nav_bringup --symlink-install

# 若要启用 Point-LIO 对比，先构建 point_lio
colcon build --packages-select point_lio --symlink-install
```

```bash
# 统一测试模板（示例：fastlio + teb）
ART=/tmp/wave_fastlio_teb_$(date +%Y%m%d_%H%M%S)
ROUNDS=5 \
WAYPOINTS='-3.8,3.3,0.0,1.0; -5.6,3.3,0.0,1.0' \
GOAL_TIMEOUT=120 \
BETWEEN_GOALS_SEC=0.5 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=amcl \
CONTROLLER=teb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
BRINGUP_LIO=fastlio \
NAV_RVIZ=false \
STARTUP_SETTLE_SEC=5 \
ACTION_WAIT_TIMEOUT=300 \
USE_ROBOSTACK=0 \
GOAL_OCCUPANCY_POLICY=snap \
GOAL_NEAREST_RADIUS=1.5 \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

说明：

- `GOAL_OCCUPANCY_POLICY=snap` 可自动把落在障碍/未知栅格的目标点吸附到最近自由栅格，减少“坏点”导致的假失败。
- 若只想严格过滤（不自动改点），改为 `GOAL_OCCUPANCY_POLICY=reject`。

## 3. 测试结果（2026-02-19）

### Case A: `fastlio + dwb`

- 工件：`/tmp/wave_fastlio_dwb_20260219_182802`
- 结果：
  - `total=10`
  - `succeeded=1`
  - `aborted=8`
  - `timeout=1`
  - `success_rate=10.0%`
- 关键日志计数：
  - `Starting point in lethal space = 121`
  - `No valid trajectories out of = 0`
  - `Controller patience exceeded = 0`

### Case B: `fastlio + teb`

- 工件：`/tmp/wave_fastlio_teb_20260219_183919`
- 结果：
  - `total=10`
  - `succeeded=5`
  - `aborted=5`
  - `timeout=0`
  - `success_rate=50.0%`
- 关键日志计数：
  - `Starting point in lethal space = 0`
  - `No valid trajectories out of = 0`
  - `Controller patience exceeded = 74`

### Case C: `pointlio + dwb`

- 工件：`/tmp/wave_pointlio_dwb2_20260219_185256`
- 结果：
  - `total=10`
  - `succeeded=6`
  - `aborted=2`
  - `timeout=2`
  - `success_rate=60.0%`
- 关键日志计数：
  - `Starting point in lethal space = 44`
  - `No valid trajectories out of = 5`
  - `Controller patience exceeded = 0`

### Case D: `pointlio + teb`

- 工件：`/tmp/wave_pointlio_teb_20260219_233706`
- 结果：
  - `total=10`
  - `succeeded=5`
  - `aborted=5`
  - `timeout=0`
  - `success_rate=50.0%`
- 关键日志计数：
  - `Starting point in lethal space = 3`
  - `No valid trajectories out of = 0`
  - `Controller patience exceeded = 80`

## 4. 结论

- 在该波浪路面上，`fastlio + dwb` 稳定性最差（10% 成功率）。
- 当前最佳组合是 `pointlio + dwb`（60%），但仍不满足稳定导航要求。
- `teb` 在 `fastlio/pointlio` 下均为 50%，主要失败特征是 `Controller patience exceeded` 偏高。
- 波浪段当前存在明显方向性差异：往 `(-3.8, 3.3)` 方向通过率显著高于返程 `(-5.6, 3.3)`。

## 5. 后续建议（最小改动优先）

1. 固定 `pointlio + teb` 组合做下一轮同口径复测（同样 10 goals）。
2. 在波浪带区域单独调 `costmap` 观测与膨胀参数，优先压低 `lethal space` 触发。
3. 再做一轮 `DWB/TEB` 对比，判定是否需要引入分段速度策略。
