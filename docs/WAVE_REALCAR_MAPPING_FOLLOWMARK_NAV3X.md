# 实车流程：建图 -> 标注起伏区 -> Follow Mark 自动发布 -> 导航往返

适用仓库：`/home/pnx/nav_ws/sentry-navigation`  
串口联动仓库：`/home/pnx/pnx_autoaim/src/rm_serial_driver`

目标：在实车中完成以下闭环：

1. 建图并保存地图（`yaml/pgm`）+ 点云地图（`pcd`）
2. 在地图中标注起伏路段区域（`zone_polygons`）
3. 导航运行时自动发布 `/chassis/follow_mark`：
   - `0`：底盘不跟随（进入起伏区）
   - `1`：底盘跟随（区外默认）
4. 串口侧按 `SendNavPacketV2` 下发 `follow_mark`，断流超时回退 `1`

---

## 1. 语义与协议约定（必须统一）

### 1.1 Follow Mark 语义

- `0`：底盘不跟随
- `1`：底盘跟随

### 1.2 串口协议（Nav V2）

`SendNavPacketV2` 字段顺序：

1. `header (0xA6)`
2. `follow_mark`
3. `linear_x/y/z`
4. `angular_x/y/z`
5. `checksum`

### 1.3 超时回退

当 `/chassis/follow_mark` 未收到或超时时，串口侧回退到 `1`（跟随）。

---

## 2. 建图模式启动（mapping）

```bash
cd /home/pnx/nav_ws/sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup.launch.py \
  mode:=mapping \
  use_sim:=false \
  localization:=slam_toolbox \
  enable_nav2:=true \
  nav_rviz:=true \
  obstacle_profile:=anti_self_lowload
```

说明：

- 建图阶段建议保留 Nav2 辅助节点，便于实时检查代价地图和局部规划状态。
- 实车手动驾驶覆盖目标区域（必须覆盖起伏路段）。

---

## 3. 保存地图（YAML/PGM + PCD）

### 3.1 保存栅格地图

```bash
MAP_NAME=highbaybe
ros2 run nav2_map_server map_saver_cli -f /tmp/${MAP_NAME}

cp /tmp/${MAP_NAME}.yaml src/rm_nav_bringup/map/${MAP_NAME}.yaml
cp /tmp/${MAP_NAME}.pgm  src/rm_nav_bringup/map/${MAP_NAME}.pgm
```

### 3.2 保存点云地图（给 icp/small_gicp）

先录包：

```bash
ros2 bag record \
  /livox/lidar/pointcloud /tf /tf_static /Odometry \
  -o /tmp/${MAP_NAME}_mapping_bag
```

录完后重建 PCD：

```bash
/usr/bin/python3 tools/build_map_pcd_from_bag.py \
  --bag /tmp/${MAP_NAME}_mapping_bag \
  --output src/rm_nav_bringup/PCD/${MAP_NAME}.pcd \
  --topic /livox/lidar/pointcloud \
  --target-frame map \
  --voxel-size 0.05 \
  --frame-stride 3 \
  --max-points-per-cloud 10000 \
  --min-range 0.2 \
  --max-range 25 \
  --min-z -0.6 \
  --max-z 2.0
```

可选：为 small_gicp 复制 companion 命名：

```bash
cp src/rm_nav_bringup/PCD/${MAP_NAME}.pcd \
   src/rm_nav_bringup/PCD/${MAP_NAME}_SGICP_MAP.pcd
```

---

## 4. 标注起伏区（写入 launch_params）

当前方案已正式并入 `fake_vel_transform`，不再需要外挂 follow_mark 发布脚本。
你只需要把起伏区多边形写入：

- `src/rm_nav_bringup/config/launch_params.yaml`
- 键：`follow_mark.zone_polygons`

示例（四边形）：

```yaml
follow_mark:
  enable: true
  mode: zone_and_hint
  topic: /chassis/follow_mark
  default_value: 1
  zone_value: 0
  enter_margin_m: 0.35
  exit_margin_m: 0.55
  zone_polygons:
    - [[-1.20, 2.10], [-0.10, 2.10], [-0.10, 2.85], [-1.20, 2.85]]
```

建议用 RViz `Publish Point` + `ros2 topic echo /clicked_point` 先采点，再填 YAML。

---

## 5. 导航模式启动（nav）

```bash
ros2 launch rm_nav_bringup bringup.launch.py \
  mode:=nav \
  use_sim:=false \
  localization:=amcl \
  enable_nav2:=true \
  nav_rviz:=true \
  obstacle_profile:=anti_self_lowload
```

往返 3 次可用已有压力脚本：

```bash
LAUNCH_BRINGUP=1 \
LOCALIZATION=amcl \
BRINGUP_MAP=${MAP_NAME} \
NAV_RVIZ=true \
OBSTACLE_PROFILE=anti_self_lowload \
WAYPOINTS="B_X,B_Y,B_QZ,B_QW; A_X,A_Y,A_QZ,A_QW" \
ROUNDS=3 \
GOAL_TIMEOUT=180 \
BETWEEN_GOALS_SEC=2 \
bash tools/stress_dynamic_nav.sh
```

---

## 6. 验证清单

### 6.1 Follow Mark 发布是否正常

```bash
ros2 topic echo /chassis/follow_mark
ros2 topic hz /chassis/follow_mark
```

预期：

- 区外稳定为 `1`
- 进入起伏区后切 `0`
- 离开后回 `1`
- 边界附近不应高频抖动（由 enter/exit 滞回抑制）

### 6.2 串口侧是否按约定回退

在串口驱动日志中确认：

- 收到值仅 `0/1`
- follow_mark 缺失/超时时回退 `1`

### 6.3 Nav2 正常性

```bash
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
```

---

## 7. 常见问题

1. `follow_mark` 一直是 `1`：检查 `zone_polygons` 是否在当前 odom/map 坐标系下，检查点位是否画偏。
2. `follow_mark` 抖动：增大 `exit_margin_m`，保证 `exit_margin_m >= enter_margin_m`。
3. 串口无响应：确认串口侧 `nav_packet_version=2`，且电控固件已按 `header 后紧跟 follow_mark` 解析。
4. RViz 不显示：检查 `nav_rviz:=true`、`nav_rviz_config` 路径与 `DISPLAY`。
