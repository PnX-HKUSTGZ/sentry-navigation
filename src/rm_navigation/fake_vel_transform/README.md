# fake_vel_transform

该包用于实现雷达和云台共用的 yaw 轴时的兼容性，即使云台处于旋转扫描状态，速度变换后，仍然可以实现较稳定的轨迹跟踪效果。

## 实现方式

创建 XYZ 轴基于 `base_link`, RPY 轴基于局部路径规划朝向的 `base_link_fake` 坐标系，使得 nav2 局部规划器能够将机器人的方向视为与当前路径规划方向一致。

nav2 发布的速度也是基于 `base_link_fake` 坐标系的，通过 tf2 将速度转换到 `base_link` 坐标系，使机器人能够正常运动。在仿真中表现即底盘小陀螺时仍能跟随轨迹。

电控端执行底盘控制命令时，机器人运动正方向为云台枪管朝向。

## fake_vel_transform_node

订阅：

- nav2 发布的基于 base_link_fake 坐标系的速度指令 `/cmd_vel`
- nav2 controller 发布的局部路径朝向 `/local_plan`
- `odom` 到 `base_link` 的 tf 变换

发布：

- 转换到 base_link 坐标系的速度 `/cmd_vel_chassis`
- Follow Mark 控制位 `/chassis/follow_mark`（可选）

静态参数：

- 底盘固定旋转速度 `spin_speed`
- TF 频率 `tf_publish_frequency`
- 局部路径超时回退 `local_plan_timeout_sec`
- 可配置 frame/topic：
  - `odom_frame`
  - `base_frame`
  - `fake_base_frame`
  - `cmd_vel_topic`
  - `cmd_vel_out_topic`
  - `local_plan_topic`
- Follow Mark 参数（可选）：
  - `follow_mark_enable`
  - `follow_mark_topic`
  - `follow_mark_manual_topic`
  - `follow_mark_hint_topic`
  - `follow_mark_mode`（`off|zone|hint|zone_and_hint`）
  - `follow_mark_default_value`（`1=常规`，`0=非常规`）
  - `follow_mark_zone_value`
  - `follow_mark_input_stale_timeout_sec`
  - `follow_mark_enter_margin_m` / `follow_mark_exit_margin_m`
  - `follow_mark_zone_rects`（每4个值表示一个矩形区域，兼容旧配置）
  - `follow_mark_zone_polygon_points` + `follow_mark_zone_polygon_sizes`（多边形区域）

多边形区域检测的算力控制：

- 节点启动时预计算每个多边形的 AABB 包围盒。
- 运行时先做 AABB 快速过滤，再做点在多边形内检测（射线法）。
- `enter/exit margin` 仅在候选多边形上做“点到边距离平方”判断，避免开方运算。

  搭配电控固定小陀螺速度，将 spin_speed 设为负，可实现移动时小陀螺减慢。

## 快速配置流程（Follow Mark 区域）

1. 在 `src/rm_nav_bringup/config/launch_params.yaml` 打开 `follow_mark.enable`，并设置 `follow_mark.mode=zone` 或 `zone_and_hint`。
2. 使用 `tools/annotate_follow_mark_zones.py` 在地图上交互式标注 `zone_polygons`。
3. 将结果写回 `launch_params.yaml` 后，重启 `bringup.launch.py` 生效。

示例：

```bash
cd /data/home/sim6g/sentry/sentry-navigation
python3 tools/annotate_follow_mark_zones.py \
  --map-yaml src/rm_nav_bringup/map/RMUL26.yaml \
  --launch-params src/rm_nav_bringup/config/launch_params.yaml \
  --write-launch-params src/rm_nav_bringup/config/launch_params.yaml
```
