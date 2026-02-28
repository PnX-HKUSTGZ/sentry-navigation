# Follow Mark 联调方案（底盘轮向一致控制）

## 1. 背景与目标

当前链路为：

- Nav2 输出 `/cmd_vel`
- `fake_vel_transform` 转换后发布 `/cmd_vel_chassis`
- `rm_serial_driver` 订阅 `/cmd_vel_chassis` 并下发串口包

在连续起伏路段（如 8 连续波峰/波谷）通过时，底盘希望“轮子朝向一致（跟随前进方向）”，以提高通过稳定性。  
因此引入 `follow_mark` 控制位，从导航链路传递到下位机。

目标：

1. 不破坏现有速度链路（`/cmd_vel_chassis` 保持不变）
2. 在串口包里新增 `uint8_t follow_mark`
3. 支持“区域判定 + 人工覆盖 + 决策提示”三种输入
4. 支持协议向后兼容（旧固件可继续跑）

---

## 2. 串口协议改造（rm_serial_driver）

## 2.1 新字段

在导航发送包 `SendNavPacket` 的 `header` 后新增：

```cpp
uint8_t follow_mark;
```

建议语义：

- `0`: 非常规模式（轮向不跟随标志位）
- `1`: 常规模式（默认）
- `2~255`: 预留

## 2.2 兼容策略

增加参数 `nav_packet_version`：

- `1`: 旧包（无 `follow_mark`）
- `2`: 新包（含 `follow_mark`）

默认建议先保持 `1`，待下位机固件升级后切 `2`。

## 2.3 navCallback 处理要求

在 `navCallback` 中：

1. 读取缓存 `mark`
2. 若超时（例如 `0.3s`）或未收到，回退 `1`
3. 组包时写入 `packet.follow_mark`

伪代码：

```cpp
uint8_t mark = 1;
{
  std::lock_guard<std::mutex> lock(mark_mutex_);
  const bool fresh = has_follow_mark_ &&
    ((now - last_follow_mark_stamp_).seconds() <= mark_stale_timeout_sec_);
  mark = fresh ? follow_mark_ : 1;
}
packet.follow_mark = mark;
```

## 2.4 关于 0.3s 超时是否“增加延迟”

不会增加正常控制延迟：

- 正常情况下每次都读取最新缓存并立即发包，`0.3s` 不是等待时间。
- `0.3s` 只用于“话题断流后的失联回退窗口”。

建议参数范围：

- 稳定链路：`0.15~0.2s`
- 抖动较大链路：`0.25~0.3s`

---

## 3. ROS 接口契约

保留：

- `/cmd_vel_chassis`：`geometry_msgs/msg/Twist`

新增：

- `/chassis/follow_mark`：`std_msgs/msg/UInt8`

可选新增：

- `/chassis/follow_mark_hint`：`std_msgs/msg/UInt8`（决策层提示）
- `/chassis/follow_mark_debug`：`std_msgs/msg/String`（调试原因）

推荐 QoS：

- `follow_mark` 使用 `Reliable + KeepLast(10)`
- 发布频率与 `/cmd_vel_chassis` 对齐（建议 20Hz）

---

## 4. 场上判定策略（导航侧）

采用“从简优先级”策略（不做 IMU 判定）：

优先级：

1. `manual_override`
2. `bt_hint`
3. `zone_detect`
4. default=1

触发建议（初始值）：

- 常规默认 `follow_mark=1`
- 若配置 `zone_detect`，可在起伏区内强制保持 `1`（带 `enter_margin=0.35m`）

释放建议（初始值）：

- 离开起伏区超过 `exit_margin=0.55m` 后，回到默认值 `1`

说明：

- 滞回用于避免 `follow_mark` 抖动，防止下位机模式频繁切换。
- 起伏区建议参数化为 YAML 区域（支持多边形），不在代码中硬编码。
- 计算优化建议：先做多边形 AABB 快速过滤，再做点内检测。

---

## 5. sentry-navigation 改造点

建议在 `fake_vel_transform` 里新增 `follow_mark` 发布逻辑（与 `cmd_vel_chassis` 同周期发布）：

- `src/rm_navigation/fake_vel_transform/include/fake_vel_transform/fake_vel_transform.hpp`
- `src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp`
- `src/rm_nav_bringup/launch/common.py`
- `src/rm_nav_bringup/config/reality/*.yaml`
- `src/rm_nav_bringup/config/simulation/*.yaml`

新增参数建议：

- `follow_mark_enable`
- `follow_mark_topic`（默认 `/chassis/follow_mark`）
- `follow_mark_mode`（`off|zone|hint|zone_and_hint`）
- `mark_stale_timeout_sec`
- `wave_zones`
- `enter_margin_m` / `exit_margin_m`
- `zone_polygons`（推荐；每个多边形至少3个点）

## 5.1 地图区域标注工具（已有地图 / 临时建图都可用）

新增工具：

- `tools/annotate_follow_mark_zones.py`

能力：

- 直接读取 `map yaml + pgm` 并在图上点选多边形区域。
- 支持把多边形写回 `src/rm_nav_bringup/config/launch_params.yaml` 的 `follow_mark.zone_polygons`。
- 兼容两类输入：
  - 仓库内已有地图（例如 `src/rm_nav_bringup/map/RMUL26.yaml`）
  - `map_saver_cli -f /tmp/wave_local_map` 生成的临时地图
- `--map-yaml` 参数同时接受 `.yaml/.yml` 或 `.pgm` 路径（传 `.pgm` 会自动找同名 `.yaml`）。

交互键位：

- 鼠标左键：添加顶点
- 鼠标右键 / `Backspace`：撤销一个顶点
- `Enter`：提交当前多边形
- `d`：删除最后一个已提交多边形
- `c`：清空当前未提交多边形
- `s`：保存（yaml + png）
- `q`：退出（退出时也会自动保存当前结果）

示例 1：编辑仓库内 RMUL26 地图并回写 launch 参数

```bash
cd /data/home/sim6g/sentry/sentry-navigation
python3 tools/annotate_follow_mark_zones.py \
  --map-yaml src/rm_nav_bringup/map/RMUL26.yaml \
  --launch-params src/rm_nav_bringup/config/launch_params.yaml \
  --write-launch-params src/rm_nav_bringup/config/launch_params.yaml
```

示例 2：编辑临时建图输出（`/tmp/wave_local_map`）

```bash
cd /data/home/sim6g/sentry/sentry-navigation
python3 tools/annotate_follow_mark_zones.py \
  --map-yaml /tmp/wave_local_map.yaml \
  --launch-params src/rm_nav_bringup/config/launch_params.yaml \
  --out-yaml /tmp/wave_local_follow_mark.yaml \
  --out-png /tmp/wave_local_follow_mark.png
```

示例 3：只有 `.pgm` 路径时（自动推断同名 `.yaml`）

```bash
cd /data/home/sim6g/sentry/sentry-navigation
python3 tools/annotate_follow_mark_zones.py \
  --map-yaml /tmp/wave_local_map.pgm
```

---

## 6. rm_serial_driver 改造点

1. 新增订阅 `/chassis/follow_mark` 并缓存最新值与时间戳
2. `navCallback` 中按超时策略回退
3. `SendNavPacketV2` 新增 `follow_mark` 字段并重新计算 CRC
4. 加入 `nav_packet_version` 参数实现 v1/v2 双协议

强制检查：

- `#pragma pack(push, 1)` / `#pragma pack(pop)`
- `static_assert(sizeof(SendNavPacketV2) == EXPECTED_SIZE)`
- 单测覆盖 “字段偏移 + CRC 正确性”

---

## 7. 决策层（可选）

如需战术控制，可在 BT 行为节点前后发布 hint：

- 进入跨越起伏任务前：`follow_mark_hint=1`
- 完成跨越后：`follow_mark_hint=0`

建议由导航侧融合：

`manual_override > hint > zone_detect > default(1)`

---

## 8. 联调与验收

## 8.1 话题联调

```bash
ros2 topic echo /chassis/follow_mark
ros2 topic hz /chassis/follow_mark
ros2 topic hz /cmd_vel_chassis
```

要求：

1. 频率匹配（同量级）
2. 时序稳定（无明显长时间掉零）

## 8.2 串口联调

在驱动日志中打印并验证：

- `packet.follow_mark`
- 与当前路段状态/策略状态一致

## 8.3 场景验收

1. 常规运行：`follow_mark` 维持 `1`（默认值）
2. 人工覆盖或特定战术动作：`follow_mark` 可切到 `0`
3. 特殊动作结束后：`follow_mark` 回到 `1`
4. 连续 8 起伏通过：`follow_mark` 无高频抖动（默认保持 `1`）

---

## 9. 提交拆分建议

1. `rm_serial_driver`：协议与驱动改造（v1/v2 + 缓存/超时）
2. `sentry-navigation`：`follow_mark` 发布与参数化判定
3. `sentry_DecisionMaking`（可选）：hint 输出
4. 文档与回归脚本

---

## 10. 关键风险与回退

风险：

- 下位机固件未同步升级导致包解释错位
- `follow_mark` 抖动导致轮向控制频繁切换

回退：

1. 将 `nav_packet_version` 切回 `1`
2. 将 `follow_mark_enable` 置 `false`
3. 保留 `/cmd_vel_chassis` 原链路继续运行
