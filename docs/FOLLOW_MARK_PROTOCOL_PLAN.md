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
3. 支持“自动判定 + 人工覆盖 + 决策提示”三种输入
4. 支持协议向后兼容（旧固件可继续跑）

---

## 2. 串口协议改造（rm_serial_driver）

## 2.1 新字段

在导航发送包 `SendNavPacket` 的 `header` 后新增：

```cpp
uint8_t follow_mark;
```

建议语义：

- `0`: 常规模式（不强制轮向一致）
- `1`: Follow 模式（强制轮向一致）
- `2~255`: 预留

## 2.2 兼容策略

增加参数 `nav_packet_version`：

- `1`: 旧包（无 `follow_mark`）
- `2`: 新包（含 `follow_mark`）

默认建议先保持 `1`，待下位机固件升级后切 `2`。

## 2.3 navCallback 处理要求

在 `navCallback` 中：

1. 读取缓存 `mark`
2. 若超时（例如 `0.3s`）或未收到，回退 `0`
3. 组包时写入 `packet.follow_mark`

伪代码：

```cpp
uint8_t mark = 0;
{
  std::lock_guard<std::mutex> lock(mark_mutex_);
  const bool fresh = has_follow_mark_ &&
    ((now - last_follow_mark_stamp_).seconds() <= mark_stale_timeout_sec_);
  mark = fresh ? follow_mark_ : 0;
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

采用“多源融合 + 优先级 + 滞回”：

优先级：

1. `manual_override`
2. `bt_hint`
3. `zone_detect`
4. `imu_detect`
5. default=0

触发建议（初始值）：

- 进入起伏区（带 `enter_margin=0.35m`）
- 或 `abs(pitch)>5.0deg` / `abs(roll)>4.0deg`

释放建议（初始值）：

- 离开起伏区超过 `exit_margin=0.55m`
- 且 `abs(pitch)<2.5deg` 且 `abs(roll)<2.0deg`
- 且保持 `hold_after_exit=1.2s`

说明：

- 滞回用于避免 `follow_mark` 抖动，防止下位机模式频繁切换。
- 起伏区建议参数化为 YAML 区域，不在代码中硬编码。

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
- `follow_mark_mode`（`off|zone|imu|hybrid`）
- `mark_stale_timeout_sec`
- `wave_zones`
- `enter_margin_m` / `exit_margin_m`
- `pitch_on_deg` / `pitch_off_deg`
- `roll_on_deg` / `roll_off_deg`
- `hold_after_exit_sec`

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

`manual_override > hint > auto_detect`

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
- 与当前路段状态/姿态状态一致

## 8.3 场景验收

1. 平路巡航：`follow_mark` 基本为 `0`
2. 进入起伏区：`follow_mark` 稳定切到 `1`
3. 离开起伏区：按滞回规则回到 `0`
4. 连续 8 起伏通过：`follow_mark` 无高频抖动

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
