# RViz 2D 位姿估计指南
# 用于 ICP 定位初始化

## 快速说明

当 ICP 定位无法自动发布 TF 变换时，需要手动设置机器人的初始位姿。这告诉 ICP 节点"机器人在这里"，它就能与 PCD 地图进行配准。

## 详细步骤

### 前置条件检查
- ✅ 系统已启动（所有节点运行）
- ✅ RViz 显示地图正常
- ✅ 地图的分辨率和比例正确（应该能识别房间/区域）

### 1. 打开 RViz（如果尚未打开）
```bash
cd /home/nyz/sentry/sentry-navigation
source install/setup.bash
rviz2
```

### 2. 配置 RViz 窗口

**设置固定框架：**
- 在左下角找到 "Fixed Frame"
- 从下拉菜单选择 **"map"**
- （不要选 "odom" 或 "base_link"，只选 "map"）

**添加地图显示：**
- 点击左下角 "Add" 按钮
- 搜索 "Map" 
- 选择 "Map" 并点击确认
- 在右侧面板设置：
  - Topic: `/map` 或 `/static_map`
  - Color Scheme: costmap / raw
  - （你应该看到 RMUC_24 场景的灰度地图）

**添加机器人模型：**
- 点击 "Add" → "RobotModel"
- 这会显示机器人的 3D 模型（一般是一个矩形或方框）

### 3. 发送初始位姿估计

**步骤 A：激活 2D Pose Estimate 工具**
- 在 RViz 顶部工具栏中找到图标（通常看起来像地图上有个箭头的按钮）
- 或使用快捷键（如果配置了）
- 按钮通常在"Interact"、"Move Camera"、"Select"等按钮旁边

**步骤 B：在地图上点击和拖动**
- 点击地图显示区域，在**机器人实际所在位置**按下鼠标
- 保持按下鼠标，向**机器人面向的方向**拖动
  - 例如：机器人面朝北，就向上拖动
  - 例如：机器人面朝东，就向右拖动
- 释放鼠标

**预期结果：**
- 一个带箭头的绿色箭头会出现在你点击的位置
- 箭头方向表示机器人的朝向
- RViz 会自动将此信息发送到 `/initialpose` 话题

### 4. 等待 ICP 配准

- **不要着急移动！** 
- ICP 需要 5-30 秒来处理点云并进行配准
- 在此期间，你会看到：
  - 机器人模型可能会在地图上跳跃或移动
  - 地图和机器人慢慢对齐
  - RViz 日志中可能出现大量消息

**预期最终结果：**
- 机器人模型（矩形框）与地图对齐
- 在 RViz 左侧的显示列表中，应该出现 TF 树
- 不再看到红色的 "Timed out waiting for transform" 错误

### 5. 验证成功

打开终端运行：
```bash
# 检查 TF 树是否完整
ros2 run tf2_ros tf2_echo map odom

# 应该输出类似：
# At time 1234567.890:
# - Translation: [0.123, 0.456, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]

# 或者查看完整 TF 树
ros2 run tf2_ros tf2_monitor
```

## 常见问题

### Q1: 点击时没有绿色箭头出现
- **原因**：2D Pose Estimate 工具未激活
- **解决**：在工具栏中明确选择 "2D Pose Estimate"（可能标记为 "2D Nav Goal" 或类似名称）

### Q2: 绿色箭头出现了，但 TF 仍未发布
- **原因**：ICP 配准失败，可能是：
  1. 初始位姿离实际位置太远（> 1 米）
  2. 方向偏差过大（> 90 度）
  3. PCD 地图与实际环境不匹配
- **解决**：
  - 尝试更准确地点击实际位置
  - 增加搜索范围参数（见后文）
  - 检查 PCD 文件是否正确

### Q3: 机器人模型与地图反向或镜像
- **原因**：方向设置反了（180 度）
- **解决**：再次使用 2D Pose Estimate，但拖动时指向相反方向

### Q4: "No transform from [base_link_fake] to [map]" 错误继续出现
- **原因**：ICP 仍未成功发布 map→odom 变换
- **解决步骤**：
  1. 检查 ICP 节点日志（如果有）
  2. 验证点云有足够的数据点
  3. 查看错误消息中提到的帧名称
  4. 确保 PCD 文件在正确位置

## 高级调试

### 监听 /initialpose 话题
```bash
ros2 topic echo /initialpose
# 当你在 RViz 中点击和拖动时，应该看到：
# header:
#   frame_id: map
# pose:
#   position: {x: X, y: Y, z: 0.0}
#   orientation: {x: 0, y: 0, z: sin(yaw/2), w: cos(yaw/2)}
```

### 监听 /tf 话题
```bash
ros2 topic echo /tf | grep -A10 'frame_id: "map"'
# 配准成功后，应该看到 "map" → "odom" 的变换
```

### 调整 ICP 搜索参数（如果配准失败）
```bash
# 增大搜索范围（默认 0.5 米）
ros2 param set /icp_registration xy_search_range 2.0

# 增大角度搜索范围（默认 30 度）
ros2 param set /icp_registration yaw_search_range 90.0

# 之后重新使用 2D Pose Estimate
```

## 流程图

```
启动系统
  ↓
打开 RViz，设置 Fixed Frame = "map"
  ↓
添加 Map 显示（应该看到 RMUC_24 场景）
  ↓
添加 RobotModel（应该看到机器人形状）
  ↓
点击 "2D Pose Estimate" 工具
  ↓
在地图上点击 + 拖动（设置初始位姿）
  ↓
等待 5-30 秒（ICP 配准）
  ↓
检查：TF 是否发布？
  ├─ 是 ✅ → 机器人已定位，可以使用 2D Nav Goal 导航
  ├─ 否 ❌ → 检查错误消息，调整参数重试
  └─ 保持等待中 ⏳ → 不要着急，继续等待
```

## 成功标志

✅ 所有以下条件都满足：
1. RViz 中机器人模型与地图在同一位置
2. `ros2 run tf2_ros tf2_echo map odom` 输出变换
3. 导航容错消息减少或消失
4. 可以使用 "2D Nav Goal" 工具设置导航目标
5. 机器人向目标移动

---

需要更多帮助？查看 launch_params.yaml 以及检查日志：
```bash
tail -50 ~/.ros/log/latest/*/stereo_image_proc-*.log
```
