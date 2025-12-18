# 🧪 Evaluation 模块功能测试任务

**创建时间**: 2025-12-18  
**优先级**: 🔴 高 (阻塞评估流程)  
**预计时间**: 60-90 分钟

---

## 📋 任务目标

验证 Evaluation 模块及相关定位算法的**功能完整性和可用性**，确保能够进行正常的导航评估测试。

### 核心问题
1. ❓ Evaluation 模块是否功能完整且可用？
2. ❓ 自定义 goal 功能是否正确实现？
3. ❓ ICP, Small-GICP, SLAM Toolbox 定位算法能否正常工作？
4. ❓ PointLIO vs FastLIO 哪个更好？

---

## 🎯 测试优先级

```
P0: Evaluation 模块基础功能
├─ 能否启动 benchmark_runner
├─ ROS 订阅和发布是否正常
├─ 数据收集器能否录制 bag
└─ 性能监控能否正常运行

P1: 导航功能验证
├─ Goal 发送机制是否正确
├─ Nav2 Action 能否接受并执行
├─ 路径规划是否正常
└─ 轨迹数据能否正确记录

P2: 定位算法对比
├─ ICP vs Small-GICP vs SLAM Toolbox
├─ 定位精度和稳定性
└─ 资源占用 (CPU/内存)

P3: LIO 算法对比
├─ PointLIO vs FastLIO
├─ 点云处理质量
└─ 性能和鲁棒性
```

---

## 🔍 测试方法论

### 阶段 1: 静态代码检查 (15 分钟)

#### 1.1 检查 Evaluation 模块完整性
```bash
# 检查所有必需文件是否存在
ls -la src/rm_nav_bringup/evaluation/
Expected:
  ✓ benchmark_runner.py
  ✓ data_collector.py
  ✓ performance_monitor.py
  ✓ trajectory_analyzer.py
  ✓ report_generator.py
  ✓ config/test_scenarios.yaml
```

#### 1.2 检查 Goal 发送逻辑
```bash
# 搜索 goal 相关代码
grep -rn "goal_publisher\|NavigateToPose\|_navigate_to_waypoint" \
  src/rm_nav_bringup/evaluation/

# 关键检查点：
- [ ] goal_publisher 是否正确初始化
- [ ] NavigateToPose Action Client 是否正确配置
- [ ] waypoint 解析逻辑是否完整
- [ ] goal 消息格式是否正确 (frame_id='map')
```

#### 1.3 检查定位算法配置
```bash
# 检查 launch_params.yaml 中的定位选项
cat src/rm_nav_bringup/config/launch_params.yaml | grep -A 5 "localization"

# 检查各定位算法的 launch 文件是否存在
ls src/rm_localization/*/launch/*.py

Expected:
  ✓ icp_localization.launch.py (或类似)
  ✓ small_gicp_localization.launch.py
  ✓ slam_toolbox launch 配置
```

#### 1.4 检查 LIO 配置
```bash
# 检查 LIO 选项
cat src/rm_nav_bringup/config/launch_params.yaml | grep -A 5 "lio"

# 检查 LIO launch 文件
ls src/rm_localization/*/launch/*lio*.py

Expected:
  ✓ point_lio.launch.py
  ✓ fast_lio.launch.py
```

---

### 阶段 2: 功能测试 (30-40 分钟)

#### 2.1 Evaluation 模块基础测试

**测试 1: 导入测试**
```bash
cd ~/sentry/sentry-navigation
source install/setup.bash

# 测试 Python 导入
python3 << EOF
import sys
sys.path.insert(0, 'src/rm_nav_bringup/evaluation')
from benchmark_runner import BenchmarkRunner
from data_collector import DataCollector
from performance_monitor import PerformanceMonitor
print("✅ All modules import successfully")
EOF
```

**测试 2: 配置文件解析**
```bash
python3 << EOF
import yaml
with open('src/rm_nav_bringup/evaluation/config/test_scenarios.yaml') as f:
    config = yaml.safe_load(f)
    scenarios = config.get('test_scenarios', {})
    methods = config.get('test_methods', {})
    print(f"✅ Scenarios: {len(scenarios)}")
    print(f"✅ Methods: {len(methods)}")
    for s in scenarios:
        print(f"  - {s}: waypoints={len(scenarios[s].get('waypoints', []))}")
EOF
```

**测试 3: 数据收集器独立测试**
```bash
# 启动 Gazebo + 导航系统 (后台)
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false &
BRINGUP_PID=$!
sleep 30  # 等待启动

# 测试数据收集器
python3 << 'EOF'
import rclpy
from data_collector import DataCollector
import time

rclpy.init()
collector = DataCollector()

print("🧪 Testing DataCollector...")
bag_file = collector.start_recording("test", "basic")
print(f"✅ Bag recording started: {bag_file}")

time.sleep(10)

collector.stop_recording()
print("✅ Bag recording stopped")

# 检查文件是否存在
from pathlib import Path
if Path(bag_file).exists():
    print(f"✅ Bag file created successfully")
else:
    print(f"❌ Bag file not found: {bag_file}")

rclpy.shutdown()
EOF

# 清理
kill $BRINGUP_PID
```

#### 2.2 Goal 发送功能测试

**测试 4: Goal 消息格式验证**
```bash
# 启动导航系统
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false &
sleep 30

# 检查 Nav2 Action Server
ros2 action list | grep navigate_to_pose
# 预期: /navigate_to_pose 存在

# 发送测试 goal
python3 << 'EOF'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

rclpy.init()
node = Node('goal_test')

client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
print("⏳ Waiting for action server...")

if not client.wait_for_server(timeout_sec=30.0):
    print("❌ Nav2 Action Server not available")
    exit(1)

print("✅ Nav2 Action Server available")

# 创建测试 goal
goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'map'
goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
goal_msg.pose.pose.position.x = 8.0
goal_msg.pose.pose.position.y = 7.0
goal_msg.pose.pose.orientation.w = 1.0

print("📤 Sending test goal...")
future = client.send_goal_async(goal_msg)

# 等待 goal 被接受
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

if future.done():
    goal_handle = future.result()
    if goal_handle.accepted:
        print("✅ Goal accepted by Nav2")
    else:
        print("❌ Goal rejected by Nav2")
else:
    print("⚠️ Goal send timeout")

rclpy.shutdown()
EOF
```

**测试 5: Waypoint 导航完整流程**
```bash
# 运行简化的评估测试 (只测试 1 个场景)
python3 << 'EOF'
import rclpy
from benchmark_runner import BenchmarkRunner

rclpy.init()
runner = BenchmarkRunner()

try:
    # 只测试 basic_navigation 场景
    print("🧪 Testing basic_navigation scenario...")
    results = runner.run_benchmark_suite(
        methods=['fastlio_slam_toolbox'],
        scenarios=['basic_navigation'],
        repeats=1
    )
    
    if results:
        print("✅ Benchmark completed successfully")
        print(f"Results: {results}")
    else:
        print("❌ Benchmark failed")
        
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    
finally:
    rclpy.shutdown()
EOF
```

#### 2.3 定位算法测试

**测试 6: ICP 定位**
```bash
# 修改配置为 ICP
python3 << 'EOF'
import yaml
config_file = 'src/rm_nav_bringup/config/launch_params.yaml'
with open(config_file) as f:
    params = yaml.safe_load(f)
params['localization'] = 'icp'
params['lio'] = 'pointlio'
with open(config_file, 'w') as f:
    yaml.dump(params, f)
print("✅ Config set to: PointLIO + ICP")
EOF

# 启动系统
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false &
sleep 30

# 检查 TF 树
ros2 run tf2_tools view_frames
# 预期: map -> odom -> base_link 完整

# 检查定位输出
ros2 topic hz /odom --window 10
# 预期: 有稳定输出 (>10Hz)

# 检查 map->odom TF
ros2 run tf2_ros tf2_echo map odom
# 预期: 有输出且无 timeout

# 清理
killall -9 gzserver gzclient
```

**测试 7: Small-GICP 定位**
```bash
# 修改配置为 Small-GICP
python3 << 'EOF'
import yaml
config_file = 'src/rm_nav_bringup/config/launch_params.yaml'
with open(config_file) as f:
    params = yaml.safe_load(f)
params['localization'] = 'small_gicp'
params['lio'] = 'pointlio'
with open(config_file, 'w') as f:
    yaml.dump(params, f)
print("✅ Config set to: PointLIO + Small-GICP")
EOF

# 重复测试 6 的检查步骤
# ...
```

**测试 8: SLAM Toolbox 定位**
```bash
# 修改配置为 SLAM Toolbox
python3 << 'EOF'
import yaml
config_file = 'src/rm_nav_bringup/config/launch_params.yaml'
with open(config_file) as f:
    params = yaml.safe_load(f)
params['localization'] = 'slam_toolbox'
params['lio'] = 'fastlio'
with open(config_file, 'w') as f:
    yaml.dump(params, f)
print("✅ Config set to: FastLIO + SLAM Toolbox")
EOF

# 重复测试 6 的检查步骤
# ...
```

#### 2.4 LIO 算法对比测试

**测试 9: PointLIO 性能**
```bash
# 配置 PointLIO
python3 << 'EOF'
import yaml
config_file = 'src/rm_nav_bringup/config/launch_params.yaml'
with open(config_file) as f:
    params = yaml.safe_load(f)
params['lio'] = 'pointlio'
with open(config_file, 'w') as f:
    yaml.dump(params, f)
EOF

# 启动并监控性能
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false &
sleep 30

# 监控 CPU/内存
ps aux | grep -E "point_lio|pointlio" | grep -v grep
# 记录: CPU%, MEM%

# 监控输出频率
ros2 topic hz /odom --window 50
# 记录: 平均频率

# 检查点云质量 (如果发布了 /cloud_registered)
ros2 topic hz /cloud_registered --window 10
ros2 topic echo /cloud_registered --once | grep "width\|height"

killall -9 gzserver gzclient
```

**测试 10: FastLIO 性能**
```bash
# 配置 FastLIO
python3 << 'EOF'
import yaml
config_file = 'src/rm_nav_bringup/config/launch_params.yaml'
with open(config_file) as f:
    params = yaml.safe_load(f)
params['lio'] = 'fastlio'
with open(config_file, 'w') as f:
    yaml.dump(params, f)
EOF

# 重复测试 9 的监控步骤
# 对比结果
```

---

### 阶段 3: 结果分析 (15 分钟)

#### 3.1 功能完整性评估

**Evaluation 模块检查清单**
```
基础功能:
- [ ] benchmark_runner 能否成功启动
- [ ] 配置文件能否正确解析
- [ ] DataCollector 能否录制 bag
- [ ] PerformanceMonitor 能否收集数据
- [ ] TrajectoryAnalyzer 能否分析轨迹
- [ ] ReportGenerator 能否生成报告

导航功能:
- [ ] Goal 消息格式是否正确
- [ ] Nav2 Action Server 能否接受 goal
- [ ] Waypoint 导航能否完整执行
- [ ] 轨迹数据能否正确记录
- [ ] Ground truth 和 estimated pose 能否对齐

已知问题:
- [ ] 是否有未实现的功能 (TODO/FIXME)
- [ ] 是否有硬编码的配置
- [ ] 是否有错误处理缺失
```

#### 3.2 定位算法对比

| 指标 | ICP | Small-GICP | SLAM Toolbox | 评价 |
|-----|-----|-----------|--------------|------|
| **功能性** |
| TF 完整性 | ✓/✗ | ✓/✗ | ✓/✗ | |
| 初始化速度 | _s | _s | _s | 越快越好 |
| 定位稳定性 | ✓/✗ | ✓/✗ | ✓/✗ | |
| **性能** |
| CPU 占用 | _%| _%| _%| 越低越好 |
| 内存占用 | _MB| _MB| _MB| 越低越好 |
| 输出频率 | _Hz| _Hz| _Hz| 越高越好 |
| **鲁棒性** |
| 启动成功率 | _/3 | _/3 | _/3 | 3次测试 |
| 运行时错误 | 有/无 | 有/无 | 有/无 | |
| **推荐度** | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | |

#### 3.3 LIO 算法对比

| 指标 | PointLIO | FastLIO | 评价 |
|-----|----------|---------|------|
| **功能性** |
| 点云输出质量 | ✓/✗ | ✓/✗ | |
| Odom 输出频率 | _Hz | _Hz | 越高越好 |
| TF 发布稳定性 | ✓/✗ | ✓/✗ | |
| **性能** |
| CPU 占用 | _%| _%| 越低越好 |
| 内存占用 | _MB| _MB| 越低越好 |
| 启动时间 | _s | _s | 越快越好 |
| **鲁棒性** |
| 启动成功率 | _/3 | _/3 | 3次测试 |
| 运行时错误 | 有/无 | 有/无 | |
| 配置复杂度 | 高/中/低 | 高/中/低 | |
| **推荐度** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | |

---

## 🔧 常见问题排查

### 问题 1: Nav2 Action Server 不可用
```bash
# 检查 Nav2 是否启动
ros2 node list | grep nav2

# 检查 Action Server 状态
ros2 action list
ros2 action info /navigate_to_pose

# 检查 Nav2 日志
ros2 run nav2_lifecycle_manager lifecycle_manager_client --help
```

### 问题 2: Goal 被拒绝
```bash
# 检查 /initialpose 是否发送
ros2 topic echo /initialpose --once

# 检查 map frame 是否存在
ros2 run tf2_ros tf2_echo map base_link

# 检查 global costmap 是否可用
ros2 topic echo /global_costmap/costmap --once
```

### 问题 3: 定位算法不工作
```bash
# 检查点云输入
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once

# 检查定位节点是否运行
ros2 node list | grep -E "icp|gicp|slam"

# 检查定位节点日志
ros2 node info <node_name>
```

### 问题 4: LIO 算法不输出
```bash
# 检查 LIO 节点
ros2 node list | grep -E "lio|fastlio|pointlio"

# 检查 LIO 输出话题
ros2 topic list | grep -E "odom|cloud"
ros2 topic hz /Odometry

# 检查 IMU 输入
ros2 topic hz /livox/imu
```

---

## 📊 预期输出

### 功能测试报告模板

```markdown
# Evaluation 模块功能测试报告

**测试时间**: YYYY-MM-DD HH:MM  
**测试环境**: Ubuntu 22.04, ROS2 Humble, Gazebo Classic

## 执行摘要

### Evaluation 模块
- **状态**: ✅ 可用 / ⚠️ 部分可用 / ❌ 不可用
- **核心问题**: 
  1. ...
  2. ...

### 定位算法
- **推荐**: ICP / Small-GICP / SLAM Toolbox
- **理由**: ...

### LIO 算法
- **推荐**: PointLIO / FastLIO
- **理由**: ...

---

## 详细测试结果

### 1. Evaluation 模块
...

### 2. 定位算法
...

### 3. LIO 算法
...

---

## 发现的问题

### 高优先级 (P0)
- [ ] 问题描述
  - **现象**: ...
  - **影响**: ...
  - **修复方案**: ...

### 中优先级 (P1)
...

### 低优先级 (P2)
...

---

## 改进建议

1. ...
2. ...
3. ...
```

---

## 🚀 快速启动命令

如果你想立即开始测试，复制以下命令：

```bash
# 1. 进入工作目录
cd ~/sentry/sentry-navigation.worktrees/worktree-2025-12-18T01-50-14
source install/setup.bash

# 2. 快速验证 Evaluation 模块导入
python3 -c "
import sys; sys.path.insert(0, 'src/rm_nav_bringup/evaluation')
from benchmark_runner import BenchmarkRunner
print('✅ Evaluation module imports OK')
"

# 3. 启动导航系统测试
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false &
sleep 30

# 4. 检查 Nav2 Action Server
ros2 action list | grep navigate_to_pose && echo "✅ Nav2 Action OK" || echo "❌ Nav2 Action Missing"

# 5. 检查定位
ros2 topic hz /odom --window 10

# 6. 清理
killall -9 gzserver gzclient
```

---

## 📝 任务检查清单

开始前确认：
- [ ] 已完成内存安全修复 (上一个任务)
- [ ] FastDDS 配置正确
- [ ] Gazebo 可以正常启动
- [ ] 有足够的磁盘空间 (至少 5GB)
- [ ] 有足够的时间 (60-90 分钟)

测试中记录：
- [ ] 每个测试的结果 (✅/⚠️/❌)
- [ ] 遇到的错误日志
- [ ] 性能指标 (CPU/内存/频率)
- [ ] 截图或 bag 文件 (如需要)

测试后输出：
- [ ] 功能测试报告 (Markdown)
- [ ] 推荐配置 (launch_params.yaml)
- [ ] 待修复问题列表 (如有)

---

## 🎯 成功标准

本次任务成功的标志：

1. ✅ **Evaluation 模块可用**
   - benchmark_runner 能成功运行
   - 至少 1 个完整场景测试通过
   - 生成了有效的评估报告

2. ✅ **定位算法确定**
   - 至少 1 个定位算法工作正常
   - 有明确的推荐算法和理由

3. ✅ **LIO 算法确定**
   - PointLIO 和 FastLIO 都完成测试
   - 有明确的推荐算法和理由

4. ✅ **问题清单**
   - 所有发现的问题都已记录
   - 有初步的修复方案

---

**准备好了吗？** 🚀

如果你已经准备好开始测试，可以：
1. 查看本文档确认测试方法
2. 运行"快速启动命令"进行初步验证
3. 或直接告诉我 "开始 Evaluation 功能测试"，我将逐步执行所有测试

---

**创建人**: GitHub Copilot CLI  
**文档版本**: 1.0  
**更新时间**: 2025-12-18
