# 🚀 哨兵导航系统 Evaluation 测试任务

在开始任何修改/调参之前：
- 先阅读 `docs/legacy/SYSTEM_FILE_ROLES.md`
- 并严格遵循其中的“文件职责与工作规则”（尤其是 TF 与 Nav2 参数的单一真相）

**任务类型**: 系统启动 + 导航评估测试  
**预计时间**: 30-45 分钟  
**工作目录**: `~/sentry/sentry-navigation`

---

## 📋 任务目标

在哨兵导航系统中执行以下任务：
1. **启动完整的导航系统**（Gazebo + FastLIO + SLAM Toolbox + Nav2）
2. **运行 Evaluation 模块测试**，验证导航功能和性能

---

## 🎯 具体要求

### Phase 1: 系统启动验证 (15 分钟)

**启动系统**:
```bash
cd ~/sentry/sentry-navigation
source install/setup.bash
# 如遇 FastDDS SHM 报错，优先使用 wrapper：
#   ./tools/legacy/launch_with_fastdds_fix.sh nav_rviz:=false
ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false &
```

**验证清单** (等待 90 秒后检查):
- [ ] ROS2 节点数 ≥ 40
- [ ] `/navigate_to_pose` Action Server 可用
- [ ] 关键话题发布: `/odom`, `/scan`, `/map`, `/cmd_vel`
- [ ] FastLIO 里程计正常运行
- [ ] SLAM Toolbox 定位正常
- [ ] Nav2 全部组件就绪

**验证命令**:
```bash
# 等待系统启动
sleep 90

# 检查节点数
ros2 node list | wc -l

# 检查 Action Server
ros2 action list | grep navigate_to_pose

# 检查关键话题
ros2 topic list | grep -E "(odom|scan|map|cmd_vel)"

# 检查话题频率
timeout 10 ros2 topic hz /odom --window 5
```

---

### Phase 2: Evaluation 测试执行 (20 分钟)

**2.1 快速验证测试**:
```bash
# 验证 Evaluation 模块可用
python3 -c "
import sys
sys.path.insert(0, 'src/rm_nav_bringup/evaluation')
from benchmark_runner import BenchmarkRunner
print('✅ Evaluation module ready')
"
```

**2.2 单场景导航测试**:
```bash
cd src/rm_nav_bringup/evaluation
python3 << 'EOF'
import sys
sys.path.insert(0, '.')
import rclpy
from benchmark_runner import BenchmarkRunner

rclpy.init()
runner = BenchmarkRunner()

print("🧪 Running basic_navigation scenario...")
results = runner.run_benchmark_suite(
    methods=['fastlio_slam_toolbox'],
    scenarios=['basic_navigation'],
    repeats=1
)

print("✅ Test completed")
print(f"Results: {results}")
rclpy.shutdown()
EOF
```

**2.3 性能监控**:
```bash
# 系统资源占用
ps aux | grep -E "(gzserver|fastlio|slam)" | grep -v grep | \
  awk '{printf "%-40s CPU: %5.1f%% MEM: %5.1f%%\n", substr($11,1,40), $3, $4}'
```

---

## ✅ 成功标准

1. ✅ 系统成功启动（40+ 节点运行）
2. ✅ Nav2 Action Server 响应正常
3. ✅ Evaluation 测试完成（至少 1 个场景）
4. ✅ 生成测试报告或结果数据

---

## 📊 已知环境状态

**系统配置**:
```yaml
lio: fastlio              # 里程计
localization: slam_toolbox # 定位
controller: teb            # 控制器
use_sim: true             # 仿真模式
world: RMUC_24            # 地图
```

**已构建的包** (14/14):
- ✅ fast_lio, point_lio
- ✅ icp_registration, small_gicp_registration
- ✅ teb_local_planner, costmap_converter
- ✅ slam_toolbox (Nav2 内置)
- ✅ rm_nav_bringup, rm_navigation

**已知问题**:
- ⚠️ FastDDS 共享内存警告（不影响功能）
- ⚠️ FastLIO 初始化需要 60-90 秒

---

## 🔧 故障排查

**如果系统未启动**:
```bash
# 检查进程
ps aux | grep -E "(gzserver|ros2)" | grep -v grep

# 如需重启，先找到进程 PID
ps aux | grep gzserver | grep -v grep | awk '{print $2}'
```

**如果 Evaluation 失败**:
```bash
# 检查 Nav2 状态
ros2 action list | grep navigate_to_pose
ros2 topic hz /odom --window 5

# 检查节点
ros2 node list | wc -l
```

---

## 📝 测试输出

**期望输出**:
1. 系统启动日志（节点列表、Action Server 状态）
2. Evaluation 测试结果（导航成功/失败、耗时、轨迹数据）
3. 性能指标（CPU、内存、话题频率）

**可选**: 生成 Markdown 格式的测试报告

---

## 🚀 快速开始 - 复制给新的 Copilot 上下文

```
请帮我执行以下任务：

1. 进入目录 ~/sentry/sentry-navigation
2. 启动导航系统: ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false
3. 等待 90 秒后验证系统状态：
   - 检查节点数（期望 40+）
   - 检查 /navigate_to_pose Action Server
   - 检查关键话题 (/odom, /scan, /map, /cmd_vel)
4. 运行 Evaluation 模块的 basic_navigation 场景测试
5. 记录性能指标（CPU、内存）
6. 生成测试结果报告

环境已就绪，所有包已构建完成（14/14）。
配置：FastLIO + SLAM Toolbox + TEB Planner
```

---

**创建时间**: 2025-12-18 04:52  
**版本**: 1.0  
**作者**: GitHub Copilot CLI
