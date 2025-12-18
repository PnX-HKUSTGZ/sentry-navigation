# 任务启动 Prompt：Evaluation 模块完整性检查

## 📋 任务背景

Sentry Navigation 系统包含一个完整的自动化评估模块，用于量化对比不同导航算法方案（FastLIO/PointLIO + SLAM Toolbox/ICP/AMCL）的性能。需要检查该模块的完整性、正确性和内存安全性。

---

## 🎯 任务目标

**核心目标**：验证评估模块是否完整且可用，确保不会在运行时爆内存

### 主要检查项

1. **代码完整性检查**
   - 所有 Python 模块是否完整实现（无 pass 占位符）
   - 必要的依赖是否声明
   - 配置文件是否完整

2. **内存安全性审查**
   - 数据收集是否有内存限制机制
   - rosbag 录制是否配置压缩和大小限制
   - 大数据结构是否及时清理
   - 是否有内存监控和预警机制

3. **功能逻辑验证**
   - 测试流程是否清晰完整
   - 错误处理是否健全
   - 结果输出是否规范

4. **可维护性评估**
   - 代码文档是否充分
   - 配置是否灵活
   - 调试功能是否完善

---

## 📁 涉及文件清单

### 核心模块（共 3052 行代码）

```
src/rm_nav_bringup/evaluation/
├── benchmark_runner.py        (38K, ~900行)  - 主控制器
├── data_collector.py          (13K, ~320行)  - 数据收集器
├── performance_monitor.py     (16K, ~400行)  - 性能监控器
├── report_generator.py        (28K, ~700行)  - 报告生成器
├── trajectory_analyzer.py     (28K, ~700行)  - 轨迹分析器
└── __init__.py                (531字节)      - 包初始化
```

### 配置文件

```
src/rm_nav_bringup/evaluation/config/
└── test_scenarios.yaml        (3.9K)  - 测试场景配置
```

### 启动脚本

```
run_evaluation.sh              (5.4K)  - 一键启动脚本
src/rm_nav_bringup/scripts/
├── run_benchmark.py           - 命令行执行脚本
└── compare_methods.py         - 结果对比脚本（需检查是否存在）
```

### 文档

```
src/rm_nav_bringup/evaluation/README.md  (9.1K)
```

---

## ⚠️ 已知风险点

### 内存安全关注点

1. **Gazebo 模拟器**
   - 当前 gzserver 进程占用 ~18.8% 内存
   - 长时间运行可能累积内存

2. **rosbag 录制**
   - 点云数据 ~5.4 Hz，每帧可能数 MB
   - 120-180 秒测试可产生大量数据
   - 需确认是否有压缩和大小限制

3. **轨迹数据存储**
   - 多个测试场景 × 多个方法 = 大量数据
   - 需检查是否有清理机制

4. **evo 轨迹分析**
   - 加载大轨迹文件可能消耗大量内存
   - 需检查是否批量处理

### 配置参数审查

```yaml
# test_scenarios.yaml 中的内存相关配置
performance:
  max_cpu_usage: 80.0%
  max_memory_usage: 2.0 GB  # ← 阈值设置
  max_processing_latency: 0.1s

# 测试时长
basic_navigation: 120s
high_dynamic: 80s
feature_sparse: 100s
complex_path: 180s
```

---

## 🔍 具体检查任务

### 1. 代码完整性检查（优先级：高）

```bash
# 检查项目：
- [ ] 所有 .py 文件没有 TODO/FIXME/INCOMPLETE 标记（已初步检查：无）
- [ ] 所有函数有完整实现（无 pass 占位符）
- [ ] 所有 import 的依赖在 README.md 中声明
- [ ] compare_methods.py 脚本是否存在
```

**检查方法**：
```bash
# 搜索未实现函数
grep -rn "^\s*pass\s*$" src/rm_nav_bringup/evaluation/ --include="*.py"

# 检查导入的外部依赖
grep -rh "^import\|^from" src/rm_nav_bringup/evaluation/*.py | \
  grep -v "^\(import\|from\) \(os\|sys\|time\|json\|pathlib\|typing\|datetime\)" | \
  sort -u

# 验证 compare_methods.py 存在
test -f src/rm_nav_bringup/scripts/compare_methods.py && echo "✅ 存在" || echo "❌ 缺失"
```

---

### 2. 内存安全性审查（优先级：极高）

#### 2.1 检查 data_collector.py

**关键问题**：
- rosbag 录制是否配置了大小/时间限制？
- 是否使用了压缩？
- 订阅的 topic 是否都必要？

**需要查看**：
```python
# 查找 rosbag2 相关配置
grep -A 10 "rosbag\|bag_recorder\|StorageOptions\|RecordOptions" \
  src/rm_nav_bringup/evaluation/data_collector.py
```

**预期配置示例**：
```python
storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id='sqlite3',
    max_bagfile_size=500 * 1024 * 1024,  # 500MB 限制
    max_cache_size=100 * 1024 * 1024      # 100MB 缓存
)

record_options = rosbag2_py.RecordOptions()
record_options.compression_mode = 'file'
record_options.compression_format = 'zstd'  # 压缩
```

#### 2.2 检查 performance_monitor.py

**关键问题**：
- 内存监控数据是否有大小限制？
- 采样频率是否合理？
- 是否有内存超限自动停止机制？

**需要查看**：
```python
# 查找内存数据存储
grep -A 5 "memory_data\|self\..*_data\.append" \
  src/rm_nav_bringup/evaluation/performance_monitor.py

# 查找清理机制
grep -n "clear\|del\|gc\.collect" \
  src/rm_nav_bringup/evaluation/performance_monitor.py
```

**预期实现**：
```python
# 应该有类似的限制
MAX_SAMPLES = 10000  # 限制样本数量
if len(self.memory_data) > MAX_SAMPLES:
    self.memory_data.pop(0)  # FIFO

# 应该有超限检查
if memory_usage_gb > self.config['max_memory_usage']:
    self.logger.error("内存超限，停止测试")
    self.emergency_stop()
```

#### 2.3 检查 benchmark_runner.py

**关键问题**：
- 每次测试后是否清理资源？
- 是否有超时保护？
- Gazebo 是否每次测试后重启？

**需要查看**：
```python
# 查找清理逻辑
grep -A 10 "cleanup\|shutdown\|kill\|terminate" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py

# 查找超时保护
grep -n "timeout\|duration\|max_time" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py
```

---

### 3. 功能逻辑验证（优先级：中）

#### 3.1 测试流程完整性

**验证步骤**：
1. 检查 `_run_single_test()` 函数逻辑
2. 确认导航目标发送机制
3. 验证成功/失败判定条件
4. 检查异常处理覆盖

**检查命令**：
```python
# 查看主测试流程
grep -A 50 "def _run_single_test" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py
```

#### 3.2 Navigation2 集成

**关键问题**：
- Action Server 名称是否正确？（`/navigate_to_pose`）
- 目标格式是否符合 Nav2 规范？
- 超时处理是否正确？

**需要验证**：
```python
# 查找 action client 创建
grep -n "ActionClient\|navigate_to_pose" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py
```

---

### 4. 可维护性评估（优先级：低）

- [ ] 每个模块是否有文档字符串？
- [ ] 关键函数是否有注释？
- [ ] 配置参数是否有说明？
- [ ] README.md 是否与实际代码一致？

---

## 🛡️ 内存爆炸防护清单

### 必须实现的保护措施

```python
# 1. 数据收集限制
MAX_BAG_SIZE = 500 * 1024 * 1024  # 500MB
MAX_SAMPLES_PER_TEST = 10000
COMPRESSION_ENABLED = True

# 2. 内存监控
MEMORY_CHECK_INTERVAL = 1.0  # 1秒
MEMORY_WARNING_THRESHOLD = 0.75  # 75%
MEMORY_CRITICAL_THRESHOLD = 0.90  # 90%

# 3. 超时保护
TEST_TIMEOUT = max(scenario['duration']) + 30  # 测试时长 + 30s buffer
LAUNCH_TIMEOUT = 60  # 启动超时
SHUTDOWN_TIMEOUT = 30  # 关闭超时

# 4. 清理机制
def cleanup_after_test():
    - 停止所有 ROS 节点
    - 删除临时文件
    - 调用 gc.collect()
    - 重启 Gazebo（清空场景）

# 5. 预检查
def pre_flight_check():
    available_memory = psutil.virtual_memory().available
    required_memory = 4 * 1024 * 1024 * 1024  # 4GB
    if available_memory < required_memory:
        raise MemoryError("可用内存不足")
```

---

## 📊 预期输出

### 检查报告格式

```markdown
# Evaluation Module Completeness Report

## 1. 代码完整性 ✅/❌
- 所有模块实现完整：✅/❌
- 依赖声明完整：✅/❌
- 配置文件完整：✅/❌
- 缺失文件：[列表]

## 2. 内存安全性 ✅/⚠️/❌
- rosbag 大小限制：✅ (500MB) / ❌ (无限制)
- rosbag 压缩：✅ (zstd) / ❌ (未压缩)
- 内存监控：✅ (实时) / ⚠️ (部分) / ❌ (无)
- 内存超限保护：✅/❌
- 数据清理机制：✅/❌
- 风险评估：低/中/高

## 3. 功能逻辑 ✅/⚠️/❌
- 测试流程完整：✅/❌
- Nav2 集成正确：✅/❌
- 错误处理健全：✅/⚠️/❌
- 超时保护：✅/❌

## 4. 可维护性 ✅/⚠️/❌
- 代码文档：✅/⚠️/❌
- 配置说明：✅/⚠️/❌
- 调试功能：✅/⚠️/❌

## 5. 发现的问题
### 严重问题（必须修复）
1. [问题描述]
   - 文件：xxx.py:123
   - 影响：导致内存泄漏/系统崩溃
   - 建议修复：[具体方案]

### 警告（建议修复）
1. [问题描述]
   - 影响：可能导致性能下降
   - 建议改进：[具体方案]

### 建议（优化）
1. [改进建议]

## 6. 测试建议
- [ ] 建议先运行 --single-test 验证基本功能
- [ ] 建议限制并行测试数量（避免内存峰值）
- [ ] 建议在测试机上预留 8GB+ 内存
- [ ] 建议启用 swap（至少 4GB）

## 7. 总体评估
- 完整性评分：X/100
- 安全性评分：X/100
- 可用性评分：X/100
- **是否可以直接运行**：✅/❌
- **建议行动**：[立即修复/谨慎测试/可以使用]
```

---

## 🚀 执行步骤建议

### 阶段 1：静态分析（不运行代码）

1. 使用 grep/view 检查所有 Python 文件
2. 验证关键函数实现
3. 检查配置文件完整性
4. 识别潜在内存风险点

### 阶段 2：代码审查

1. 重点审查 data_collector.py 的 rosbag 配置
2. 审查 performance_monitor.py 的内存监控
3. 审查 benchmark_runner.py 的清理逻辑
4. 检查错误处理和超时机制

### 阶段 3：生成报告

1. 汇总发现的问题
2. 评估风险等级
3. 提供修复建议
4. 给出测试建议

### 阶段 4：（可选）最小化测试

如果代码看起来安全，可以运行：
```bash
# 在监控下运行单个快速测试
./run_evaluation.sh --single-test --scenarios basic_navigation --methods pointlio_icp
```

**监控命令**（另开终端）：
```bash
watch -n 1 "free -h && echo '---' && ps aux | grep -E 'gazebo|python' | grep -v grep"
```

---

## ⚠️ 重要约束

1. **不要在检查阶段运行完整测试**（可能需要数小时且占用大量资源）
2. **不要同时运行多个测试**（当前 Gazebo 已在运行）
3. **如发现严重内存安全问题，建议先修复再测试**
4. **所有修改必须最小化，保持现有功能不变**

---

## 📝 补充信息

### 当前系统状态

- ✅ Gazebo 正在运行（PID: 2525120, MEM: 18.8%）
- ✅ Navigation 系统正常运行
- ✅ Livox 点云正常发布（~5.4 Hz）
- ✅ Map frame 可用

### 可用资源

- 主仓库：`/home/nyz/sentry/sentry-navigation`
- 当前 worktree：`/home/nyz/sentry/sentry-navigation.worktrees/worktree-2025-12-18T00-56-23`
- 结果目录：`~/sentry_evaluation_results`（将创建）

### Python 环境

```bash
# 必需依赖（需在报告中验证是否都检查到）
- psutil       # 系统监控
- matplotlib   # 图表生成
- numpy        # 数值计算
- pyyaml       # YAML 解析
- evo          # 轨迹评估
- rclpy        # ROS2 Python
- rosbag2_py   # ROS2 bag 录制
```

---

## 🎯 成功标准

报告完成后，应该能够回答：

1. ✅ **该模块是否完整？**（所有代码都实现了）
2. ✅ **是否安全？**（不会爆内存/崩溃）
3. ✅ **能否运行？**（依赖完整、逻辑正确）
4. ✅ **如何测试？**（具体的测试步骤）
5. ✅ **有哪些风险？**（已知问题和缓解措施）

---

**任务开始时间**: 2025-12-18T01:26:00Z  
**预计完成时间**: 30-60 分钟（静态分析）  
**优先级**: 高（内存安全）> 中（功能完整）> 低（代码质量）

**准备好了吗？开始检查！** 🔍
