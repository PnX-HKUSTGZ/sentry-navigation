# Evaluation Module 快速检查清单

## 🎯 核心任务
检查评估模块完整性，**确保不会爆内存**

---

## ⚡ 快速检查命令

### 1. 代码完整性（2分钟）
```bash
cd /home/nyz/sentry/sentry-navigation.worktrees/worktree-2025-12-18T00-56-23

# 检查未实现函数
grep -rn "^\s*pass\s*$" src/rm_nav_bringup/evaluation/ --include="*.py"

# 检查缺失文件
test -f src/rm_nav_bringup/scripts/compare_methods.py && echo "✅" || echo "❌ 缺失"

# 检查外部依赖
grep -rh "^import\|^from" src/rm_nav_bringup/evaluation/*.py | \
  grep -v "^\(import\|from\) \(os\|sys\|time\|json\|pathlib\|typing\|datetime\|collections\|threading\|subprocess\)" | \
  sort -u
```

### 2. 内存安全审查（10分钟）⚠️ **最重要**

#### rosbag 配置检查
```bash
# 查找 rosbag 配置
grep -B5 -A15 "StorageOptions\|RecordOptions\|max_bagfile_size\|compression" \
  src/rm_nav_bringup/evaluation/data_collector.py

# 必须找到：
# - max_bagfile_size 限制
# - compression_mode 压缩设置
```

#### 内存监控检查
```bash
# 查找内存数据存储
grep -n "memory_data\|\.append\|MAX.*SAMPLE" \
  src/rm_nav_bringup/evaluation/performance_monitor.py

# 查找清理机制
grep -n "clear()\|pop()\|del \|gc.collect" \
  src/rm_nav_bringup/evaluation/performance_monitor.py

# 查找内存超限保护
grep -n "memory.*exceed\|memory.*limit\|emergency.*stop" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py
```

#### 清理机制检查
```bash
# 查找测试后清理
grep -B5 -A10 "cleanup\|_cleanup\|shutdown.*gazebo\|terminate.*process" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py
```

### 3. 功能逻辑（5分钟）

#### 导航集成检查
```bash
# 检查 Nav2 Action Client
grep -n "ActionClient\|navigate_to_pose\|NavigateToPose" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py

# 检查超时处理
grep -n "timeout\|duration.*exceed\|max.*time" \
  src/rm_nav_bringup/evaluation/benchmark_runner.py
```

---

## 🚨 关键风险点速查

### ❌ 危险信号
- [ ] rosbag 没有 `max_bagfile_size` 限制
- [ ] rosbag 没有压缩配置
- [ ] `memory_data` 无限 append 且无上限
- [ ] 没有 `gc.collect()` 调用
- [ ] 没有测试后的清理函数
- [ ] 没有内存超限自动停止机制

### ✅ 安全信号
- [ ] rosbag 有 500MB+ 大小限制
- [ ] rosbag 启用 zstd/lz4 压缩
- [ ] 内存数据有 FIFO 或大小限制
- [ ] 每次测试后调用 cleanup
- [ ] 有内存监控和报警
- [ ] Gazebo 每次测试后重启

---

## 📋 检查结果模板

```
## 快速检查结果

### 1. 代码完整性
- 未实现函数: ___个
- 缺失文件: ___
- 依赖检查: ✅/❌

### 2. 内存安全 ⚠️
- rosbag 大小限制: ✅ ___MB / ❌ 无
- rosbag 压缩: ✅ ___ / ❌ 无
- 内存数据限制: ✅ ___条 / ❌ 无
- 内存监控: ✅ / ⚠️ / ❌
- 清理机制: ✅ / ❌
- **风险等级**: 🟢低 / 🟡中 / 🔴高

### 3. 主要问题
1. [问题描述 + 文件:行号]
2. ...

### 4. 建议
- [ ] 可以直接测试
- [ ] 需要修复后测试
- [ ] 不建议运行（风险太高）
```

---

## 🔥 内存安全必查代码段

### data_collector.py（最关键）
```python
# 应该有类似配置：
storage_options = rosbag2_py.StorageOptions(
    max_bagfile_size=500 * 1024 * 1024,  # ← 必须有
    max_cache_size=100 * 1024 * 1024      # ← 必须有
)
record_options.compression_mode = 'file'  # ← 必须有
record_options.compression_format = 'zstd' # ← 推荐
```

### performance_monitor.py
```python
# 应该有类似限制：
MAX_SAMPLES = 10000  # ← 必须有
if len(self.memory_data) > MAX_SAMPLES:
    self.memory_data.pop(0)  # FIFO

# 应该有超限检查：
if memory_gb > self.max_memory:
    self.emergency_stop()  # ← 必须有
```

### benchmark_runner.py
```python
# 应该有清理函数：
def cleanup_test(self):
    self.stop_navigation()
    self.kill_gazebo()
    gc.collect()  # ← 推荐
    time.sleep(5)

# 应该有超时保护：
result = action_client.wait_for_result(timeout=duration)
```

---

## ⏱️ 时间分配建议

- **5分钟**: 运行所有快速检查命令
- **10分钟**: 详细审查 data_collector.py
- **5分钟**: 详细审查 performance_monitor.py
- **5分钟**: 审查 benchmark_runner.py 清理逻辑
- **5分钟**: 编写检查报告

**总计**: ~30分钟

---

## 🚀 检查后行动

### 如果发现严重问题（🔴）
1. **不要运行测试**
2. 记录问题并提供修复建议
3. 等待修复后再测试

### 如果只有轻微问题（🟡）
1. 记录问题
2. 可以谨慎地运行 `--single-test`
3. 全程监控内存使用

### 如果一切正常（🟢）
1. 文档化检查结果
2. 可以安全运行测试
3. 建议先 `--single-test` 验证

---

## 📞 紧急情况处理

如果测试过程中发现内存飙升：

```bash
# 立即停止测试（Ctrl+C in launch terminal）

# 检查内存使用
free -h
ps aux | grep -E 'gazebo|python' | grep -v grep

# 杀死 Gazebo（如果失控）
ps aux | grep gzserver | awk '{print $2}' | xargs kill -9

# 清理缓存
sync && echo 3 | sudo tee /proc/sys/vm/drop_caches
```

---

**开始检查！优先级：内存安全 > 代码完整 > 功能逻辑** 🔍
