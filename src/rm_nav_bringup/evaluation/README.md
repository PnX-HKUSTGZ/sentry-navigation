# 哨兵导航系统评估模块

## 📋 概述

这是一个完整的自动化评估系统，用于量化对比不同的哨兵导航算法方案。系统支持多种LIO算法和定位方法的组合测试，并生成详细的性能分析报告。

## 🚀 快速开始

### 1. 一键运行完整评估

```bash
cd /home/nyz/sentry/sentry-navigation
./run_evaluation.sh
```

说明：`run_evaluation.sh` 会构建工作区，然后运行 `python3 src/rm_nav_bringup/scripts/run_benchmark.py`。
该入口会启动导航系统、按场景 `waypoints` 自动发送 `NavigateToPose` 目标、录制数据并生成报告。

### 2. 自定义评估

```bash
# 指定测试方法和场景
./run_evaluation.sh --methods fastlio_slam_toolbox pointlio_icp --scenarios basic_navigation high_dynamic

# 调试模式（运行单个测试）
./run_evaluation.sh --single-test

# 查看帮助
./run_evaluation.sh --help
```

### 3. 手动运行

```bash
# 设置环境
source install/setup.bash

# 运行基准测试
python3 src/rm_nav_bringup/scripts/run_benchmark.py \
    --methods fastlio_slam_toolbox pointlio_slam_toolbox pointlio_icp \
    --scenarios basic_navigation high_dynamic feature_sparse

# 分析结果
python3 src/rm_nav_bringup/scripts/compare_methods.py \
    --results-dir ~/sentry_evaluation_results \
    --generate-charts
```

### 4. 调试建议（强烈推荐）

1) 先跑单测（只跑一个方法+一个场景）：

```bash
./run_evaluation.sh --single-test
```

2) 如果提示 Action Server 不可用，先在另一终端检查：

```bash
ros2 action list | grep -i navigate
```

3) 启动日志默认落盘在结果目录下的 `logs/`：

```text
~/sentry_evaluation_results/logs/
```

## 📊 测试方法

### 支持的算法组合

| 方法名称 | LIO算法 | 定位算法 | 描述 |
|---------|---------|----------|------|
| `fastlio_slam_toolbox` | FastLIO | SLAM Toolbox | 当前默认方案 |
| `pointlio_slam_toolbox` | PointLIO | SLAM Toolbox | 改进方案一 |
| `pointlio_icp` | PointLIO | ICP | 改进方案二 |
| `fastlio_amcl` | FastLIO | AMCL | 对比方案 |

### 测试场景

| 场景名称 | 描述 | 测试重点 |
|---------|------|----------|
| `basic_navigation` | 基础导航测试 | 直线和简单转弯 |
| `high_dynamic` | 高动态运动测试 | 高速运动和急转弯 |
| `feature_sparse` | 特征稀少环境测试 | 长廊等单调环境 |
| `complex_path` | 复杂路径测试 | 模拟比赛路径 |

## 📈 评估指标

### 精度指标
- **ATE (绝对轨迹误差)**: 估计轨迹与真实轨迹的全局一致性
- **RPE (相对位姿误差)**: 局部运动的准确性
- **轨迹平滑度**: 速度、加速度和加加速度的变化率

### 性能指标
- **CPU使用率**: 算法对处理器的占用
- **内存使用**: 算法的内存消耗
- **处理延迟**: 从传感器输入到位姿输出的时间延迟

### 鲁棒性指标
- **定位失败次数**: 算法完全跟丢的频率
- **重定位时间**: 从失去定位到恢复的时间
- **漂移率**: 位姿估计随时间的累积误差

## 📁 输出结果

评估完成后，结果保存在 `~/sentry_evaluation_results/`（或你通过 `--output-dir` 指定的目录）下：

```
sentry_evaluation_results/
├── evaluation_report_YYYYMMDD_HHMMSS.html    # HTML格式的详细报告
├── evaluation_data_YYYYMMDD_HHMMSS.json      # JSON格式的原始数据
├── ate_comparison.png                         # ATE对比图
├── cpu_comparison.png                         # CPU使用率对比图
├── memory_comparison.png                     # 内存使用对比图
├── performance_radar.png                     # 综合性能雷达图
├── logs/                                     # bringup 与导航系统输出日志
└── sentry_evaluation_data/                   # 原始数据包和轨迹数据
    ├── fastlio_slam_toolbox_basic_navigation_TIMESTAMP/
    ├── fastlio_slam_toolbox_basic_navigation_TIMESTAMP.json
    └── ...
```

## ⚙️ 配置

### 修改测试场景

编辑 `src/rm_nav_bringup/evaluation/config/test_scenarios.yaml` 来：
- 添加新的测试场景
- 修改现有场景的参数
- 调整评估指标阈值

### 添加新的算法组合

在 `test_scenarios.yaml` 的 `test_methods` 部分添加新的方法：

```yaml
test_methods:
  my_new_method:
    name: "我的新方法"
    description: "新算法组合的描述"
    config:
      lio: "fastlio"
      localization: "my_localization"
```

## 🛠️ 系统要求

### 软件依赖
- ROS 2 Humble
- Python 3.8+
- 必要的Python包：
  - `psutil` (系统监控)
  - `matplotlib` (图表生成)
  - `numpy` (数值计算)
  - `pyyaml` (配置文件处理)
  - `evo` (轨迹评估，自动安装)

### 在 CI / 干净环境中安装依赖 (可复制命令)
下面给出两种常用的安装方式：在 Python 虚拟环境中安装（推荐），以及在基于 Debian/Ubuntu 的系统上先安装系统依赖再安装 Python 包。

1) 使用 Python 虚拟环境（推荐，适用于 CI）

```bash
# 创建并激活虚拟环境
python3 -m venv .venv_evaluation
source .venv_evaluation/bin/activate

# 升级 pip 并安装必需的 Python 包
pip install --upgrade pip
pip install numpy matplotlib psutil pyyaml

# evo 有时需要编译其部分依赖，若希望强制从源码安装（与本项目中的自动安装行为一致）：
pip install evo --upgrade --no-binary evo

# 可选：验证 evo 可用
evo_ape --help || python -m pip show evo
```

2) 在 Debian/Ubuntu 系统上（系统依赖 + 虚拟环境或全局安装）

```bash
# 安装常用系统依赖（在 CI runner 上执行）
sudo apt-get update && sudo apt-get install -y build-essential python3-dev python3-venv python3-pip libeigen3-dev

# 然后按上面的虚拟环境步骤安装 Python 包
python3 -m venv .venv_evaluation
source .venv_evaluation/bin/activate
pip install --upgrade pip
pip install numpy matplotlib psutil pyyaml
pip install evo --upgrade --no-binary evo
```

注：如果在受限网络或无编译工具的环境中，`pip install evo --no-binary evo` 可能失败。建议在 CI 镜像中预先缓存或使用带有 evo 的基础镜像 / 虚拟环境。若希望避免编译，可尝试 `pip install evo`（允许 wheel 安装），但在某些平台可能没有可用 wheel。

### 硬件要求
- 至少4GB RAM
- 多核CPU（推荐4核以上）
- 足够的存储空间（每次完整测试约1-2GB）

## 🔍 调试

### 运行单个测试
```bash
./run_evaluation.sh --single-test
```

### 查看详细日志
```bash
# 检查ROS节点日志
ros2 node list
ros2 topic list

# 查看数据收集状态
ros2 topic echo /odom --once
ros2 topic echo /gazebo/model_states --once
```

### 常见问题

1. **Gazebo启动失败**
   - 检查是否有其他Gazebo实例在运行
   - 确保有足够的内存和GPU资源

2. **导航系统无响应**
   - 检查Nav2是否正确启动
   - 确认地图文件存在且正确

3. **数据收集为空**
   - 检查话题名称是否正确
   - 确认机器人模型在Gazebo中正确加载

## 📚 模块说明

### 核心模块

- **`benchmark_runner.py`**: 主控制器，协调整个测试流程
- **`data_collector.py`**: 数据收集器，记录ROS话题和轨迹数据
- **`trajectory_analyzer.py`**: 轨迹分析器，使用evo工具计算精度指标
- **`performance_monitor.py`**: 性能监控器，监控CPU、内存和延迟
- **`report_generator.py`**: 报告生成器，生成HTML报告和可视化图表

### 脚本工具

- **`run_benchmark.py`**: 命令行执行脚本
- **`compare_methods.py`**: 结果分析和对比脚本
- **`run_evaluation.sh`**: 一键启动脚本

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个评估系统：

1. 添加新的评估指标
2. 支持更多的算法组合
3. 改进可视化效果
4. 优化性能和稳定性

## 📄 许可证

该项目遵循与主项目相同的许可证。
