# 哨兵导航系统 (Sentry Navigation)

深圳北理莫斯科大学 北极熊战队 哨兵导航仿真/实车包

> **版本更新 (2025.06)**: 
> - 重构启动文件架构，采用统一配置文件管理
> - 完善错误处理和参数验证机制  
> - 添加详细的代码注释和使用文档
> - 优化节点启动顺序和时序控制

> **RMUL_26 联调可复现指南（导航 + 决策）**  
> 见 `docs/RMUL26_JOINT_REPRO.md`，包含单入口 bringup、AMCL 初始位姿 QoS、决策桩话题、DWB/TEB A/B 命令。

## 一. 项目介绍

本项目使用全向移动小车，附加 Livox Mid360 雷达与 IMU，在 RMUC/RMUL 地图进行导航算法仿真，仅需要调整参数即可移植到真实机器人中导航。

**新特性**:
- ✅ 统一配置文件管理，告别复杂命令行参数
- ✅ 模块化启动文件设计，易于维护和扩展  
- ✅ 完善的参数验证和错误处理机制
- ✅ 详细的代码注释和使用文档
- ✅ 支持多种算法组合的灵活切换

早期功能演示视频：[寒假在家，怎么调车！？更适合新手宝宝的 RM 导航仿真](https://b23.tv/xSNQGmb)

|Gazebo 仿真|Fast_LIO/Point_LIO + Navigation2|
|:-:|:-:|
|![Gazebo 仿真](.docs/gazebo_RMUL.png)|![Fast_LIO/Point_LIO + Navigation2](.docs/FAST_LIO+Nav2.png)|

|动态避障|
|:-:|
|![Gazebo 仿真](.docs/2024.5.11RMUL动态避障2.gif)|

|一键偷家|
|:-:|
|![Gazebo 仿真](.docs/2024.5.11RMUC轨迹跟踪一键偷家.gif)|

### 1.1 rm_simulation 话题接口

| **Topic name**      | **Type**                        | **Note**                         |
|:-------------------:|:-------------------------------:|:--------------------------------:|
| /livox/lidar             | livox_ros_driver2/msg/CustomMsg | Mid360 自定义消息类型   |
| /livox/lidar/pointcloud | sensor_msgs/msg/PointCloud2     | ROS2 点云消息类型                      |
| /livox/imu                | sensor_msgs/msg/Imu             | Gazebo 插件仿真 IMU                  |
| /cmd_vel            | geometry_msgs/msg/Twist         | 麦克纳姆轮小车运动控制接口                  |

### 1.2 整体框图

![功能包流程图](.docs/功能包流程图.png)

## 二. 环境配置

当前开发环境基线为 Ubuntu22.04, ROS2 humble, Gazebo Classic 11.10.0。  
从 2026 起推荐使用 **RoboStack（conda）** 管理 ROS 依赖，避免系统 `/opt/ros/humble` 与工作区混装导致的问题。

### 2.1 方案A：RoboStack（推荐，替代系统 ROS2）

#### 2.1.1 快速上手（复制即用）

```bash
cd sentry-navigation

# 1) 创建/复用 RoboStack 环境并编译
bash tools/setup_robostack_env.sh

# 2) 激活 RoboStack（每个新终端都要执行）
source tools/activate_robostack.sh

# 3) 自检环境
bash tools/check_robostack_env.sh

# 4) 启动导航（防 conda 污染 + FastDDS SHM 处理）
bash tools/launch_nav_safe.sh

# 5) 启动 Foxglove bridge（可选）
bash tools/run_foxglove_bridge.sh
```

如果你需要在当前 shell 中切回系统 ROS2：

```bash
conda deactivate 2>/dev/null || true
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROS_DISTRO
source /opt/ros/humble/setup.bash
```

1. 克隆仓库

    ```sh
    git clone --recursive <your_repo_url>/sentry-navigation.git
    ```

2. 创建 RoboStack 环境并编译工作区

    ```sh
    cd sentry-navigation
    bash tools/setup_robostack_env.sh
    ```

3. 激活 RoboStack 环境（每次新终端都要执行）

    ```sh
    source tools/activate_robostack.sh
    ```

4. 启动 Foxglove Bridge（默认 `ws://0.0.0.0:8765`）

    ```sh
    bash tools/run_foxglove_bridge.sh
    ```

5. 在 Foxglove 客户端连接

    ```text
    ws://<机器人IP或本机IP>:8765
    ```

6. 运行 RoboStack 环境自检

    ```sh
    bash tools/check_robostack_env.sh
    ```

7. （可选）加入 `~/.bashrc`，提供快捷命令 `srs`

    ```sh
    grep -q "robostack_bashrc_snippet.sh" ~/.bashrc || \
      echo "source $(pwd)/tools/robostack_bashrc_snippet.sh" >> ~/.bashrc
    ```

注意:
- 使用 RoboStack 时，不要再 `source /opt/ros/humble/setup.bash`。
- `tools/activate_robostack.sh` 会自动 source conda ROS 和本工作区 overlay。
- 若你需要让 Foxglove 在局域网访问，保留默认 `FOXGLOVE_ADDRESS=0.0.0.0` 即可。
- 若处于离线/受限网络，可尝试 `ROBOSTACK_OFFLINE=1 RUN_ROSDEP=0 RUN_COLCON_BUILD=0 bash tools/setup_robostack_env.sh`（要求本地 conda 缓存已包含依赖）。
- 启用自动激活（可选）：在 `~/.bashrc` 里加 `export AUTO_SENTRY_RS=1`，进入工作区目录时会自动执行 `srs`。

### 2.2 方案B：系统 ROS2（兼容旧流程）

1. 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

    ```sh
    sudo apt install cmake
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2
    mkdir build && cd build
    cmake .. && make -j
    sudo make install
    ```

2. 安装依赖（系统 ROS2）

    ```sh
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

3. 编译

    ```sh
    colcon build --symlink-install
    ```

## 三. 运行

### 3.1 启动文件说明

本项目提供了统一的启动系统，所有配置参数都可以通过修改 `launch_params.yaml` 文件来设置，无需在命令行中传递大量参数。

#### 3.1.1 配置文件结构

主要配置文件位于 `src/rm_nav_bringup/config/launch_params.yaml`：

```yaml
base_link2livox_frame:
   xyz: '"0.12 0.0 0.175"'
   rpy: '"0.0  0.0  0.0"'

# 局部控制器: teb(默认) 或 dwb
controller: teb

# LIO算法: fastlio 或 pointlio
lio: fastlio

# 定位算法: slam_toolbox, amcl, icp, small_gicp (仅 nav 模式生效)
localization: slam_toolbox

# 运行模式: mapping(建图) 或 nav(导航)
mode: nav

# 是否使用仿真：true(仿真) / false(真实)
use_sim: true

# 是否启动 LIO 的 RViz
use_lio_rviz: false

# 世界/地图名称
world: RMUC_24
```

#### 3.1.2 参数权责与分层规则（重要）

为了避免“同名配置不同行为/线上线下差异”，本仓库对 Nav2 参数的权责边界约定如下：

1) **唯一权威入口：rm_nav_bringup**

- 正式运行（仿真/实车）的 Nav2 参数文件由 rm_nav_bringup 选择并传入 Nav2。
- 选择逻辑在 `rm_nav_bringup/launch/common.py`：
   - `use_sim: true` → 读取 `rm_nav_bringup/config/simulation/` 下的参数
   - `use_sim: false` → 读取 `rm_nav_bringup/config/reality/` 下的参数

2) **rm_navigation 包内的 params 不是“运行时真相”**

- `rm_navigation/rm_navigation/params/nav2_params.yaml` 是 rm_navigation 启动文件的默认值（只有你直接 `ros2 launch rm_navigation ...` 且不传 `params_file` 才会用到）。
- 当你按推荐方式 `ros2 launch rm_nav_bringup bringup.launch.py` 启动时，rm_nav_bringup 会显式把 `params_file` 传给 rm_navigation 的 bringup，因此 rm_navigation 包内那份默认 params 不参与运行。

3) **参数分层：场景层 + 控制器层（overlay）**

- 场景层（simulation/reality）为主：
   - `rm_nav_bringup/config/simulation/nav2_params.yaml`
   - `rm_nav_bringup/config/reality/nav2_params.yaml`
- 控制器层只覆盖 `controller_server`：
   - `rm_nav_bringup/config/simulation/nav2_controller_dwb.yaml`
   - `rm_nav_bringup/config/reality/nav2_controller_dwb.yaml`
- `controller: teb` 时不生成文件，直接使用场景层的 `nav2_params.yaml`。
- `controller: dwb` 时，启动阶段会生成合并后的 params 文件到 `/tmp/rm_nav_bringup/` 并作为 `params_file` 传给 Nav2。

4) **base_link_fake / fake_vel_transform 属于“控制层配套机制”**

- 当前默认 Nav2 配置使用 `robot_base_frame: base_link_fake`，并配套启动 `fake_vel_transform`（用于将规划朝向与底盘朝向解耦，避免小陀螺时跟踪失效）。
- 因此：不要在未理解该机制的情况下把 `robot_base_frame` 直接改回 `base_link`，否则会导致局部控制行为与评估结论不一致。

#### 3.1.2 启动方式

修改配置文件后，使用统一的启动命令：

```bash
# 启动完整导航系统
ros2 launch rm_nav_bringup bringup.launch.py

# 命令行选择地图（覆盖 launch_params.yaml 的 world）
# 例如选择新增 RMUL2026：
ros2 launch rm_nav_bringup bringup.launch.py map:=RMUL2026 nav_rviz:=false
```

推荐在本机开发环境使用防污染启动脚本（自动处理 conda Python 污染 + FastDDS SHM）：

```bash
# 先做一次环境预检查（不启动）
DRY_RUN=1 bash tools/launch_nav_safe.sh

# 正式启动
bash tools/launch_nav_safe.sh
```

### 3.2 配置参数详解

1. **world** - 世界环境名称:
   - 仿真模式:
     - `RMUL` - [2024 Robomaster 3V3 场地](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22942&extra=page%3D1)
     - `RMUC` - [2024 Robomaster 7V7 场地](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22942&extra=page%3D1)
   - 真实环境:
     - 自定义名称，对应 `.pcd`(ICP点云图) 和 `.yaml`(栅格地图) 文件名

2. **mode** - 运行模式:
   - `mapping` - 边建图边导航模式，使用SLAM实时构建地图
   - `nav` - 导航模式，使用预构建地图进行定位和导航

3. **lio** - 激光雷达惯性里程计算法:
   - `fastlio` - [FAST-LIO](https://github.com/LihanChen2004/FAST_LIO/tree/ROS2)，基于卡尔曼滤波，计算效率高，里程计约10Hz
   - `pointlio` - [Point-LIO](https://github.com/LihanChen2004/Point-LIO/tree/RM2024_SMBU_auto_sentry)，基于点特征，可输出100+Hz里程计，对导航更友好但CPU占用更高

4. **localization** - 定位算法 (仅在 `mode: nav` 时生效):
   - `slam_toolbox` - 使用 [SLAM工具箱](https://github.com/SteveMacenski/slam_toolbox) 定位模式，动态场景效果更好
   - `amcl` - 使用 [AMCL](https://navigation.ros.org/configuration/packages/configuring-amcl.html) 蒙特卡罗定位算法
   - `icp` - 使用 [ICP点云配准](https://github.com/baiyeweiguang/CSU-RM-Sentry/tree/main/src/rm_localization/icp_registration) 定位，仅初始化时配准，长期运行可能累积误差
   - `small_gicp` - 使用 [Small GICP高性能点云配准](https://github.com/koide3/small_gicp) 定位，相比传统ICP提升2倍性能

5. **use_sim** - 环境类型:
   - `true` - 启动Gazebo仿真环境
   - `false` - 使用真实硬件环境

6. **controller** - 局部控制器:
   - `teb` - 默认控制器（TEB），配置在 `rm_nav_bringup/config/{simulation|reality}/nav2_params.yaml`
   - `dwb` - 切换到 DWB（只覆盖 `controller_server`），配置在 `rm_nav_bringup/config/{simulation|reality}/nav2_controller_dwb.yaml`

7. **可视化选项**:
   - `use_lio_rviz` - 是否启动LIO算法的RViz点云可视化
   - Navigation2 的 RViz 由 launch 参数 `nav_rviz` 控制（默认 true，可在命令行覆盖）

#### 重要提示:
1. **AMCL定位**: 启动后需在RViz中手动设置初始位姿
2. **SLAM工具箱定位**: 需要提供 `.posegraph` 地图文件
3. **ICP定位**: 需要提供 `.pcd` 点云地图文件
4. **Small GICP定位**: 需要提供 `.pcd` 点云地图文件，支持多线程并行处理

### 3.3 使用示例

#### 3.3.1 仿真环境示例

1. **仿真建图模式**:
   ```bash
   # 编辑 launch_params.yaml
   world: "RMUL"
   mode: "mapping"
   lio: "fastlio"
   use_sim: true
   use_lio_rviz: false
   # Navigation2 RViz 在命令行用 nav_rviz 控制
   
   # 启动
   ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=true
   ```

2. **仿真导航模式**:
   ```bash
   # 编辑 launch_params.yaml
   world: "RMUL"
   mode: "nav"
   lio: "fastlio"
   localization: "slam_toolbox"
   use_sim: true
   use_lio_rviz: false
   # Navigation2 RViz 在命令行用 nav_rviz 控制
   
   # 启动
   ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=true
   ```

#### 3.3.2 真实环境示例

1. **真实环境建图**:
   ```bash
   # 编辑 launch_params.yaml
   world: "YOUR_WORLD_NAME"
   mode: "mapping"
   lio: "fastlio"
   use_sim: false
   use_lio_rviz: false
   # Navigation2 RViz 在命令行用 nav_rviz 控制
   
   # 启动
   ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=true
   ```

   **建图完成后的保存操作**:
   - 保存点云地图: `ros2 service call /map_save std_srvs/srv/Trigger`
   - 保存栅格地图: 参考 [如何保存 .pgm 和 .posegraph 地图？](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/issues/I9427I)

2. **真实环境导航**:
   ```bash
   # 编辑 launch_params.yaml
   world: "YOUR_WORLD_NAME"
   mode: "nav"
   lio: "fastlio"
   localization: "slam_toolbox"
   use_sim: false
   use_lio_rviz: false
   # Navigation2 RViz 在命令行用 nav_rviz 控制
   
   # 启动
   ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=true
   ```

   **注意**: 确保栅格地图文件 `YOUR_WORLD_NAME.yaml` 存放在 `src/rm_nav_bringup/map/` 目录，点云地图文件 `YOUR_WORLD_NAME.pcd` 存放在 `src/rm_nav_bringup/PCD/` 目录。

### 3.4 启动文件架构说明

项目采用模块化启动文件设计：

- **`bringup.launch.py`** - 主启动文件，调用通用模块
- **`common.py`** - 通用模块，包含所有节点定义和参数配置
- **`launch_params.yaml`** - 统一配置文件，所有启动参数集中管理

这种设计的优势：
1. **配置集中化** - 所有参数在一个文件中管理
2. **代码复用** - 仿真和真实环境共享相同的节点定义
3. **易于维护** - 修改配置无需修改代码，降低出错概率
4. **灵活组合** - 可以轻松切换不同的算法组合

### 3.5 启动文件问题排查

#### 3.5.1 常见问题及解决方案

1. **配置文件找不到**:
   ```
   错误: FileNotFoundError: launch_params.yaml not found

---

## 四. Evaluation 自动评估（推荐）

本仓库提供一套自动化评估流程，用于在仿真中对比不同方法/场景并生成报告。

### 4.1 一键运行（最推荐）

在仓库根目录执行：

```bash
./run_evaluation.sh --single-test
```

说明：该入口会负责构建（必要时）、启动系统、按场景自动发送导航目标、录制数据并生成结果（HTML + JSON）。

### 4.2 文档入口

- 评估模块使用说明：`src/rm_nav_bringup/evaluation/README.md`
- 系统文件职责与数据流：`SYSTEM_FILE_ROLES.md`
   解决: 确保 launch_params.yaml 文件存在于 src/rm_nav_bringup/config/ 目录
   ```

2. **参数验证失败**:
   ```
   错误: Invalid mode: xxx. Must be one of: mapping, nav
   解决: 检查 launch_params.yaml 中的参数值是否正确
   ```

3. **仿真环境启动失败**:
   ```
   错误: start_rm_simulation not defined
   解决: 确保仿真相关的启动文件存在，检查 common.py 中的导入
   ```

4. **真实硬件连接问题**:
   ```
   错误: Livox雷达连接失败
   解决: 检查 MID360_config.json 中的IP配置，确保网络连接正常
   ```

#### 3.5.2 调试技巧

1. **查看启动参数**:
   启动时会在终端输出当前配置参数，确认配置是否正确。

2. **分步调试**:
   可以在 `common.py` 中注释掉部分节点，逐步启动来定位问题。

3. **日志输出**:
   所有节点都设置了 `output="screen"`，可以查看详细的运行日志。

4. **话题检查**:
   ```bash
   # 检查话题是否正常发布
   ros2 topic list
   ros2 topic echo /topic_name
   ```

### 3.6 小工具 - 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 四. 启动文件详细说明

### 4.1 代码架构设计

本项目采用模块化设计，将启动逻辑分为三个主要文件：

#### 4.1.1 文件结构
```
src/rm_nav_bringup/launch/
├── bringup.launch.py          # 主启动文件
├── common.py                  # 通用节点定义和配置
└── config/
    └── launch_params.yaml     # 统一配置文件
```

#### 4.1.2 设计原则

1. **配置与代码分离**: 所有可变参数集中在 `launch_params.yaml` 中
2. **代码复用**: 仿真和真实环境共享相同的节点定义
3. **模块化**: 功能按类型分组，便于维护和扩展
4. **错误处理**: 完善的参数验证和异常处理机制

### 4.2 关键组件说明

#### 4.2.1 common.py 模块

**参数加载与验证**:
- 使用 YAML 加载配置文件
- 对所有关键参数进行有效性验证
- 提供默认值和错误提示

**节点分类管理**:
- **基础功能节点**: 地面分割、点云转换、IMU滤波等
- **LIO算法节点**: FAST-LIO 和 Point-LIO
- **定位算法节点**: SLAM工具箱、AMCL、ICP定位
- **环境相关节点**: 仿真环境和真实硬件驱动

**坐标变换管理**:
- 机器人描述生成 (URDF)
- 静态坐标变换发布
- 动态坐标变换处理

#### 4.2.2 bringup.launch.py 主启动文件

**启动逻辑**:
1. 根据 `use_sim` 参数选择环境类型
2. 启动基础功能节点
3. 根据 `lio` 参数启动相应的LIO算法
4. 根据 `mode` 和 `localization` 参数启动定位系统
5. 启动 Navigation2 导航系统

**时序控制**:
- ICP定位延迟启动（等待LIO稳定）
- 节点启动顺序优化
- 异常处理和重启机制

### 4.3 配置文件说明

#### 4.3.1 launch_params.yaml 结构详解

```yaml
# ========= 环境配置 =========
world: "RMUL"                    # 世界名称，对应地图文件前缀
use_sim: true                    # 环境类型选择

# ========= 算法配置 =========  
mode: "mapping"                  # 运行模式选择
lio: "fastlio"                   # LIO算法选择
localization: "slam_toolbox"     # 定位算法选择

# ========= 可视化配置 =========
use_lio_rviz: false             # LIO可视化开关
nav_rviz:=true                  # 导航 RViz（launch 参数，默认 true）

# ========= 硬件配置 =========
base_link2livox_frame:          # 雷达坐标变换
  xyz: "\"0.12 0.0 0.175\""    # 位置偏移
  rpy: "\"0.0  0.0  0.0\""     # 姿态偏移
```

#### 4.3.2 参数配置指导

**世界环境配置**:
- 仿真: 选择预设的 RMUL 或 RMUC 场地
- 真实: 设置自定义名称，确保对应的地图文件存在

**算法组合推荐**:
- 高精度建图: `fastlio + mapping`
- 高频导航: `pointlio + nav + slam_toolbox`
- 静态环境: `fastlio + nav + icp`
- 动态环境: `fastlio + nav + amcl`

**性能优化建议**:
- CPU性能一般: 关闭 `use_lio_rviz`，使用 `fastlio`
- 高精度要求: 使用 `pointlio`，开启相关可视化
- 实时性要求: 使用 `icp` 定位，关闭不必要的可视化

### 4.4 启动文件的改进

相比传统的启动方式，本项目的改进包括：

#### 4.4.1 问题解决

**原有问题**:
- 参数传递复杂，命令行参数过多
- 配置分散，难以统一管理  
- 代码重复，维护困难
- 缺乏参数验证，容易出错

**改进方案**:
- 统一配置文件，简化启动命令
- 模块化设计，提高代码复用性
- 完善的错误处理和提示信息
- 详细的代码注释和文档

#### 4.4.2 扩展性设计

**新算法接入**:
1. 在 `common.py` 中添加节点定义
2. 在配置文件中添加相应参数选项
3. 在主启动文件中添加条件启动逻辑

**新功能模块**:
- 传感器融合模块
- 多机器人协作模块  
- 高级规划算法模块
- 安全监控模块

## 五. 实车适配关键参数

1. 雷达 ip

    本导航包已内置 [livox_ros_driver2](https://gitee.com/SMBU-POLARBEAR/livox_ros_driver2_humble)，可直接修改 [MID360_config.json](./src/rm_nav_bringup/config/reality/MID360_config.json) - `lidar_configs` - `ip`

2. 测量机器人底盘正中心到雷达的相对坐标

    x, y 距离比较重要，将影响云台旋转时解算到 base_link 的坐标准确性

    填入 [measurement_params_real.yaml](./src/rm_nav_bringup/config/reality/measurement_params_real.yaml)

    若雷达倾斜放置，无需在此处填入 rpy，而是将点云旋转角度填入 [MID360_config.json](./src/rm_nav_bringup/config/reality/MID360_config.json) - `extrinsic_parameter`

3. 测量雷达与地面的垂直距离

    此参数影响点云分割效果

    填入 [segmentation_real.yaml](./src/rm_nav_bringup/config/reality/segmentation_real.yaml) - `sensor_height`

4. nav2_params

    参数很多，比较重要的是 robot_radius 和 速度相关参数。详见 [nav2官方文档](https://docs.nav2.org/)

## 后记

> 这个仿真包也是我学习的一个记录，也是我学习的一个起点。

很难想象一年前的五月，我连 Ubuntu 和 ROS 是都不知道。笔者大一时只是个混子，北极熊第一届视觉组成员（其实啥也没干），但误打误撞去了 2023 联盟赛广东站赛场。当时只是作为一个观众+摄影师，但赛场的氛围是无以言表的，回来后 RM 浓度就开始逐渐增高，尝试部署华师2023的视觉开源，也在这段时间逐渐学会了自学的方式，写下了第一篇 [算法组文档](https://flowus.cn/lihanchen/facb28a9-5d34-42a7-9bc8-630a182c3571) 。

暑假时参加了“快递速达”的支线任务（比赛），那是第一次初识导航，还记得举着电脑拿着2D雷达在实验楼建图的喜悦。后来被春茧里华农的“自动驾驶”哨兵震撼到了，于是乎大二上和“实验楼安家组”出去喝粥路上，队长问我 24 赛季你想做啥，我毫不犹豫的答出了“导航”。

抱着玩一玩练练手的心态创建了本仓库，第一个 [commit](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/commit/a3e475ce59c60d68462e4555a76113e4ba1295f1) 在 2023-09-27，当时借鉴的还是 [华农 2023 RMUL 哨兵导航开源包](https://github.com/SCAU-RM-NAV/rm2023_auto_sentry_ws) 和 [湖工大RMUC地图](https://github.com/HBUTHUANGPX/Hbut_LC_sentry)，在此基础上只是加了 mid360 的仿真。后来 2023.10 月 中南大学 FYT 战队开源了[RM 哨兵上位机算法](https://github.com/baiyeweiguang/CSU-RM-Sentry)，是当时少见的 ROS2 nav2 导航框架，于是乎当时就想着缝进初代导航包（直到现在也是中南 FYT 的模样）。在不断尝试新算法的过程中，对 Gazebo 和 ros2_launch 也有了更深入的了解。

如果没有开发这个仿真包，我可能也不会有机会接触到导航算法，也不会有机会接触到这么多优秀的开源项目。我还记得那是 2023.1.26，在港中深的哨兵上部署了我的仿真包（ RMUC 联队），改改 launch 文件居然就能让实车动起来了，意料之外地实现了 Sim2Real。由于学校没有机械专业，哨兵在联盟赛前不到两周才完全出生，于是乎备赛期间一直都是在赛博调车优化。

上赛场了，从观众席到检录区。但当时由于在仿真中忽略了雷达偏心放置时的 tf 问题，导致实车云台旋转时定位不准，最终也没能在 RMUL 中发挥丝滑的导航走位...挺遗憾的。

鄙人非大佬，只是个缝合怪菜鸡。我的学习路径个人觉得是有点畸形的，并不是自下而上地先学理论再实践，而是自上而下地先实践再不断补理论的坑。但这种学习方式也不断地给我正反馈，梦里都在改代码，每天醒来都有盼头。

## 致谢（不分先后）

Mid360 点云仿真：参考了 [livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation/blob/main/src/livox_points_plugin.cpp)、 [livox_laser_simulation_RO2](https://github.com/stm32f303ret6/livox_laser_simulation_RO2/blob/main/src/livox_points_plugin.cpp)、 [Issue15: CustomMsg](https://github.com/Livox-SDK/livox_laser_simulation/issues/15)。

导航算法框架：基于 [中南大学 FYT 战队 RM 哨兵上位机算法](https://github.com/baiyeweiguang/CSU-RM-Sentry) 修改并适配仿真，在原有基础上添加对 base_link 的建模，提供多种可选定位方式并完善 launch 文件。

感谢深技大 Shockley，对雷达跟随云台旋转时的速度变换问题提供了很好的解决思路，从而有了 [fake_vel_transform](./src/rm_navigation/fake_vel_transform/) 功能包。

感谢上海工程技术大学、辽宁科技大学、上海电力大学对本开源包的深度使用与交流，给了我很多优化方向。

还有很多很多很多 RM 网友给了我很多鼓励和帮助，这里就不一一列举了（已达成龙王结局，QQ 里 RM 分组九十多人）。
