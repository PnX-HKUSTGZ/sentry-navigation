# 系统待处理问题：文件角色与工作规则（Agent 必须遵循）

本文件用于：
- 在我们优先解决“TF 断树/坐标系合约”这一核心问题期间，**冻结“文件分散治理”**，用一份文档明确各文件职责。
- 让后续 Copilot/Agent 在修改时有一致的“单一真相（single source of truth）”，避免到处改一圈导致更乱。

## 0. 工作规则（最高优先级）

1. **不要为同一用途新增第二份配置文件**（尤其是 Nav2/TF/launch 参数）。
2. **优先修改 rm_nav_bringup 的 config/launch**；除非明确切换启动链路，否则不要去改 rm_navigation 的模板参数当作“生效配置”。
3. **合并/重构文件分散的问题先不做**：只在本文件里标注“现状与职责”，等 TF 问题解决后再集中治理。
4. **子模块先本地保留即可**：不要删除子模块目录/记录；如远端缺失导致递归更新失败，先用“定向对齐/定向更新”策略绕开。
5. **大文件资产策略**：`stl / stp / pcd` 必须走 Git LFS（由 `.gitattributes` 约束）。

## 1. 启动链路（我们默认的系统入口）

- 主入口：`src/rm_nav_bringup/launch/bringup.launch.py`
  - 作用：一键启动整套系统（仿真/实机、LIO、定位、Nav2 等）。
  - 说明：该文件依赖 `common.py` 来集中加载参数和拼装节点。

- 参数汇聚与“单一真相”：`src/rm_nav_bringup/launch/common.py`
  - 作用：读取 `launch_params.yaml`，决定 world/mode/localization/lio/use_sim，并拼出各类 config 路径与 Node 定义。
  - 重要：**我们在处理 TF/坐标系时，优先在这里找“谁在发布/重映射/启动 TF 相关节点”。**

- 启动参数：`src/rm_nav_bringup/config/launch_params.yaml`
  - 作用：选择 `world / mode(nav|mapping) / lio(fastlio|pointlio) / localization(slam_toolbox|amcl|icp|small_gicp) / use_sim` 等。
  - 规则：系统行为选择应集中到这里，避免在多个 launch 中重复写死。

## 2. Nav2 参数（当前“生效配置”在哪里）

当前 bringup 链路会根据 `use_sim` 选择：
- 仿真：`src/rm_nav_bringup/config/simulation/nav2_params.yaml`
- 实机：`src/rm_nav_bringup/config/reality/nav2_params.yaml`

规则：
- **要改 costmap/controller/BT 等 Nav2 参数，默认改上述两份**（按场景拆分）。
- `src/rm_navigation/rm_navigation/params/nav2_params.yaml` 更像“rm_navigation 包的默认模板/参考”，
  - 只有在你明确使用 `rm_navigation` 自己的 `navigation_launch.py` 且未被 bringup 覆盖 params 时，才会作为默认生效。
  - 在我们解决 TF 核心问题之前，不把它当作主配置入口。

## 3. TF 合约（核心问题）

我们要达成的最小 TF 合约（导航稳定运行所需）：
- `map -> odom`：由定位/建图组件提供（如 slam_toolbox / amcl / icp 系列之一，取决于模式）
- `odom -> base_link`：由里程计/LIO 或融合节点提供（fastlio/pointlio/融合）
- `base_link -> 传感器帧(livox/imu/...)`：由 robot_state_publisher + URDF/xacro 提供

与 TF 相关的“入口点”主要在：
- `src/rm_nav_bringup/launch/common.py`：决定 robot_description、robot_state_publisher、以及各定位/LIO节点是否启动。
- `src/rm_navigation/rm_navigation/launch/*.py`：Nav2 stack 启动（内部做 /tf /tf_static remap）。

在 TF 问题彻底解决前，Agent 修改 TF/frames 必须：
- 先明确“哪个节点负责哪个 TF 边”，避免两个节点同时发布同一条 TF。
- 修改后必须能通过 `check_tf.sh`（或等价检查）验证 TF 连通。

## 4. 仿真与资产（world / mesh / map / pcd）

- 仿真启动：`src/rm_simulation/pb_rm_simulation/launch/rm_simulation.launch.py`
- 地图/点云索引（当前做法）：`src/rm_nav_bringup/config/maps_index.yaml`
- 大文件策略：`stl / stp / pcd` 必须由 Git LFS 管理（见 `.gitattributes`）

补充（评测所需的 GT 数据来源）：
- 仿真中用于评测的地面真值里程计（GT odom）由 `src/rm_nav_bringup/urdf/sentry_robot_sim.xacro` 中的 Gazebo planar_move 插件发布。
  - 约定：优先 remap 到 `/ground_truth/odom`（若实际 bringup 出现不同 topic 名称，以评测模块的动态解析为准）。

规则：
- 不要手动把 LFS 指针文件当“真实资产”提交。
- 若出现“内容看似一样但 git 仍显示 modified”，优先检查：是否存在 LFS/非 LFS 历史混用、filter 配置或 worktree 分支差异。

## 5. 子模块（先保留，后治理）

现阶段目标：**保证不删就不会一无所有**。
- 如果 `git submodule update --recursive` 因远端缺失失败：
  - 不做递归更新
  - 只对关键子模块做“定向 checkout 到 superproject 记录的 commit + reset/clean”

后续（TF 问题解决后）再决定：替换远端/移除子模块/vendor 化。

## 6. 可观测性/评测（改进方向）

文档与评测入口集中在仓库根目录的 Evaluation/Diagnosis 报告与脚本（例如 `run_evaluation.sh` 等）。
规则：
- 新增调试手段优先做成脚本/文档，不要在核心节点里长期留 printf。
- 每次定位 TF/导航异常，最小证据集应包括：tf tree、关键 topic hz、Nav2 lifecycle 状态、关键参数快照。

补充（评测链路的“单一真相”与数据流）：
- 评测模块位于 `src/rm_nav_bringup/evaluation/`，其中轨迹精度评测以“录包结果”为准，不依赖在线 `ros2 bag play + topic echo` 的脆弱导出。
  - 数据采集：`data_collector.py` 会按类型动态解析并录制 GT/EST 里程计（若存在），并始终录制 `/tf`、`/tf_static` 等轻量话题。
  - 轨迹提取：`trajectory_analyzer.py` 优先直接读取 rosbag2 sqlite（包含对 `.db3.zstd` 的解压处理），GT 默认取 ground-truth odom，EST 优先 `/odom`，否则用 TF 组合得到 `odom->base_link`。
