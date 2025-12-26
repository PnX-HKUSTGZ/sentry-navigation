# Nav2 参数权责与“同名不同行为”说明

本文件回答三个问题：

1) **Nav2 参数到底从哪里来？谁是权威？**
2) 为什么会出现 **`nav2_params.yaml` 同名但行为不同**？
3) 如何收敛到 **一个默认控制器 + 显式切换策略 + 参数分层**。

---

## 1. 运行时参数来源（实际生效链路）

在哨兵系统的一般启动路径中（推荐入口）：

- 入口：`rm_nav_bringup/launch/bringup.launch.py`
- 选择逻辑：`rm_nav_bringup/launch/common.py`
  - 读取 `rm_nav_bringup/config/launch_params.yaml` 的 `use_sim`
  - 根据 `use_sim` 选择配置目录：
    - 仿真：`rm_nav_bringup/config/simulation/`
    - 实车：`rm_nav_bringup/config/reality/`
  - 组装 `nav2_params_file_dir = <config_dir>/nav2_params.yaml`
- 传递方式：`common.py` 通过 `start_navigation2` 将 `params_file := nav2_params_file_dir` 传给 rm_navigation 的启动文件
  - `rm_navigation/launch/bringup_rm_navigation.py`
  - 它再 include `rm_navigation/launch/navigation_launch.py`
- 关键点：`rm_navigation/launch/navigation_launch.py` 使用 `RewrittenYaml(source_file=params_file, ...)`
  - 会把 `use_sim_time`、`autostart` 等 launch 参数覆盖到临时 yaml（`configured_params`）
  - 然后把 `configured_params` 作为 **各 nav2 节点唯一的 parameters 输入**

结论：

- **哨兵 bringup 这条链路下，真正生效的 Nav2 参数文件是**：
  - `rm_nav_bringup/config/simulation/nav2_params.yaml` 或
  - `rm_nav_bringup/config/reality/nav2_params.yaml`
- `rm_navigation` 包在该链路下更像“Nav2 启动封装”，**不是参数权威源**。

---

## 2. “同名不同行为”的根因

仓库内至少存在两套同名文件：

- `rm_nav_bringup/config/simulation/nav2_params.yaml`
- `rm_nav_bringup/config/reality/nav2_params.yaml`
- `rm_navigation/params/nav2_params.yaml`

它们名字相同，但**控制器/代价地图/频率/插件族**可能完全不同。

这会导致：

- 你如果启动 `rm_nav_bringup` 的 bringup（系统入口），得到的是 rm_nav_bringup 目录下的配置（目前是 TEB 控制器配置）。
- 你如果单独启动 `rm_navigation/launch/bringup_rm_navigation.py` 或 `rm_navigation/launch/navigation_launch.py` 而不传 `params_file`，则会用 rm_navigation 自己的默认：`rm_navigation/params/nav2_params.yaml`（目前是 DWB 控制器配置）。

因此“同名不同行为”不是 Nav2 本身的玄学，而是**启动入口不同 → 传入的 params_file 不同 → 最终参数树不同**。

---

## 3. 现状观察（收敛前的关键不一致）

### 3.1 控制器插件

- `rm_nav_bringup/config/*/nav2_params.yaml`：
  - `controller_server.controller_plugins: ["FollowPath"]`
  - `FollowPath.plugin: teb_local_planner::TebLocalPlannerROS`
- `rm_navigation/params/nav2_params.yaml`：
  - `FollowPath.plugin: dwb_core::DWBLocalPlanner`

### 3.2 sim vs reality 两份 Nav2 params

目前 `rm_nav_bringup/config/simulation/nav2_params.yaml` 与 `.../reality/nav2_params.yaml` 大量重复，差异主要体现在速度/加速度等少量约束参数上。

这类“复制粘贴+局部改动”的结构，长期会导致：

- 同名字段在不同文件中漂移
- 排障时不清楚“改的是哪个生效文件”
- 很难引入“明确的切换策略”

---

## 4. 推荐的权责边界（治理原则）

建议明确以下边界：

- **rm_nav_bringup 负责：** 具体机器人/赛场/仿真与实车差异（传感器 topic、frame、速度约束、costmap layer 开关、控制器选择策略等）。
- **rm_navigation 负责：** nav2 节点编排（启动哪些节点、composition/respawn、通用 remapping），以及“可运行的默认示例”，但不承诺是哨兵机器人的最终参数权威。

这样做可以允许 rm_navigation 继续独立测试/复用，但哨兵系统不会再被“rm_navigation 默认 params”反向影响。

---

## 5. 收敛方案（一个默认控制器 + 显式切换 + 参数分层）

### 5.1 一个默认控制器

以当前系统入口实际使用为准：**默认继续使用 TEB**（保持现状不变）。

### 5.2 显式切换策略

把“选择 controller”变成 **launch_params.yaml 的显式参数**（例如 `controller: teb|dwb`），并在 `rm_nav_bringup/launch/common.py` 中据此决定“叠加哪份控制器 overlay”。

### 5.3 参数分层（减少复制粘贴）

ROS2 launch 的 Node 支持传入多个 parameter 文件，后面的覆盖前面的。

建议把 Nav2 参数拆成：

1. `nav2_common.yaml`：两端共用、尽量稳定（BT、planner、recoveries、costmap 通用结构、smoother 等）
2. `nav2_env_sim.yaml` / `nav2_env_real.yaml`：只放 sim/real 的差异（frame、sensor topic、速度上限、是否启用某些 layer）
3. `nav2_controller_teb.yaml` / `nav2_controller_dwb.yaml`：只放 controller_server 相关（`controller_plugins`、FollowPath plugin 与其私参）

启动时按顺序叠加：

- `parameters=[nav2_common, nav2_env_<sim|real>, nav2_controller_<teb|dwb>]`

这样可以把“差异”集中在 overlay 文件中，避免全文件复制。

---

## 6. 下一步（如果要我直接落地改造）

我可以按“不改变默认行为（仍然 TEB）”的原则做一次最小侵入改造：

- 新建 `rm_nav_bringup/config/nav2_layers/` 放分层 yaml
- `rm_nav_bringup/launch/common.py` 改为传递多份 yaml（common + env + controller）
- `launch_params.yaml` 增加 `controller` 字段，默认 `teb`
- 保留旧的 `nav2_params.yaml` 一段时间，但在文件头加明确的 DEPRECATED 注释，避免继续扩散

需要你确认：默认控制器最终是 **TEB** 还是希望切到 **DWB/RPP**。
