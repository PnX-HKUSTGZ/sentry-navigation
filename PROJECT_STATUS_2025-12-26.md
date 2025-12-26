# 哨兵导航系统现状总结（截至 2025-12-26）

> 本文用于记录 **总体项目现状**、已完成/未完成事项，以及下一阶段的待办任务。

## 1. 概览

- 仓库：`sentry-navigation`
- 当前工作分支（本地）：`consolidate/worktrees-2025-12-18`
- 近期主线目标：
  - 让评估（evaluation）链路可跑、可复现、产物齐全且指标可信
  - 撤回不合理的 Nav2 frame workaround，保持 `map/odom/base_link` 合约
  - 文档治理：删除一次性报告、统一入口文档与运行指南
  - 解决“Nav2 参数/控制器配置分散导致同名不同行为”的权责问题（已写入 README）

## 2. 当前可用能力（现在能稳定做到什么）

### 2.1 评估链路（Evaluation）

- 能按仓库入口脚本运行评估：`run_evaluation.sh`（支持 `--single-test`）
- Single-test 与 suite 产物齐全（包含 JSON 数据与 HTML 报告），用于后续对比与归档
- ATE/RPE 不再出现“恒为 0”的假象：
  - 根因已定位为 bag 中缺少 GT/odom 轨迹（或未能提取）
  - 现已通过 **仿真发布 GT odom + rosbag2 sqlite 直读提取 + TF 回退构造估计轨迹** 使指标真实可用

### 2.2 Nav2 配置与 frame 合约

- `global_frame` 的不当回退（`map -> odom`）已撤回，恢复为 `map`
- 仿真/实车的 Nav2 参数入口逻辑明确：由 `rm_nav_bringup/launch/common.py` 读取 `launch_params.yaml` 的 `use_sim` 决定 `config/simulation` 或 `config/reality`

### 2.3 文档与入口治理

- 清理了大量一次性诊断/测试报告类 Markdown，避免后续误用
- 更新了入口文档与 checklist，使其与当前实现一致
- README 中已补充 **Nav2 参数权责边界 + controller 切换策略 + 参数分层规则**（避免“同名配置不同行为”）

## 3. 已完成事项（Done）

### 3.1 Evaluation 相关

- 修复 `performance_monitor.py` 缺失 import 导致的运行错误
- 修复 single-test 不产出完整 report/JSON 的问题：现在会生成必要的 `single_test_result_*.json`、`evaluation_data_*.json`、`evaluation_report_*.html`
- 修复 ATE/RPE=0 的根因与实现：
  - 支持 rosbag2 sqlite 解析 `.db3`/`.db3.zstd`
  - GT/EST 轨迹选择逻辑完善（缺 topic 时用 TF 回退）
  - evo CLI 调用与输出解析更稳定

### 3.2 配置与启动稳定性

- `use_sim_time` 下发改为更稳健的 bool 处理，降低时间基准混乱风险
- FastDDS SHM 报错相关路径提供了可用的启动脚本/建议（作为运行指引保留）

### 3.3 Nav2 参数治理（权责明确）

- `launch_params.yaml` 增加 `controller: teb|dwb` 的显式开关
- `rm_nav_bringup` 增加 DWB overlay：
  - `config/{simulation|reality}/nav2_controller_dwb.yaml`
- `common.py` 支持：
  - 默认 `controller=teb` 使用原 `nav2_params.yaml`
  - `controller=dwb` 时在启动阶段生成合并参数到 `/tmp/rm_nav_bringup/` 并作为 `params_file` 传给 Nav2
- README 已同步“真实配置字段”，移除文档中不存在的 `use_nav_rviz`，改为 launch 参数 `nav_rviz`

### 3.4 Git/LFS/工作区卫生

- 处理了 Git LFS / STL mesh 规范化与“假脏”问题，降低合并摩擦
- worktree 分支的合并路径已基本理顺，避免后续在旧 worktree 路径上迷失

## 4. 未完成事项（Not Done Yet）

### 4.1 Nav2 参数进一步收敛（仍有风险点）

- 仓库内仍存在 3 份 `nav2_params.yaml`：
  - `rm_nav_bringup/config/simulation/nav2_params.yaml`
  - `rm_nav_bringup/config/reality/nav2_params.yaml`
  - `rm_navigation/rm_navigation/params/nav2_params.yaml`
- 虽然 README 已明确“权威入口是 rm_nav_bringup”，但 **rm_navigation 包内 params 仍可能在某些直接 launch 路径下被误用**。
  - 是否要做“软性 deprecate（文档/README/注释）”或“硬性收敛（移除/重定向）”尚未落地。

### 4.2 Controller 切换的工程化验证

- `controller=dwb` 的 overlay 已提供，但：
  - 仍需要在仿真/实车分别跑一次基本导航（至少 1-2 个场景）验证参数集可用
  - DWB 与 `base_link_fake/fake_vel_transform` 的配套关系需要更明确的“适用范围与禁忌”说明（已在 README 提醒，但缺少验证结论）

### 4.3 文档一致性与落地检查

- 目前存在一个未跟踪文件 `NAV2_PARAMS_GOVERNANCE.md`（工作区里有，但未纳入版本控制）。
  - 需要决定：删除/合并到 README/或正式提交为设计文档。

## 5. 待办任务（Next TODOs）

### 5.1 必做：根据 git 日志回溯所有改动分析合理性

目标：把过去一段时间的关键改动按“动机→影响→风险→是否应保留”逐条复盘，形成可审计的结论。

建议的执行方式（可落地到 checklist）：

1) 以时间窗口为线索（例如 2025-12 全月）导出提交列表（含 merge），分组：
   - Evaluation/评估链路
   - Nav2 参数/控制器
   - TF/frame/时间基准
   - Git LFS/大文件/mesh
   - 文档治理
2) 对每组挑选关键提交做深入复盘：
   - 为什么改（Bug/性能/可复现性/合并卫生）
   - 改动是否引入新耦合或隐性假设
   - 是否需要补充测试/回滚/二次重构
3) 对“高风险改动点”做 `git blame` 回溯到具体文件段落，补充原因说明（写入设计/运行文档）。

### 5.2 建议：Nav2 params 的“软性收敛”落地

- 在 `rm_navigation/rm_navigation/params/nav2_params.yaml` 顶部加显式注释（或 README 引导）说明：
  - 系统入口使用 rm_nav_bringup 传入的 `params_file`
  - 本文件仅作为 rm_navigation 包独立运行的默认示例

### 5.3 建议：controller=dwb 走一轮最小验证

- 仿真：选一个 world + 1 个场景，验证能规划、能跟踪、无明显振荡/超速
- 实车：至少验证启动、TF 就绪、速度输出方向正确

### 5.4 建议：处理未跟踪文档

- 明确 `NAV2_PARAMS_GOVERNANCE.md` 的归宿：
  - A) 合并进 README 并删除该文件
  - B) 作为设计文档提交（建议改名为 `docs/` 下带日期/版本）
  - C) 删除（若 README 已覆盖且不希望维护第二份）

## 6. 风险与注意事项

- **入口不同 → 参数文件不同** 是“同名不同行为”的核心风险源；必须持续通过 README/脚本约束入口。
- `base_link_fake` 机制属于控制层配套设计，不建议未经验证就切回 `base_link`。
- 大文件（地图/mesh）依赖 Git LFS 的一致性仍需持续关注（尤其是多人协作与 CI 拉取）。

---

维护人：GitHub Copilot（GPT-5.2）  
更新时间：2025-12-26
