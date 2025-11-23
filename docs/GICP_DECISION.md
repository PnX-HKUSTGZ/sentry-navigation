# GICP 决策文档

## 目的
确定导航子系统中用于精配准/复位的算法和集成策略：采用 GICP（及其工程化变体 Small-GICP）作为精配准模块的标准实现，并说明对比、集成要点与验证计划。

---

## 方案对比（摘要）

| 对比项 | 基于静态地图的 Nav2（map_server + Nav2） | ICP / Small-GICP（基于 PCD 精配准） | 在线 SLAM（FAST-LIO / Point-LIO） |
|---:|:---|:---|:---|
| 方案简述 | 使用预先生成且维护的 YAML 地图供 Nav2 做全局定位与路径规划 | 使用离线采集的高质量 PCD 与实时点云做精配准，Small-GICP 为轻量化/稳健化的 GICP 实现 | 使用传感器（LiDAR/IMU/视觉）在线构建地图并定位 |
| 典型用途 | 常规巡逻、全局路径规划、确定性行为树任务 | 精确复位、局部对齐与高精度定位（对接部署点或充电桩等） | 探索、未知环境运行、环境动态更新 |
| 优点 | 稳定、实现简单、低资源 | 精度高、对局部几何对齐有优势，Small-GICP 更稳健/更快 | 能在无预置地图场景运作，持续优化地图库 |
| 缺点 | 对环境变动/动态对象敏感，需要维护地图 | 依赖高质量 PCD，缺图则不可用；大量计算资源 | 实现复杂、对时间同步/IMU 依赖强、调试与维护成本高 |
| 资源 | 低 | 中到高（取决于点云密度与频率） | 中到高 |

---

## 为什么选择 GICP / Small-GICP（结论）

- 目标场景：哨兵类系统在关键部署点需要高精度对齐（定位与姿态），以保证自动瞄准、路径复位与任务稳定性。
- 权衡理由：Nav2 负责主航行（低延迟、可靠），SLAM 作为在线冗余；而 GICP（Small-GICP）提供按需的高精度复位能力。
- Small-GICP 优势：在保留 GICP 精度的同时，通过稀疏化、参数优化与工程化加速，降低在线计算负担并提升稳健性，适合作为按需服务。

---

## Small-GICP 简要说明

- 实现要点：
  - 体素下采样（Voxel Grid） + 法线估计作为预处理。
  - 使用点到面的匹配（GICP 原理），并对协方差矩阵做稳健化处理以抑制离群点影响。
  - 限制最大迭代次数与残差阈值以保证在线响应时延可控。
- 推荐默认参数（起点）：
  - voxel_size: 0.05 m（基于传感器分辨率与场景）
  - normal_radius: 0.1 m
  - max_iter: 50
  - transformation_epsilon: 1e-6
  - fitness_threshold: 0.5（或根据点云密度调节）
- 性能优化：
  - 在多核机器上并行计算对应批次法线/邻域搜索；
  - 对于大区间匹配，先做粗配准（低分辨率）再做精配准（高分辨率）；
  - 使用里程计/IMU 提供粗位姿作为初值以减少迭代与收敛风险。

---

## 集成架构（最终方案）

1. 主导航层
   - `Nav2`（`map_server + nav2`）负责全局路径规划与行为执行，使用 `map/<world>.yaml`。
2. 在线定位层
   - `FAST-LIO` / `Point-LIO` 在启动时并行运行（若启用），负责实时建图与局部定位；作为对 Nav2 的冗余/补偿。
3. 精配准服务（按需）
   - `small_gicp` 节点/服务，按需启动：
     - 仅在 `maps_index.yaml` 对应 world 有 PCD 且 launch 参数 `localization=icp` 时启动；
     - 提供同步服务接口：`/small_gicp/align` 接收目标 PCD 路径与初始位姿，返回匹配结果（success, pose, score）。
4. 控制与派发
   - `rm_nav_bringup/launch/common.py` 负责根据 `launch_params.yaml` 的 `world` 字段计算并注入 `map_file_path`、`pcd_path` 等到各节点；
   - 启动时打印 `maps_index.yaml` 的友好摘要并打印当前 world 的状态（ok / missing-pcd / missing-yaml）。
5. 降级策略
   - 当选择 `localization=icp` 且 PCD 缺失：打印警告并自动退回至 `Nav2`（使用 map_server）或切换到 `SLAM`（若启用）；
   - 当 `small_gicp` 失败或得分低：发出告警并允许手动或自动切换到备用定位策略。
6. 数据管理
   - 大文件（`*.pcd`, `*.ply`, `*.stl`, `*.lvx`, `*.bag`）使用 Git LFS 管理；
   - 使用 `tools/update_maps_index.py` 生成 `src/rm_nav_bringup/config/maps_index.yaml`，并在必要时触发刷新与 CI 检查。

---

## 接口与运行模式建议

- small_gicp 节点 API（示例）：
  - 服务：`/small_gicp/align`:
    - 请求：`{ "target_pcd": "/path/to/pcd", "init_pose": [x,y,z,roll,pitch,yaw], "params": { ... } }`
    - 返回：`{ "success": bool, "pose": [x,y,z,roll,pitch,yaw], "score": float }
`  
- Launch 参数：
  - `world`（maps_index 上的键名）
  - `localization` ∈ {`nav2`, `icp`, `slam`}，决定优先级
  - `enable_small_gicp`（bool）: 强制启用/禁用 Small-GICP

---

## 验证与测试计划

1. 单元与仿真测试
   - 在仿真环境或已有 PCD 数据上做回归测试：
     - 准确性测试：给定若干初始扰动位姿，统计 Small-GICP 收敛率与平均误差（位置/角度）。
     - 稳健性测试：加入噪声/遮挡/部分重叠，评估失败率与得分分布。
2. 端到端场测
   - 在若干不同 world 上执行完整启动流程：
     - 启用 `localization=icp` 时，验证 small_gicp 服务能成功复位到参考位置（阈值内）。
     - 在缺 PCD 情况下验证系统优雅降级（Nav2 或 SLAM）。
3. 性能测试
   - 在目标硬件上测量单次配准延迟分布（P50/P90/P99）并调整 `voxel_size` 与 `max_iter` 以控制延迟。
4. 回归门槛
   - 定义可接受的 `fitness_threshold` 与 `score`（例如成功时 `score < 0.3`，需根据实际数据调优）。

---

## 回退与监控

- 若 small_gicp 连续 N 次失败（例如 N=3），自动标记该 world 的 PCD 状态为 suspect 并通知运维；
- 在 ROS 日志中记录详细 match 信息与得分；并导出可视化 marker（RViz）帮助离线分析；
- 在 UI/控制台突出显示当前定位模式与 small_gicp 成功/失败统计。

---

## 里程碑与交付项

1. Small-GICP 原型（代码 + 单元测试）
2. 集成 wrapper（服务接口 + launch 参数）
3. 性能验证报告（延迟/成功率/误差）
4. CI 检查：`tools/update_maps_index.py` 与 maps_index 一致性检查

---

## 结语

Small-GICP 作为按需精配准服务，与 Nav2 + SLAM 的混合架构能在保证日常导航可靠性的同时，为关键点位提供高精度复位能力。建议把 Small-GICP 设为可配置的服务并通过 CI/测试不断优化其参数与回退阈值。
