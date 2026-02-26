# RMUL_26_WAVE 真实性对齐复核（2026-02-25）

## 目标
排除旧数据和口径不一致影响，验证“波浪路段通过性”结论是否真实。

## 本次对齐动作
1. 只使用 **新生成** artifact，不复用旧 `docs/logs/*` 曲线。
2. 显式对齐 launch 实际使用的 world 文件：
   - `src/rm_simulation/pb_rm_simulation/launch/rm_simulation.launch.py`
   - `RMUL_26_WAVE -> RMUL2026_wave.world`
3. 对齐波浪条几何与当前通道：
   - `tools/generate_wave_world.py` 默认更新为 `y_center=2.20`
   - 波浪条高度从悬空状态调整为地面邻近：`z_base=0.096`（top range `0.061~0.131`）
4. 修正 spawn 高度，去掉长时间坠落瞬态：
   - `src/rm_simulation/pb_rm_simulation/launch/rm_simulation.launch.py`
   - `RMUL_26 / RMUL_26_WAVE` 的 `z: 1.16 -> 0.08`

## 新鲜数据集

### A. 悬空波浪条（改前，作为对照）
- Artifact: `artifacts/fresh_wave_settle40_20260225_233223`
- 条件: old wave z（top mean≈1.135），40s settle，2 goals
- 结果（波浪区内，post-settle）:
  - `odom z range = 0.001595 m`
  - `odom tilt p95 = 0.793 deg`（max=0.864）
  - `wave_top_mean - odom_mean_z = +1.074832 m`
- 结论: 机器人几乎不受波浪影响，且与波浪条有约 1.07m 高差，实质未接触。

### B. 高度对齐后（接触波浪条）
- Artifact: `artifacts/fresh_wave_realign_20260225_233600`
- 条件: new wave z（top range 0.061~0.131），40s settle，2 goals
- 结果（波浪区内，post-settle）:
  - `odom z range = 0.129223 m`
  - `odom tilt p95 = 83.788 deg`（max=98.838）
  - `imu tilt p95 = 84.663 deg`
- 结论: 已发生显著接触，但姿态剧烈失稳（接近翻车级）。

### C. 轴向直穿工况（减少转向干扰）
- Artifact: `artifacts/fresh_wave_axispass_20260225_233924`
- 条件: 3 goals (`-5.18,2.20 -> -3.90,2.20 -> -5.18,2.20`)，40s settle
- 结果（波浪区内，post-settle）:
  - `odom z range = 0.133584 m`
  - `odom tilt p95 = 155.817 deg`（max=175.417）
  - `imu tilt p95 = 155.764 deg`
- 结论: 工况更“直穿”后仍出现极端翻转姿态，问题并非仅由路径回环导致。

### D. Spawn 修正后快速复测（去长等待）
- Artifact: `artifacts/fresh_wave_spawnfix_main_20260225_234421`
- 条件: spawn z=0.08，5s settle，2 goals
- 结果（波浪区内）:
  - `odom z range = 0.043425 m`
  - `odom tilt p95 = 178.552 deg`（max=179.621）
  - `imu tilt p95 = 178.563 deg`
- 结论: 启动瞬态被消除，但接触后仍可进入近倒置姿态。

## 关键结论（贴合真实场景口径）
1. **旧“20/20 成功”不能用于证明波浪通过性。**
   - 在悬空几何下，确实能 20/20，但那是“未接触波浪”的成功。

2. **一旦波浪高度对齐到实际地面，系统出现严重姿态失稳。**
   - 说明当前仿真链路对“起伏通过”并不稳定，且 `goal SUCCEEDED` 与“车辆可控”并不等价。

3. **当前模型对实车可比性不足，必须引入姿态安全约束才有工程意义。**
   - 仅看 Nav2 action 成功率会掩盖翻车/倒置等失效模式。

## 当前最可信的模拟结论
在当前实现下（Nav2 2D + `base_link_fake` 导航参考 + box 叠加波浪地形），
- “导航任务成功率”可以维持高值，
- 但“车体姿态稳定性”在真实接触波浪时明显不达标。

因此，**此版本尚不能宣称“实车可通过 70mm / 240mm 波浪路段”。**
它只能说明：
- 不接触波浪时导航稳定；
- 接触后存在高风险姿态失稳。

## 建议的验收门槛（后续必须同时满足）
- `goal success_rate >= 95%`
- 波浪区内车体倾角（tilt）
  - `p95 <= 12 deg`
  - `max <= 20 deg`
- IMU 竖直加速度去均值 RMS：`<= 2.0 m/s^2`
- 无倒置/翻滚（tilt 不接近 90/180）。

## 参考图
- `artifacts/fresh_wave_settle40_20260225_233223/analysis_fresh/fresh_path_and_attitude.png`
- `artifacts/fresh_wave_realign_20260225_233600/analysis_fresh/fresh_path_and_attitude.png`
- `artifacts/fresh_wave_axispass_20260225_233924/analysis_fresh/fresh_path_and_attitude.png`
- `artifacts/fresh_wave_spawnfix_main_20260225_234421/analysis_fresh/fresh_path_and_attitude.png`
