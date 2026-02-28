# legacy 脚本说明

本目录存放历史阶段使用的兼容脚本，统一从仓库根目录迁入，避免根目录继续堆积杂项文件。

## 脚本清单

- `tools/legacy/run_evaluation.sh`：历史一键评估入口
- `tools/legacy/launch_with_fastdds_fix.sh`：FastDDS SHM 兼容启动包装
- `tools/legacy/save_grid_map.sh`：保存 Nav2 栅格地图
- `tools/legacy/save_pcd.sh`：触发 FastLIO 点云保存
- `tools/legacy/restart_with_pointlio.sh`：历史 Point-LIO 重启脚本
- `tools/legacy/initialize_icp_pose.sh`：历史 ICP 初始位姿脚本
- `tools/legacy/test_icp_initialization.sh`：历史 ICP 初始化诊断
- `tools/legacy/verify_fastlio_fix.sh`：历史 FAST-LIO 修复验证
- `tools/legacy/fix_pcd_naming.sh`：历史 PCD 命名修复脚本

## 使用约定

- 这些脚本以“历史兼容”为目的保留，不是当前推荐主流程。
- 部分脚本包含旧路径或旧流程假设，执行前请先阅读脚本内容再使用。
- 当前推荐主流程以 `ros2 launch rm_nav_bringup bringup.launch.py` 单入口和 `tools/` 目录下非 `legacy` 脚本为准。
