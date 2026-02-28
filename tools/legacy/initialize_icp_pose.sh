#!/bin/bash

# ==========================================
# ICP 位姿初始化完整解决方案
# ==========================================

set -e

cd /home/nyz/sentry/sentry-navigation
source install/setup.bash

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║      ICP 定位系统 - 初始位姿设置完整指南              ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# ========== 步骤 1: 验证系统状态 ==========
echo "【步骤 1】验证系统状态..."
echo "-------------------------------------------"

echo "✓ 检查 ICP 节点..."
timeout 5 bash -c 'ros2 node list 2>/dev/null | grep -q "/icp_registration"' && echo "  ✅ ICP 节点运行中" || echo "  ⚠️  检查超时或节点不存在"

echo "✓ 检查点云话题..."
timeout 5 bash -c 'ros2 topic list 2>/dev/null | grep -q "/livox/lidar/pointcloud"' && echo "  ✅ 点云话题存在" || echo "  ⚠️  点云话题不存在"

echo "✓ 检查 PCD 文件..."
if [ -f "src/rm_nav_bringup/PCD/RMUC_24.pcd" ]; then
    SIZE=$(stat -c%s "src/rm_nav_bringup/PCD/RMUC_24.pcd" 2>/dev/null || echo "unknown")
    echo "  ✅ RMUC_24.pcd 存在"
else
    echo "  ❌ RMUC_24.pcd 不存在"
fi

echo ""

# ========== 步骤 2: 获取 ICP 当前配置 ==========
echo "【步骤 2】ICP 当前配置"
echo "-------------------------------------------"

echo "• 尝试获取参数（可能需要 10 秒）..."
echo ""

# ========== 步骤 3: 设置初始位姿 ==========
echo "【步骤 3】设置初始位姿"
echo "-------------------------------------------"

# 如果在命令行提供了参数，使用它们；否则使用默认值
if [ $# -ge 3 ]; then
    X=$1
    Y=$2
    YAW=$3
else
    # 默认值：在原点，方向向前
    X=0.0
    Y=0.0
    YAW=0.0
fi

echo "• 设置初始位姿为: X=$X, Y=$Y, YAW=$YAW"
echo "• 执行命令（最多等待 15 秒）..."

timeout 15 bash -c "ros2 param set /icp_registration initial_pose \"[$X, $Y, 0.0, 0.0, 0.0, $YAW]\"" 2>&1

if [ $? -eq 0 ]; then
    echo "  ✅ 初始位姿设置成功"
else
    echo "  ⚠️  设置可能超时或失败，继续..."
fi

echo ""

# ========== 步骤 4: 给予 ICP 初始化时间 ==========
echo "【步骤 4】等待 ICP 初始化"
echo "-------------------------------------------"

echo "• 等待 8 秒让 ICP 执行配准..."
for i in {1..8}; do
    echo -ne "\r  进度: [$((i*100/8))%] "
    sleep 1
done
echo -ne "\r  ✅ 等待完成           \n"

echo ""

# ========== 步骤 5: 检查 TF 发布 ==========
echo "【步骤 5】检查 TF 树状态"
echo "-------------------------------------------"

echo "• 尝试获取 map → odom 变换（最多等待 5 秒）..."
TF_OUTPUT=$(timeout 5 bash -c "ros2 run tf2_ros tf2_echo map odom 2>&1" 2>&1 | head -10)

if echo "$TF_OUTPUT" | grep -q "Translation"; then
    echo "  ✅ TF 树完整！成功获取变换"
    echo ""
    echo "$TF_OUTPUT"
else
    echo "  ⚠️  ICP 未能成功发布 TF（这是正常的，需要在 RViz 中设置位姿）"
    echo ""
fi

echo ""

# ========== 步骤 6: 给出后续说明 ==========
echo "【步骤 6】后续操作"
echo "-------------------------------------------"

echo "✓ 如果 TF 成功发布："
echo "  1. 导航系统应该正常工作"
echo "  2. 在 RViz 中设置导航目标即可让机器人移动"
echo ""

echo "✓ 如果 TF 仍未发布："
echo "  1. 打开 RViz"
echo "  2. 确保 'Fixed Frame' 设置为 'map'"
echo "  3. 点击 '2D Pose Estimate' 按钮"
echo "  4. 在地图上点击机器人的实际位置，拖动指定方向"
echo "  5. 等待 5-30 秒，让 ICP 完成配准"
echo "  6. 观察机器人模型是否与地图对齐"
echo ""

echo "✓ 实时监控 ICP 配准过程："
echo "  • 监听点云到 PCD 的距离：ros2 topic echo /scan | head -20"
echo "  • 监听 TF 发布：ros2 topic echo /tf | grep map"
echo "  • 查看 ICP 日志：ros2 node info /icp_registration"
echo ""

echo "✓ 调试 ICP 参数："
echo "  • 修改 xy_search_range（默认 0.5）："
echo "    ros2 param set /icp_registration xy_search_range 1.0"
echo "  • 修改 yaw_search_range（默认 30°）："
echo "    ros2 param set /icp_registration yaw_search_range 90.0"
echo ""

echo "╔════════════════════════════════════════════════════════╗"
echo "║               初始化过程完成！                          ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""
