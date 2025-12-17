#!/bin/bash

# ICP 初始化测试脚本
# 用于诊断和解决 ICP 节点的初始化问题

set -e

cd /home/nyz/sentry/sentry-navigation
source install/setup.bash

echo "========================================="
echo "   ICP 初始化和位姿设置测试脚本"
echo "========================================="
echo ""

# 1. 检查 PCD 文件
echo "[1/5] 检查 PCD 文件..."
PCD_FILE="install/rm_nav_bringup/share/rm_nav_bringup/PCD/RMUC_24.pcd"
if [ -f "$PCD_FILE" ]; then
    echo "✅ PCD 文件存在: $PCD_FILE"
    FILE_SIZE=$(ls -lh "$PCD_FILE" | awk '{print $5}')
    echo "   文件大小: $FILE_SIZE"
else
    echo "❌ PCD 文件不存在: $PCD_FILE"
    exit 1
fi
echo ""

# 2. 检查 ICP 节点是否运行
echo "[2/5] 检查 ICP 节点运行状态..."
if ros2 node list 2>/dev/null | grep -q "/icp_registration"; then
    echo "✅ ICP 节点正在运行"
else
    echo "❌ ICP 节点未运行"
    exit 1
fi
echo ""

# 3. 检查点云话题
echo "[3/5] 检查点云话题..."
if timeout 2 bash -c "ros2 topic hz /livox/lidar/pointcloud 2>&1 | grep -q 'average rate'" 2>/dev/null; then
    CLOUD_HZ=$(timeout 2 bash -c "ros2 topic hz /livox/lidar/pointcloud" 2>&1 | grep "average rate" | awk '{print $3}')
    echo "✅ 点云话题正常，频率: $CLOUD_HZ Hz"
else
    echo "⚠️  点云话题无数据或离线"
fi
echo ""

# 4. 设置初始位姿
echo "[4/5] 设置初始位姿..."
echo "当前配置："
ros2 param get /icp_registration initial_pose
echo ""
echo "正在尝试设置初始位姿为 [0, 0, 0, 0, 0, 0.1] (在原点，偏转 0.1 弧度)..."
ros2 param set /icp_registration initial_pose "[0.0, 0.0, 0.0, 0.0, 0.0, 0.1]"
echo "✅ 初始位姿已设置"
echo ""

# 5. 等待 ICP 初始化并检查 TF
echo "[5/5] 等待 ICP 配准和 TF 发布..."
echo "等待 5 秒以便 ICP 进行配准..."
sleep 5

echo ""
echo "检查 map → odom TF 变换:"
timeout 3 bash -c "ros2 run tf2_ros tf2_echo map odom" 2>&1 | head -20 || true

echo ""
echo "========================================="
echo "测试完成！"
echo ""
echo "下一步操作："
echo "1. 打开 RViz"
echo "2. 点击 'Initialize Global Localization' 或在地图上设置 '2D Pose Estimate'"
echo "3. 等待 ICP 完成配准（通常 10-30 秒）"
echo "4. 观察 TF 树是否完整，地图和机器人位置是否对齐"
echo ""
echo "如果仍然无法发布 TF，请运行："
echo "  ros2 topic echo /tf | grep map"
echo "  来检查 ICP 是否有任何输出"
echo ""
