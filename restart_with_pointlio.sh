#!/bin/bash

# ==========================================
# 使用 Point-LIO 重启导航系统
# ==========================================

set -e

cd /home/nyz/sentry/sentry-navigation
source install/setup.bash

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║     使用 Point-LIO 重启导航系统                        ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

echo "【配置信息】"
echo "-------------------------------------------"
echo "• LIO 算法: Point-LIO"
echo "• 定位方法: ICP"
echo "• 地图文件: RMUC_24.pcd"
echo "• 模式: 导航模式"
echo ""

echo "【启动说明】"
echo "-------------------------------------------"
echo "1. 请先停止当前运行的系统（Ctrl+C）"
echo "2. 然后运行此脚本"
echo "3. 系统启动后等待 10-15 秒让 Point-LIO 初始化"
echo "4. 打开 RViz 并使用 '2D Pose Estimate' 设置初始位姿"
echo ""

read -p "按 Enter 继续启动，或 Ctrl+C 取消..."

echo ""
echo "【正在启动】"
echo "-------------------------------------------"

ros2 launch rm_nav_bringup bringup.launch.py
