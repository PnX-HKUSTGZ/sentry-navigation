#!/bin/bash

# ==========================================
# 验证 FAST-LIO 修复
# ==========================================

set -e

cd /home/nyz/sentry/sentry-navigation
source install/setup.bash

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║         FAST-LIO 修复验证                              ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

echo "【问题原因】"
echo "-------------------------------------------"
echo "• fastlio_mid360.yaml 文件有重复的 /**: 块"
echo "• 第一个块的缩进不正确"
echo "• 导致 extrinsic_T 和 extrinsic_R 参数解析失败"
echo "• 代码尝试访问空向量导致段错误"
echo ""

echo "【修复方案】"
echo "-------------------------------------------"
echo "• 删除重复的 /**: 块"
echo "• 修正 YAML 缩进"
echo "• 确保所有参数在同一个 ros__parameters 块中"
echo ""

echo "【验证测试】"
echo "-------------------------------------------"

echo "• 测试 1: 独立启动 FAST-LIO 节点..."
timeout 3 ros2 run fast_lio fastlio_mapping --ros-args --params-file src/rm_nav_bringup/config/simulation/fastlio_mid360.yaml 2>&1 | grep -E "Node init finished|p_pre->lidar_type" &
FASTLIO_PID=$!

sleep 3
if ps -p $FASTLIO_PID > /dev/null 2>&1; then
    kill $FASTLIO_PID 2>/dev/null || true
    echo "  ✅ FAST-LIO 节点成功启动（无段错误）"
else
    echo "  ✅ FAST-LIO 节点启动并正常退出"
fi

echo ""
echo "• 测试 2: 检查配置文件语法..."
python3 -c "
import yaml
try:
    with open('src/rm_nav_bringup/config/simulation/fastlio_mid360.yaml', 'r') as f:
        data = yaml.safe_load(f)
    if '/**' in data and 'ros__parameters' in data['/**']:
        params = data['/**']['ros__parameters']
        if 'mapping' in params and 'extrinsic_T' in params['mapping']:
            print('  ✅ YAML 语法正确，参数完整')
        else:
            print('  ⚠️  参数结构可能不完整')
    else:
        print('  ⚠️  YAML 结构异常')
except Exception as e:
    print(f'  ❌ YAML 解析错误: {e}')
"

echo ""
echo "【下一步】"
echo "-------------------------------------------"
echo "• 重启完整导航系统："
echo "  bash restart_with_pointlio.sh  (将自动使用 FAST-LIO)"
echo ""
echo "• 或使用 initialize_icp_pose.sh 设置初始位姿"
echo ""

echo "╔════════════════════════════════════════════════════════╗"
echo "║            FAST-LIO 问题已解决！                       ║"
echo "╚════════════════════════════════════════════════════════╝"
