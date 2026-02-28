#!/bin/bash

# ==========================================
# 修复 PCD 文件命名混乱问题
# ==========================================

set -e

cd /home/nyz/sentry/sentry-navigation

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║         PCD 文件命名修复                               ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

echo "【问题分析】"
echo "-------------------------------------------"
echo "• 48033ba 提交时 map 文件重命名为 *_24 格式"
echo "• 但 PCD 文件没有同步重命名"
echo "• 导致命名不一致：RMUL.pcd vs RMUL_24.yaml"
echo ""

echo "【当前状态】"
echo "-------------------------------------------"
ls -lh src/rm_nav_bringup/PCD/
echo ""

echo "【修复方案】"
echo "-------------------------------------------"

# 1. 重命名 RMUL.pcd 为 RMUL_24.pcd
if [ -f "src/rm_nav_bringup/PCD/RMUL.pcd" ]; then
    echo "• 重命名 RMUL.pcd → RMUL_24.pcd"
    git mv src/rm_nav_bringup/PCD/RMUL.pcd src/rm_nav_bringup/PCD/RMUL_24.pcd
    echo "  ✅ 完成"
else
    echo "  ⚠️  RMUL.pcd 不存在，跳过"
fi

# 2. 添加 RMUC_24.pcd（如果存在且未跟踪）
if [ -f "src/rm_nav_bringup/PCD/RMUC_24.pcd" ] && ! git ls-files --error-unmatch src/rm_nav_bringup/PCD/RMUC_24.pcd >/dev/null 2>&1; then
    echo "• 添加 RMUC_24.pcd 到 git"
    git add src/rm_nav_bringup/PCD/RMUC_24.pcd
    echo "  ✅ 完成"
else
    echo "  ℹ️  RMUC_24.pcd 已跟踪或不存在"
fi

# 3. 检查 RMUC26.pcd 是否是 LFS 指针
if [ -f "src/rm_nav_bringup/PCD/RMUC26.pcd" ]; then
    SIZE=$(stat -c%s src/rm_nav_bringup/PCD/RMUC26.pcd)
    if [ "$SIZE" -lt 1000 ]; then
        echo "• RMUC26.pcd 是 Git LFS 指针文件 ($SIZE 字节)"
        echo "  ⚠️  需要运行: git lfs pull"
    else
        echo "  ✅ RMUC26.pcd 是完整文件"
    fi
fi

echo ""
echo "【修复后状态】"
echo "-------------------------------------------"
ls -lh src/rm_nav_bringup/PCD/
echo ""

echo "【对应的 map 文件】"
echo "-------------------------------------------"
ls -1 src/rm_nav_bringup/map/*.yaml | sed 's|.*/||' | sed 's|\.yaml||'
echo ""

echo "【Git 状态】"
echo "-------------------------------------------"
git status src/rm_nav_bringup/PCD/ --short
echo ""

echo "【建议】"
echo "-------------------------------------------"
echo "• 提交这些更改："
echo "  git commit -m 'fix(pcd): rename PCD files to match map naming convention'"
echo ""
echo "• 如果需要 RMUC26.pcd 完整文件："
echo "  git lfs pull"
echo ""

echo "╔════════════════════════════════════════════════════════╗"
echo "║            命名一致性修复完成！                        ║"
echo "╚════════════════════════════════════════════════════════╝"
