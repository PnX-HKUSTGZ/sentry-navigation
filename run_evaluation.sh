#!/bin/bash

# 哨兵导航系统自动评估脚本
# ====================================

echo "🤖 哨兵导航系统自动评估"
echo "====================================="

# 检查当前目录
if [ ! -f "src/rm_nav_bringup/config/launch_params.yaml" ]; then
    echo "❌ 错误: 请在 sentry-navigation 项目根目录下运行此脚本"
    exit 1
fi

# 设置颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}📋 检查依赖...${NC}"

# 检查Python依赖
REQUIRED_PACKAGES=("psutil" "matplotlib" "numpy" "pyyaml")

for package in "${REQUIRED_PACKAGES[@]}"; do
    if python3 -c "import $package" 2>/dev/null; then
        echo -e "✅ $package 已安装"
    else
        echo -e "${YELLOW}📦 安装 $package...${NC}"
        pip3 install $package
    fi
done

# 检查evo工具
if command -v evo_ape &> /dev/null; then
    echo "✅ evo 评估工具已安装"
else
    echo -e "${YELLOW}📦 安装 evo 评估工具...${NC}"
    pip3 install evo --upgrade --no-binary evo
fi

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2 环境未设置，请先source ROS2 setup脚本${NC}"
    exit 1
fi

echo -e "${GREEN}✅ 依赖检查完成${NC}"

# 构建项目
echo -e "${YELLOW}🔨 构建项目...${NC}"
if [ ! -d "build" ] || [ ! -d "install" ]; then
    echo "首次构建，这可能需要几分钟..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "增量构建..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select rm_nav_bringup
fi

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ 项目构建失败${NC}"
    exit 1
fi

# 设置环境
echo -e "${YELLOW}⚙️ 设置环境...${NC}"
source install/setup.bash

# 创建结果目录
RESULTS_DIR="$HOME/sentry_evaluation_results"
mkdir -p "$RESULTS_DIR"

echo -e "${GREEN}📁 结果将保存到: $RESULTS_DIR${NC}"

# 解析命令行参数
METHODS=("fastlio_slam_toolbox" "pointlio_slam_toolbox" "pointlio_icp")
SCENARIOS=("basic_navigation" "high_dynamic" "feature_sparse")
SINGLE_TEST=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --methods)
            shift
            METHODS=()
            while [[ $# -gt 0 && ! "$1" =~ ^-- ]]; do
                METHODS+=("$1")
                shift
            done
            ;;
        --scenarios)
            shift
            SCENARIOS=()
            while [[ $# -gt 0 && ! "$1" =~ ^-- ]]; do
                SCENARIOS+=("$1")
                shift
            done
            ;;
        --single-test)
            SINGLE_TEST=true
            shift
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  --methods METHOD1 METHOD2 ...    指定测试方法"
            echo "  --scenarios SCENARIO1 SCENARIO2  指定测试场景"
            echo "  --single-test                     运行单个测试(调试模式)"
            echo "  --help                            显示帮助信息"
            echo ""
            echo "可用方法: fastlio_slam_toolbox, pointlio_slam_toolbox, pointlio_icp, fastlio_amcl"
            echo "可用场景: basic_navigation, high_dynamic, feature_sparse, complex_path"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

echo -e "${YELLOW}🚀 开始评估...${NC}"
echo "测试方法: ${METHODS[*]}"
echo "测试场景: ${SCENARIOS[*]}"

# 构建Python命令
PYTHON_CMD="python3 src/rm_nav_bringup/scripts/simple_test.py"
PYTHON_CMD="$PYTHON_CMD --methods ${METHODS[*]}"
PYTHON_CMD="$PYTHON_CMD --scenarios ${SCENARIOS[*]}"
PYTHON_CMD="$PYTHON_CMD --output-dir $RESULTS_DIR"

if [ "$SINGLE_TEST" = true ]; then
    # 单个测试模式，只测试第一个方法和场景
    PYTHON_CMD="python3 src/rm_nav_bringup/scripts/simple_test.py"
    PYTHON_CMD="$PYTHON_CMD --methods ${METHODS[0]}"
    PYTHON_CMD="$PYTHON_CMD --scenarios ${SCENARIOS[0]}"
    PYTHON_CMD="$PYTHON_CMD --output-dir $RESULTS_DIR"
    echo -e "${YELLOW}🔍 运行单个测试 (调试模式)${NC}"
fi

# 运行评估
echo "执行命令: $PYTHON_CMD"
eval $PYTHON_CMD

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ 评估完成!${NC}"
    echo ""
    echo "📊 结果文件:"
    ls -la "$RESULTS_DIR"/*.html 2>/dev/null
    ls -la "$RESULTS_DIR"/*.json 2>/dev/null
    ls -la "$RESULTS_DIR"/*.png 2>/dev/null
    
    echo ""
    echo -e "${YELLOW}📈 生成对比分析...${NC}"
    python3 src/rm_nav_bringup/scripts/compare_methods.py --results-dir "$RESULTS_DIR" --generate-charts
    
    echo ""
    echo -e "${GREEN}🎉 全部完成!${NC}"
    echo "📁 详细结果请查看: $RESULTS_DIR"
    
    # 尝试打开HTML报告
    HTML_FILE=$(ls "$RESULTS_DIR"/evaluation_report_*.html 2>/dev/null | head -1)
    if [ -n "$HTML_FILE" ]; then
        echo "🌐 评估报告: $HTML_FILE"
        if command -v xdg-open &> /dev/null; then
            echo "正在打开浏览器查看报告..."
            xdg-open "$HTML_FILE" &
        fi
    fi
else
    echo -e "${RED}❌ 评估失败${NC}"
    echo "请检查错误信息并重试"
    exit 1
fi
