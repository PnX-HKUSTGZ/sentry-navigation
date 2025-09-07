#!/usr/bin/env python3
"""
方案对比脚本
用于分析和对比不同方案的性能
"""

import sys
import os
import json
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def load_evaluation_results(results_dir: str):
    """加载评估结果"""
    results_dir = Path(results_dir)
    
    # 查找最新的评估结果文件
    json_files = list(results_dir.glob("evaluation_data_*.json"))
    
    if not json_files:
        print(f"在目录 {results_dir} 中未找到评估结果文件")
        return None
    
    # 选择最新的文件
    latest_file = max(json_files, key=lambda f: f.stat().st_mtime)
    
    print(f"加载评估结果: {latest_file}")
    
    with open(latest_file, 'r', encoding='utf-8') as f:
        return json.load(f)

def analyze_results(results):
    """分析结果"""
    print("\n📊 结果分析")
    print("=" * 50)
    
    methods = list(results.keys())
    scenarios = []
    
    # 获取所有场景
    for method_results in results.values():
        if isinstance(method_results, dict):
            scenarios.extend(method_results.keys())
    scenarios = list(set(scenarios))
    
    print(f"测试方法: {methods}")
    print(f"测试场景: {scenarios}")
    
    # 分析每个指标
    print("\n🎯 精度分析 (ATE RMSE):")
    ate_data = {}
    for method in methods:
        ate_values = []
        for scenario in scenarios:
            try:
                result = results[method][scenario]
                if 'error' not in result:
                    ate_rmse = result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', float('inf'))
                    ate_values.append(ate_rmse)
                else:
                    ate_values.append(float('inf'))
            except KeyError:
                ate_values.append(float('inf'))
        
        ate_data[method] = ate_values
        avg_ate = np.mean([v for v in ate_values if v != float('inf')])
        print(f"  {method}: 平均ATE = {avg_ate:.4f}m")
    
    print("\n⚡ 性能分析 (CPU使用率):")
    cpu_data = {}
    for method in methods:
        cpu_values = []
        for scenario in scenarios:
            try:
                result = results[method][scenario]
                if 'error' not in result:
                    cpu_max = result.get('performance_metrics', {}).get('cpu', {}).get('max', 0)
                    cpu_values.append(cpu_max)
                else:
                    cpu_values.append(0)
            except KeyError:
                cpu_values.append(0)
        
        cpu_data[method] = cpu_values
        avg_cpu = np.mean([v for v in cpu_values if v > 0])
        print(f"  {method}: 平均CPU = {avg_cpu:.1f}%")
    
    print("\n💾 内存分析:")
    memory_data = {}
    for method in methods:
        memory_values = []
        for scenario in scenarios:
            try:
                result = results[method][scenario]
                if 'error' not in result:
                    memory_max = result.get('performance_metrics', {}).get('memory', {}).get('max', 0)
                    memory_values.append(memory_max)
                else:
                    memory_values.append(0)
            except KeyError:
                memory_values.append(0)
        
        memory_data[method] = memory_values
        avg_memory = np.mean([v for v in memory_values if v > 0])
        print(f"  {method}: 平均内存 = {avg_memory:.1f}GB")
    
    return {
        'ate_data': ate_data,
        'cpu_data': cpu_data,
        'memory_data': memory_data,
        'scenarios': scenarios
    }

def generate_comparison_charts(analysis_data, output_dir):
    """生成对比图表"""
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    scenarios = analysis_data['scenarios']
    
    # 1. ATE对比图
    plt.figure(figsize=(12, 6))
    x = np.arange(len(scenarios))
    width = 0.25
    
    methods = list(analysis_data['ate_data'].keys())
    for i, method in enumerate(methods):
        ate_values = analysis_data['ate_data'][method]
        # 处理无穷大值
        ate_values = [min(v, 1.0) if v != float('inf') else 1.0 for v in ate_values]
        plt.bar(x + i * width, ate_values, width, label=method)
    
    plt.xlabel('测试场景')
    plt.ylabel('ATE RMSE (米)')
    plt.title('绝对轨迹误差对比')
    plt.xticks(x + width, scenarios, rotation=45)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    ate_chart = output_dir / 'ate_comparison.png'
    plt.savefig(ate_chart, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"ATE对比图已保存: {ate_chart}")
    
    # 2. 综合性能对比
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # CPU对比
    for i, method in enumerate(methods):
        cpu_values = analysis_data['cpu_data'][method]
        ax1.bar(x + i * width, cpu_values, width, label=method)
    
    ax1.set_xlabel('测试场景')
    ax1.set_ylabel('最大CPU使用率 (%)')
    ax1.set_title('CPU使用率对比')
    ax1.set_xticks(x + width)
    ax1.set_xticklabels(scenarios, rotation=45)
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 内存对比
    for i, method in enumerate(methods):
        memory_values = analysis_data['memory_data'][method]
        ax2.bar(x + i * width, memory_values, width, label=method)
    
    ax2.set_xlabel('测试场景')
    ax2.set_ylabel('最大内存使用 (GB)')
    ax2.set_title('内存使用对比')
    ax2.set_xticks(x + width)
    ax2.set_xticklabels(scenarios, rotation=45)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    perf_chart = output_dir / 'performance_comparison.png'
    plt.savefig(perf_chart, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"性能对比图已保存: {perf_chart}")

def generate_recommendations(analysis_data):
    """生成改进建议"""
    print("\n🎯 改进建议")
    print("=" * 50)
    
    methods = list(analysis_data['ate_data'].keys())
    
    # 找出最佳方法
    method_scores = {}
    for method in methods:
        ate_values = [v for v in analysis_data['ate_data'][method] if v != float('inf')]
        cpu_values = [v for v in analysis_data['cpu_data'][method] if v > 0]
        memory_values = [v for v in analysis_data['memory_data'][method] if v > 0]
        
        # 计算综合评分
        ate_score = 1.0 / (1.0 + np.mean(ate_values)) if ate_values else 0
        cpu_score = 100.0 / (100.0 + np.mean(cpu_values)) if cpu_values else 0
        memory_score = 4.0 / (4.0 + np.mean(memory_values)) if memory_values else 0
        
        overall_score = ate_score * 0.5 + cpu_score * 0.3 + memory_score * 0.2
        method_scores[method] = {
            'overall': overall_score,
            'ate_avg': np.mean(ate_values) if ate_values else float('inf'),
            'cpu_avg': np.mean(cpu_values) if cpu_values else 0,
            'memory_avg': np.mean(memory_values) if memory_values else 0
        }
    
    # 排序方法
    sorted_methods = sorted(method_scores.keys(), key=lambda m: method_scores[m]['overall'], reverse=True)
    
    print("🏆 方法排名 (按综合性能):")
    for i, method in enumerate(sorted_methods):
        score = method_scores[method]
        print(f"  {i+1}. {method}")
        print(f"     - 平均ATE: {score['ate_avg']:.4f}m")
        print(f"     - 平均CPU: {score['cpu_avg']:.1f}%")
        print(f"     - 平均内存: {score['memory_avg']:.1f}GB")
        print(f"     - 综合评分: {score['overall']:.3f}")
    
    # 具体建议
    best_method = sorted_methods[0]
    best_score = method_scores[best_method]
    
    print(f"\n💡 建议:")
    print(f"  🥇 推荐使用: {best_method}")
    
    if best_score['ate_avg'] < 0.1:
        print("  ✅ 定位精度优秀")
    elif best_score['ate_avg'] < 0.2:
        print("  ⚠️ 定位精度良好，可进一步优化传感器标定")
    else:
        print("  ❌ 定位精度需要改进，建议检查算法参数和传感器配置")
    
    if best_score['cpu_avg'] < 50:
        print("  ✅ CPU负载合理")
    elif best_score['cpu_avg'] < 80:
        print("  ⚠️ CPU负载较高，考虑算法优化")
    else:
        print("  ❌ CPU负载过高，建议升级硬件或优化算法")
    
    if best_score['memory_avg'] < 1:
        print("  ✅ 内存使用效率高")
    elif best_score['memory_avg'] < 2:
        print("  ⚠️ 内存使用适中")
    else:
        print("  ❌ 内存使用较高，检查内存泄漏")

def main():
    parser = argparse.ArgumentParser(description='分析和对比导航系统评估结果')
    parser.add_argument(
        '--results-dir',
        default=str(Path.home() / 'sentry_evaluation_results'),
        help='评估结果目录'
    )
    parser.add_argument(
        '--generate-charts',
        action='store_true',
        help='生成对比图表'
    )
    
    args = parser.parse_args()
    
    print("📊 哨兵导航系统评估结果分析")
    print("=" * 50)
    
    # 加载结果
    results = load_evaluation_results(args.results_dir)
    if not results:
        return
    
    # 分析结果
    analysis_data = analyze_results(results)
    
    # 生成图表
    if args.generate_charts:
        print("\n📈 生成对比图表...")
        generate_comparison_charts(analysis_data, args.results_dir)
    
    # 生成建议
    generate_recommendations(analysis_data)
    
    print(f"\n✅ 分析完成！详细结果请查看: {args.results_dir}")

if __name__ == '__main__':
    main()
