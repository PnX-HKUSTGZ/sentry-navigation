#!/usr/bin/env python3
"""
报告生成器
生成HTML格式的详细评估报告
"""

import json
import time
from pathlib import Path
from typing import Dict, List, Any
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

class ReportGenerator:
    """报告生成器"""
    
    def __init__(self, output_dir: str | None = None):
        self.output_dir = Path(output_dir) if output_dir else (Path.home() / 'sentry_evaluation_results')
        self.output_dir.mkdir(exist_ok=True)
        
    def generate_comparison_report(self, all_results: Dict[str, Any]) -> str:
        """生成对比报告"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = self.output_dir / f"evaluation_report_{timestamp}.html"
        
        # 生成可视化图表
        charts = self._generate_charts(all_results)
        
        # 生成HTML报告
        html_content = self._generate_html_report(all_results, charts, timestamp)
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(html_content)
        
        # 同时保存JSON格式的原始数据
        json_file = self.output_dir / f"evaluation_data_{timestamp}.json"
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(all_results, f, indent=2, ensure_ascii=False)
        
        print(f"评估报告已生成: {report_file}")
        print(f"原始数据已保存: {json_file}")
        
        return str(report_file)
    
    def _generate_charts(self, all_results: Dict[str, Any]) -> Dict[str, str]:
        """生成可视化图表"""
        charts = {}
        
        try:
            # 准备数据
            methods = list(all_results.keys())
            scenarios = []
            
            # 获取所有场景
            for method_results in all_results.values():
                if isinstance(method_results, dict):
                    scenarios.extend(method_results.keys())
            scenarios = list(set(scenarios))
            
            # 1. ATE对比图
            charts['ate_comparison'] = self._create_ate_comparison_chart(all_results, methods, scenarios)
            
            # 2. CPU使用率对比图
            charts['cpu_comparison'] = self._create_cpu_comparison_chart(all_results, methods, scenarios)
            
            # 3. 内存使用对比图
            charts['memory_comparison'] = self._create_memory_comparison_chart(all_results, methods, scenarios)
            
            # 4. 综合性能雷达图
            charts['radar_chart'] = self._create_performance_radar_chart(all_results, methods)
            
        except Exception as e:
            print(f"生成图表时发生错误: {e}")
            
        return charts
    
    def _create_ate_comparison_chart(self, all_results: Dict, methods: List[str], scenarios: List[str]) -> str:
        """创建ATE对比图"""
        try:
            fig, ax = plt.subplots(figsize=(12, 6))
            
            x = np.arange(len(scenarios))
            width = 0.2
            
            for i, method in enumerate(methods):
                ate_values = []
                for scenario in scenarios:
                    try:
                        result = all_results[method][scenario]
                        if 'error' not in result:
                            ate_rmse = result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', 0)
                            ate_values.append(ate_rmse)
                        else:
                            ate_values.append(float('inf'))
                    except (KeyError, TypeError):
                        ate_values.append(0)
                
                # 处理无穷大值
                ate_values = [min(v, 10) if v != float('inf') else 10 for v in ate_values]
                
                ax.bar(x + i * width, ate_values, width, label=method)
            
            ax.set_xlabel('测试场景')
            ax.set_ylabel('ATE RMSE (米)')
            ax.set_title('绝对轨迹误差对比')
            ax.set_xticks(x + width * (len(methods) - 1) / 2)
            ax.set_xticklabels(scenarios, rotation=45)
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            plt.tight_layout()
            chart_file = self.output_dir / 'ate_comparison.png'
            plt.savefig(chart_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            return str(chart_file)
            
        except Exception as e:
            print(f"创建ATE对比图失败: {e}")
            return ""
    
    def _create_cpu_comparison_chart(self, all_results: Dict, methods: List[str], scenarios: List[str]) -> str:
        """创建CPU使用率对比图"""
        try:
            fig, ax = plt.subplots(figsize=(12, 6))
            
            x = np.arange(len(scenarios))
            width = 0.2
            
            for i, method in enumerate(methods):
                cpu_values = []
                for scenario in scenarios:
                    try:
                        result = all_results[method][scenario]
                        if 'error' not in result:
                            cpu_max = result.get('performance_metrics', {}).get('cpu', {}).get('max', 0)
                            cpu_values.append(cpu_max)
                        else:
                            cpu_values.append(0)
                    except (KeyError, TypeError):
                        cpu_values.append(0)
                
                ax.bar(x + i * width, cpu_values, width, label=method)
            
            ax.set_xlabel('测试场景')
            ax.set_ylabel('最大CPU使用率 (%)')
            ax.set_title('CPU使用率对比')
            ax.set_xticks(x + width * (len(methods) - 1) / 2)
            ax.set_xticklabels(scenarios, rotation=45)
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            plt.tight_layout()
            chart_file = self.output_dir / 'cpu_comparison.png'
            plt.savefig(chart_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            return str(chart_file)
            
        except Exception as e:
            print(f"创建CPU对比图失败: {e}")
            return ""
    
    def _create_memory_comparison_chart(self, all_results: Dict, methods: List[str], scenarios: List[str]) -> str:
        """创建内存使用对比图"""
        try:
            fig, ax = plt.subplots(figsize=(12, 6))
            
            x = np.arange(len(scenarios))
            width = 0.2
            
            for i, method in enumerate(methods):
                memory_values = []
                for scenario in scenarios:
                    try:
                        result = all_results[method][scenario]
                        if 'error' not in result:
                            memory_max = result.get('performance_metrics', {}).get('memory', {}).get('max', 0)
                            memory_values.append(memory_max)
                        else:
                            memory_values.append(0)
                    except (KeyError, TypeError):
                        memory_values.append(0)
                
                ax.bar(x + i * width, memory_values, width, label=method)
            
            ax.set_xlabel('测试场景')
            ax.set_ylabel('最大内存使用 (GB)')
            ax.set_title('内存使用对比')
            ax.set_xticks(x + width * (len(methods) - 1) / 2)
            ax.set_xticklabels(scenarios, rotation=45)
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            plt.tight_layout()
            chart_file = self.output_dir / 'memory_comparison.png'
            plt.savefig(chart_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            return str(chart_file)
            
        except Exception as e:
            print(f"创建内存对比图失败: {e}")
            return ""
    
    def _create_performance_radar_chart(self, all_results: Dict, methods: List[str]) -> str:
        """创建性能雷达图"""
        try:
            # 计算每个方法的综合性能指标
            performance_data = {}
            
            for method in methods:
                method_results = all_results.get(method, {})
                
                # 计算平均指标
                ate_scores = []
                cpu_scores = []
                memory_scores = []
                stability_scores = []
                
                for scenario_result in method_results.values():
                    if isinstance(scenario_result, dict) and 'error' not in scenario_result:
                        # ATE评分 (越小越好，转换为0-100分)
                        ate_rmse = scenario_result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', 0)
                        ate_score = max(0, 100 - ate_rmse * 100)  # 假设1米误差为0分
                        ate_scores.append(ate_score)
                        
                        # 性能评分
                        perf_metrics = scenario_result.get('performance_metrics', {})
                        summary = perf_metrics.get('summary', {})
                        cpu_scores.append(summary.get('cpu_score', 0))
                        memory_scores.append(summary.get('memory_score', 0))
                        stability_scores.append(summary.get('stability_score', 0))
                
                performance_data[method] = {
                    'accuracy': np.mean(ate_scores) if ate_scores else 0,
                    'cpu_efficiency': np.mean(cpu_scores) if cpu_scores else 0,
                    'memory_efficiency': np.mean(memory_scores) if memory_scores else 0,
                    'stability': np.mean(stability_scores) if stability_scores else 0
                }
            
            # 创建雷达图
            categories = ['Accuracy', 'CPU Efficiency', 'Memory Efficiency', 'Stability']
            
            fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
            
            angles = [n / float(len(categories)) * 2 * np.pi for n in range(len(categories))]
            angles += angles[:1]  # 闭合图形
            
            for method in methods:
                values = [
                    performance_data[method]['accuracy'],
                    performance_data[method]['cpu_efficiency'],
                    performance_data[method]['memory_efficiency'],
                    performance_data[method]['stability']
                ]
                values += values[:1]  # 闭合图形
                
                ax.plot(angles, values, 'o-', linewidth=2, label=method)
                ax.fill(angles, values, alpha=0.25)
            
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(categories)
            ax.set_ylim(0, 100)
            ax.set_title('综合性能对比', size=16, y=1.1)
            ax.legend(loc='upper right', bbox_to_anchor=(1.2, 1.0))
            ax.grid(True)
            
            plt.tight_layout()
            chart_file = self.output_dir / 'performance_radar.png'
            plt.savefig(chart_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            return str(chart_file)
            
        except Exception as e:
            print(f"创建雷达图失败: {e}")
            return ""
    
    def _generate_html_report(self, all_results: Dict, charts: Dict[str, str], timestamp: str) -> str:
        """生成HTML报告"""
        
        # 计算汇总统计
        summary_stats = self._calculate_summary_statistics(all_results)
        
        html_template = f"""
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>哨兵导航系统评估报告</title>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }}
        h1 {{
            color: #2c3e50;
            text-align: center;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
        }}
        h2 {{
            color: #34495e;
            border-left: 4px solid #3498db;
            padding-left: 15px;
            margin-top: 30px;
        }}
        h3 {{
            color: #7f8c8d;
        }}
        .summary {{
            background-color: #ecf0f1;
            padding: 20px;
            border-radius: 8px;
            margin-bottom: 30px;
        }}
        .method-section {{
            margin-bottom: 40px;
            padding: 20px;
            border: 1px solid #bdc3c7;
            border-radius: 8px;
        }}
        .scenario {{
            margin-bottom: 20px;
            padding: 15px;
            background-color: #f8f9fa;
            border-left: 3px solid #3498db;
        }}
        .metrics-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin: 15px 0;
        }}
        .metric-card {{
            background-color: white;
            padding: 15px;
            border-radius: 6px;
            border: 1px solid #e0e0e0;
            text-align: center;
        }}
        .metric-value {{
            font-size: 24px;
            font-weight: bold;
            color: #2c3e50;
        }}
        .metric-label {{
            font-size: 12px;
            color: #7f8c8d;
            margin-top: 5px;
        }}
        .chart {{
            text-align: center;
            margin: 20px 0;
        }}
        .chart img {{
            max-width: 100%;
            height: auto;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }}
        .good {{ color: #27ae60; }}
        .warning {{ color: #f39c12; }}
        .error {{ color: #e74c3c; }}
        .info-box {{
            background-color: #d4edda;
            border-left: 4px solid #28a745;
            padding: 15px;
            margin: 15px 0;
            border-radius: 4px;
        }}
        .warning-box {{
            background-color: #fff3cd;
            border-left: 4px solid #ffc107;
            padding: 15px;
            margin: 15px 0;
            border-radius: 4px;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }}
        th, td {{
            border: 1px solid #ddd;
            padding: 12px;
            text-align: left;
        }}
        th {{
            background-color: #f8f9fa;
            font-weight: bold;
        }}
        .timestamp {{
            text-align: center;
            color: #7f8c8d;
            font-size: 14px;
            margin-top: 30px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 哨兵导航系统评估报告</h1>
        
        <div class="summary">
            <h2>📊 评估概要</h2>
            <p><strong>测试时间:</strong> {timestamp}</p>
            <p><strong>测试方法数量:</strong> {len(all_results)}</p>
            <p><strong>测试场景:</strong> {', '.join(summary_stats.get('scenarios', []))}</p>
            
            <div class="metrics-grid">
                <div class="metric-card">
                    <div class="metric-value">{summary_stats.get('best_method', 'N/A')}</div>
                    <div class="metric-label">最佳方法</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">{summary_stats.get('best_ate', 0):.3f}m</div>
                    <div class="metric-label">最佳ATE RMSE</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">{summary_stats.get('avg_cpu', 0):.1f}%</div>
                    <div class="metric-label">平均CPU使用率</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">{summary_stats.get('avg_memory', 0):.1f}GB</div>
                    <div class="metric-label">平均内存使用</div>
                </div>
            </div>
        </div>

        <h2>📈 可视化分析</h2>
        {self._generate_charts_html(charts)}

        <h2>📋 详细结果</h2>
        {self._generate_detailed_results_html(all_results)}

        <h2>🎯 建议与结论</h2>
        {self._generate_recommendations_html(all_results, summary_stats)}

        <div class="timestamp">
            报告生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        </div>
    </div>
</body>
</html>
"""
        
        return html_template
    
    def _generate_charts_html(self, charts: Dict[str, str]) -> str:
        """生成图表HTML"""
        html = ""
        
        chart_titles = {
            'ate_comparison': '绝对轨迹误差对比',
            'cpu_comparison': 'CPU使用率对比',
            'memory_comparison': '内存使用对比',
            'radar_chart': '综合性能雷达图'
        }
        
        for chart_key, chart_file in charts.items():
            if chart_file and Path(chart_file).exists():
                title = chart_titles.get(chart_key, chart_key)
                # 使用相对路径
                relative_path = Path(chart_file).name
                html += f"""
                <div class="chart">
                    <h3>{title}</h3>
                    <img src="{relative_path}" alt="{title}">
                </div>
                """
        
        return html
    
    def _generate_detailed_results_html(self, all_results: Dict) -> str:
        """生成详细结果HTML"""
        html = ""
        
        for method, method_results in all_results.items():
            html += f'<div class="method-section">'
            html += f'<h3>🔧 {method}</h3>'
            
            if isinstance(method_results, dict):
                for scenario, result in method_results.items():
                    html += f'<div class="scenario">'
                    html += f'<h4>📍 {scenario}</h4>'
                    
                    if isinstance(result, dict) and 'error' not in result:
                        # 轨迹指标
                        trajectory_metrics = result.get('trajectory_metrics', {})
                        ate = trajectory_metrics.get('ate', {})
                        rpe = trajectory_metrics.get('rpe', {})
                        
                        # 性能指标
                        performance_metrics = result.get('performance_metrics', {})
                        cpu = performance_metrics.get('cpu', {})
                        memory = performance_metrics.get('memory', {})
                        
                        html += '<div class="metrics-grid">'
                        
                        # ATE指标
                        ate_rmse = ate.get('rmse', 0)
                        ate_class = 'good' if ate_rmse < 0.1 else 'warning' if ate_rmse < 0.2 else 'error'
                        html += f'''
                        <div class="metric-card">
                            <div class="metric-value {ate_class}">{ate_rmse:.3f}m</div>
                            <div class="metric-label">ATE RMSE</div>
                        </div>
                        '''
                        
                        # RPE指标
                        rpe_rmse = rpe.get('rmse', 0)
                        rpe_class = 'good' if rpe_rmse < 0.05 else 'warning' if rpe_rmse < 0.1 else 'error'
                        html += f'''
                        <div class="metric-card">
                            <div class="metric-value {rpe_class}">{rpe_rmse:.3f}m</div>
                            <div class="metric-label">RPE RMSE</div>
                        </div>
                        '''
                        
                        # CPU指标
                        cpu_max = cpu.get('max', 0)
                        cpu_class = 'good' if cpu_max < 50 else 'warning' if cpu_max < 80 else 'error'
                        html += f'''
                        <div class="metric-card">
                            <div class="metric-value {cpu_class}">{cpu_max:.1f}%</div>
                            <div class="metric-label">最大CPU</div>
                        </div>
                        '''
                        
                        # 内存指标
                        memory_max = memory.get('max', 0)
                        memory_class = 'good' if memory_max < 1 else 'warning' if memory_max < 2 else 'error'
                        html += f'''
                        <div class="metric-card">
                            <div class="metric-value {memory_class}">{memory_max:.1f}GB</div>
                            <div class="metric-label">最大内存</div>
                        </div>
                        '''
                        
                        html += '</div>'  # metrics-grid
                        
                    else:
                        html += f'<div class="warning-box">❌ 测试失败: {result.get("error", "未知错误")}</div>'
                    
                    html += '</div>'  # scenario
            
            html += '</div>'  # method-section
        
        return html
    
    def _generate_recommendations_html(self, all_results: Dict, summary_stats: Dict) -> str:
        """生成建议HTML"""
        recommendations = []
        
        # 分析结果并生成建议
        best_method = summary_stats.get('best_method', '')
        if best_method:
            recommendations.append(f"🏆 **推荐方案**: {best_method} 在综合性能方面表现最佳")
        
        # 精度分析
        best_ate = summary_stats.get('best_ate', float('inf'))
        if best_ate < 0.1:
            recommendations.append("✅ 整体定位精度优秀，ATE误差控制在10cm以内")
        elif best_ate < 0.2:
            recommendations.append("⚠️ 定位精度良好，但仍有优化空间")
        else:
            recommendations.append("❌ 定位精度需要改进，建议检查传感器标定和算法参数")
        
        # 性能分析
        avg_cpu = summary_stats.get('avg_cpu', 0)
        if avg_cpu < 50:
            recommendations.append("✅ CPU使用率合理，系统负载适中")
        elif avg_cpu < 80:
            recommendations.append("⚠️ CPU使用率较高，建议优化算法或升级硬件")
        else:
            recommendations.append("❌ CPU使用率过高，可能影响实时性能")
        
        # 内存分析
        avg_memory = summary_stats.get('avg_memory', 0)
        if avg_memory < 1:
            recommendations.append("✅ 内存使用效率高")
        elif avg_memory < 2:
            recommendations.append("⚠️ 内存使用适中")
        else:
            recommendations.append("❌ 内存使用较高，建议检查内存泄漏")
        
        html = '<div class="info-box">'
        for rec in recommendations:
            html += f'<p>{rec}</p>'
        html += '</div>'
        
        return html
    
    def _calculate_summary_statistics(self, all_results: Dict) -> Dict[str, Any]:
        """计算汇总统计"""
        summary = {
            'scenarios': [],
            'best_method': '',
            'best_ate': float('inf'),
            'avg_cpu': 0,
            'avg_memory': 0
        }
        
        try:
            # 收集所有场景
            all_scenarios = set()
            for method_results in all_results.values():
                if isinstance(method_results, dict):
                    all_scenarios.update(method_results.keys())
            summary['scenarios'] = list(all_scenarios)
            
            # 计算每个方法的平均性能
            method_scores = {}
            all_cpu = []
            all_memory = []
            
            for method, method_results in all_results.items():
                if isinstance(method_results, dict):
                    ate_values = []
                    cpu_values = []
                    memory_values = []
                    
                    for result in method_results.values():
                        if isinstance(result, dict) and 'error' not in result:
                            # ATE
                            ate_rmse = result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', float('inf'))
                            if ate_rmse != float('inf'):
                                ate_values.append(ate_rmse)
                            
                            # CPU和内存
                            perf = result.get('performance_metrics', {})
                            cpu_max = perf.get('cpu', {}).get('max', 0)
                            memory_max = perf.get('memory', {}).get('max', 0)
                            
                            if cpu_max > 0:
                                cpu_values.append(cpu_max)
                                all_cpu.append(cpu_max)
                            if memory_max > 0:
                                memory_values.append(memory_max)
                                all_memory.append(memory_max)
                    
                    # 计算方法得分 (简单的加权平均)
                    avg_ate = np.mean(ate_values) if ate_values else float('inf')
                    avg_cpu = np.mean(cpu_values) if cpu_values else 100
                    avg_memory = np.mean(memory_values) if memory_values else 10
                    
                    # 综合评分 (ATE越小越好，CPU和内存使用越少越好)
                    ate_score = max(0, 1 - avg_ate)  # ATE < 1m 为满分
                    cpu_score = max(0, (100 - avg_cpu) / 100)
                    memory_score = max(0, (4 - avg_memory) / 4)  # 4GB为基准
                    
                    total_score = ate_score * 0.5 + cpu_score * 0.3 + memory_score * 0.2
                    method_scores[method] = {
                        'score': total_score,
                        'ate': avg_ate
                    }
            
            # 找出最佳方法
            if method_scores:
                best_method = max(method_scores.keys(), key=lambda m: method_scores[m]['score'])
                summary['best_method'] = best_method
                summary['best_ate'] = method_scores[best_method]['ate']
            
            # 平均性能
            summary['avg_cpu'] = np.mean(all_cpu) if all_cpu else 0
            summary['avg_memory'] = np.mean(all_memory) if all_memory else 0
            
        except Exception as e:
            print(f"计算汇总统计时发生错误: {e}")
        
        return summary

def main():
    """测试报告生成器"""
    # 创建模拟数据进行测试
    test_results = {
        'fastlio_slam_toolbox': {
            'basic_navigation': {
                'trajectory_metrics': {
                    'ate': {'rmse': 0.15, 'mean': 0.12},
                    'rpe': {'rmse': 0.08, 'mean': 0.06}
                },
                'performance_metrics': {
                    'cpu': {'max': 65, 'mean': 45},
                    'memory': {'max': 1.2, 'mean': 1.0}
                }
            }
        },
        'pointlio_slam_toolbox': {
            'basic_navigation': {
                'trajectory_metrics': {
                    'ate': {'rmse': 0.12, 'mean': 0.10},
                    'rpe': {'rmse': 0.06, 'mean': 0.05}
                },
                'performance_metrics': {
                    'cpu': {'max': 75, 'mean': 55},
                    'memory': {'max': 1.5, 'mean': 1.3}
                }
            }
        }
    }
    
    generator = ReportGenerator()
    report_file = generator.generate_comparison_report(test_results)
    print(f"测试报告已生成: {report_file}")

if __name__ == '__main__':
    main()
