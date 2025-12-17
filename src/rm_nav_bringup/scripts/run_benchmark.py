#!/usr/bin/env python3
"""
基准测试执行脚本
"""

import sys
import os
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='运行哨兵导航系统基准测试')
    parser.add_argument(
        '--methods', 
        nargs='+', 
        default=['fastlio_slam_toolbox', 'pointlio_slam_toolbox', 'pointlio_icp'],
        help='要测试的方法列表'
    )
    parser.add_argument(
        '--scenarios',
        nargs='+',
        default=['basic_navigation', 'high_dynamic', 'feature_sparse'],
        help='要测试的场景列表'
    )
    parser.add_argument(
        '--output-dir',
        default=str(Path.home() / 'sentry_evaluation_results'),
        help='结果输出目录'
    )
    parser.add_argument(
        '--single-test',
        action='store_true',
        help='运行单个测试用于调试'
    )
    
    args = parser.parse_args()
    
    print("🤖 哨兵导航系统基准测试")
    print("=" * 50)
    print(f"📋 测试方法: {args.methods}")
    print(f"📍 测试场景: {args.scenarios}")
    print(f"📁 输出目录: {args.output_dir}")
    print("=" * 50)
    
    # 创建输出目录
    os.makedirs(args.output_dir, exist_ok=True)
    
    # 添加评估模块路径
    evaluation_path = str(Path(__file__).parent.parent / 'evaluation')
    sys.path.insert(0, evaluation_path)
    
    # 现在导入ROS和评估模块
    try:
        import rclpy
        from benchmark_runner import BenchmarkRunner
    except ImportError as e:
        print(f"❌ 导入错误: {e}")
        print("请确保：")
        print("1. ROS2环境已正确设置 (source /opt/ros/humble/setup.bash)")
        print("2. 工作空间已构建并sourced (source install/setup.bash)")
        return 1
    
    # 初始化ROS
    rclpy.init()
    
    try:
        # 创建基准测试运行器
        runner = BenchmarkRunner(output_dir=args.output_dir)
        
        if args.single_test:
            # 运行单个测试用于调试
            print("🔍 运行单个测试 (调试模式)")
            method = args.methods[0]
            scenario = args.scenarios[0]
            
            scenario_config = runner.config['test_scenarios'].get(scenario, {})
            if scenario_config:
                result = runner._run_single_test(method, scenario, scenario_config)
                print(f"\n单个测试结果:")
                print(f"方法: {method}")
                print(f"场景: {scenario}")
                if 'error' not in result:
                    print(f"ATE RMSE: {result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', 'N/A')}")
                    print(f"最大CPU: {result.get('performance_metrics', {}).get('cpu', {}).get('max', 'N/A')}%")
                else:
                    print(f"错误: {result['error']}")
            else:
                print(f"场景配置不存在: {scenario}")
        else:
            # 运行完整测试套件
            results = runner.run_benchmark_suite(args.methods, args.scenarios)
            
            print(f"\n🎉 测试完成！")
            print(f"📊 结果已保存到: {args.output_dir}")
            
            # 打印简要结果
            print("\n📈 测试结果概览:")
            for method, method_results in results.items():
                print(f"\n🔧 {method}:")
                for scenario, result in method_results.items():
                    if isinstance(result, dict) and 'error' not in result:
                        ate_rmse = result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', 'N/A')
                        cpu_max = result.get('performance_metrics', {}).get('cpu', {}).get('max', 'N/A')
                        memory_max = result.get('performance_metrics', {}).get('memory', {}).get('max', 'N/A')
                        duration = result.get('test_duration', 0)
                        
                        if isinstance(ate_rmse, (int, float)):
                            ate_str = f"{ate_rmse:.4f}m"
                        else:
                            ate_str = str(ate_rmse)
                            
                        if isinstance(cpu_max, (int, float)):
                            cpu_str = f"{cpu_max:.1f}%"
                        else:
                            cpu_str = str(cpu_max)
                            
                        if isinstance(memory_max, (int, float)):
                            mem_str = f"{memory_max:.1f}GB"
                        else:
                            mem_str = str(memory_max)
                        
                        print(f"  📍 {scenario}: ATE={ate_str}, CPU={cpu_str}, MEM={mem_str}, 时长={duration:.1f}s")
                    else:
                        print(f"  📍 {scenario}: ❌ 测试失败 - {result.get('error', '未知错误')}")
    
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
        return 1
    
    except Exception as e:
        print(f"\n❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    exit(main())
