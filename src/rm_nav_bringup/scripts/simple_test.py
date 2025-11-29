#!/usr/bin/env python3
"""
简化的基准测试脚本
直接在脚本中实现基本功能，避免复杂的模块导入
"""

import sys
import os
import time
import subprocess
import signal
import json
import argparse
from pathlib import Path
import yaml

def print_colored(text, color=""):
    """打印彩色文本"""
    colors = {
        "green": "\033[0;32m",
        "yellow": "\033[1;33m", 
        "red": "\033[0;31m",
        "blue": "\033[0;34m",
        "reset": "\033[0m"
    }
    print(f"{colors.get(color, '')}{text}{colors['reset']}")

def load_test_config():
    """加载测试配置"""
    config_path = Path(__file__).parent.parent / 'evaluation' / 'config' / 'test_scenarios.yaml'
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print_colored(f"❌ 加载配置文件失败: {e}", "red")
        return None

def update_launch_params(method, config):
    """更新启动参数"""
    params_file = Path('/home/nyz/sentry/sentry-navigation/src/rm_nav_bringup/config/launch_params.yaml')
    
    try:
        with open(params_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        
        # 获取方法配置
        method_config = config.get('test_methods', {}).get(method, {}).get('config', {})
        
        # 更新参数
        if 'lio' in method_config:
            params['lio'] = method_config['lio']
        if 'localization' in method_config:
            params['localization'] = method_config['localization']
        
        # 设置测试环境
        params['mode'] = 'nav'
        params['use_sim'] = True
        
        with open(params_file, 'w', encoding='utf-8') as f:
            yaml.dump(params, f, default_flow_style=False, allow_unicode=True)
            
        print_colored(f"✅ 启动参数已更新: LIO={params.get('lio')}, 定位={params.get('localization')}", "green")
        return True
        
    except Exception as e:
        print_colored(f"❌ 更新启动参数失败: {e}", "red")
        return False

def run_simple_test(method, scenario, config):
    """运行简化的测试"""
    print_colored(f"🚀 开始测试: {method} - {scenario}", "blue")
    
    scenario_config = config['test_scenarios'].get(scenario, {})
    if not scenario_config:
        return {'error': f'场景配置不存在: {scenario}'}
    
    # 更新启动参数
    if not update_launch_params(method, config):
        return {'error': '更新启动参数失败'}
    
    print_colored("🔧 启动导航系统...", "yellow")
    
    # 启动导航系统
    launch_cmd = [
        'bash', '-c', 
        'cd /home/nyz/sentry/sentry-navigation && source install/setup.bash && ros2 launch rm_nav_bringup bringup.launch.py'
    ]
    
    try:
        process = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        
        print_colored(f"导航系统PID: {process.pid}", "blue")
        print_colored("等待系统启动...", "yellow")
        time.sleep(20)  # 等待系统启动
        
        # 检查进程是否还在运行
        if process.poll() is not None:
            stdout, stderr = process.communicate()
            return {
                'error': f'导航系统启动失败',
                'stdout': stdout.decode('utf-8')[-1000:],  # 最后1000字符
                'stderr': stderr.decode('utf-8')[-1000:]
            }
        
        print_colored("✅ 导航系统启动成功", "green")
        
        # 模拟测试运行一段时间
        test_duration = 30  # 简化测试，只运行30秒
        print_colored(f"🧪 运行测试 {test_duration} 秒...", "yellow")
        
        start_time = time.time()
        time.sleep(test_duration)
        actual_duration = time.time() - start_time
        
        # 收集一些基本的系统信息
        cpu_info = get_system_info()
        
        result = {
            'method': method,
            'scenario': scenario,
            'test_duration': actual_duration,
            'status': 'completed',
            'system_info': cpu_info,
            'timestamp': time.time()
        }
        
        print_colored(f"✅ 测试完成: {method} - {scenario}", "green")
        return result
        
    except Exception as e:
        return {'error': f'测试执行失败: {e}'}
        
    finally:
        # 清理进程
        try:
            if process and process.poll() is None:
                print_colored("🧹 清理导航系统进程...", "yellow")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                time.sleep(5)
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                print_colored("✅ 进程清理完成", "green")
        except Exception as e:
            print_colored(f"⚠️ 进程清理警告: {e}", "yellow")

def get_system_info():
    """获取系统信息"""
    try:
        # 获取CPU信息
        cpu_cmd = ['bash', '-c', "top -bn1 | grep 'Cpu(s)' | awk '{print $2}' | sed 's/%us,//'"]
        cpu_result = subprocess.run(cpu_cmd, capture_output=True, text=True, timeout=5)
        cpu_usage = cpu_result.stdout.strip() if cpu_result.returncode == 0 else "unknown"
        
        # 获取内存信息
        mem_cmd = ['bash', '-c', "free -m | awk 'NR==2{printf \"%.1f\", $3/1024}'"]
        mem_result = subprocess.run(mem_cmd, capture_output=True, text=True, timeout=5)
        memory_usage = mem_result.stdout.strip() if mem_result.returncode == 0 else "unknown"
        
        return {
            'cpu_usage': cpu_usage,
            'memory_usage_gb': memory_usage
        }
    except Exception:
        return {'cpu_usage': 'unknown', 'memory_usage_gb': 'unknown'}

def save_results(results, output_dir):
    """保存结果"""
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    timestamp = int(time.time())
    result_file = output_dir / f"simple_test_results_{timestamp}.json"
    
    try:
        with open(result_file, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        
        print_colored(f"📁 结果已保存: {result_file}", "green")
        return str(result_file)
        
    except Exception as e:
        print_colored(f"❌ 保存结果失败: {e}", "red")
        return None

def main():
    parser = argparse.ArgumentParser(description='简化的哨兵导航系统测试')
    parser.add_argument(
        '--methods', 
        nargs='+', 
        default=['fastlio_slam_toolbox'],
        help='要测试的方法列表'
    )
    parser.add_argument(
        '--scenarios',
        nargs='+',
        default=['basic_navigation'],
        help='要测试的场景列表'
    )
    parser.add_argument(
        '--output-dir',
        default=str(Path.home() / 'sentry_evaluation_results'),
        help='结果输出目录'
    )
    
    args = parser.parse_args()
    
    print_colored("🤖 哨兵导航系统简化测试", "blue")
    print("=" * 50)
    print(f"📋 测试方法: {args.methods}")
    print(f"📍 测试场景: {args.scenarios}")
    print(f"📁 输出目录: {args.output_dir}")
    print("=" * 50)
    
    # 加载配置
    config = load_test_config()
    if not config:
        return 1
    
    # 运行测试
    all_results = {}
    
    for method in args.methods:
        method_results = {}
        
        for scenario in args.scenarios:
            print_colored(f"\n🔄 测试: {method} - {scenario}", "blue")
            result = run_simple_test(method, scenario, config)
            method_results[scenario] = result
            
            if 'error' in result:
                print_colored(f"❌ 测试失败: {result['error']}", "red")
            else:
                print_colored(f"✅ 测试成功", "green")
            
            # 测试间隔
            print_colored("⏱️ 等待系统稳定...", "yellow")
            time.sleep(10)
        
        all_results[method] = method_results
    
    # 保存结果
    result_file = save_results(all_results, args.output_dir)
    
    # 打印总结
    print_colored("\n📊 测试总结", "blue")
    print("=" * 50)
    
    for method, method_results in all_results.items():
        print(f"\n🔧 {method}:")
        for scenario, result in method_results.items():
            if 'error' not in result:
                duration = result.get('test_duration', 0)
                cpu = result.get('system_info', {}).get('cpu_usage', 'N/A')
                memory = result.get('system_info', {}).get('memory_usage_gb', 'N/A')
                print(f"  📍 {scenario}: ✅ 成功 (时长={duration:.1f}s, CPU={cpu}%, 内存={memory}GB)")
            else:
                print(f"  📍 {scenario}: ❌ 失败 - {result['error']}")
    
    print_colored(f"\n🎉 测试完成！结果保存在: {args.output_dir}", "green")
    return 0

if __name__ == '__main__':
    exit(main())
