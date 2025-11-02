#!/usr/bin/env python3
"""
基准测试运行器
负责协调整个评估流程的执行
"""

import os
import yaml
import time
import subprocess
import threading
import signal
import argparse
import random
import csv
import json
from pathlib import Path
from typing import Dict, List, Any, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose
import action_msgs.msg
from rclpy.action import ActionClient

from data_collector import DataCollector
from performance_monitor import PerformanceMonitor
from trajectory_analyzer import TrajectoryAnalyzer
from report_generator import ReportGenerator

class BenchmarkRunner(Node):
    """基准测试运行器"""
    
    def __init__(self):
        super().__init__('benchmark_runner')
        self.get_logger().info("初始化基准测试运行器...")
        
        # 加载配置
        self.config = self._load_config()
        
        # 初始化各个模块
        self.data_collector = DataCollector()
        self.performance_monitor = PerformanceMonitor()
        self.trajectory_analyzer = TrajectoryAnalyzer()
        self.report_generator = ReportGenerator()
        
        # 测试状态
        self.current_test = None
        self.test_start_time = None
        self.test_results = {}
        
        # ROS发布器和订阅器
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # Nav2 Action Client
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 导航状态监控
        self.navigation_goal_reached = False
        self.current_waypoint_index = 0
        self.navigation_timeout = False
        
        # 配置文件路径
        self.launch_params_file = Path('/home/nyz/sentry/sentry-navigation/src/rm_nav_bringup/config/launch_params.yaml')
        
    def _load_config(self):
        """加载测试配置"""
        config_path = Path(__file__).parent / "config" / "test_scenarios.yaml"
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"加载配置文件失败: {e}")
            return {'test_scenarios': {}, 'test_methods': {}}
    
    def run_benchmark_suite(self, methods: List[str], scenarios: Optional[List[str]] = None, repeats: int = 1, base_seed: Optional[int] = None) -> Dict[str, Any]:
        """运行完整的基准测试套件"""
        self.get_logger().info(f"开始运行基准测试套件")
        self.get_logger().info(f"测试方法: {methods}")
        
        # 如果没有指定场景，使用所有场景
        if scenarios is None:
            scenarios = list(self.config.get('test_scenarios', {}).keys())
        
        self.get_logger().info(f"测试场景: {scenarios}")
        
        all_results = {}
        
        summary_rows = []

        for method in methods:
            self.get_logger().info(f"======= 开始测试方法: {method} =======")
            method_results = {}
            
            for scenario_name in scenarios:
                scenario_config = self.config['test_scenarios'].get(scenario_name)
                if not scenario_config:
                    self.get_logger().warning(f"场景配置不存在: {scenario_name}")
                    continue
                    
                self.get_logger().info(f"  --- 测试场景: {scenario_name} ---")
                
                # 支持重复运行
                repeats_results = []
                for rep in range(repeats):
                    # 生成或派生随机种子
                    if base_seed is not None:
                        seed = int(base_seed) + rep
                    else:
                        seed = int(time.time()) + rep

                    self.get_logger().info(f"  --- 运行 {scenario_name} (repeat {rep+1}/{repeats}) seed={seed} ---")
                    result = self._run_single_test(method, scenario_name, scenario_config, repeat_index=rep+1, seed=seed)
                    repeats_results.append(result)

                    # 保存单次结果到文件
                    try:
                        out_dir = self.data_collector.output_dir
                        out_dir.mkdir(exist_ok=True)
                        result_file = out_dir / f"result_{method}_{scenario_name}_{int(time.time())}_rep{rep+1}.json"
                        with open(result_file, 'w', encoding='utf-8') as rf:
                            json.dump(result, rf, indent=2, ensure_ascii=False)
                        self.get_logger().info(f"已保存结果: {result_file}")
                    except Exception as e:
                        self.get_logger().warning(f"保存单次结果失败: {e}")

                    # 追加汇总行
                    try:
                        ate_rmse = result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', 'N/A')
                        rpe_rmse = result.get('trajectory_metrics', {}).get('rpe', {}).get('rmse', 'N/A')
                        cpu_max = result.get('performance_metrics', {}).get('cpu', {}).get('max', 'N/A')
                        mem_max = result.get('performance_metrics', {}).get('memory', {}).get('max', 'N/A') if result.get('performance_metrics') else 'N/A'
                    except Exception:
                        ate_rmse = rpe_rmse = cpu_max = mem_max = 'N/A'

                    summary_rows.append({
                        'method': method,
                        'scenario': scenario_name,
                        'repeat': rep+1,
                        'seed': seed,
                        'ate_rmse': ate_rmse,
                        'rpe_rmse': rpe_rmse,
                        'cpu_max': cpu_max,
                        'mem_max': mem_max,
                        'success': result.get('success', False),
                        'bag_file': result.get('bag_file', ''),
                        'timestamp': result.get('timestamp', time.time())
                    })

                method_results[scenario_name] = repeats_results
                
                # 测试间隔，让系统稳定
                self.get_logger().info("等待系统稳定...")
                time.sleep(5)
            
            all_results[method] = method_results
            self.get_logger().info(f"======= 方法 {method} 测试完成 =======")
        
        # 生成对比报告
        self.get_logger().info("生成评估报告...")
        report_file = self.report_generator.generate_comparison_report(all_results)
        self.get_logger().info(f"评估报告已生成: {report_file}")

        # 保存汇总 CSV
        try:
            csv_file = self.data_collector.output_dir / f"benchmark_summary_{int(time.time())}.csv"
            with open(csv_file, 'w', newline='', encoding='utf-8') as cf:
                writer = csv.DictWriter(cf, fieldnames=['method','scenario','repeat','seed','ate_rmse','rpe_rmse','cpu_max','mem_max','success','bag_file','timestamp'])
                writer.writeheader()
                for row in summary_rows:
                    writer.writerow(row)
            self.get_logger().info(f"汇总CSV已保存: {csv_file}")
        except Exception as e:
            self.get_logger().warning(f"保存汇总CSV失败: {e}")
        
        return all_results
    
    def _run_single_test(self, method: str, scenario_name: str, scenario_config: Dict, repeat_index: int = 1, seed: Optional[int] = None) -> Dict[str, Any]:
        """运行单个测试案例"""
        self.get_logger().info(f"开始测试: {method} - {scenario_name}")
        
        # 1. 启动对应的导航系统
        process = self._launch_navigation_system(method, scenario_config)
        
        if not process:
            return {'error': '启动导航系统失败'}
        
        # 等待系统启动
        self.get_logger().info("等待导航系统启动...")
        time.sleep(15)
        
        try:
            # 2. 开始数据收集
            bag_file = self.data_collector.start_recording(method, scenario_name)
            
            if not bag_file:
                return {'error': '启动数据收集失败'}
            
            # 3. 开始性能监控
            self.performance_monitor.start_monitoring()
            
            # 4. 执行测试场景
            self.test_start_time = time.time()
            success = self._execute_scenario(scenario_config)
            
            # 5. 等待测试完成
            if success:
                self._wait_for_test_completion(scenario_config.get('duration', 120))
            
            # 6. 停止数据收集
            self.data_collector.stop_recording()
            performance_data = self.performance_monitor.stop_monitoring()
            
            # 7. 分析结果
            self.get_logger().info("分析轨迹数据...")
            trajectory_metrics = self.trajectory_analyzer.analyze_trajectory(bag_file)
            
            # 8. 汇总结果
            test_duration = time.time() - self.test_start_time

            result = {
                'scenario': scenario_name,
                'method': method,
                'trajectory_metrics': trajectory_metrics,
                'performance_metrics': performance_data,
                'test_duration': test_duration,
                'bag_file': bag_file,
                'timestamp': time.time(),
                'success': success,
                'repeat': repeat_index,
                'seed': seed
            }
            
            self.get_logger().info(f"测试完成: {method} - {scenario_name}")
            self.get_logger().info(f"  轨迹ATE RMSE: {trajectory_metrics.get('ate', {}).get('rmse', 'N/A')}")
            self.get_logger().info(f"  最大CPU使用: {performance_data.get('cpu', {}).get('max', 'N/A')}%")
            self.get_logger().info(f"  测试时长: {test_duration:.1f}s")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"测试执行失败: {e}")
            return {'error': str(e)}
        
        finally:
            # 清理：关闭导航系统
            self._cleanup_navigation_system(process)
    
    def _launch_navigation_system(self, method: str, scenario_config: Dict) -> Optional[subprocess.Popen]:
        """启动对应的导航系统"""
        try:
            # 修改launch_params.yaml以使用指定方法
            self._update_launch_params(method, scenario_config)
            
            # 构建启动命令
            cmd = [
                'bash', '-c', 
                'cd /home/nyz/sentry/sentry-navigation && source install/setup.bash && ros2 launch rm_nav_bringup bringup.launch.py'
            ]
            
            # 启动导航系统
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 创建新的进程组
            )
            
            self.get_logger().info(f"导航系统启动中... PID: {process.pid}")
            return process
            
        except Exception as e:
            self.get_logger().error(f"启动导航系统失败: {e}")
            return None
    
    def _update_launch_params(self, method: str, scenario_config: Dict):
        """更新启动参数以使用指定的方法"""
        try:
            # 读取当前配置
            with open(self.launch_params_file, 'r', encoding='utf-8') as f:
                params = yaml.safe_load(f)
            
            # 获取方法配置
            method_config = self.config.get('test_methods', {}).get(method, {}).get('config', {})
            
            # 更新参数
            if 'lio' in method_config:
                params['lio'] = method_config['lio']
            if 'localization' in method_config:
                params['localization'] = method_config['localization']
            
            # 设置测试环境
            params['mode'] = 'nav'
            params['use_sim'] = True
            params['world'] = scenario_config.get('world', 'RMUC_25')
            
            # 保存配置
            with open(self.launch_params_file, 'w', encoding='utf-8') as f:
                yaml.dump(params, f, default_flow_style=False, allow_unicode=True)
                
            self.get_logger().info(f"启动参数已更新: LIO={params.get('lio')}, 定位={params.get('localization')}")
            
        except Exception as e:
            self.get_logger().error(f"更新启动参数失败: {e}")
    
    def _execute_scenario(self, scenario_config: Dict) -> bool:
        """执行测试场景"""
        try:
            waypoints = scenario_config.get('waypoints', [])
            initial_pose = scenario_config.get('initial_pose', [0, 0, 0, 0, 0, 0])
            
            # 设置初始位置
            self._set_initial_pose(initial_pose)
            time.sleep(2)
            
            # 设置初始位姿估计
            self._set_initial_pose_estimate(initial_pose)
            time.sleep(3)
            
            # 逐个导航到各个航点
            for i, waypoint in enumerate(waypoints):
                self.current_waypoint_index = i
                self.get_logger().info(f"导航到航点 {i+1}/{len(waypoints)}: {waypoint}")
                
                success = self._navigate_to_waypoint(waypoint)
                if not success:
                    self.get_logger().warning(f"导航到航点 {i+1} 失败")
                    return False
                    
                # 短暂停留
                time.sleep(1)
            
            self.get_logger().info("所有航点导航完成")
            return True
            
        except Exception as e:
            self.get_logger().error(f"执行测试场景失败: {e}")
            return False
    
    def _set_initial_pose(self, pose: List[float]):
        """在仿真中设置机器人初始位姿"""
        try:
            if len(pose) >= 6:
                # 使用Gazebo服务设置机器人位置
                cmd = [
                    'ros2', 'service', 'call', '/gazebo/set_entity_state',
                    'gazebo_msgs/srv/SetEntityState',
                    f'{{state: {{name: "robot", pose: {{position: {{x: {pose[0]}, y: {pose[1]}, z: {pose[2]}}}, orientation: {{x: 0, y: 0, z: {pose[5]}, w: 1}}}}}}}}'
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    self.get_logger().info(f"机器人位姿已设置: {pose}")
                else:
                    self.get_logger().warning(f"设置机器人位姿失败: {result.stderr}")
                    
        except Exception as e:
            self.get_logger().error(f"设置初始位姿失败: {e}")
    
    def _set_initial_pose_estimate(self, pose: List[float]):
        """设置初始位姿估计"""
        try:
            if len(pose) >= 6:
                initial_pose = PoseWithCovarianceStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = self.get_clock().now().to_msg()
                
                initial_pose.pose.pose.position.x = float(pose[0])
                initial_pose.pose.pose.position.y = float(pose[1])
                initial_pose.pose.pose.position.z = float(pose[2])
                
                # 简单的yaw角度转四元数
                import math
                yaw = pose[5]
                initial_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
                initial_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                # 设置协方差矩阵
                initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
                
                self.initial_pose_publisher.publish(initial_pose)
                self.get_logger().info("初始位姿估计已发布")
                
        except Exception as e:
            self.get_logger().error(f"设置初始位姿估计失败: {e}")
    
    def _navigate_to_waypoint(self, waypoint: List[float], timeout: float = 30.0) -> bool:
        """导航到指定航点"""
        try:
            if len(waypoint) < 3:
                return False
            
            # 创建导航目标
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.pose.pose.position.x = float(waypoint[0])
            goal_msg.pose.pose.position.y = float(waypoint[1])
            goal_msg.pose.pose.position.z = float(waypoint[2])
            goal_msg.pose.pose.orientation.w = 1.0
            
            # 等待Action服务器
            if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Nav2 Action服务器不可用")
                return False
            
            # 发送目标
            future = self.nav_action_client.send_goal_async(goal_msg)
            
            # 等待目标被接受
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                self.get_logger().error("发送导航目标超时")
                return False
                
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("导航目标被拒绝")
                return False
            
            # 等待导航完成
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if result_future.done():
                result = result_future.result()
                if result.status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info("导航成功")
                    return True
                else:
                    self.get_logger().warning(f"导航失败，状态: {result.status}")
                    return False
            else:
                self.get_logger().warning("导航超时")
                # 取消目标
                goal_handle.cancel_goal_async()
                return False
                
        except Exception as e:
            self.get_logger().error(f"导航失败: {e}")
            return False
    
    def _wait_for_test_completion(self, max_duration: float):
        """等待测试完成"""
        self.get_logger().info(f"等待测试完成，最大时长: {max_duration}s")
        time.sleep(max_duration)
    
    def _cleanup_navigation_system(self, process: subprocess.Popen):
        """清理导航系统进程"""
        try:
            if process and process.poll() is None:
                self.get_logger().info("正在关闭导航系统...")
                
                # 发送SIGTERM信号给整个进程组
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                
                # 等待进程正常退出
                try:
                    process.wait(timeout=10)
                    self.get_logger().info("导航系统已正常关闭")
                except subprocess.TimeoutExpired:
                    # 如果超时，强制杀死
                    self.get_logger().warning("强制关闭导航系统")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait()
                    
        except Exception as e:
            self.get_logger().error(f"清理导航系统失败: {e}")

def main():
    """主函数"""
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument('--repeats', type=int, default=1, help='每个场景的重复次数')
    parser.add_argument('--seed', type=int, default=None, help='可选的基准种子（将与重复索引相加）')
    args = parser.parse_args()

    runner = BenchmarkRunner()

    try:
        # 定义要测试的方法
        test_methods = [
            'fastlio_slam_toolbox',
            'pointlio_slam_toolbox', 
            'pointlio_icp'
        ]
        
        # 定义要测试的场景
        test_scenarios = [
            'basic_navigation',
            'high_dynamic',
            'feature_sparse'
        ]

        # 运行基准测试
        print("\n" + "="*50)
        print("🤖 哨兵导航系统基准测试")
        print("="*50)

        results = runner.run_benchmark_suite(test_methods, test_scenarios, repeats=args.repeats, base_seed=args.seed)
        
        print("\n" + "="*50)
        print("✅ 基准测试完成!")
        print("="*50)
        
        # 简要输出结果
        for method, method_results in results.items():
            print(f"\n🔧 {method}:")
            for scenario, result in method_results.items():
                if isinstance(result, dict) and 'error' not in result:
                    ate_rmse = result.get('trajectory_metrics', {}).get('ate', {}).get('rmse', 'N/A')
                    cpu_max = result.get('performance_metrics', {}).get('cpu', {}).get('max', 'N/A')
                    print(f"  📍 {scenario}: ATE={ate_rmse:.4f}m, CPU={cpu_max:.1f}%")
                else:
                    print(f"  📍 {scenario}: ❌ 失败 - {result.get('error', '未知错误')}")
    
    except KeyboardInterrupt:
        print("\n用户中断测试")
    
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
