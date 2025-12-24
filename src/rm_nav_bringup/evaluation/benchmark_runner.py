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
import math
import psutil
from pathlib import Path
from typing import Dict, List, Any, Optional
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as NavPath
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

    def __init__(self, output_dir: Optional[str] = None):
        super().__init__('benchmark_runner')
        self.get_logger().info("初始化基准测试运行器...")

        # 评估输出目录（报告/汇总/日志/数据包）
        # 约定：output_dir 下同时包含报告文件与 sentry_evaluation_data 子目录
        if output_dir:
            self.output_dir = Path(output_dir)
        else:
            self.output_dir = Path.home() / 'sentry_evaluation_results'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.data_dir = self.output_dir / 'sentry_evaluation_data'
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir = self.output_dir / 'logs'
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        
        # 加载配置
        self.config = self._load_config()
        
        # 初始化各个模块
        self.data_collector = DataCollector(output_dir=str(self.data_dir))
        self.performance_monitor = PerformanceMonitor()
        self.trajectory_analyzer = TrajectoryAnalyzer()
        self.report_generator = ReportGenerator(output_dir=str(self.output_dir))

        # 记录启动进程的日志句柄，便于清理
        self._launch_log_handle = None
        
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

        # TF（用于等待定位初始化完成）
        self._tf_buffer = None
        self._tf_listener = None
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        except Exception as e:
            self.get_logger().warning(f"TF2 初始化失败，部分等待逻辑将跳过: {e}")
        
        # 导航状态监控
        self.navigation_goal_reached = False
        self.current_waypoint_index = 0
        self.navigation_timeout = False
        
        # 配置文件路径
        self.launch_params_file = Path('/home/nyz/sentry/sentry-navigation/src/rm_nav_bringup/config/launch_params.yaml')

        # 当前测试所使用的定位模式（由 _update_launch_params 写入）
        self._current_localization: Optional[str] = None

        # 内存看门狗：避免长测把机器拖到 OOM
        self._abort_requested = threading.Event()
        self._mem_watchdog_thread: Optional[threading.Thread] = None
        self._current_bringup_process: Optional[subprocess.Popen] = None
        
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

                # 兼容 report_generator.py / compare_methods.py 的数据结构：
                # - repeats==1 时保存单个 dict
                # - repeats>1 时保存聚合后的 dict（附带 repeats 列表）
                if repeats <= 1:
                    method_results[scenario_name] = repeats_results[0] if repeats_results else {'error': 'no result'}
                else:
                    method_results[scenario_name] = self._aggregate_repeat_results(method, scenario_name, repeats_results)
                
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

    def _aggregate_repeat_results(self, method: str, scenario_name: str, repeats_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """将多次重复结果聚合成单个结果 dict，便于生成报告/对比分析。"""
        valid = [r for r in repeats_results if isinstance(r, dict) and 'error' not in r]
        aggregated: Dict[str, Any] = {
            'method': method,
            'scenario': scenario_name,
            'repeats': repeats_results,
            'success': any(r.get('success', False) for r in valid) if valid else False,
        }

        def mean(values: List[float]) -> float:
            return sum(values) / len(values) if values else 0.0

        # 聚合 ATE/RPE
        try:
            ate_rmses = []
            rpe_rmses = []
            for r in valid:
                ate = r.get('trajectory_metrics', {}).get('ate', {}).get('rmse', None)
                rpe = r.get('trajectory_metrics', {}).get('rpe', {}).get('rmse', None)
                if isinstance(ate, (int, float)):
                    ate_rmses.append(float(ate))
                if isinstance(rpe, (int, float)):
                    rpe_rmses.append(float(rpe))

            aggregated['trajectory_metrics'] = {
                'ate': {'rmse': mean(ate_rmses)},
                'rpe': {'rmse': mean(rpe_rmses)},
            }
        except Exception:
            aggregated['trajectory_metrics'] = {}

        # 聚合 CPU/MEM（用多次运行的 max 的均值）
        try:
            cpu_maxes = []
            mem_maxes = []
            for r in valid:
                cpu_max = r.get('performance_metrics', {}).get('cpu', {}).get('max', None)
                mem_max = r.get('performance_metrics', {}).get('memory', {}).get('max', None)
                if isinstance(cpu_max, (int, float)):
                    cpu_maxes.append(float(cpu_max))
                if isinstance(mem_max, (int, float)):
                    mem_maxes.append(float(mem_max))

            aggregated['performance_metrics'] = {
                'cpu': {'max': mean(cpu_maxes)},
                'memory': {'max': mean(mem_maxes)},
            }
        except Exception:
            aggregated['performance_metrics'] = {}

        return aggregated
    
    def _run_single_test(self, method: str, scenario_name: str, scenario_config: Dict, repeat_index: int = 1, seed: Optional[int] = None) -> Dict[str, Any]:
        """运行单个测试案例"""
        self.get_logger().info(f"开始测试: {method} - {scenario_name}")
        
        # 1. 启动对应的导航系统
        process = self._launch_navigation_system(method, scenario_name, scenario_config)
        self._current_bringup_process = process
        self._abort_requested.clear()
        
        if not process:
            return {'error': '启动导航系统失败'}
        
        # 等待系统启动并确保 Nav2 Action Server 可用
        self.get_logger().info("等待导航系统启动...")
        time.sleep(5)
        if not self._wait_for_nav2_ready(timeout_sec=scenario_config.get('startup_timeout', 120)):
            self.get_logger().error("Nav2 Action Server 在超时内未就绪，终止本次测试")
            return {'error': 'Nav2 Action服务器不可用/未就绪'}
        
        try:
            # 2. 开始数据收集
            bag_file = self.data_collector.start_recording(method, scenario_name)
            
            if not bag_file:
                return {'error': '启动数据收集失败'}
            
            # 3. 开始性能监控
            self.performance_monitor.start_monitoring()

            # 3.1 启动内存看门狗（尽早终止并清理，避免机器被 OOM 重启）
            self._start_memory_watchdog()
            
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
            self._stop_memory_watchdog()
            self._current_bringup_process = None
    
    def _launch_navigation_system(self, method: str, scenario_name: str, scenario_config: Dict) -> Optional[subprocess.Popen]:
        """启动对应的导航系统"""
        try:
            # Gazebo Classic 默认使用 11345 端口作为 master。
            # 若上一轮未清理干净，会导致新 gzserver 启动直接 exit code 255（bind: Address already in use）。
            # 这里先做一次“尽量温和”的清理，避免评估链路被端口占用卡死。
            self._cleanup_stale_sim_processes()

            # 若仍有进程占用 Gazebo master 端口，直接失败（避免后续长时间等待）
            try:
                import socket
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(0.2)
                    if s.connect_ex(('127.0.0.1', 11345)) == 0:
                        self.get_logger().error(
                            "检测到 Gazebo master 端口 11345 仍被占用（可能有遗留 gzserver 或其它 Gazebo 实例）。"
                        )
                        self.get_logger().error(
                            "请先关闭占用 11345 的进程，或结束其它 Gazebo 会话后再运行评估。"
                        )
                        return None
            except Exception:
                # 端口探测失败不阻断流程
                pass

            # 修改launch_params.yaml以使用指定方法
            self._update_launch_params(method, scenario_config)
            
            # 构建启动命令
            cmd = [
                'bash', '-c', 
                # 评估默认关闭 RViz，避免 rviz 退出触发全局 Shutdown 级联
                'cd /home/nyz/sentry/sentry-navigation && source install/setup.bash && ros2 launch rm_nav_bringup bringup.launch.py nav_rviz:=false'
            ]
            
            # 启动导航系统：输出落盘，避免 PIPE 写满导致启动进程卡死
            ts = int(time.time())
            log_file = self.logs_dir / f"bringup_{method}_{scenario_name}_{ts}.log"
            self._launch_log_handle = open(log_file, 'wb')

            process = subprocess.Popen(
                cmd,
                stdout=self._launch_log_handle,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid  # 创建新的进程组
            )
            
            self.get_logger().info(f"导航系统启动中... PID: {process.pid}")
            self.get_logger().info(f"导航系统日志: {log_file}")
            return process
            
        except Exception as e:
            self.get_logger().error(f"启动导航系统失败: {e}")
            return None

    def _cleanup_stale_sim_processes(self):
        """清理遗留的评估相关进程。

        目标：避免上一轮评估遗留的进程占用端口/资源，导致本轮 Nav2 action server 不就绪或仿真失败。

        注意：仅清理命令行与本工程强相关的进程，避免误杀用户其它会话。
        """

        def _matches_target(name: str, cmdline: str) -> bool:
            if not name and not cmdline:
                return False

            is_bringup_launch = (
                'ros2 launch rm_nav_bringup bringup.launch.py' in cmdline
                or ('rm_nav_bringup' in cmdline and 'bringup.launch.py' in cmdline)
            )

            is_nav2_container = (
                'component_container_mt' in name
                and ('__node:=nav2_container' in cmdline or ' __node:=nav2_container' in cmdline)
            )

            is_gazebo = (
                'gzserver' in name
                or 'gzclient' in name
                or cmdline.startswith('gzserver ')
                or cmdline.startswith('gzclient ')
                or ' gzserver ' in cmdline
                or ' gzclient ' in cmdline
            )

            is_spawn_entity = 'spawn_entity.py' in cmdline and 'gazebo_ros' in cmdline

            if not (is_bringup_launch or is_nav2_container or is_gazebo or is_spawn_entity):
                return False

            # bringup / nav2_container 视为评估“独占资源”，直接清理（同用户下）。
            # 这是为了解决评估过程中断/异常后 nav2_container 脱离进程组、长期残留的问题。
            if is_bringup_launch or is_nav2_container:
                return True

            # 仅清理与本工程相关的实例
            return (
                ('/sentry-navigation/' in cmdline)
                or ('/pb_rm_simulation/' in cmdline)
                or ('pb_rm_simulation' in cmdline)
                or ('rm_nav_bringup' in cmdline)
                or ('src/rm_nav_bringup' in cmdline)
            )

        try:
            current_username = psutil.Process().username()
        except Exception:
            current_username = None

        targets: List[psutil.Process] = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'username']):
            try:
                if current_username is not None and proc.info.get('username') != current_username:
                    continue

                name = (proc.info.get('name') or '').lower()
                cmdline_list = proc.info.get('cmdline') or []
                cmdline = ' '.join(cmdline_list)
                if _matches_target(name, cmdline):
                    targets.append(psutil.Process(proc.info['pid']))
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        if not targets:
            return

        self.get_logger().warning(
            f"检测到遗留评估进程 {len(targets)} 个，尝试清理以释放端口/资源（Gazebo/Nav2/bringup）"
        )

        # 先 SIGINT
        for p in targets:
            try:
                p.send_signal(signal.SIGINT)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        gone, alive = psutil.wait_procs(targets, timeout=3.0)
        if alive:
            # 再 SIGKILL
            for p in alive:
                try:
                    p.kill()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            psutil.wait_procs(alive, timeout=2.0)

    def _wait_for_nav2_ready(self, timeout_sec: float = 120.0) -> bool:
        """等待 Nav2 NavigateToPose action server 就绪"""
        start = time.time()
        last_log = 0.0
        while (time.time() - start) < float(timeout_sec):
            # 若 bringup 已经退出，就没必要继续等
            try:
                proc = self._current_bringup_process
                if proc is not None and proc.poll() is not None:
                    self.get_logger().error(
                        f"导航系统进程已提前退出（rc={proc.returncode}），Nav2 Action Server 不会就绪。"
                    )
                    return False
            except Exception:
                pass

            # 进程事件/发现可能需要 spin
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception:
                pass

            if self.nav_action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("Nav2 Action Server 已就绪: navigate_to_pose")
                return True

            elapsed = time.time() - start
            if elapsed - last_log >= 5.0:
                self.get_logger().info(f"等待 Nav2 Action Server 就绪... {elapsed:.0f}s/{float(timeout_sec):.0f}s")
                last_log = elapsed

        return False
    
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

            self._current_localization = params.get('localization')
            
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
            if self._abort_requested.is_set():
                return False

            waypoints = scenario_config.get('waypoints', [])
            initial_pose = scenario_config.get('initial_pose', [0, 0, 0, 0, 0, 0])
            
            # 设置初始位置
            # 评估默认不再调用 /gazebo/set_entity_state（该服务易阻塞/超时，并可能导致资源堆积）。
            # 依赖仿真 spawn_entity 的默认出生点 + /initialpose 来完成定位初始化。
            if os.environ.get('SENTRY_EVAL_SET_GAZEBO_POSE', '0') == '1':
                self._set_initial_pose(initial_pose)
            time.sleep(2)
            
            # 设置初始位姿估计：对 ICP/Small-GICP 等延迟启动节点，采用持续发布方式提升命中率
            # ICP/Small-GICP 通常需要初始位姿触发配准并开始发布 map->odom
            loc = (self._current_localization or '').lower()
            if loc in ('icp', 'small_gicp'):
                self._publish_initial_pose_for_localization(initial_pose, publish_sec=10.0, publish_hz=2.0)
                if not self._wait_for_transform('map', 'odom', timeout_sec=20.0):
                    self.get_logger().error('定位 TF 未就绪（map <- odom 超时），终止本次测试以避免 goal 被拒绝')
                    return False
            else:
                self._publish_initial_pose_for_localization(initial_pose)
            time.sleep(1)
            
            # 逐个导航到各个航点
            for i, waypoint in enumerate(waypoints):
                if self._abort_requested.is_set():
                    self.get_logger().error('触发内存看门狗：终止测试')
                    return False
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
                yaw = float(pose[5])
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                request = {
                    'state': {
                        'name': 'robot',
                        'pose': {
                            'position': {
                                'x': float(pose[0]),
                                'y': float(pose[1]),
                                'z': float(pose[2]),
                            },
                            'orientation': {
                                'x': 0.0,
                                'y': 0.0,
                                'z': float(qz),
                                'w': float(qw),
                            },
                        },
                    }
                }
                request_yaml = yaml.safe_dump(request, default_flow_style=True).strip()
                # 使用Gazebo服务设置机器人位置
                cmd = [
                    'ros2', 'service', 'call', '/gazebo/set_entity_state',
                    'gazebo_msgs/srv/SetEntityState',
                    request_yaml
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    self.get_logger().info(f"机器人位姿已设置: {pose}")
                else:
                    self.get_logger().warning(f"设置机器人位姿失败: {result.stderr}")
                    
        except Exception as e:
            self.get_logger().error(f"设置初始位姿失败: {e}")
    
    def _publish_initial_pose_for_localization(self, pose: List[float], publish_sec: float = 6.0, publish_hz: float = 2.0):
        """发布 /initialpose 以初始化定位（尤其是 ICP/Small-GICP）。

        说明：ICP/Small-GICP 节点在 bringup 中会延迟启动（TimerAction 7s）。
        为提高命中率，这里在一段时间内重复发布，并更新 stamp。
        """
        try:
            if len(pose) >= 6:
                publish_hz = max(0.5, float(publish_hz))
                publish_sec = max(0.5, float(publish_sec))
                count = int(publish_sec * publish_hz)

                yaw = float(pose[5])
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)

                cov = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

                loc = (self._current_localization or '').lower()
                self.get_logger().info(
                    f"发布 /initialpose 用于定位初始化 (localization={loc or 'unknown'})，持续 {publish_sec:.1f}s @ {publish_hz:.1f}Hz"
                )

                for _ in range(count):
                    msg = PoseWithCovarianceStamped()
                    msg.header.frame_id = 'map'
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.pose.pose.position.x = float(pose[0])
                    msg.pose.pose.position.y = float(pose[1])
                    msg.pose.pose.position.z = float(pose[2])
                    msg.pose.pose.orientation.z = float(qz)
                    msg.pose.pose.orientation.w = float(qw)
                    msg.pose.covariance = cov
                    self.initial_pose_publisher.publish(msg)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    time.sleep(1.0 / publish_hz)

                # ICP/Small-GICP 常见需要一点时间完成首帧配准并开始发布 map->odom
                if loc in ('icp', 'small_gicp'):
                    time.sleep(2.0)
                
        except Exception as e:
            self.get_logger().error(f"设置初始位姿估计失败: {e}")

    def _wait_for_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 10.0) -> bool:
        """等待 TF 变换可用（用于判断定位是否开始工作）。"""
        if self._tf_buffer is None:
            return False

        start = time.time()
        last_log = 0.0
        while (time.time() - start) < float(timeout_sec):
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._tf_buffer.can_transform(
                    target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2)
                ):
                    self.get_logger().info(f"TF 已就绪: {target_frame} <- {source_frame}")
                    return True
            except Exception:
                pass

            elapsed = time.time() - start
            if elapsed - last_log >= 2.0:
                self.get_logger().info(f"等待 TF: {target_frame} <- {source_frame} ... {elapsed:.0f}s/{float(timeout_sec):.0f}s")
                last_log = elapsed

        self.get_logger().warning(f"等待 TF 超时: {target_frame} <- {source_frame}")
        return False
    
    def _navigate_to_waypoint(self, waypoint: List[float], timeout: float = 30.0) -> bool:
        """导航到指定航点"""
        try:
            if self._abort_requested.is_set():
                return False
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
                if self._abort_requested.is_set():
                    return False
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
                if self._abort_requested.is_set():
                    return False
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

    def _start_memory_watchdog(self):
        if self._mem_watchdog_thread and self._mem_watchdog_thread.is_alive():
            return

        def _watch():
            # 阈值：可用内存 < 1GB 或内存占用 > 95% 就触发
            min_available_gb = float(os.environ.get('SENTRY_EVAL_MIN_AVAILABLE_GB', '1.0'))
            max_used_percent = float(os.environ.get('SENTRY_EVAL_MAX_MEM_PERCENT', '95.0'))
            while rclpy.ok() and not self._abort_requested.is_set():
                try:
                    vm = psutil.virtual_memory()
                    available_gb = vm.available / (1024**3)
                    used_percent = float(vm.percent)
                    if available_gb < min_available_gb or used_percent > max_used_percent:
                        self.get_logger().error(
                            f"内存告警：available={available_gb:.2f}GB, used={used_percent:.1f}% -> 触发保护终止"
                        )
                        self._abort_requested.set()

                        # 尽可能快速释放：停止 rosbag + 关闭 bringup
                        try:
                            if self.data_collector and self.data_collector.bag_process:
                                os.killpg(os.getpgid(self.data_collector.bag_process.pid), signal.SIGINT)
                        except Exception:
                            pass

                        try:
                            proc = self._current_bringup_process
                            if proc and proc.poll() is None:
                                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        except Exception:
                            pass
                        return

                except Exception:
                    pass

                time.sleep(1.0)

        self._mem_watchdog_thread = threading.Thread(target=_watch, daemon=True)
        self._mem_watchdog_thread.start()

    def _stop_memory_watchdog(self):
        # 线程是 daemon，靠 abort 标志退出；这里仅尽量触发退出并 join 一下。
        self._abort_requested.set()
        try:
            if self._mem_watchdog_thread and self._mem_watchdog_thread.is_alive():
                self._mem_watchdog_thread.join(timeout=1.0)
        except Exception:
            pass
    
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

            # 兜底清理：防止 Gazebo/SpawnEntity 因异常脱离 launch 进程组而残留
            self._cleanup_stale_sim_processes()

            # 关闭日志句柄
            if self._launch_log_handle is not None:
                try:
                    self._launch_log_handle.flush()
                    self._launch_log_handle.close()
                except Exception:
                    pass
                self._launch_log_handle = None
                    
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
