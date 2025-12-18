#!/usr/bin/env python3
"""
性能监控器
监控CPU、内存使用和处理延迟
"""

import psutil
import time
import threading
from typing import Dict, List, Any, Optional
import subprocess
import re
import json
from pathlib import Path

class PerformanceMonitor:
    """性能监控器"""
    
    def __init__(self):
        self.monitoring = False
        self.monitor_thread = None
        self.cpu_data = []
        self.memory_data = []
        self.latency_data = []
        self.node_processes = {}
        self.system_info = self._get_system_info()
        
        # 监控数据上限保护 (防止长时间运行内存堆积)
        self.MAX_SAMPLES = int(os.environ.get('SENTRY_EVAL_MAX_SAMPLES', '1000'))
        
    def _get_system_info(self) -> Dict[str, Any]:
        """获取系统信息"""
        try:
            return {
                'cpu_count': psutil.cpu_count(),
                'cpu_freq': psutil.cpu_freq()._asdict() if psutil.cpu_freq() else {},
                'memory_total_gb': psutil.virtual_memory().total / (1024**3),
                'platform': psutil.Platform if hasattr(psutil, 'Platform') else 'unknown'
            }
        except Exception:
            return {'cpu_count': 1, 'cpu_freq': {}, 'memory_total_gb': 1, 'platform': 'unknown'}
    
    def start_monitoring(self):
        """开始性能监控"""
        self.monitoring = True
        self.cpu_data.clear()
        self.memory_data.clear()
        self.latency_data.clear()
        
        # 查找相关的ROS节点进程
        self._find_ros_processes()
        
        # 启动监控线程
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        print("开始性能监控...")
    
    def stop_monitoring(self) -> Dict[str, Any]:
        """停止性能监控并返回结果"""
        self.monitoring = False
        
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        
        print("停止性能监控...")
        
        return self._analyze_performance_data()
    
    def _find_ros_processes(self):
        """查找相关的ROS节点进程"""
        # 要监控的关键节点
        target_nodes = [
            'fast_lio',
            'point_lio', 
            'slam_toolbox',
            'icp_registration',
            'amcl',
            'gazebo',
            'gzserver',
            'gzclient',
            'nav2',
            'pointcloud_to_laserscan',
            'static_transform_publisher'
        ]
        
        self.node_processes = {}
        
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    proc_name = proc.info['name'] or ''
                    
                    for node in target_nodes:
                        if (node in cmdline or node in proc_name) and node not in self.node_processes:
                            self.node_processes[node] = {
                                'pid': proc.info['pid'],
                                'name': proc_name,
                                'cmdline': cmdline[:100]  # 截断命令行避免过长
                            }
                            print(f"找到节点进程: {node} (PID: {proc.info['pid']})")
                            break
                            
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue
                    
        except Exception as e:
            print(f"查找ROS进程时发生错误: {e}")
    
    def _monitor_loop(self):
        """监控循环"""
        while self.monitoring:
            try:
                timestamp = time.time()
                
                # 监控系统级CPU和内存
                cpu_percent = psutil.cpu_percent(interval=0.1)
                memory_info = psutil.virtual_memory()
                
                # 监控各个节点的资源使用
                node_stats = {}
                for node_name, proc_info in self.node_processes.items():
                    try:
                        proc = psutil.Process(proc_info['pid'])
                        if proc.is_running():
                            node_stats[node_name] = {
                                'cpu_percent': proc.cpu_percent(),
                                'memory_mb': proc.memory_info().rss / (1024 * 1024),
                                'memory_percent': proc.memory_percent(),
                                'threads': proc.num_threads(),
                                'status': proc.status()
                            }
                        else:
                            node_stats[node_name] = None
                    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                        node_stats[node_name] = None
                
                # 记录系统级数据 (限制列表大小防止内存堆积)
                if len(self.cpu_data) >= self.MAX_SAMPLES:
                    self.cpu_data.pop(0)
                self.cpu_data.append({
                    'timestamp': timestamp,
                    'total_cpu': cpu_percent,
                    'cpu_per_core': psutil.cpu_percent(percpu=True),
                    'nodes': {k: v['cpu_percent'] if v else 0 for k, v in node_stats.items()}
                })
                
                if len(self.memory_data) >= self.MAX_SAMPLES:
                    self.memory_data.pop(0)
                self.memory_data.append({
                    'timestamp': timestamp,
                    'total_memory_percent': memory_info.percent,
                    'total_memory_gb': memory_info.used / (1024**3),
                    'available_memory_gb': memory_info.available / (1024**3),
                    'nodes': {k: v['memory_mb'] if v else 0 for k, v in node_stats.items()}
                })
                
                # 监控网络延迟和话题频率
                latency_info = self._measure_topic_metrics()
                if latency_info:
                    if len(self.latency_data) >= self.MAX_SAMPLES:
                        self.latency_data.pop(0)
                    self.latency_data.append({
                        'timestamp': timestamp,
                        **latency_info
                    })
                
                time.sleep(1.0)  # 每秒监控一次
                
            except Exception as e:
                print(f"监控循环中发生错误: {e}")
                time.sleep(1.0)
    
    def _measure_topic_metrics(self) -> Optional[Dict[str, float]]:
        """测量话题相关指标"""
        try:
            metrics = {}
            
            # 监控重要话题的频率
            important_topics = ['/odom', '/livox/lidar', '/scan', '/tf']
            
            for topic in important_topics:
                try:
                    # 使用timeout防止hang住
                    cmd = ['timeout', '3', 'ros2', 'topic', 'hz', topic, '--window', '5']
                    result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
                    
                    if result.returncode == 0:
                        # 解析输出获取频率信息
                        for line in result.stdout.split('\n'):
                            if 'average rate' in line.lower():
                                match = re.search(r'(\d+\.?\d*)', line)
                                if match:
                                    freq = float(match.group(1))
                                    topic_name = topic.replace('/', '_').replace('__', '_').strip('_')
                                    metrics[f'{topic_name}_hz'] = freq
                                    break
                                    
                except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
                    continue
            
            # 计算整体延迟指标
            if metrics:
                odom_hz = metrics.get('odom_hz', 0)
                lidar_hz = metrics.get('livox_lidar_hz', 0)
                
                if odom_hz > 0:
                    metrics['odom_latency_ms'] = 1000.0 / odom_hz
                if lidar_hz > 0:
                    metrics['lidar_latency_ms'] = 1000.0 / lidar_hz
            
            return metrics if metrics else None
            
        except Exception as e:
            # print(f"话题指标测量失败: {e}")
            return None
    
    def _analyze_performance_data(self) -> Dict[str, Any]:
        """分析性能数据"""
        if not self.cpu_data or not self.memory_data:
            return {
                'cpu': {'max': 0, 'mean': 0, 'std': 0},
                'memory': {'max': 0, 'mean': 0},
                'latency': {'mean': 0, 'max': 0},
                'nodes': {},
                'system_info': self.system_info,
                'duration': 0,
                'summary': {
                    'overall_score': 0,
                    'cpu_score': 0,
                    'memory_score': 0,
                    'stability_score': 0
                }
            }
        
        try:
            # CPU分析
            total_cpu_values = [d['total_cpu'] for d in self.cpu_data]
            cpu_stats = {
                'max': float(max(total_cpu_values)),
                'mean': float(sum(total_cpu_values) / len(total_cpu_values)),
                'std': float(self._calculate_std(total_cpu_values)),
                'min': float(min(total_cpu_values))
            }
            
            # 内存分析
            total_memory_values = [d['total_memory_gb'] for d in self.memory_data]
            memory_stats = {
                'max': float(max(total_memory_values)),
                'mean': float(sum(total_memory_values) / len(total_memory_values)),
                'std': float(self._calculate_std(total_memory_values)),
                'min': float(min(total_memory_values))
            }
            
            # 延迟分析
            latency_stats = {'mean': 0, 'max': 0, 'odom_hz_mean': 0, 'lidar_hz_mean': 0}
            if self.latency_data:
                odom_latencies = [d.get('odom_latency_ms', 0) for d in self.latency_data if d.get('odom_latency_ms', 0) > 0]
                odom_freqs = [d.get('odom_hz', 0) for d in self.latency_data if d.get('odom_hz', 0) > 0]
                lidar_freqs = [d.get('livox_lidar_hz', 0) for d in self.latency_data if d.get('livox_lidar_hz', 0) > 0]
                
                if odom_latencies:
                    latency_stats['mean'] = float(sum(odom_latencies) / len(odom_latencies))
                    latency_stats['max'] = float(max(odom_latencies))
                if odom_freqs:
                    latency_stats['odom_hz_mean'] = float(sum(odom_freqs) / len(odom_freqs))
                if lidar_freqs:
                    latency_stats['lidar_hz_mean'] = float(sum(lidar_freqs) / len(lidar_freqs))
            
            # 各节点分析
            node_stats = {}
            for node_name in self.node_processes.keys():
                node_cpu_values = [d['nodes'].get(node_name, 0) for d in self.cpu_data]
                node_mem_values = [d['nodes'].get(node_name, 0) for d in self.memory_data]
                
                # 过滤掉0值（进程不存在时）
                node_cpu_values = [v for v in node_cpu_values if v > 0]
                node_mem_values = [v for v in node_mem_values if v > 0]
                
                if node_cpu_values or node_mem_values:
                    node_stats[node_name] = {
                        'cpu_max': float(max(node_cpu_values)) if node_cpu_values else 0,
                        'cpu_mean': float(sum(node_cpu_values) / len(node_cpu_values)) if node_cpu_values else 0,
                        'memory_max': float(max(node_mem_values)) if node_mem_values else 0,
                        'memory_mean': float(sum(node_mem_values) / len(node_mem_values)) if node_mem_values else 0,
                        'active_samples': len(node_cpu_values)
                    }
            
            # 计算总体测试时长
            duration = self.cpu_data[-1]['timestamp'] - self.cpu_data[0]['timestamp'] if self.cpu_data else 0
            
            # 计算性能评分
            summary = self._calculate_performance_scores(cpu_stats, memory_stats, latency_stats, duration)
            
            return {
                'cpu': cpu_stats,
                'memory': memory_stats,
                'latency': latency_stats,
                'nodes': node_stats,
                'system_info': self.system_info,
                'duration': float(duration),
                'summary': summary,
                'data_points': len(self.cpu_data)
            }
            
        except Exception as e:
            print(f"性能数据分析失败: {e}")
            return {
                'error': str(e),
                'cpu': {'max': 0, 'mean': 0, 'std': 0},
                'memory': {'max': 0, 'mean': 0},
                'latency': {'mean': 0, 'max': 0},
                'nodes': {},
                'system_info': self.system_info,
                'duration': 0
            }
    
    def _calculate_std(self, values: List[float]) -> float:
        """计算标准差"""
        if not values or len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
        return variance ** 0.5
    
    def _calculate_performance_scores(self, cpu_stats: Dict, memory_stats: Dict, 
                                    latency_stats: Dict, duration: float) -> Dict[str, float]:
        """计算性能评分"""
        try:
            # CPU评分 (0-100, 越低越好转换为越高越好)
            cpu_usage_penalty = min(cpu_stats['mean'] / 100.0, 1.0)
            cpu_score = max(0, 100 - cpu_usage_penalty * 50)
            
            # 内存评分
            memory_usage_gb = memory_stats['mean']
            memory_penalty = min(memory_usage_gb / 4.0, 1.0)  # 假设4GB为合理上限
            memory_score = max(0, 100 - memory_penalty * 50)
            
            # 稳定性评分（基于标准差，越小越稳定）
            cpu_stability = max(0, 100 - cpu_stats['std'])
            memory_stability = max(0, 100 - memory_stats['std'] * 10)
            stability_score = (cpu_stability + memory_stability) / 2
            
            # 整体评分
            overall_score = (cpu_score * 0.4 + memory_score * 0.3 + stability_score * 0.3)
            
            return {
                'overall_score': float(overall_score),
                'cpu_score': float(cpu_score),
                'memory_score': float(memory_score),
                'stability_score': float(stability_score)
            }
            
        except Exception as e:
            print(f"性能评分计算失败: {e}")
            return {
                'overall_score': 0,
                'cpu_score': 0,
                'memory_score': 0,
                'stability_score': 0
            }

def main():
    """测试性能监控器"""
    monitor = PerformanceMonitor()
    
    try:
        print("开始性能监控测试...")
        monitor.start_monitoring()
        
        # 运行10秒测试
        time.sleep(10)
        
        results = monitor.stop_monitoring()
        
        print("\n性能监控结果:")
        print(json.dumps(results, indent=2))
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
        results = monitor.stop_monitoring()
        print("监控结果:", results)

if __name__ == '__main__':
    main()
