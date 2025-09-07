#!/usr/bin/env python3
"""
轨迹分析器
使用evo库和自定义方法分析轨迹精度和性能
"""

import numpy as np
import pandas as pd
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional
import subprocess
import tempfile
import json
import math
import sys

class TrajectoryAnalyzer:
    """轨迹分析器"""
    
    def __init__(self):
        self.evo_available = self._check_evo_installation()
        
    def _check_evo_installation(self) -> bool:
        """检查evo工具是否安装"""
        try:
            result = subprocess.run(['evo_ape', '--help'], 
                                  capture_output=True, text=True, timeout=10)
            return result.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            print("警告: evo工具未安装，将使用简化分析方法")
            return False
    
    def install_evo(self) -> bool:
        """尝试安装evo工具"""
        try:
            print("正在安装evo评估工具...")
            subprocess.run([sys.executable, '-m', 'pip', 'install', 'evo', '--upgrade', '--no-binary', 'evo'], 
                         check=True, timeout=300)
            self.evo_available = self._check_evo_installation()
            if self.evo_available:
                print("evo工具安装成功")
            return self.evo_available
        except Exception as e:
            print(f"evo工具安装失败: {e}")
            return False
    
    def analyze_trajectory(self, bag_file: str) -> Dict[str, Any]:
        """分析轨迹数据"""
        
        if not self.evo_available:
            # 尝试安装evo
            if not self.install_evo():
                print("使用简化分析方法...")
                return self._simple_analysis(bag_file)
        
        try:
            # 从bag文件提取轨迹数据
            ground_truth_file, estimated_file = self._extract_trajectories_from_bag(bag_file)
            
            if not ground_truth_file or not estimated_file:
                print("轨迹数据提取失败，使用简化分析")
                return self._simple_analysis(bag_file)
            
            # 使用evo进行分析
            ate_results = self._calculate_ate(ground_truth_file, estimated_file)
            rpe_results = self._calculate_rpe(ground_truth_file, estimated_file)
            
            # 计算额外指标
            smoothness_metrics = self._calculate_trajectory_smoothness(estimated_file)
            drift_analysis = self._analyze_drift(ground_truth_file, estimated_file)
            
            return {
                'ate': ate_results,
                'rpe': rpe_results,
                'smoothness': smoothness_metrics,
                'drift': drift_analysis,
                'files': {
                    'ground_truth': ground_truth_file,
                    'estimated': estimated_file
                },
                'analysis_method': 'evo'
            }
            
        except Exception as e:
            print(f"evo分析失败: {e}，使用简化分析")
            return self._simple_analysis(bag_file)
    
    def _extract_trajectories_from_bag(self, bag_file: str) -> Tuple[Optional[str], Optional[str]]:
        """从ROS bag文件中提取轨迹数据并转换为TUM格式"""
        
        # 首先尝试从JSON文件获取轨迹数据
        json_file = Path(bag_file).with_suffix('.json')
        if json_file.exists():
            return self._extract_from_json(str(json_file))
        
        # 如果没有JSON文件，尝试从bag文件提取
        return self._extract_from_rosbag(bag_file)
    
    def _extract_from_json(self, json_file: str) -> Tuple[Optional[str], Optional[str]]:
        """从JSON文件提取轨迹数据"""
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            gt_poses = data.get('ground_truth_poses', [])
            est_poses = data.get('estimated_poses', [])
            
            if not gt_poses or not est_poses:
                print("JSON文件中没有轨迹数据")
                return None, None
            
            # 创建临时TUM文件
            gt_file = tempfile.NamedTemporaryFile(mode='w', suffix='_gt.tum', delete=False)
            est_file = tempfile.NamedTemporaryFile(mode='w', suffix='_est.tum', delete=False)
            
            # 转换地面真值
            for pose in gt_poses:
                ts = pose['timestamp']
                pos = pose['position']
                ori = pose['orientation']
                gt_file.write(f"{ts} {pos['x']} {pos['y']} {pos['z']} "
                             f"{ori['x']} {ori['y']} {ori['z']} {ori['w']}\n")
            
            # 转换估计轨迹
            for pose in est_poses:
                ts = pose['timestamp']
                pos = pose['position']
                ori = pose['orientation']
                est_file.write(f"{ts} {pos['x']} {pos['y']} {pos['z']} "
                              f"{ori['x']} {ori['y']} {ori['z']} {ori['w']}\n")
            
            gt_file.close()
            est_file.close()
            
            print(f"从JSON提取轨迹数据: GT={len(gt_poses)}, EST={len(est_poses)}")
            return gt_file.name, est_file.name
            
        except Exception as e:
            print(f"从JSON提取轨迹数据失败: {e}")
            return None, None
    
    def _extract_from_rosbag(self, bag_file: str) -> Tuple[Optional[str], Optional[str]]:
        """从ROS bag文件提取轨迹数据"""
        try:
            # 这里需要实现从rosbag读取的代码
            # 由于rosbag2的Python API比较复杂，暂时返回None
            print("从rosbag提取轨迹数据的功能尚未实现")
            return None, None
            
        except Exception as e:
            print(f"从rosbag提取轨迹数据失败: {e}")
            return None, None
    
    def _calculate_ate(self, gt_file: str, est_file: str) -> Dict[str, float]:
        """计算绝对轨迹误差(ATE)"""
        try:
            # 创建临时结果文件
            result_file = tempfile.NamedTemporaryFile(suffix='.zip', delete=False)
            result_file.close()
            
            cmd = [
                'evo_ape', 'tum', gt_file, est_file, 
                '--verbose', '--no_plot', '--no_warnings',
                '--save_results', result_file.name
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            
            if result.returncode != 0:
                print(f"evo_ape执行失败: {result.stderr}")
                return self._fallback_ate_calculation(gt_file, est_file)
            
            # 解析结果
            output_lines = result.stdout.split('\n')
            ate_stats = {}
            
            for line in output_lines:
                line_lower = line.lower()
                if 'rmse' in line_lower and ':' in line:
                    ate_stats['rmse'] = self._extract_number_from_line(line)
                elif 'mean' in line_lower and ':' in line:
                    ate_stats['mean'] = self._extract_number_from_line(line)
                elif 'median' in line_lower and ':' in line:
                    ate_stats['median'] = self._extract_number_from_line(line)
                elif 'std' in line_lower and ':' in line:
                    ate_stats['std'] = self._extract_number_from_line(line)
                elif 'min' in line_lower and ':' in line:
                    ate_stats['min'] = self._extract_number_from_line(line)
                elif 'max' in line_lower and ':' in line:
                    ate_stats['max'] = self._extract_number_from_line(line)
            
            # 清理临时文件
            Path(result_file.name).unlink(missing_ok=True)
            
            return ate_stats
            
        except Exception as e:
            print(f"ATE计算失败: {e}")
            return self._fallback_ate_calculation(gt_file, est_file)
    
    def _calculate_rpe(self, gt_file: str, est_file: str) -> Dict[str, float]:
        """计算相对位姿误差(RPE)"""
        try:
            result_file = tempfile.NamedTemporaryFile(suffix='.zip', delete=False)
            result_file.close()
            
            cmd = [
                'evo_rpe', 'tum', gt_file, est_file, 
                '--delta', '1', '--delta_unit', 's',
                '--verbose', '--no_plot', '--no_warnings',
                '--save_results', result_file.name
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            
            if result.returncode != 0:
                print(f"evo_rpe执行失败: {result.stderr}")
                return self._fallback_rpe_calculation(gt_file, est_file)
            
            # 解析结果
            output_lines = result.stdout.split('\n')
            rpe_stats = {}
            
            for line in output_lines:
                line_lower = line.lower()
                if 'rmse' in line_lower and ':' in line:
                    rpe_stats['rmse'] = self._extract_number_from_line(line)
                elif 'mean' in line_lower and ':' in line:
                    rpe_stats['mean'] = self._extract_number_from_line(line)
                elif 'median' in line_lower and ':' in line:
                    rpe_stats['median'] = self._extract_number_from_line(line)
                elif 'std' in line_lower and ':' in line:
                    rpe_stats['std'] = self._extract_number_from_line(line)
            
            # 清理临时文件
            Path(result_file.name).unlink(missing_ok=True)
            
            return rpe_stats
            
        except Exception as e:
            print(f"RPE计算失败: {e}")
            return self._fallback_rpe_calculation(gt_file, est_file)
    
    def _extract_number_from_line(self, line: str) -> float:
        """从行中提取数字"""
        import re
        numbers = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', line)
        if numbers:
            return float(numbers[-1])  # 通常最后一个数字是我们要的
        return 0.0
    
    def _calculate_trajectory_smoothness(self, traj_file: str) -> Dict[str, float]:
        """计算轨迹平滑度"""
        try:
            # 读取轨迹文件
            data = np.loadtxt(traj_file)
            if len(data.shape) == 1:
                data = data.reshape(1, -1)
                
            if data.shape[0] < 3:
                return {'velocity_std': 0, 'acceleration_std': 0, 'jerk_mean': 0, 'jerk_max': 0}
            
            positions = data[:, 1:4]  # x, y, z
            timestamps = data[:, 0]
            
            # 计算速度和加速度
            dt = np.diff(timestamps)
            dt = np.maximum(dt, 1e-6)  # 避免除零
            
            velocities = np.diff(positions, axis=0) / dt[:, np.newaxis]
            
            if len(velocities) < 2:
                return {'velocity_std': 0, 'acceleration_std': 0, 'jerk_mean': 0, 'jerk_max': 0}
                
            accelerations = np.diff(velocities, axis=0) / dt[1:, np.newaxis]
            
            # 计算平滑度指标
            velocity_magnitude = np.linalg.norm(velocities, axis=1)
            acceleration_magnitude = np.linalg.norm(accelerations, axis=1)
            
            # Jerk (加加速度)
            if len(accelerations) > 1:
                jerks = np.diff(accelerations, axis=0) / dt[2:, np.newaxis]
                jerk_magnitude = np.linalg.norm(jerks, axis=1)
            else:
                jerk_magnitude = np.array([0])
            
            return {
                'velocity_std': float(np.std(velocity_magnitude)),
                'acceleration_std': float(np.std(acceleration_magnitude)),
                'jerk_mean': float(np.mean(jerk_magnitude)),
                'jerk_max': float(np.max(jerk_magnitude))
            }
            
        except Exception as e:
            print(f"平滑度计算失败: {e}")
            return {'velocity_std': 0, 'acceleration_std': 0, 'jerk_mean': 0, 'jerk_max': 0}
    
    def _analyze_drift(self, gt_file: str, est_file: str) -> Dict[str, float]:
        """分析轨迹漂移"""
        try:
            gt_data = np.loadtxt(gt_file)
            est_data = np.loadtxt(est_file)
            
            if len(gt_data.shape) == 1:
                gt_data = gt_data.reshape(1, -1)
            if len(est_data.shape) == 1:
                est_data = est_data.reshape(1, -1)
            
            # 计算起点和终点的误差
            start_error = np.linalg.norm(gt_data[0, 1:4] - est_data[0, 1:4])
            end_error = np.linalg.norm(gt_data[-1, 1:4] - est_data[-1, 1:4])
            
            # 计算漂移率 (米/秒)
            time_duration = gt_data[-1, 0] - gt_data[0, 0]
            drift_rate = (end_error - start_error) / max(time_duration, 1e-6)
            
            return {
                'start_error': float(start_error),
                'end_error': float(end_error),
                'drift_rate': float(drift_rate),
                'total_drift': float(end_error - start_error)
            }
            
        except Exception as e:
            print(f"漂移分析失败: {e}")
            return {'start_error': 0, 'end_error': 0, 'drift_rate': 0, 'total_drift': 0}
    
    def _fallback_ate_calculation(self, gt_file: str, est_file: str) -> Dict[str, float]:
        """备用ATE计算方法"""
        try:
            gt_data = np.loadtxt(gt_file)
            est_data = np.loadtxt(est_file)
            
            if len(gt_data.shape) == 1:
                gt_data = gt_data.reshape(1, -1)
            if len(est_data.shape) == 1:
                est_data = est_data.reshape(1, -1)
            
            # 简单的点对点距离计算
            min_len = min(len(gt_data), len(est_data))
            errors = []
            
            for i in range(min_len):
                error = np.linalg.norm(gt_data[i, 1:4] - est_data[i, 1:4])
                errors.append(error)
            
            errors = np.array(errors)
            
            return {
                'rmse': float(np.sqrt(np.mean(errors**2))),
                'mean': float(np.mean(errors)),
                'median': float(np.median(errors)),
                'std': float(np.std(errors)),
                'min': float(np.min(errors)),
                'max': float(np.max(errors))
            }
            
        except Exception as e:
            print(f"备用ATE计算失败: {e}")
            return {'rmse': float('inf'), 'mean': float('inf'), 'median': float('inf'), 
                   'std': 0, 'min': 0, 'max': float('inf')}
    
    def _fallback_rpe_calculation(self, gt_file: str, est_file: str) -> Dict[str, float]:
        """备用RPE计算方法"""
        try:
            gt_data = np.loadtxt(gt_file)
            est_data = np.loadtxt(est_file)
            
            if len(gt_data.shape) == 1:
                gt_data = gt_data.reshape(1, -1)
            if len(est_data.shape) == 1:
                est_data = est_data.reshape(1, -1)
            
            # 计算相对位移
            min_len = min(len(gt_data), len(est_data))
            relative_errors = []
            
            for i in range(1, min_len):
                gt_rel = gt_data[i, 1:4] - gt_data[i-1, 1:4]
                est_rel = est_data[i, 1:4] - est_data[i-1, 1:4]
                rel_error = np.linalg.norm(gt_rel - est_rel)
                relative_errors.append(rel_error)
            
            relative_errors = np.array(relative_errors)
            
            return {
                'rmse': float(np.sqrt(np.mean(relative_errors**2))),
                'mean': float(np.mean(relative_errors)),
                'median': float(np.median(relative_errors)),
                'std': float(np.std(relative_errors))
            }
            
        except Exception as e:
            print(f"备用RPE计算失败: {e}")
            return {'rmse': float('inf'), 'mean': float('inf'), 'median': float('inf'), 'std': 0}
    
    def _simple_analysis(self, bag_file: str) -> Dict[str, Any]:
        """简化版分析（当evo不可用时）"""
        try:
            # 尝试从JSON文件读取数据进行简单分析
            json_file = Path(bag_file).with_suffix('.json')
            if json_file.exists():
                return self._analyze_from_json(str(json_file))
            
        except Exception as e:
            print(f"简化分析失败: {e}")
        
        return {
            'ate': {'rmse': 0, 'mean': 0, 'median': 0, 'std': 0, 'min': 0, 'max': 0},
            'rpe': {'rmse': 0, 'mean': 0, 'median': 0, 'std': 0},
            'smoothness': {'velocity_std': 0, 'acceleration_std': 0, 'jerk_mean': 0, 'jerk_max': 0},
            'drift': {'start_error': 0, 'end_error': 0, 'drift_rate': 0, 'total_drift': 0},
            'analysis_method': 'simplified',
            'note': 'evo工具不可用，使用简化分析方法'
        }
    
    def _analyze_from_json(self, json_file: str) -> Dict[str, Any]:
        """从JSON文件进行分析"""
        gt_file, est_file = self._extract_from_json(json_file)
        
        if not gt_file or not est_file:
            return self._simple_analysis("")
        
        # 使用备用计算方法
        ate_results = self._fallback_ate_calculation(gt_file, est_file)
        rpe_results = self._fallback_rpe_calculation(gt_file, est_file)
        smoothness_metrics = self._calculate_trajectory_smoothness(est_file)
        drift_analysis = self._analyze_drift(gt_file, est_file)
        
        # 清理临时文件
        Path(gt_file).unlink(missing_ok=True)
        Path(est_file).unlink(missing_ok=True)
        
        return {
            'ate': ate_results,
            'rpe': rpe_results,
            'smoothness': smoothness_metrics,
            'drift': drift_analysis,
            'analysis_method': 'fallback'
        }

def main():
    """测试轨迹分析器"""
    analyzer = TrajectoryAnalyzer()
    
    # 测试分析功能
    test_bag = "/tmp/test_bag"
    result = analyzer.analyze_trajectory(test_bag)
    
    print("轨迹分析结果:")
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    main()
