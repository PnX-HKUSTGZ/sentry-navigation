"""
哨兵导航系统评估模块
提供自动化的导航算法性能评估和对比功能
"""

__version__ = "1.0.0"
__author__ = "Sentry Navigation Team"

from .benchmark_runner import BenchmarkRunner
from .data_collector import DataCollector
from .trajectory_analyzer import TrajectoryAnalyzer
from .performance_monitor import PerformanceMonitor
from .report_generator import ReportGenerator

__all__ = [
    'BenchmarkRunner',
    'DataCollector', 
    'TrajectoryAnalyzer',
    'PerformanceMonitor',
    'ReportGenerator'
]
