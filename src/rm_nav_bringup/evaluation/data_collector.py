#!/usr/bin/env python3
"""
数据收集器
负责记录测试过程中的ROS数据包和地面真值
"""

import os
import subprocess
import time
import signal
from pathlib import Path
from typing import Optional, List, Dict, Any
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, Imu
import yaml
import json

class DataCollector(Node):
    """数据收集器"""
    
    def __init__(self):
        super().__init__('data_collector')
        
        # 数据存储路径
        self.output_dir = Path.home() / 'sentry_evaluation_data'
        self.output_dir.mkdir(exist_ok=True)
        
        # ROS bag 相关
        self.bag_process = None
        self.current_bag_file = None
        
        # 地面真值和估计轨迹数据
        self.ground_truth_poses = []
        self.estimated_poses = []
        self.recording = False
        
        # 订阅器
        self.model_states_sub = None
        self.odom_sub = None
        
        self.get_logger().info("数据收集器初始化完成")
    
    def start_recording(self, method: str, scenario: str) -> str:
        """开始记录数据"""
        timestamp = int(time.time())
        bag_name = f"{method}_{scenario}_{timestamp}"
        bag_path = self.output_dir / bag_name
        
        self.get_logger().info(f"开始数据收集: {bag_name}")
        
        # 要记录的话题列表
        topics_to_record = [
            '/gazebo/model_states',     # 地面真值
            '/odom',                    # 里程计输出
            '/tf',                      # TF变换
            '/tf_static',               # 静态TF变换
            '/livox/lidar',             # 点云数据
            '/livox/imu',               # IMU数据
            '/scan',                    # 激光扫描
            '/map',                     # 地图
            '/goal_pose',               # 目标位姿
            '/local_plan',              # 局部路径
            '/global_plan',             # 全局路径
            '/cmd_vel',                 # 速度命令
            '/joint_states',            # 关节状态
        ]
        
        # 构建ros2 bag record命令
        cmd = ['ros2', 'bag', 'record'] + topics_to_record + ['-o', str(bag_path)]
        
        try:
            # 启动录制进程
            self.bag_process = subprocess.Popen(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 创建新的进程组
            )
            self.current_bag_file = str(bag_path)
            
            self.get_logger().info(f"开始录制数据包: {bag_path}")
            
            # 清空之前的数据
            self.ground_truth_poses.clear()
            self.estimated_poses.clear()
            self.recording = True
            
            # 启动ROS订阅器
            self._start_ros_subscribers()
            
            # 等待bag recording开始
            time.sleep(2)
            
            return self.current_bag_file
            
        except Exception as e:
            self.get_logger().error(f"启动数据录制失败: {e}")
            return None
    
    def stop_recording(self):
        """停止记录数据"""
        self.recording = False
        
        # 停止ROS订阅器
        self._stop_ros_subscribers()
        
        if self.bag_process:
            try:
                # 发送SIGINT信号给整个进程组
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
                
                # 等待进程正常退出
                self.bag_process.wait(timeout=10)
                self.get_logger().info(f"数据录制正常停止: {self.current_bag_file}")
                
            except subprocess.TimeoutExpired:
                # 如果超时，强制杀死进程
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)
                self.get_logger().warning("数据录制进程被强制停止")
                
            except Exception as e:
                self.get_logger().error(f"停止数据录制时发生错误: {e}")
                
            finally:
                self.bag_process = None
        
        # 保存收集到的轨迹数据
        if self.current_bag_file and (self.ground_truth_poses or self.estimated_poses):
            self._save_trajectory_data()
    
    def _start_ros_subscribers(self):
        """启动ROS订阅器"""
        self.model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', 
            self.model_states_callback, 10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10
        )
        
        self.get_logger().info("ROS订阅器已启动")
    
    def _stop_ros_subscribers(self):
        """停止ROS订阅器"""
        if self.model_states_sub:
            self.destroy_subscription(self.model_states_sub)
            self.model_states_sub = None
            
        if self.odom_sub:
            self.destroy_subscription(self.odom_sub)
            self.odom_sub = None
            
        self.get_logger().info("ROS订阅器已停止")
    
    def model_states_callback(self, msg: ModelStates):
        """Gazebo模型状态回调 - 获取地面真值"""
        if not self.recording:
            return
            
        try:
            # 查找机器人模型（可能的名称）
            robot_names = ['robot', 'sentry', 'sentry_robot']
            robot_index = None
            
            for name in robot_names:
                try:
                    robot_index = msg.name.index(name)
                    break
                except ValueError:
                    continue
            
            if robot_index is not None:
                robot_pose = msg.pose[robot_index]
                
                pose_data = {
                    'timestamp': self.get_clock().now().nanoseconds / 1e9,
                    'position': {
                        'x': robot_pose.position.x,
                        'y': robot_pose.position.y,
                        'z': robot_pose.position.z
                    },
                    'orientation': {
                        'x': robot_pose.orientation.x,
                        'y': robot_pose.orientation.y,
                        'z': robot_pose.orientation.z,
                        'w': robot_pose.orientation.w
                    }
                }
                
                self.ground_truth_poses.append(pose_data)
                
        except Exception as e:
            self.get_logger().debug(f"获取地面真值时发生错误: {e}")
    
    def odom_callback(self, msg: Odometry):
        """里程计回调 - 获取估计位姿"""
        if not self.recording:
            return
            
        try:
            pose_data = {
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            }
            
            self.estimated_poses.append(pose_data)
            
        except Exception as e:
            self.get_logger().debug(f"获取估计位姿时发生错误: {e}")
    
    def _save_trajectory_data(self):
        """保存轨迹数据到JSON文件"""
        try:
            trajectory_file = Path(self.current_bag_file).with_suffix('.json')
            
            data = {
                'ground_truth_poses': self.ground_truth_poses,
                'estimated_poses': self.estimated_poses,
                'collection_info': {
                    'start_time': self.ground_truth_poses[0]['timestamp'] if self.ground_truth_poses else 0,
                    'end_time': self.ground_truth_poses[-1]['timestamp'] if self.ground_truth_poses else 0,
                    'gt_count': len(self.ground_truth_poses),
                    'est_count': len(self.estimated_poses)
                }
            }
            
            with open(trajectory_file, 'w') as f:
                json.dump(data, f, indent=2)
                
            self.get_logger().info(f"轨迹数据已保存: {trajectory_file}")
            
        except Exception as e:
            self.get_logger().error(f"保存轨迹数据失败: {e}")
    
    def get_collected_data(self) -> dict:
        """获取收集到的数据"""
        return {
            'ground_truth_poses': self.ground_truth_poses,
            'estimated_poses': self.estimated_poses,
            'bag_file': self.current_bag_file
        }

def main():
    """测试数据收集器"""
    rclpy.init()
    
    collector = DataCollector()
    
    try:
        # 开始录制
        bag_file = collector.start_recording("test_method", "test_scenario")
        
        if bag_file:
            print(f"数据录制已开始: {bag_file}")
            print("按 Ctrl+C 停止录制...")
            
            rclpy.spin(collector)
        else:
            print("启动数据录制失败")
            
    except KeyboardInterrupt:
        print("用户中断录制")
        
    finally:
        collector.stop_recording()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
