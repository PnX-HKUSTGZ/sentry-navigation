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
    
    def __init__(self, output_dir: Optional[str] = None):
        super().__init__('data_collector')
        
        # 数据存储路径
        if output_dir:
            self.output_dir = Path(output_dir)
        else:
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

    def _ros_time_to_sec(self, stamp) -> float:
        """将 ROS 时间转换为秒（支持不同字段命名）"""
        try:
            # rclpy Header.stamp has sec & nanosec
            sec = getattr(stamp, 'sec', None)
            nanosec = getattr(stamp, 'nanosec', None)
            if sec is not None and nanosec is not None:
                return float(sec) + float(nanosec) * 1e-9

            # fallback for builtin time type with seconds+nsecs
            sec = getattr(stamp, 'sec', None)
            nsec = getattr(stamp, 'nsec', None)
            if sec is not None and nsec is not None:
                return float(sec) + float(nsec) * 1e-9

        except Exception:
            pass

        # 最后退回到节点时钟
        return self.get_clock().now().nanoseconds / 1e9
    
    def start_recording(self, method: str, scenario: str) -> str:
        """开始记录数据"""
        timestamp = int(time.time())
        bag_name = f"{method}_{scenario}_{timestamp}"
        bag_path = self.output_dir / bag_name
        
        self.get_logger().info(f"开始数据收集: {bag_name}")
        
        # 要记录的话题列表
        # 默认只记录轻量话题，避免 rosbag2 缓存 + 大体量点云导致内存飙升（甚至触发 OOM 重启）。
        topics_to_record = [
            '/gazebo/model_states',     # 地面真值
            '/odom',                    # 里程计输出
            '/tf',                      # TF变换
            '/tf_static',               # 静态TF变换
            '/goal_pose',               # 目标位姿
            '/cmd_vel',                 # 速度命令
        ]

        # 可选：重话题（点云/IMU/地图/scan 等）。需要时显式开启：SENTRY_EVAL_RECORD_HEAVY_TOPICS=1
        if os.environ.get('SENTRY_EVAL_RECORD_HEAVY_TOPICS', '0') == '1':
            topics_to_record += [
                '/livox/lidar/pointcloud',
                '/livox/imu',
                '/scan',
                '/map',
                '/local_plan',
                '/global_plan',
                '/joint_states',
            ]
        
        # 构建 ros2 bag record 命令
        # 注：不使用 PIPE，避免输出无人消费时导致阻塞/资源堆积。
        cmd = ['ros2', 'bag', 'record'] + topics_to_record + ['-o', str(bag_path)]
        
        try:
            # 启动录制进程
            self.bag_process = subprocess.Popen(
                cmd, 
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
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

                # 如果消息包含 header，优先使用消息的时间戳
                timestamp = None
                try:
                    header = getattr(msg, 'header', None)
                    if header is not None:
                        timestamp = self._ros_time_to_sec(header.stamp)
                except Exception:
                    timestamp = None

                if timestamp is None:
                    timestamp = self.get_clock().now().nanoseconds / 1e9

                pose_data = {
                    'timestamp': float(timestamp),
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
            # 优先使用消息自带的时间戳
            try:
                header = getattr(msg, 'header', None)
                if header is not None:
                    timestamp = self._ros_time_to_sec(header.stamp)
                else:
                    timestamp = self.get_clock().now().nanoseconds / 1e9
            except Exception:
                timestamp = self.get_clock().now().nanoseconds / 1e9

            pose_data = {
                'timestamp': float(timestamp),
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
            # 确保按照时间戳排序并去除可能的 NaN
            def _clean_and_sort(poses):
                cleaned = [p for p in poses if p and isinstance(p.get('timestamp', None), (int, float))]
                return sorted(cleaned, key=lambda x: x['timestamp'])

            gt_sorted = _clean_and_sort(self.ground_truth_poses)
            est_sorted = _clean_and_sort(self.estimated_poses)

            data = {
                'ground_truth_poses': gt_sorted,
                'estimated_poses': est_sorted,
                'collection_info': {
                    'start_time': gt_sorted[0]['timestamp'] if gt_sorted else 0,
                    'end_time': gt_sorted[-1]['timestamp'] if gt_sorted else 0,
                    'gt_count': len(gt_sorted),
                    'est_count': len(est_sorted)
                }
            }
            
            with open(trajectory_file, 'w') as f:
                json.dump(data, f, indent=2)
                
            self.get_logger().info(f"轨迹数据已保存: {trajectory_file}")

        except Exception as e:
            self.get_logger().error(f"保存轨迹数据失败: {e}")

    def force_save(self):
        """强制保存当前收集到的轨迹数据（即使未处于 recording 状态）"""
        try:
            if self.current_bag_file and (self.ground_truth_poses or self.estimated_poses):
                self._save_trajectory_data()
                self.get_logger().info("强制保存轨迹数据完成")
            else:
                self.get_logger().info("没有足够的数据可保存")
        except Exception as e:
            self.get_logger().error(f"force_save 失败: {e}")
    
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
