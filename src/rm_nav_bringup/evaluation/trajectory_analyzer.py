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
import os
import time
import signal
import yaml
import sqlite3

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
            print(f"尝试从rosbag提取轨迹数据: {bag_file}")

            # 优先使用 sqlite 直接解析（不依赖 ros2 bag play + topic echo，且支持 TF-only 估计轨迹）
            gt_file, est_file = self._extract_from_rosbag_sqlite(bag_file)
            if gt_file and est_file:
                return gt_file, est_file

            # 确保 bag_file 指向已经记录的 ros2 bag 目录（或 .db3 文件所在目录）
            # 我们使用 subprocess 调用 `ros2 bag play` 并同时用 `ros2 topic echo -p` 导出消息为 YAML，
            # 之后解析 YAML 并构建 TUM 文件。

            import yaml
            import signal

            tmp_gt = tempfile.NamedTemporaryFile(mode='w', suffix='_gt.tum', delete=False)
            tmp_est = tempfile.NamedTemporaryFile(mode='w', suffix='_est.tum', delete=False)
            tmp_gt.close()
            tmp_est.close()

            # 临时文件保存 topic echo 输出
            tmp_gz = tempfile.NamedTemporaryFile(mode='w', suffix='_model_states.yaml', delete=False)
            tmp_odom = tempfile.NamedTemporaryFile(mode='w', suffix='_odom.yaml', delete=False)
            tmp_gz.close()
            tmp_odom.close()

            # 启动 ros2 bag play
            play_cmd = ['ros2', 'bag', 'play', str(bag_file)]
            play_proc = subprocess.Popen(play_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)

            # 启动 topic echo 进程，输出原始 YAML
            gz_cmd = ['ros2', 'topic', 'echo', '-p', '/gazebo/model_states']
            odom_cmd = ['ros2', 'topic', 'echo', '-p', '/odom']

            gz_proc = subprocess.Popen(gz_cmd, stdout=open(tmp_gz.name, 'w'), stderr=subprocess.PIPE, preexec_fn=os.setsid)
            odom_proc = subprocess.Popen(odom_cmd, stdout=open(tmp_odom.name, 'w'), stderr=subprocess.PIPE, preexec_fn=os.setsid)

            # 等待 play 进程结束（注意：对于大型 bag 这可能比较久）
            try:
                out, err = play_proc.communicate(timeout=3600)
            except subprocess.TimeoutExpired:
                # 超时则终止 play
                os.killpg(os.getpgid(play_proc.pid), signal.SIGTERM)
                play_proc.wait()

            # play 结束后，停止 echo 订阅器
            try:
                if gz_proc.poll() is None:
                    os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
            except Exception:
                pass
            try:
                if odom_proc.poll() is None:
                    os.killpg(os.getpgid(odom_proc.pid), signal.SIGINT)
            except Exception:
                pass

            # 小等待让输出文件写完
            time.sleep(1.0)

            # 解析 model_states YAML 文件
            gt_count = 0
            est_count = 0
            try:
                with open(tmp_gz.name, 'r', encoding='utf-8') as f:
                    # ros2 topic echo -p 会输出多段 YAML/JSON 风格的记录，逐个解析
                    docs = list(yaml.safe_load_all(f))
                    for doc in docs:
                        if not doc:
                            continue
                        # doc 结构可能为 {'name': [...], 'pose': [...] , ...}
                        names = doc.get('name') or doc.get('model_name') or []
                        poses = doc.get('pose') or doc.get('poses') or []
                        if not names or not poses:
                            continue
                        # 找到机器人索引
                        robot_names = ['robot', 'sentry', 'sentry_robot']
                        robot_index = None
                        for rn in robot_names:
                            if rn in names:
                                robot_index = names.index(rn)
                                break
                        if robot_index is None and len(names) > 0:
                            # fallback: assume first
                            robot_index = 0

                        try:
                            pose = poses[robot_index]
                        except Exception:
                            continue

                        # 提取时间戳（如果 doc 包含 header）
                        stamp = None
                        header = doc.get('header')
                        if header and header.get('stamp'):
                            s = header['stamp']
                            sec = s.get('sec') or s.get('secs') or 0
                            nsec = s.get('nanosec') or s.get('nsecs') or s.get('nsec') or 0
                            stamp = float(sec) + float(nsec) * 1e-9

                        if stamp is None:
                            # fallback: use current time
                            stamp = time.time()

                        # pose 里可能是 pose: {position: {x,y,z}, orientation: {x,y,z,w}}
                        pos = pose.get('position', {}) if isinstance(pose, dict) else {}
                        ori = pose.get('orientation', {}) if isinstance(pose, dict) else {}

                        x = pos.get('x', 0)
                        y = pos.get('y', 0)
                        z = pos.get('z', 0)
                        qx = ori.get('x', 0)
                        qy = ori.get('y', 0)
                        qz = ori.get('z', 0)
                        qw = ori.get('w', 1)

                        with open(tmp_gt.name, 'a', encoding='utf-8') as gf:
                            gf.write(f"{stamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
                            gt_count += 1
            except Exception as e:
                print(f"解析 model_states 失败: {e}")

            # 解析 odom YAML 文件
            try:
                with open(tmp_odom.name, 'r', encoding='utf-8') as f:
                    docs = list(yaml.safe_load_all(f))
                    for doc in docs:
                        if not doc:
                            continue
                        # doc 结构通常包含 header and pose
                        header = doc.get('header') or {}
                        stamp = None
                        if header.get('stamp'):
                            s = header['stamp']
                            sec = s.get('sec') or s.get('secs') or 0
                            nsec = s.get('nanosec') or s.get('nsecs') or s.get('nsec') or 0
                            stamp = float(sec) + float(nsec) * 1e-9
                        if stamp is None:
                            stamp = time.time()

                        pose = doc.get('pose', {}).get('pose') if doc.get('pose') else doc.get('pose')
                        if not pose:
                            # some outputs may directly expose position/orientation
                            pose = doc.get('pose', {})

                        pos = {}
                        ori = {}
                        if isinstance(pose, dict):
                            position = pose.get('position') or {}
                            orientation = pose.get('orientation') or {}
                            pos = position
                            ori = orientation

                        x = pos.get('x', 0)
                        y = pos.get('y', 0)
                        z = pos.get('z', 0)
                        qx = ori.get('x', 0)
                        qy = ori.get('y', 0)
                        qz = ori.get('z', 0)
                        qw = ori.get('w', 1)

                        with open(tmp_est.name, 'a', encoding='utf-8') as ef:
                            ef.write(f"{stamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
                            est_count += 1
            except Exception as e:
                print(f"解析 odom 失败: {e}")

            # 清理中间 YAML 文件
            try:
                Path(tmp_gz.name).unlink(missing_ok=True)
                Path(tmp_odom.name).unlink(missing_ok=True)
            except Exception:
                pass

            if gt_count == 0 and est_count == 0:
                print("从rosbag中未提取到轨迹数据")
                # 清理生成的 tum
                Path(tmp_gt.name).unlink(missing_ok=True)
                Path(tmp_est.name).unlink(missing_ok=True)
                return None, None

            print(f"从rosbag提取完成: GT={gt_count}, EST={est_count}")
            return tmp_gt.name, tmp_est.name

        except Exception as e:
            print(f"从rosbag提取轨迹数据失败: {e}")
            return None, None
            
        except Exception as e:
            print(f"从rosbag提取轨迹数据失败: {e}")
            return None, None

    def _extract_from_rosbag_sqlite(self, bag_file: str) -> Tuple[Optional[str], Optional[str]]:
        """直接从 rosbag2 sqlite3 存储中提取轨迹。

        支持：
        - 地面真值：优先 /ground_truth/odom (nav_msgs/msg/Odometry)
        - 估计轨迹：优先 /odom (nav_msgs/msg/Odometry)，否则从 /tf + /tf_static 组合 odom->base_link
        """

        bag_path = Path(bag_file)
        bag_dir = bag_path if bag_path.is_dir() else bag_path.parent

        db3_files = sorted(bag_dir.glob('*.db3'))
        if not db3_files:
            # 如果只有压缩文件，尝试解压到同名 .db3（rosbag2_player 也会这样做，但评估流程未必触发）
            zstd_files = sorted(bag_dir.glob('*.db3.zstd'))
            if zstd_files:
                try:
                    subprocess.run(['zstd', '-d', '-f', str(zstd_files[0])], check=False, timeout=120)
                except Exception:
                    pass
            db3_files = sorted(bag_dir.glob('*.db3'))

        if not db3_files:
            return None, None

        db_path = str(db3_files[0])

        try:
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except Exception as e:
            print(f"无法导入 rclpy/rosidl 以解析 rosbag2 数据: {e}")
            return None, None

        def bag_ts_to_sec(ts_ns: int) -> float:
            return float(ts_ns) * 1e-9

        def normalize_quat(q):
            x, y, z, w = q
            n = math.sqrt(x*x + y*y + z*z + w*w)
            if n <= 1e-12:
                return (0.0, 0.0, 0.0, 1.0)
            return (x/n, y/n, z/n, w/n)

        def quat_mul(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            return (
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2,
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
            )

        def quat_conj(q):
            x, y, z, w = q
            return (-x, -y, -z, w)

        def rotate_vec(q, v):
            # v' = q * (v,0) * q_conj
            vx, vy, vz = v
            qv = (vx, vy, vz, 0.0)
            qc = quat_conj(q)
            r = quat_mul(quat_mul(q, qv), qc)
            return (r[0], r[1], r[2])

        def compose(t1, q1, t2, q2):
            # T = (t1,q1) * (t2,q2)
            q1n = normalize_quat(q1)
            q2n = normalize_quat(q2)
            t2r = rotate_vec(q1n, t2)
            t = (t1[0] + t2r[0], t1[1] + t2r[1], t1[2] + t2r[2])
            q = quat_mul(q1n, q2n)
            return t, normalize_quat(q)

        con = sqlite3.connect(db_path)
        cur = con.cursor()
        cur.execute('select id, name, type from topics')
        topics = {row[0]: (row[1], row[2]) for row in cur.fetchall()}

        # Identify topic IDs
        odom_topics: List[Tuple[int, str]] = []
        tf_topic_id = None
        tf_static_topic_id = None
        for topic_id, (name, typ) in topics.items():
            if typ == 'nav_msgs/msg/Odometry':
                odom_topics.append((topic_id, name))
            elif name == '/tf' and typ == 'tf2_msgs/msg/TFMessage':
                tf_topic_id = topic_id
            elif name == '/tf_static' and typ == 'tf2_msgs/msg/TFMessage':
                tf_static_topic_id = topic_id

        def _pick_gt_odom_topic_id() -> Optional[int]:
            if not odom_topics:
                return None

            def score(name: str) -> int:
                n = name.lower()
                if name == '/ground_truth/odom':
                    return 100
                if 'ground_truth' in n or 'groundtruth' in n or '/gt' in n or 'gt_' in n:
                    return 90
                if name == '/Odometry':
                    return 80
                return 10

            best = max(odom_topics, key=lambda it: (score(it[1]), it[1]))
            return best[0]

        def _pick_est_odom_topic_id() -> Optional[int]:
            for topic_id, name in odom_topics:
                if name == '/odom':
                    return topic_id
            return None

        gt_odom_topic_id = _pick_gt_odom_topic_id()
        est_odom_topic_id = _pick_est_odom_topic_id()

        gt_file_path: Optional[str] = None
        est_file_path: Optional[str] = None

        # Extract GT from /ground_truth/odom
        if gt_odom_topic_id is not None:
            odom_type = get_message('nav_msgs/msg/Odometry')
            tmp_gt = tempfile.NamedTemporaryFile(mode='w', suffix='_gt.tum', delete=False)
            gt_count = 0
            for ts_ns, data in cur.execute('select timestamp, data from messages where topic_id=? order by timestamp', (gt_odom_topic_id,)):
                msg = deserialize_message(data, odom_type)
                ts = bag_ts_to_sec(ts_ns)
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                tmp_gt.write(f"{ts} {p.x} {p.y} {p.z} {o.x} {o.y} {o.z} {o.w}\n")
                gt_count += 1
            tmp_gt.close()
            if gt_count > 0:
                gt_file_path = tmp_gt.name
                print(f"从sqlite提取GT Odometry: {gt_count} 条")
            else:
                try:
                    Path(tmp_gt.name).unlink(missing_ok=True)
                except Exception:
                    pass

        # Extract EST
        if est_odom_topic_id is not None:
            odom_type = get_message('nav_msgs/msg/Odometry')
            tmp_est = tempfile.NamedTemporaryFile(mode='w', suffix='_est.tum', delete=False)
            est_count = 0
            for ts_ns, data in cur.execute('select timestamp, data from messages where topic_id=? order by timestamp', (est_odom_topic_id,)):
                msg = deserialize_message(data, odom_type)
                ts = bag_ts_to_sec(ts_ns)
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                tmp_est.write(f"{ts} {p.x} {p.y} {p.z} {o.x} {o.y} {o.z} {o.w}\n")
                est_count += 1
            tmp_est.close()
            if est_count > 0:
                est_file_path = tmp_est.name
                print(f"从sqlite提取EST Odometry: {est_count} 条")
            else:
                try:
                    Path(tmp_est.name).unlink(missing_ok=True)
                except Exception:
                    pass

        # TF-based estimate fallback
        if est_file_path is None and tf_topic_id is not None and tf_static_topic_id is not None:
            tf_type = get_message('tf2_msgs/msg/TFMessage')

            # static odom->lidar_odom
            static_t = (0.0, 0.0, 0.0)
            static_q = (0.0, 0.0, 0.0, 1.0)
            for ts_ns, data in cur.execute('select timestamp, data from messages where topic_id=?', (tf_static_topic_id,)):
                msg = deserialize_message(data, tf_type)
                for tr in msg.transforms:
                    if tr.header.frame_id == 'odom' and tr.child_frame_id == 'lidar_odom':
                        t = tr.transform.translation
                        r = tr.transform.rotation
                        static_t = (t.x, t.y, t.z)
                        static_q = (r.x, r.y, r.z, r.w)

            # dynamic lidar_odom->base_link
            dyn = []
            for ts_ns, data in cur.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tf_topic_id,)):
                msg = deserialize_message(data, tf_type)
                for tr in msg.transforms:
                    if tr.header.frame_id == 'lidar_odom' and tr.child_frame_id == 'base_link':
                        ts = bag_ts_to_sec(ts_ns)
                        t = tr.transform.translation
                        r = tr.transform.rotation
                        dyn.append((ts, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)))

            if dyn:
                tmp_est = tempfile.NamedTemporaryFile(mode='w', suffix='_est.tum', delete=False)
                for ts, t_lidar_base, q_lidar_base in dyn:
                    t_odom_base, q_odom_base = compose(static_t, static_q, t_lidar_base, q_lidar_base)
                    tmp_est.write(
                        f"{ts} {t_odom_base[0]} {t_odom_base[1]} {t_odom_base[2]} "
                        f"{q_odom_base[0]} {q_odom_base[1]} {q_odom_base[2]} {q_odom_base[3]}\n"
                    )
                tmp_est.close()
                est_file_path = tmp_est.name
                print(f"从sqlite提取EST TF(odom->base_link): {len(dyn)} 条")

        con.close()

        if not gt_file_path or not est_file_path:
            # 如果缺任何一个，直接返回失败（调用方会退回到旧方案或简化分析）
            try:
                if gt_file_path:
                    Path(gt_file_path).unlink(missing_ok=True)
                if est_file_path:
                    Path(est_file_path).unlink(missing_ok=True)
            except Exception:
                pass
            return None, None

        return gt_file_path, est_file_path
    
    def _calculate_ate(self, gt_file: str, est_file: str) -> Dict[str, float]:
        """计算绝对轨迹误差(ATE)"""
        try:
            # 创建临时结果文件
            result_file = tempfile.NamedTemporaryFile(suffix='.zip', delete=False)
            result_file.close()

            # 保存 stdout/stderr 的临时文件，便于调试
            stdout_file = tempfile.NamedTemporaryFile(mode='w', suffix='_evo_ape_stdout.txt', delete=False)
            stderr_file = tempfile.NamedTemporaryFile(mode='w', suffix='_evo_ape_stderr.txt', delete=False)
            stdout_file.close()
            stderr_file.close()

            cmd = [
                'evo_ape', 'tum', gt_file, est_file,
                '-v', '--no_warnings',
                '--save_results', result_file.name
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)

            # 将输出写入文件
            try:
                with open(stdout_file.name, 'w', encoding='utf-8') as sf:
                    sf.write(result.stdout or '')
                with open(stderr_file.name, 'w', encoding='utf-8') as ef:
                    ef.write(result.stderr or '')
            except Exception:
                pass

            if result.returncode != 0:
                print(f"evo_ape执行失败，请检查: {stderr_file.name}")
                # 保留 result_file 以便调试
                return self._fallback_ate_calculation(gt_file, est_file)

            # 解析结果（从 stdout）
            output_lines = result.stdout.split('\n')
            ate_stats = {}

            for line in output_lines:
                line_lower = line.lower().strip()
                # evo 不同版本可能输出：
                #   "rmse: 0.123"  或  "rmse      0.123"
                if line_lower.startswith('rmse'):
                    ate_stats['rmse'] = self._extract_number_from_line(line)
                elif line_lower.startswith('mean'):
                    ate_stats['mean'] = self._extract_number_from_line(line)
                elif line_lower.startswith('median'):
                    ate_stats['median'] = self._extract_number_from_line(line)
                elif line_lower.startswith('std'):
                    ate_stats['std'] = self._extract_number_from_line(line)
                elif line_lower.startswith('min'):
                    ate_stats['min'] = self._extract_number_from_line(line)
                elif line_lower.startswith('max'):
                    ate_stats['max'] = self._extract_number_from_line(line)

            # 返回并包含生成的文件路径以便追踪
            ate_stats['evo_result_file'] = result_file.name
            ate_stats['evo_stdout'] = stdout_file.name
            ate_stats['evo_stderr'] = stderr_file.name

            return ate_stats
            
        except Exception as e:
            print(f"ATE计算失败: {e}")
            return self._fallback_ate_calculation(gt_file, est_file)
    
    def _calculate_rpe(self, gt_file: str, est_file: str) -> Dict[str, float]:
        """计算相对位姿误差(RPE)"""
        try:
            result_file = tempfile.NamedTemporaryFile(suffix='.zip', delete=False)
            result_file.close()

            stdout_file = tempfile.NamedTemporaryFile(mode='w', suffix='_evo_rpe_stdout.txt', delete=False)
            stderr_file = tempfile.NamedTemporaryFile(mode='w', suffix='_evo_rpe_stderr.txt', delete=False)
            stdout_file.close()
            stderr_file.close()

            cmd = [
                'evo_rpe', 'tum', gt_file, est_file,
                '--delta', '1', '--delta_unit', 'f',
                '-v', '--no_warnings',
                '--save_results', result_file.name
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)

            try:
                with open(stdout_file.name, 'w', encoding='utf-8') as sf:
                    sf.write(result.stdout or '')
                with open(stderr_file.name, 'w', encoding='utf-8') as ef:
                    ef.write(result.stderr or '')
            except Exception:
                pass

            if result.returncode != 0:
                print(f"evo_rpe执行失败，请检查: {stderr_file.name}")
                return self._fallback_rpe_calculation(gt_file, est_file)

            # 解析结果
            output_lines = result.stdout.split('\n')
            rpe_stats = {}

            for line in output_lines:
                line_lower = line.lower().strip()
                if line_lower.startswith('rmse'):
                    rpe_stats['rmse'] = self._extract_number_from_line(line)
                elif line_lower.startswith('mean'):
                    rpe_stats['mean'] = self._extract_number_from_line(line)
                elif line_lower.startswith('median'):
                    rpe_stats['median'] = self._extract_number_from_line(line)
                elif line_lower.startswith('std'):
                    rpe_stats['std'] = self._extract_number_from_line(line)

            rpe_stats['evo_result_file'] = result_file.name
            rpe_stats['evo_stdout'] = stdout_file.name
            rpe_stats['evo_stderr'] = stderr_file.name

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
