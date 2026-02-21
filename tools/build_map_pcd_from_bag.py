#!/usr/bin/env python3
"""Build a map PCD from rosbag pointcloud + TF replay.

This tool replays a bag and accumulates `/livox/lidar/pointcloud` into a target frame
(default: `odom`) using TF lookups at each cloud stamp.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


@dataclass
class Stats:
    clouds_seen: int = 0
    clouds_used: int = 0
    tf_failures: int = 0
    points_seen: int = 0
    points_kept: int = 0


class MapPcdBuilder(Node):
    def __init__(
        self,
        *,
        pointcloud_topic: str,
        target_frame: str,
        voxel_size: float,
        min_range: float,
        max_range: float,
        min_z: float,
        max_z: float,
        frame_stride: int,
        max_points_per_cloud: int,
    ) -> None:
        super().__init__("map_pcd_builder")
        self.set_parameters([
            Parameter("use_sim_time", value=True),
        ])

        self.pointcloud_topic = pointcloud_topic
        self.target_frame = target_frame
        self.voxel_size = voxel_size
        self.min_range = min_range
        self.max_range = max_range
        self.min_z = min_z
        self.max_z = max_z
        self.frame_stride = max(1, frame_stride)
        self.max_points_per_cloud = max(0, max_points_per_cloud)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.stats = Stats()
        self._voxel_acc: Dict[Tuple[int, int, int], np.ndarray] = {}
        self._lock = threading.Lock()

        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._on_cloud,
            10,
        )

    def _lookup_transform(self, source_frame: str, stamp: RosTimeMsg):
        when = Time.from_msg(stamp)
        timeout = Duration(seconds=0.08)
        return self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            when,
            timeout=timeout,
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.stats.clouds_seen += 1
        if (self.stats.clouds_seen - 1) % self.frame_stride != 0:
            return

        self.stats.clouds_used += 1

        try:
            tf_msg = self._lookup_transform(msg.header.frame_id, msg.header.stamp)
        except Exception:
            self.stats.tf_failures += 1
            if self.stats.tf_failures % 30 == 1:
                self.get_logger().warn(
                    f"TF lookup failed {self.stats.tf_failures} times: "
                    f"{self.target_frame} <- {msg.header.frame_id}"
                )
            return

        if hasattr(point_cloud2, "read_points_numpy"):
            pts = point_cloud2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            pts = np.asarray(pts, dtype=np.float32)
        else:
            pts_iter = point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            raw = np.asarray(list(pts_iter))
            if raw.size == 0:
                pts = np.zeros((0, 3), dtype=np.float32)
            elif raw.dtype.names:
                pts = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(
                    np.float32
                )
            else:
                pts = np.asarray(raw, dtype=np.float32)
        if pts.size == 0:
            return

        self.stats.points_seen += int(pts.shape[0])

        if self.max_points_per_cloud > 0 and pts.shape[0] > self.max_points_per_cloud:
            idx = np.linspace(0, pts.shape[0] - 1, self.max_points_per_cloud, dtype=np.int64)
            pts = pts[idx]

        xy_norm = np.linalg.norm(pts[:, :2], axis=1)
        mask = (
            (xy_norm >= self.min_range)
            & (xy_norm <= self.max_range)
            & (pts[:, 2] >= self.min_z)
            & (pts[:, 2] <= self.max_z)
        )
        pts = pts[mask]
        if pts.size == 0:
            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rot = self._quat_to_rot(q.x, q.y, q.z, q.w)
        pts_tf = (rot @ pts.T).T + np.array([t.x, t.y, t.z], dtype=np.float32)

        # Voxel accumulation with running average (x,y,z,intensity,count).
        voxel_idx = np.floor(pts_tf / self.voxel_size).astype(np.int32)
        with self._lock:
            for i in range(pts_tf.shape[0]):
                key = (int(voxel_idx[i, 0]), int(voxel_idx[i, 1]), int(voxel_idx[i, 2]))
                if key not in self._voxel_acc:
                    self._voxel_acc[key] = np.array(
                        [pts_tf[i, 0], pts_tf[i, 1], pts_tf[i, 2], 0.0, 1.0],
                        dtype=np.float64,
                    )
                else:
                    acc = self._voxel_acc[key]
                    n = acc[4] + 1.0
                    acc[0] = (acc[0] * acc[4] + pts_tf[i, 0]) / n
                    acc[1] = (acc[1] * acc[4] + pts_tf[i, 1]) / n
                    acc[2] = (acc[2] * acc[4] + pts_tf[i, 2]) / n
                    acc[4] = n

        self.stats.points_kept += int(pts_tf.shape[0])

    @staticmethod
    def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )

    def snapshot_points(self) -> np.ndarray:
        with self._lock:
            if not self._voxel_acc:
                return np.zeros((0, 4), dtype=np.float32)
            arr = np.zeros((len(self._voxel_acc), 4), dtype=np.float32)
            for i, acc in enumerate(self._voxel_acc.values()):
                arr[i, 0] = float(acc[0])
                arr[i, 1] = float(acc[1])
                arr[i, 2] = float(acc[2])
                arr[i, 3] = float(acc[3])
            return arr


def write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {points.shape[0]}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {points.shape[0]}\n")
        f.write("DATA ascii\n")
        for row in points:
            f.write(f"{row[0]:.6f} {row[1]:.6f} {row[2]:.6f} {row[3]:.6f}\n")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Build map PCD by replaying rosbag pointcloud + TF")
    p.add_argument("--bag", required=True, help="ros2 bag directory")
    p.add_argument("--output", required=True, help="output .pcd path")
    p.add_argument("--topic", default="/livox/lidar/pointcloud", help="pointcloud topic")
    p.add_argument("--target-frame", default="odom", help="output frame for accumulated map")
    p.add_argument("--voxel-size", type=float, default=0.05, help="voxel size in meters")
    p.add_argument("--frame-stride", type=int, default=3, help="use every Nth cloud")
    p.add_argument("--max-points-per-cloud", type=int, default=12000, help="cap points per cloud (0 disables)")
    p.add_argument("--min-range", type=float, default=0.2)
    p.add_argument("--max-range", type=float, default=25.0)
    p.add_argument("--min-z", type=float, default=-0.8)
    p.add_argument("--max-z", type=float, default=2.0)
    p.add_argument("--play-rate", type=float, default=1.0)
    p.add_argument("--quiet-sec", type=float, default=2.0, help="extra settle time after bag play ends")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    bag = Path(args.bag)
    if not bag.exists():
        print(f"[ERROR] bag not found: {bag}", file=sys.stderr)
        return 2

    rclpy.init()
    node = MapPcdBuilder(
        pointcloud_topic=args.topic,
        target_frame=args.target_frame,
        voxel_size=args.voxel_size,
        min_range=args.min_range,
        max_range=args.max_range,
        min_z=args.min_z,
        max_z=args.max_z,
        frame_stride=args.frame_stride,
        max_points_per_cloud=args.max_points_per_cloud,
    )
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    play_cmd = [
        "ros2",
        "bag",
        "play",
        str(bag),
        "--clock",
        "--rate",
        str(args.play_rate),
    ]
    print("[INFO] running:", " ".join(play_cmd))
    play_proc = subprocess.Popen(play_cmd)

    try:
        rc = play_proc.wait()
    except KeyboardInterrupt:
        play_proc.send_signal(signal.SIGINT)
        rc = play_proc.wait(timeout=5)

    if rc != 0:
        print(f"[ERROR] ros2 bag play exited with {rc}", file=sys.stderr)

    time.sleep(max(0.0, args.quiet_sec))

    points = node.snapshot_points()
    stats = node.stats
    out_path = Path(args.output)
    write_ascii_pcd(out_path, points)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    print("[SUMMARY]")
    print(f"clouds_seen={stats.clouds_seen}")
    print(f"clouds_used={stats.clouds_used}")
    print(f"tf_failures={stats.tf_failures}")
    print(f"points_seen={stats.points_seen}")
    print(f"points_kept={stats.points_kept}")
    print(f"voxel_points={points.shape[0]}")
    print(f"output={out_path}")

    return 0 if rc == 0 else rc


if __name__ == "__main__":
    raise SystemExit(main())
