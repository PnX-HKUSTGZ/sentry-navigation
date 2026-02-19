#!/usr/bin/env /usr/bin/python3
"""Merge dual lidar obstacle pointclouds into a single topic.

Design goal:
- Keep existing segmentation chains untouched.
- Optionally transform each cloud into one target frame.
- Publish a bounded-size merged cloud for downstream scan/costmap use.
"""

from __future__ import annotations

import argparse
import math
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


Point3 = Tuple[float, float, float]


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> Tuple[Tuple[float, float, float], ...]:
    """Quaternion to 3x3 rotation matrix."""
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _apply_transform(points: Sequence[Point3], tf_msg: TransformStamped) -> List[Point3]:
    """Apply rigid transform to point list."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    rot = _quat_to_rot(q.x, q.y, q.z, q.w)
    tx, ty, tz = t.x, t.y, t.z

    out: List[Point3] = []
    for px, py, pz in points:
        ox = rot[0][0] * px + rot[0][1] * py + rot[0][2] * pz + tx
        oy = rot[1][0] * px + rot[1][1] * py + rot[1][2] * pz + ty
        oz = rot[2][0] * px + rot[2][1] * py + rot[2][2] * pz + tz
        out.append((ox, oy, oz))
    return out


class ObstacleCloudMerger(Node):
    def __init__(
        self,
        *,
        target_frame: str,
        left_topic: str,
        right_topic: str,
        output_topic: str,
        publish_rate_hz: float,
        max_points_per_cloud: int,
        stale_timeout_sec: float,
        use_sim_time: bool,
    ) -> None:
        super().__init__("obstacle_cloud_merger")
        if self.has_parameter("use_sim_time"):
            self.set_parameters([Parameter("use_sim_time", value=use_sim_time)])
        else:
            self.declare_parameter("use_sim_time", use_sim_time)
        self.target_frame = target_frame
        self.max_points_per_cloud = max(1, max_points_per_cloud)
        self.stale_timeout = Duration(seconds=max(stale_timeout_sec, 0.1))
        self.warn_period = Duration(seconds=1.0)
        self.last_warn_time = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=8.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.left_msg: PointCloud2 | None = None
        self.right_msg: PointCloud2 | None = None
        self.left_points: List[Point3] = []
        self.right_points: List[Point3] = []
        self.left_stamp = self.get_clock().now()
        self.right_stamp = self.get_clock().now()

        self.left_sub = self.create_subscription(
            PointCloud2, left_topic, self._left_cb, qos_profile_sensor_data
        )
        self.right_sub = self.create_subscription(
            PointCloud2, right_topic, self._right_cb, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._on_timer)

        self.get_logger().info(
            "merge_obstacle_clouds started: "
            f"left={left_topic}, right={right_topic}, output={output_topic}, target={target_frame}"
        )

    def _left_cb(self, msg: PointCloud2) -> None:
        self.left_msg = msg
        self.left_points = self._to_target_frame(msg)
        self.left_stamp = self.get_clock().now()

    def _right_cb(self, msg: PointCloud2) -> None:
        self.right_msg = msg
        self.right_points = self._to_target_frame(msg)
        self.right_stamp = self.get_clock().now()

    def _is_fresh(self, stamp) -> bool:
        return (self.get_clock().now() - stamp) <= self.stale_timeout

    def _warn_once_per_sec(self, text: str) -> None:
        now = self.get_clock().now()
        if self.last_warn_time is None or (now - self.last_warn_time) >= self.warn_period:
            self.last_warn_time = now
            self.get_logger().warn(text)

    def _extract_points(self, msg: PointCloud2) -> List[Point3]:
        """Read xyz points with bounded size."""
        total = max(1, int(msg.width) * int(msg.height))
        stride = max(1, total // self.max_points_per_cloud)

        out: List[Point3] = []
        for idx, point in enumerate(
            point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ):
            if idx % stride != 0:
                continue
            px, py, pz = float(point[0]), float(point[1]), float(point[2])
            if math.isfinite(px) and math.isfinite(py) and math.isfinite(pz):
                out.append((px, py, pz))
            if len(out) >= self.max_points_per_cloud:
                break
        return out

    def _to_target_frame(self, msg: PointCloud2) -> List[Point3]:
        pts = self._extract_points(msg)
        if not pts:
            return []
        if msg.header.frame_id == self.target_frame:
            return pts

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08),
            )
        except TransformException as exc:
            self._warn_once_per_sec(
                f"TF missing for {msg.header.frame_id}->{self.target_frame}: {exc}"
            )
            return []
        return _apply_transform(pts, tf_msg)

    def _on_timer(self) -> None:
        merged: List[Point3] = []
        has_left = self.left_msg is not None and self._is_fresh(self.left_stamp)
        has_right = self.right_msg is not None and self._is_fresh(self.right_stamp)

        if has_left and self.left_points:
            merged.extend(self.left_points)
        if has_right and self.right_points:
            merged.extend(self.right_points)

        if not merged:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, merged))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge dual obstacle pointcloud topics.")
    parser.add_argument("--target-frame", default="livox_frame")
    parser.add_argument("--left-topic", default="/segmentation/obstacle")
    parser.add_argument("--right-topic", default="/segmentation/obstacle_right")
    parser.add_argument("--output-topic", default="/segmentation/obstacle_merged")
    parser.add_argument("--publish-rate", type=float, default=8.0)
    parser.add_argument("--max-points-per-cloud", type=int, default=12000)
    parser.add_argument("--stale-timeout", type=float, default=0.8)
    parser.add_argument("--use-sim-time", action="store_true")
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = ObstacleCloudMerger(
        target_frame=args.target_frame,
        left_topic=args.left_topic,
        right_topic=args.right_topic,
        output_topic=args.output_topic,
        publish_rate_hz=args.publish_rate,
        max_points_per_cloud=args.max_points_per_cloud,
        stale_timeout_sec=args.stale_timeout,
        use_sim_time=args.use_sim_time,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
