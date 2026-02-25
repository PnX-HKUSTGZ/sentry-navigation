#!/usr/bin/env python3
"""
Publish a moving obstacle cluster as PointCloud2 for Nav2 dynamic-obstacle tests.
"""

import argparse
import math
import signal
import struct
import sys
from typing import List, Tuple

import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.node import Node


def _build_points(
    cx: float, cy: float, z: float, radius: float, step: float
) -> List[Tuple[float, float, float]]:
    pts: List[Tuple[float, float, float]] = []
    n = max(1, int(radius / max(0.01, step)))
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            x = cx + ix * step
            y = cy + iy * step
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius * radius:
                pts.append((x, y, z))
    return pts


def _to_cloud2(frame_id: str, stamp, points: List[Tuple[float, float, float]]) -> PointCloud2:
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    buf = bytearray()
    for p in points:
        buf.extend(struct.pack("fff", p[0], p[1], p[2]))

    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = bytes(buf)
    return msg


class DynamicObstacleCloudPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("dynamic_obstacle_cloud_publisher")
        self.args = args
        self.running = True
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(PointCloud2, args.topic, qos)
        self.timer = self.create_timer(1.0 / max(1.0, args.rate_hz), self.on_timer)
        self.start_time = self.get_clock().now()
        self.get_logger().info(
            "dynamic obstacle cloud active: "
            f"topic={args.topic}, frame={args.frame_id}, "
            f"track=({args.start_x:.2f}, {args.start_y:.2f})->({args.end_x:.2f}, {args.end_y:.2f}), "
            f"period={args.period_sec:.2f}s"
        )

    def on_timer(self) -> None:
        if not self.running:
            return
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        half_period = max(0.2, self.args.period_sec / 2.0)
        phase = (elapsed % self.args.period_sec) / half_period
        if phase <= 1.0:
            alpha = phase
        else:
            alpha = 2.0 - phase
        alpha = max(0.0, min(1.0, alpha))
        blend = 0.5 - 0.5 * math.cos(alpha * math.pi)

        cx = self.args.start_x + (self.args.end_x - self.args.start_x) * blend
        cy = self.args.start_y + (self.args.end_y - self.args.start_y) * blend
        points = _build_points(
            cx, cy, self.args.z, self.args.radius, self.args.grid_step
        )
        msg = _to_cloud2(self.args.frame_id, now.to_msg(), points)
        self.pub.publish(msg)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Publish moving obstacle cloud")
    p.add_argument("--topic", default="/segmentation/obstacle_right")
    p.add_argument("--frame-id", default="map")
    p.add_argument("--start-x", type=float, default=-4.7)
    p.add_argument("--start-y", type=float, default=2.5)
    p.add_argument("--end-x", type=float, default=-4.7)
    p.add_argument("--end-y", type=float, default=4.1)
    p.add_argument("--z", type=float, default=0.15)
    p.add_argument("--radius", type=float, default=0.22)
    p.add_argument("--grid-step", type=float, default=0.06)
    p.add_argument("--period-sec", type=float, default=6.0)
    p.add_argument("--rate-hz", type=float, default=12.0)
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = DynamicObstacleCloudPublisher(args)

    def _stop(_sig: int, _frame) -> None:
        node.running = False
        node.destroy_timer(node.timer)
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
