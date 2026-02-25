#!/usr/bin/env python3
"""
Spawn and drive a simple moving obstacle in Gazebo classic (ROS 2 Humble).

This helper is intentionally lightweight and only depends on rclpy + gazebo_msgs.
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from rclpy.node import Node


def _build_box_sdf(name: str, size_x: float, size_y: float, size_z: float) -> str:
    # A plain collision/visual box that can be teleported by set_entity_state.
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <allow_auto_disable>false</allow_auto_disable>
    <link name='body'>
      <inertial>
        <mass>8.0</mass>
        <inertia>
          <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{size_x:.3f} {size_y:.3f} {size_z:.3f}</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{size_x:.3f} {size_y:.3f} {size_z:.3f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.8 0.1 1.0</ambient>
          <diffuse>0.1 0.8 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class DynamicObstacleDriver(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("dynamic_obstacle_driver")
        self.args = args
        self.running = True

        self.spawn_cli = self.create_client(SpawnEntity, args.spawn_service)
        self.set_cli = self.create_client(SetEntityState, args.set_service)
        self.delete_cli = self.create_client(DeleteEntity, args.delete_service)

    def wait_services(self, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline and rclpy.ok():
            ok_spawn = self.spawn_cli.wait_for_service(timeout_sec=0.2)
            ok_set = self.set_cli.wait_for_service(timeout_sec=0.2)
            if ok_spawn and ok_set:
                return True
        return False

    def spawn_obstacle(self) -> bool:
        req = SpawnEntity.Request()
        req.name = self.args.name
        req.xml = _build_box_sdf(
            self.args.name, self.args.size_x, self.args.size_y, self.args.size_z
        )
        req.robot_namespace = ""
        req.reference_frame = self.args.reference_frame
        req.initial_pose = Pose()
        req.initial_pose.position.x = self.args.start_x
        req.initial_pose.position.y = self.args.start_y
        req.initial_pose.position.z = self.args.z
        req.initial_pose.orientation.w = 1.0

        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error("spawn service call failed or timed out")
            return False

        res = fut.result()
        if res.success:
            self.get_logger().info(f"spawned dynamic obstacle: {self.args.name}")
            return True

        msg = (res.status_message or "").lower()
        if "already exists" in msg:
            self.get_logger().warn(
                f"entity already exists ({self.args.name}), reusing existing one"
            )
            return True

        self.get_logger().error(
            f"spawn failed for {self.args.name}: {res.status_message}"
        )
        return False

    def drive_loop(self) -> None:
        state = EntityState()
        state.name = self.args.name
        state.reference_frame = self.args.reference_frame
        state.pose.orientation.w = 1.0
        state.twist = Twist()

        half_period = max(0.2, self.args.period_sec / 2.0)
        dt = max(0.02, self.args.dt_sec)
        p0 = (self.args.start_x, self.args.start_y)
        p1 = (self.args.end_x, self.args.end_y)
        z = self.args.z
        t0 = time.time()

        while rclpy.ok() and self.running:
            elapsed = time.time() - t0
            phase = (elapsed % self.args.period_sec) / half_period
            if phase <= 1.0:
                alpha = phase
            else:
                alpha = 2.0 - phase
            alpha = max(0.0, min(1.0, alpha))

            # Cosine easing prevents sudden velocity jumps near turn points.
            blend = 0.5 - 0.5 * math.cos(alpha * math.pi)
            x = p0[0] + (p1[0] - p0[0]) * blend
            y = p0[1] + (p1[1] - p0[1]) * blend

            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = z

            req = SetEntityState.Request()
            req.state = state
            fut = self.set_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.3)
            time.sleep(dt)

    def delete_obstacle(self) -> None:
        if not self.args.delete_on_exit:
            return
        if not self.delete_cli.wait_for_service(timeout_sec=0.5):
            return
        req = DeleteEntity.Request()
        req.name = self.args.name
        fut = self.delete_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is not None and fut.result().success:
            self.get_logger().info(f"deleted dynamic obstacle: {self.args.name}")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Drive a moving dynamic obstacle in Gazebo")
    p.add_argument("--name", default="dyn_obs_cross_1")
    p.add_argument("--start-x", type=float, default=-4.7)
    p.add_argument("--start-y", type=float, default=2.5)
    p.add_argument("--end-x", type=float, default=-4.7)
    p.add_argument("--end-y", type=float, default=4.1)
    p.add_argument("--z", type=float, default=0.35)
    p.add_argument("--size-x", type=float, default=0.35)
    p.add_argument("--size-y", type=float, default=0.35)
    p.add_argument("--size-z", type=float, default=0.70)
    p.add_argument("--period-sec", type=float, default=6.0)
    p.add_argument("--dt-sec", type=float, default=0.08)
    p.add_argument("--wait-service-timeout", type=float, default=30.0)
    p.add_argument("--reference-frame", default="world")
    p.add_argument("--spawn-service", default="/spawn_entity")
    p.add_argument("--set-service", default="/gazebo/set_entity_state")
    p.add_argument("--delete-service", default="/delete_entity")
    p.add_argument("--delete-on-exit", action="store_true")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = DynamicObstacleDriver(args)

    def _stop(_sig: int, _frame) -> None:
        node.running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        if not node.wait_services(args.wait_service_timeout):
            node.get_logger().error("gazebo services not ready in time")
            return 2
        if not node.spawn_obstacle():
            return 3
        node.drive_loop()
        return 0
    finally:
        node.delete_obstacle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
