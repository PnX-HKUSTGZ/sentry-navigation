# RMUL_26 Repro Release (Navigation + Decision)

Release date: 2026-02-18

This document pins a known-good cross-repo pair for RMUL_26 joint debugging.

## Pinned commits

- Navigation repo: `b93f3d4f6ed2a5b343b5555d47e102e34aeadcea`
- Decision repo: `924b5dc19ef7e16646dca32bb3b9e826447cde28`

## Goals covered

- Single entrypoint Nav2 bringup: `bringup.launch.py`
- AMCL `/initialpose` with `BEST_EFFORT + VOLATILE`
- Clean ROS graph requirement (no duplicated nav2/amcl stacks)
- Decision BT continuously emits `NavigateToPose` goals when stub topics are present
- DWB / TEB controller switch and A/B verification

## Minimal joint run (copy/paste)

Terminal 1 (navigation):

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup.launch.py localization:=amcl nav_rviz:=false
```

Terminal 2 (AMCL init pose):

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 6 ros2 topic pub --rate 5 --qos-reliability best_effort --qos-durability volatile \
  /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -5.0, y: 3.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'
```

Terminal 3 (decision node):

```bash
cd ../sentry_DecisionMaking
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_decision decision_node_launch.py \
  map_profile:=RMUL_26 \
  strategy_profile:=vp_center_control \
  use_sim_time:=true
```

Terminal 4 (mandatory BT stubs):

```bash
cd ../sentry_DecisionMaking
source /opt/ros/humble/setup.bash
source install/setup.bash
bash tools/publish_bt_stubs.sh
```

Verification:

```bash
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose
ros2 topic echo -n 1 /amcl_pose
```

## DWB / TEB A/B

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

LAUNCH_BRINGUP=1 CONTROLLER=dwb ROUNDS=1 GOAL_TIMEOUT=45 BETWEEN_GOALS_SEC=1 \
WAYPOINTS='-5.0,3.0,0.0,1.0; -4.5,2.6,0.0,1.0' tools/stress_dynamic_nav.sh

LAUNCH_BRINGUP=1 CONTROLLER=teb ROUNDS=1 GOAL_TIMEOUT=45 BETWEEN_GOALS_SEC=1 \
WAYPOINTS='-5.0,3.0,0.0,1.0; -4.5,2.6,0.0,1.0' tools/stress_dynamic_nav.sh
```

Artifacts are written to:

- `sentry-navigation/artifacts/nav_stress_<timestamp>/`
