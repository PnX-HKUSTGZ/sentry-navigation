# RMUL_26 Repro Release (Navigation + Decision)

Release date: 2026-02-18

This document pins a known-good cross-repo pair for RMUL_26 joint debugging.

## Pinned commits

- Navigation repo: `3e26b2bf52fb9da17ed5d4ad5549d0f4f4f53494`
- Decision repo: `bd8f3489fb40e51da7e1f30db0fa143ab158a915`

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

## Frame contract checks (recommended before stress tests)

These checks prevent silent config drift between localization and control frames.

```bash
# one-shot checker
bash tools/check_frame_contract.sh

# AMCL should stay on real body frame
ros2 param get /amcl base_frame_id

# Nav2 controller stack should consume fake base frame
ros2 param get /controller_server robot_base_frame
ros2 param get /bt_navigator robot_base_frame

# fake_vel_transform should publish base_link -> base_link_fake
ros2 run tf2_ros tf2_echo base_link base_link_fake
ros2 topic hz /tf

# Optional: monitor fallback warning if /local_plan becomes stale
ros2 topic echo /rosout | grep -E "fake_vel_transform|No fresh local plan"
```

Expected values:

- `/amcl base_frame_id` => `base_link`
- `/controller_server robot_base_frame` => `base_link_fake`
- `/bt_navigator robot_base_frame` => `base_link_fake`

## DWB / TEB A/B

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

LAUNCH_BRINGUP=1 CONTROLLER=dwb ROUNDS=1 GOAL_TIMEOUT=45 BETWEEN_GOALS_SEC=1 \
GOAL_OCCUPANCY_POLICY=reject \
WAYPOINTS='-5.0,3.0,0.0,1.0; -4.5,2.6,0.0,1.0' tools/stress_dynamic_nav.sh

LAUNCH_BRINGUP=1 CONTROLLER=teb ROUNDS=1 GOAL_TIMEOUT=45 BETWEEN_GOALS_SEC=1 \
GOAL_OCCUPANCY_POLICY=reject \
WAYPOINTS='-5.0,3.0,0.0,1.0; -4.5,2.6,0.0,1.0' tools/stress_dynamic_nav.sh
```

Artifacts are written to:

- `sentry-navigation/artifacts/nav_stress_<timestamp>/`

Goal occupancy policy options:

- `off`: disable map occupancy check
- `warn`: print warning only
- `reject`: skip occupied/unknown/out-of-map goals (recommended)
- `snap`: auto-snap invalid goals to nearest free cell within `GOAL_NEAREST_RADIUS` (m)
