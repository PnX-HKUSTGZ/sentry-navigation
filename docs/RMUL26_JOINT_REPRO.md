# RMUL_26 Joint Reproduction (Navigation + Decision)

This runbook reproduces the full chain in ROS 2 Humble:
- headless Gazebo + AMCL + Nav2 (single bringup entrypoint)
- decision BT continuously sending `NavigateToPose` goals
- required BT subscriber topics stubbed to avoid short-circuit

## 0. Repositories

Assume sibling layout:

```text
<ws_parent>/
  sentry-navigation/
  sentry_DecisionMaking/
```

## 1. Build Navigation Repo

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

Recommended launch defaults in `src/rm_nav_bringup/config/launch_params.yaml`:
- `world: RMUL_26`
- `mode: nav`
- `localization: amcl`
- `use_sim: true`
- `controller: teb` or `dwb`

## 2. Build Decision Repo

```bash
cd ../sentry_DecisionMaking
bash tools/setup_behaviortree_ros2_dep.sh
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

## 3. Bringup Navigation (single entrypoint)

Terminal A:

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup.launch.py localization:=amcl nav_rviz:=false
```

Do not start extra `map_server` / `lifecycle_manager` manually.

## 4. Publish Initial Pose (AMCL QoS)

Terminal B:

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

timeout 6 ros2 topic pub --rate 5 --qos-reliability best_effort --qos-durability volatile \
  /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -5.0, y: 3.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'
```

## 5. Start Decision Node + Topic Stubs

Terminal C:

```bash
cd sentry_DecisionMaking
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_decision decision_node_launch.py \
  map_profile:=RMUL_26 \
  strategy_profile:=vp_center_control \
  use_sim_time:=true
```

Terminal D:

```bash
cd sentry_DecisionMaking
source /opt/ros/humble/setup.bash
source install/setup.bash
bash tools/publish_bt_stubs.sh
```

## 6. Validation Checklist

```bash
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose
ros2 node list
ros2 topic echo -n 1 /amcl_pose
```

Expectations:
- no duplicate Nav2/AMCL stacks
- `/amcl_pose` is available after initial pose
- decision keeps issuing `NavigateToPose` goals while stubs are alive

## 7. DWB/TEB A/B Reproduction

Run from `sentry-navigation`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

LAUNCH_BRINGUP=1 CONTROLLER=dwb ROUNDS=1 GOAL_TIMEOUT=45 BETWEEN_GOALS_SEC=1 \
WAYPOINTS='-5.0,3.0,0.0,1.0; -4.5,2.6,0.0,1.0' tools/stress_dynamic_nav.sh

LAUNCH_BRINGUP=1 CONTROLLER=teb ROUNDS=1 GOAL_TIMEOUT=45 BETWEEN_GOALS_SEC=1 \
WAYPOINTS='-5.0,3.0,0.0,1.0; -4.5,2.6,0.0,1.0' tools/stress_dynamic_nav.sh
```

The script writes per-run artifacts under:
- `sentry-navigation/artifacts/nav_stress_<timestamp>/`

## 8. One-shot Nav Goal Repro

`tools/repro_nav2_sim_goal.sh` now supports controller selection:

```bash
cd sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

CONTROLLER=teb tools/repro_nav2_sim_goal.sh
CONTROLLER=dwb tools/repro_nav2_sim_goal.sh
```
