# Fast-LIO2 + DWB + small-gicp Closing Report (RMUL_26_WAVE)

Date: 2026-02-21

## 1. Goal

Move from truth-dependent validation to real localization chain validation:

- Odometry input for Nav2: `/Odometry` (Fast-LIO2)
- Global correction: `small_gicp_registration` (`map -> odom`)
- Controller baseline: `DWB`
- Stress target: bidirectional 20 goals on wave road, success rate >= 95%

## 2. What Changed

### 2.1 Runtime/Test tooling

- `tools/stress_dynamic_nav.sh`
  - Added optional bag recording:
    - `BAG_RECORD`
    - `BAG_TOPICS`
    - `BAG_STORAGE`
    - `BAG_OUTPUT`
- New: `tools/evaluate_small_gicp_from_bag.sh`
  - Offline replay evaluator (`small_gicp + rosbag play`), outputs:
    - `iterations.csv`
    - `summary.txt`
- New: `tools/build_map_pcd_from_bag.py`
  - Rebuilds map PCD from bag replay (`pointcloud + TF`) in a chosen target frame.

### 2.2 Map/PCD assets

- New PCD used by final run:
  - `src/rm_nav_bringup/PCD/RMUL26_WAVE_SGICP_MAP.pcd`
- New map alias yaml:
  - `src/rm_nav_bringup/map/RMUL26_WAVE_SGICP_MAP.yaml`

### 2.3 small-gicp acceptance policy (simulation config)

In `src/rm_nav_bringup/config/simulation/small_gicp_registration.yaml`:

- `max_acceptable_fitness_score`: `3.0 -> 0.25`
- `recovery_max_correspondence_distance`: `5.0 -> 2.5`
- `recovery_acceptable_fitness_score`: `5.0 -> 0.35`

Reason: prevent large map->odom jumps from low-quality matches that were causing planner aborts (`Starting point in lethal space`, `no valid path found`).

## 3. Repro Commands

### 3.1 Capture dataset bag

```bash
cd /data/home/sim6g/sentry/sentry-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

ART=artifacts/small_gicp_dataset_$(date +%Y%m%d_%H%M%S)
ROUNDS=2 \
WAYPOINTS='-3.8,3.3,0.0,1.0; -5.6,3.3,0.0,1.0' \
GOAL_TIMEOUT=120 \
BETWEEN_GOALS_SEC=0.5 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=amcl \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
BRINGUP_LIO=fastlio \
LIDAR_NOISE_STDDEV=0.004 \
NAV_RVIZ=false \
STARTUP_SETTLE_SEC=5 \
ACTION_WAIT_TIMEOUT=240 \
GOAL_OCCUPANCY_POLICY=snap \
GOAL_NEAREST_RADIUS=1.5 \
BAG_RECORD=1 \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

### 3.2 Rebuild map-frame PCD for small-gicp

```bash
/usr/bin/python3 tools/build_map_pcd_from_bag.py \
  --bag artifacts/small_gicp_dataset_20260221_153400/rosbag_dataset \
  --output src/rm_nav_bringup/PCD/RMUL26_WAVE_SGICP_MAP.pcd \
  --topic /livox/lidar/pointcloud \
  --target-frame map \
  --voxel-size 0.05 \
  --frame-stride 3 \
  --max-points-per-cloud 10000 \
  --min-range 0.2 \
  --max-range 25 \
  --min-z -0.6 \
  --max-z 2.0
```

### 3.3 Offline registration evaluation

```bash
BAG_PATH=artifacts/small_gicp_dataset_20260221_153400/rosbag_dataset \
PCD_PATH=src/rm_nav_bringup/PCD/RMUL26_WAVE_SGICP_MAP.pcd \
INIT_X=-5.0 INIT_Y=3.0 INIT_QZ=0.0 INIT_QW=1.0 \
OUT_DIR=artifacts/small_gicp_offline_eval_$(date +%Y%m%d_%H%M%S) \
bash tools/evaluate_small_gicp_from_bag.sh
```

### 3.4 20-goal baseline / target matrix

Baseline (`amcl`):

```bash
ART=artifacts/regress_amcl_20_$(date +%Y%m%d_%H%M%S)
ROUNDS=10 \
WAYPOINTS='-3.8,3.3,0.0,1.0; -5.6,3.3,0.0,1.0' \
GOAL_TIMEOUT=120 \
BETWEEN_GOALS_SEC=0.5 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=amcl \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
BRINGUP_LIO=fastlio \
LIDAR_NOISE_STDDEV=0.004 \
NAV_RVIZ=false \
STARTUP_SETTLE_SEC=5 \
ACTION_WAIT_TIMEOUT=240 \
GOAL_OCCUPANCY_POLICY=snap \
GOAL_NEAREST_RADIUS=1.5 \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

Target (`small_gicp`):

```bash
ART=artifacts/regress_smallgicp_20_cons_$(date +%Y%m%d_%H%M%S)
ROUNDS=10 \
WAYPOINTS='-3.8,3.3,0.0,1.0; -5.6,3.3,0.0,1.0' \
GOAL_TIMEOUT=120 \
BETWEEN_GOALS_SEC=0.5 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=small_gicp \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE_SGICP_MAP \
BRINGUP_LIO=fastlio \
LIDAR_NOISE_STDDEV=0.004 \
NAV_RVIZ=false \
STARTUP_SETTLE_SEC=5 \
ACTION_WAIT_TIMEOUT=300 \
NAV_START_DELAY=22 \
MAP_TF_REQUIRED=1 \
GOAL_OCCUPANCY_POLICY=snap \
GOAL_NEAREST_RADIUS=1.5 \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

## 4. Results

- Baseline artifact:
  - `artifacts/regress_amcl_20_20260221_155310`
  - `20/20`, success rate `100%`
- small-gicp before conservative acceptance tuning:
  - `artifacts/regress_smallgicp_20_20260221_160509`
  - `18/20`, success rate `90%`
  - failures mainly when planning to `(-5.6, 3.3)`
- small-gicp after conservative acceptance tuning:
  - `artifacts/regress_smallgicp_20_cons_20260221_162644`
  - `20/20`, success rate `100%`

Observed failure signatures before tuning:

- `Starting point in lethal space`
- `no valid path found`
- followed by action `ABORTED`

After tuning, these abort signatures disappeared in the 20-goal run above.

## 5. Notes

- Keep `bringup.launch.py` as the only startup entrypoint.
- Keep full chain `use_sim_time` enabled.
- For replay/build scripts in mixed conda environments, prefer `/usr/bin/python3` to avoid `rclpy` ABI mismatch.

## 6. 2026-02-25 Realistic-Chain Regression (small-gicp + DWB)

This round uses a stricter "real-chain" setup:

- odom from `/Odometry` (no default ground-truth fallback)
- localization: `small_gicp`
- controller: `dwb`
- world/map: `RMUL_26_WAVE` / `RMUL26_WAVE`
- lidar noise enabled
- map->odom pre-action gate enabled

### 6.1 Stable loop profile (21 goals)

Command:

```bash
ART=artifacts/realstress_smallgicp_loop3_final_$(date +%Y%m%d_%H%M%S)
WAYPOINTS='-5.6,3.3,0.0,1.0; -3.8,3.3,0.0,1.0; -4.6,1.8,0.0,1.0' \
INIT_X=-5.375 INIT_Y=3.425 INIT_QZ=-0.2798765 INIT_QW=0.9600360 \
ROUNDS=7 \
GOAL_TIMEOUT=90 \
BETWEEN_GOALS_SEC=0.5 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=small_gicp \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
BRINGUP_LIO=fastlio \
LIDAR_NOISE_STDDEV=0.004 \
NAV_RVIZ=false \
STARTUP_SETTLE_SEC=5 \
ACTION_WAIT_TIMEOUT=300 \
NAV_START_DELAY=22 \
MAP_TF_REQUIRED=1 \
MAP_TF_PRE_ACTION_CHECK=1 \
GOAL_OCCUPANCY_POLICY=reject \
CLEAR_COSTMAP_BEFORE_GOAL=1 \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

Result:

- `artifacts/realstress_smallgicp_loop3_final_20260225_010349`
- `21/21`, success rate `100%`

### 6.2 Higher-noise check (12 goals)

Same profile with `LIDAR_NOISE_STDDEV=0.006`:

- `artifacts/realstress_smallgicp_loop3_noise006_20260225_010822`
- `11/12`, success rate `91.7%`
- dominant miss type: timeout (no ABORT / no planner crash)

### 6.3 Practical takeaways

- Setting initial pose to simulation spawn (`-5.375, 3.425, qz=-0.2798765, qw=0.9600360`) removed frequent first-goal bootstrap misses.
- The selected loop avoids round-boundary duplicate goals and gives stable, reproducible success in wave-road conditions.
- For this wave scenario, global planning is more stable when it is static-map-driven, while dynamic obstacle reaction is left primarily to local costmap/controller.

### 6.4 Dynamic-obstacle injection regression

To validate dynamic obstacle avoidance without depending on Gazebo model-state services,
inject a moving obstacle cloud into `/segmentation/obstacle_right` (consumed by local costmap obstacle layer):

```bash
ART=artifacts/realstress_smallgicp_dynobs_cloud_$(date +%Y%m%d_%H%M%S)
WAYPOINTS='-5.6,3.3,0.0,1.0; -3.8,3.3,0.0,1.0' \
ROUNDS=4 \
GOAL_TIMEOUT=90 \
BETWEEN_GOALS_SEC=0.5 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=small_gicp \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
BRINGUP_LIO=fastlio \
LIDAR_NOISE_STDDEV=0.004 \
NAV_RVIZ=false \
STARTUP_SETTLE_SEC=5 \
ACTION_WAIT_TIMEOUT=300 \
NAV_START_DELAY=22 \
MAP_TF_REQUIRED=1 \
MAP_TF_PRE_ACTION_CHECK=1 \
GOAL_OCCUPANCY_POLICY=reject \
CLEAR_COSTMAP_BEFORE_GOAL=1 \
DYNAMIC_OBS_ENABLE=1 \
DYNAMIC_OBS_MODE=cloud \
DYNAMIC_OBS_NAME=dyn_obs_cross_lane \
DYNAMIC_OBS_START_X=-4.7 DYNAMIC_OBS_START_Y=2.5 \
DYNAMIC_OBS_END_X=-4.7 DYNAMIC_OBS_END_Y=4.1 \
DYNAMIC_OBS_PERIOD_SEC=5.5 \
DYNAMIC_OBS_RATE_HZ=14 \
DYNAMIC_OBS_RADIUS=0.26 \
DYNAMIC_OBS_GRID_STEP=0.05 \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

Observed:

- `artifacts/realstress_smallgicp_dynobs_mid_20260225_085546`
- `6/6`, success rate `100%`
- obstacle publisher log confirms dynamic cloud injection is active (`dynamic_obstacle.log`).
- avg duration increased (dynamic obstacle interaction), no `ABORTED` / `TIMEOUT`.

### 6.5 STVL selection A/B (simulation global costmap)

To compare static global map vs STVL global map under the same dynamic-obstacle setup:

- Baseline: `NAV_USE_STVL=0` (global `static_layer + inflation_layer`)
- Variant: `NAV_USE_STVL=1` (global `static_layer + stvl_layer + inflation_layer`)
- Other settings unchanged: `small_gicp + DWB`, same waypoints, same noise and dynamic cloud profile.

Command template (baseline / STVL only differ in `NAV_USE_STVL`):

```bash
ART=artifacts/realstress_smallgicp_dynobs20_${TAG}_$(date +%Y%m%d_%H%M%S)
WAYPOINTS='-5.6,3.3,0.0,1.0; -3.8,3.3,0.0,1.0' \
ROUNDS=10 \
GOAL_TIMEOUT=90 \
BETWEEN_GOALS_SEC=0.5 \
LAUNCH_BRINGUP=1 \
LOCALIZATION=small_gicp \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
BRINGUP_LIO=fastlio \
LIDAR_NOISE_STDDEV=0.004 \
NAV_RVIZ=false \
STARTUP_SETTLE_SEC=8 \
ACTION_WAIT_TIMEOUT=180 \
NAV_START_DELAY=24 \
MAP_TF_REQUIRED=1 \
MAP_TF_PRE_ACTION_CHECK=1 \
GOAL_OCCUPANCY_POLICY=reject \
CLEAR_COSTMAP_BEFORE_GOAL=1 \
DYNAMIC_OBS_ENABLE=1 \
DYNAMIC_OBS_MODE=cloud \
DYNAMIC_OBS_NAME=dyn_obs_cross_lane \
DYNAMIC_OBS_START_X=-4.7 DYNAMIC_OBS_START_Y=2.5 \
DYNAMIC_OBS_END_X=-4.7 DYNAMIC_OBS_END_Y=4.1 \
DYNAMIC_OBS_PERIOD_SEC=5.5 \
DYNAMIC_OBS_RATE_HZ=14 \
DYNAMIC_OBS_RADIUS=0.26 \
DYNAMIC_OBS_GRID_STEP=0.05 \
NAV_USE_STVL=${NAV_USE_STVL} \
ARTIFACT_DIR=$ART \
bash tools/stress_dynamic_nav.sh
```

Results:

- Baseline `NAV_USE_STVL=0`:
  - `artifacts/realstress_smallgicp_dynobs20_baseline_20260225_094408`
  - `20/20`, success `100%`
  - `avg_duration_sec=15.405`
  - timeout retries observed in goal logs: `2`
- STVL `NAV_USE_STVL=1`:
  - `artifacts/realstress_smallgicp_dynobs20_stvl_retry_20260225_130032`
  - `20/20`, success `100%`
  - `avg_duration_sec=5.191`
  - timeout retries observed in goal logs: `0`

Selection conclusion for current simulation profile:

- Keep STVL as a selectable option (not hard-forced).
- For this tested dynamic-cloud setup, `NAV_USE_STVL=1` is preferred:
  - same success rate (`100%`)
  - significantly lower average goal duration
  - no timeout-retry events in the 20-goal run
