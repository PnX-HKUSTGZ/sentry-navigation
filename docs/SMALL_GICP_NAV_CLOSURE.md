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
