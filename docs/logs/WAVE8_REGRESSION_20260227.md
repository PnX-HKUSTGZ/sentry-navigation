# WAVE8 Regression Log (2026-02-27)

## Scope
- ROS2 Humble + Nav2 (DWB) + AMCL
- World: `RMUL_26_WAVE`
- Goal: verify passability over 8 consecutive bumps with measurable trajectory coverage

## Key Environment Adjustments
- Wave strip moved onto a long free corridor to avoid map-obstacle truncation.
  - strip x-range: `[-1.40, 0.52]` (length `1.92m`)
  - strip y-center: `2.75`
  - strip peak-to-peak kept `0.07m`
  - strip z-offset raised so bumps are not buried by base terrain
- `RMUL_26_WAVE` spawn moved near strip left end for deterministic entry.
- `wavepass` obstacle profile keeps costmap obstacle layers disabled during this passability regression.

## Baseline Command (single-goal wave pass)
```bash
RM_NAV_OBSTACLE_PROFILE=wavepass \
LAUNCH_BRINGUP=1 \
LOCALIZATION=amcl \
CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE \
BRINGUP_MAP=RMUL26_WAVE \
LIDAR_NOISE_STDDEV=0.0 \
INIT_X=-1.60 INIT_Y=3.30 INIT_QZ=0.0 INIT_QW=1.0 \
ROUNDS=1 \
WAYPOINTS='1.9,3.3,0,1' \
GOAL_TIMEOUT=180 TIMEOUT_RETRY_COUNT=0 \
GOAL_OCCUPANCY_POLICY=snap GOAL_NEAREST_RADIUS=2.0 \
BAG_RECORD=1 \
BAG_TOPICS='/clock /tf /tf_static /amcl_pose /ground_truth/odom' \
GT_TRACE_RECORD=0 \
ARTIFACT_DIR=<artifact_path> \
bash tools/stress_dynamic_nav.sh
```

## Coverage Metric
From `analysis_drift/loc_vs_ground_truth.csv`:
- wave x-range: `[-1.40, 0.52]`
- wave y-range used for tolerance: `[2.20, 3.20]`
- wavelength: `0.24m`
- `estimated_waves_crossed = inside_x_span / 0.24`

Command:
```bash
python3 tools/evaluate_wave_coverage.py \
  --csv <artifact>/analysis_drift/loc_vs_ground_truth.csv \
  --wave-x-min -1.40 --wave-x-max 0.52 \
  --wave-y-min 2.20 --wave-y-max 3.20 \
  --wavelength 0.24
```

## Repeated Results (goal `x=1.9,y=3.3`)
- `wave8_run27_rep1_20260227_032017`: success `1/1`, waves `7.885`
- `wave8_run27_rep2_20260227_032104`: success `0/1` (timeout), waves `7.921`
- `wave8_run27_rep3_20260227_032659`: success `1/1`, waves `7.610`

Summary:
- coverage is consistently close to 8-wave target (`~7.6-7.9`)
- action success is not yet fully stable (2/3 in this set)

## Current Conclusion
- Methodology is now measuring real traversal span instead of just action status.
- The platform can repeatedly reach near-8-wave physical coverage in nav mode.
- Remaining gap is terminal stability (`SUCCEEDED` consistency), not traversal capability itself.

## 2026-02-27 Latest Iteration (Same Day Update)

### Script / Runtime Fixes
- `tools/stress_dynamic_nav.sh`
  - fixed AMCL init gate parsing: no more `awk` expression crash on `/amcl_pose` format variants.
  - RMUL wave defaults aligned to current spawn corridor:
    - `INIT_X=-1.42`, `INIT_Y=2.75`
    - wave default waypoints: `1.90,2.75 -> -1.42,2.75`
  - preflight ROS-graph check now ignores ghost-only `/static_transform_publisher_*` entries.
- `src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp`
  - when `spin_speed=0`, angular velocity now passthroughs `/cmd_vel.angular.z` instead of forced zero.
  - non-zero `spin_speed` keeps legacy fixed-spin behavior.
- `src/rm_nav_bringup/launch/common.py`
  - clarified `spin_speed` semantics in launch comment (`0.0=passthrough`).

### Confirmed Regression Commands

1) Stable short-range baseline (10 goals, 100%)
```bash
LAUNCH_BRINGUP=1 \
LOCALIZATION=amcl CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE BRINGUP_MAP=RMUL26_WAVE \
LIDAR_NOISE_STDDEV=0.0 \
ROUNDS=5 \
WAYPOINTS='0.0,2.75,0,1; -1.20,2.75,0,1' \
GOAL_TIMEOUT=80 TIMEOUT_RETRY_COUNT=0 ABORT_RETRY_COUNT=0 \
PROGRESS_REQUIRED_RADIUS=0.05 \
PROGRESS_TIME_ALLOWANCE=12.0 \
CONTROLLER_FAILURE_TOLERANCE=0.35 \
GOAL_OCCUPANCY_POLICY=snap GOAL_NEAREST_RADIUS=2.0 \
BAG_RECORD=0 GT_TRACE_RECORD=0 \
ARTIFACT_DIR=/tmp/wave_stability_progress_tuned_20260227_123553 \
bash tools/stress_dynamic_nav.sh
```

Result:
- `succeeded=10 / total_goals=10`
- `success_rate_percent=100.0`
- artifact: `/tmp/wave_stability_progress_tuned_20260227_123553`

2) Longer-span target still unstable (needs further work)
```bash
LAUNCH_BRINGUP=1 \
LOCALIZATION=amcl CONTROLLER=dwb \
BRINGUP_WORLD=RMUL_26_WAVE BRINGUP_MAP=RMUL26_WAVE \
LIDAR_NOISE_STDDEV=0.0 \
ROUNDS=3 \
WAYPOINTS='1.90,2.75,0,1; -1.20,2.75,0,1' \
GOAL_TIMEOUT=120 TIMEOUT_RETRY_COUNT=0 ABORT_RETRY_COUNT=0 \
PROGRESS_REQUIRED_RADIUS=0.05 \
PROGRESS_TIME_ALLOWANCE=12.0 \
CONTROLLER_FAILURE_TOLERANCE=0.35 \
GOAL_OCCUPANCY_POLICY=snap GOAL_NEAREST_RADIUS=2.0 \
BAG_RECORD=0 GT_TRACE_RECORD=0 \
ARTIFACT_DIR=<artifact> \
bash tools/stress_dynamic_nav.sh
```

Observed failure mode:
- repeated `[follow_path] [ActionServer] Aborting handle`
- frequent planner warning: `Starting point in lethal space`
- implies long-span route still sensitive to local/global costmap start-cell validity.

### Updated Practical Conclusion
- The closed-loop stack is now reproducibly stable for short-range oscillation regression on wave lane.
- The long-span “single-goal far crossing” case remains the blocker to claim robust 8-bump-through nav by action success alone.
- Next debug focus should stay on start-cell lethal mitigation for long goals (not on AMCL init parser / fake_vel angular passthrough, which are now fixed).

## 2026-02-27 Late Update (Determinism Re-check)

### What Was Re-validated
- Re-ran multiple 2-goal wave-lane loops on the current workspace state (AMCL + DWB + `wavepass`).
- Added/validated a simulation-safe option in `fake_vel_transform`:
  - new param `use_local_plan_transform` (set `false` in simulation launch)
  - angular passthrough remains enabled when `spin_speed=0`
- Added extra DWB runtime overrides in `tools/stress_dynamic_nav.sh` (`min/max_vel_x` support) for controlled experiments.

### Representative Artifacts (current state)
- `artifacts/nav_stress_20260227_133217` -> `0/2` (both TIMEOUT)
- `artifacts/nav_stress_20260227_133746` -> `1/2`
- `artifacts/nav_stress_20260227_135916` -> `1/2` (with sim fake-vel local-plan transform disabled)
- `artifacts/nav_stress_20260227_141511` -> `1/2` (timeout retry enabled)
- `artifacts/nav_stress_20260227_141856` -> `1/2` (return yaw=pi)
- `artifacts/nav_stress_20260227_142443` -> `1/2` (xy tol=0.40)
- `artifacts/nav_stress_20260227_142905` -> `0/2` (TEB trial, first goal timeout)

### Stable Failure Pattern (most important)
1. First goal often reports `SUCCEEDED`, but BT navigator starts next goal from about `(-0.48, 2.72)` rather than near `(0.00, 2.75)`.
2. After second goal starts, controller enters repeated
   - `[follow_path] [ActionServer] Aborting handle`
   loop until timeout.
3. During that phase, there are usually no `Passing new path to controller` logs for the second goal, indicating path handoff/replan is not effectively progressing.

### Practical Interpretation
- Current 2-goal round-trip regression is **not** at competition target yet.
- Existing "100% short-run" claim is not representative for the current configuration/state and should not be used as the final acceptance evidence.
- Main blocker is no longer simple AMCL init parsing or fake-vel angular passthrough; it is a closed-loop progression failure after mid-lane stop.

### Next Focus (P1)
- Capture one minimal failing bag for second-goal abort-loop with:
  - `/local_plan`, `/cmd_vel`, `/cmd_vel_chassis`, `/tf`, `/amcl_pose`, `/local_costmap/costmap`.
- Diagnose why second goal lacks usable new path handoff (planner vs controller progression gate).
- Only after this, re-run 20-goal metric for acceptance.

## 2026-02-27 Diagnostic Evidence (Single-goal Timeout)

Artifact: `artifacts/nav_stress_20260227_145726`

Command profile:
- one goal: `(0.0, 2.75)`
- `GOAL_TIMEOUT=70`
- DWB with `progress_required_radius=0.005`, `progress_time_allowance=30`
- `bag_record=1` + `ground_truth` trace enabled

Result:
- `succeeded=0/1`, timeout

Evidence summary:
- `ros2 bag info` shows controller and planner were active during timeout:
  - `/local_plan`: `798` msgs
  - `/cmd_vel`: `1350` msgs
  - `/cmd_vel_chassis`: `1350` msgs
- `bringup.log` shows many `Passing new path to controller` followed by:
  - `Failed to make progress`
  - then repeated `[follow_path] [ActionServer] Aborting handle`
- Ground-truth trace (`ground_truth_pose.csv`) over the failed interval:
  - `path_len_xy ~= 0.6975m`
  - `span_x ~= 0.0834m`
  - `span_y ~= 0.5530m`

Interpretation:
- This is not a "no-plan/no-cmd" failure.
- The stack is continuously planning and commanding, but effective progress toward the goal criteria is insufficient, eventually triggering progress failure.
