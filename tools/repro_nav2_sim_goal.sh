#!/usr/bin/env bash
set -euo pipefail

# Repro script: headless Gazebo + AMCL + Nav2 NavigateToPose
# Single entrypoint: rm_nav_bringup/bringup.launch.py

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

GOAL_X="${1:-3.85}"
GOAL_Y="${2:--3.65}"
GOAL_QZ="${3:-0.0}"
GOAL_QW="${4:-1.0}"
CONTROLLER="${CONTROLLER:-}"
STARTUP_SETTLE_SEC="${STARTUP_SETTLE_SEC:-3}"

# RMUL_26 default spawn pose (from pb_rm_simulation)
INIT_X="${INIT_X:--5.375}"
INIT_Y="${INIT_Y:-3.425}"
INIT_QZ="${INIT_QZ:--0.2798765}"
INIT_QW="${INIT_QW:-0.9600360}"

# setup.bash in some deployments reads these vars under `set -u`.
# Provide safe defaults so the repro script is portable across shells/envs.
: "${COLCON_TRACE:=0}"
: "${AMENT_TRACE_SETUP_FILES:=0}"
: "${AMENT_PYTHON_EXECUTABLE:=/usr/bin/python3}"
: "${PYTHONPATH:=}"
: "${LD_LIBRARY_PATH:=}"
: "${AMENT_PREFIX_PATH:=}"
: "${CMAKE_PREFIX_PATH:=}"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

# 0) Require clean ROS graph (avoids duplicate nodes and non-determinism)
if ros2 node list 2>/dev/null | grep -q .; then
  echo "[ERROR] ROS graph is not clean. Existing nodes:" >&2
  ros2 node list 2>/dev/null | sed 's/^/  /' >&2
  echo "Stop existing bringup(s) then rerun." >&2
  exit 2
fi

echo "[1/5] Launch bringup (AMCL, no RViz)"
# Run in background; keep stdout/stderr visible in this terminal.
launch_cmd=(ros2 launch rm_nav_bringup bringup.launch.py localization:=amcl nav_rviz:=false)
if [[ -n "${CONTROLLER}" ]]; then
  launch_cmd+=("controller:=${CONTROLLER}")
fi
"${launch_cmd[@]}" &
BRINGUP_PID=$!

cleanup() {
  if kill -0 "${BRINGUP_PID}" 2>/dev/null; then
    echo "[cleanup] stopping bringup (pid=${BRINGUP_PID})" >&2
    kill "${BRINGUP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[2/5] Wait for /amcl and /navigate_to_pose"
# action server readiness
for i in {1..60}; do
  if ros2 action list 2>/dev/null | grep -qx "/navigate_to_pose"; then
    break
  fi
  sleep 0.5
  if [[ $i -eq 60 ]]; then
    echo "[ERROR] /navigate_to_pose not available" >&2
    exit 3
  fi
done

if [[ "${STARTUP_SETTLE_SEC}" != "0" ]]; then
  echo "[INFO] Settling ${STARTUP_SETTLE_SEC}s for lifecycle stabilization"
  sleep "${STARTUP_SETTLE_SEC}"
fi

# 3) Initialize AMCL (QoS must match AMCL subscription: BEST_EFFORT + VOLATILE)
# Publish repeatedly for a short burst.
echo "[3/5] Publish /initialpose (BEST_EFFORT+VOLATILE) at spawn pose"
# NOTE: stamp omitted (0) to avoid future/past extrapolation issues.
timeout 3 ros2 topic pub --rate 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: ${INIT_X}, y: ${INIT_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${INIT_QZ}, w: ${INIT_QW}}}}}" \
  --qos-reliability best_effort --qos-durability volatile >/dev/null || true

# Wait until AMCL starts publishing pose
for i in {1..40}; do
  if timeout 1 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
  if [[ $i -eq 40 ]]; then
    echo "[ERROR] /amcl_pose not received" >&2
    exit 4
  fi
done

echo "[4/5] Send NavigateToPose goal: (${GOAL_X}, ${GOAL_Y})"
# Don't pipe to head/tail; ros2action can crash with BrokenPipeError.
RESULT_LINE="$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${GOAL_QZ}, w: ${GOAL_QW}}}}}" \
  | grep -E "Goal finished with status:" || true)"

echo "[5/5] ${RESULT_LINE:-Goal finished with status: <UNKNOWN>}"

# Exit non-zero if not SUCCEEDED
if [[ "${RESULT_LINE}" != *"SUCCEEDED"* ]]; then
  exit 10
fi
