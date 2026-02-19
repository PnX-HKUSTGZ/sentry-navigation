#!/usr/bin/env bash
set -euo pipefail

# Multi-goal stress test for dynamic-obstacle-oriented Nav2 tuning.
# Optional launch mode keeps the single entrypoint requirement:
#   ros2 launch rm_nav_bringup bringup.launch.py

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ROUNDS="${ROUNDS:-2}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-90}"
BETWEEN_GOALS_SEC="${BETWEEN_GOALS_SEC:-1}"
LAUNCH_BRINGUP="${LAUNCH_BRINGUP:-0}"
LOCALIZATION="${LOCALIZATION:-amcl}"
NAV_RVIZ="${NAV_RVIZ:-false}"
DRY_RUN="${DRY_RUN:-0}"
USE_ROBOSTACK="${USE_ROBOSTACK:-0}"
ACTION_NAME="${ACTION_NAME:-}"
CONTROLLER="${CONTROLLER:-}"
NAV_START_DELAY="${NAV_START_DELAY:-}"
STARTUP_SETTLE_SEC="${STARTUP_SETTLE_SEC:-3}"
ACTION_WAIT_TIMEOUT="${ACTION_WAIT_TIMEOUT:-90}"
MAP_TF_WAIT_TIMEOUT="${MAP_TF_WAIT_TIMEOUT:-45}"

INIT_X="${INIT_X:--5.0}"
INIT_Y="${INIT_Y:-3.0}"
INIT_QZ="${INIT_QZ:-0.0}"
INIT_QW="${INIT_QW:-1.0}"

# x,y,qz,qw; x,y,qz,qw; ...
WAYPOINTS="${WAYPOINTS:--5.0,3.0,0.0,1.0; -2.0,2.0,0.0,1.0; 0.0,0.0,0.0,1.0; 1.8,-1.2,0.0,1.0; -0.8,0.8,0.0,1.0; -5.0,3.0,0.0,1.0}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${ARTIFACT_DIR:-${WS_DIR}/artifacts/nav_stress_${TIMESTAMP}}"
mkdir -p "${ARTIFACT_DIR}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog_nav_stress_${TIMESTAMP}}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] Missing ${WS_DIR}/install/setup.bash. Build workspace first." >&2
  exit 1
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  echo "[DRY_RUN] stress_dynamic_nav.sh"
  echo "  rounds=${ROUNDS}"
  echo "  goal_timeout=${GOAL_TIMEOUT}s"
  echo "  launch_bringup=${LAUNCH_BRINGUP}"
  echo "  use_robostack=${USE_ROBOSTACK}"
  echo "  localization=${LOCALIZATION}"
  echo "  controller=${CONTROLLER:-<default>}"
  echo "  nav_start_delay=${NAV_START_DELAY:-<auto>}"
  echo "  action_wait_timeout=${ACTION_WAIT_TIMEOUT}s"
  echo "  map_tf_wait_timeout=${MAP_TF_WAIT_TIMEOUT}s"
  echo "  nav_rviz=${NAV_RVIZ}"
  echo "  init_pose=(${INIT_X}, ${INIT_Y}, ${INIT_QZ}, ${INIT_QW})"
  echo "  waypoints=${WAYPOINTS}"
  echo "  artifacts=${ARTIFACT_DIR}"
  echo "  ros_log_dir=${ROS_LOG_DIR}"
  exit 0
fi

setup_ros_env() {
  if [[ "${USE_ROBOSTACK}" == "1" ]]; then
    local activate_script="${WS_DIR}/tools/activate_robostack.sh"
    if [[ ! -f "${activate_script}" ]]; then
      echo "[ERROR] USE_ROBOSTACK=1 but missing ${activate_script}" >&2
      exit 6
    fi
    set +u
    # shellcheck source=/dev/null
    source "${activate_script}"
    set -u
    return
  fi

  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER
  unset PYTHONPATH PYTHONHOME
  export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

  FASTDDS_PROFILE_FILE="${FASTDDS_PROFILE_FILE:-/tmp/fastdds_nav_stress_${TIMESTAMP}.xml}"
  cat > "${FASTDDS_PROFILE_FILE}" << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>UDPv4Transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="no_shm_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>UDPv4Transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_PROFILE_FILE}"
  export RMW_FASTRTPS_USE_SHM=0

  set +u
  source /opt/ros/humble/setup.bash
  source "${WS_DIR}/install/setup.bash"
  set -u
}

setup_ros_env

# Refresh ros2 daemon to avoid stale graph cache during rapid relaunch loops.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

BRINGUP_PID=""

cleanup() {
  if [[ -n "${BRINGUP_PID}" ]] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
    echo "[cleanup] stopping bringup pid=${BRINGUP_PID}" >&2
    kill "${BRINGUP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

wait_for_action() {
  local timeout_s="$1"
  local loops=$((timeout_s * 2))
  local i
  for ((i = 1; i <= loops; i++)); do
    if [[ -n "${ACTION_NAME}" ]]; then
      if ros2 action list 2>/dev/null | grep -qx "${ACTION_NAME}"; then
        return 0
      fi
    else
      ACTION_NAME="$(ros2 action list 2>/dev/null | grep -E '(^|/)navigate_to_pose$' | head -n 1 || true)"
      if [[ -n "${ACTION_NAME}" ]]; then
        return 0
      fi
    fi
    sleep 0.5
  done
  return 1
}

wait_for_amcl_pose() {
  local timeout_s="$1"
  local loops=$((timeout_s * 4))
  local i
  for ((i = 1; i <= loops; i++)); do
    if timeout 1 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

wait_for_map_tf() {
  local timeout_s="$1"
  local loops=$((timeout_s * 2))
  local i
  for ((i = 1; i <= loops; i++)); do
    if timeout 1 ros2 topic echo /tf --once 2>/dev/null | grep -q "frame_id: map"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

publish_initialpose() {
  timeout 4 ros2 topic pub --rate 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, pose: {pose: {position: {x: ${INIT_X}, y: ${INIT_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${INIT_QZ}, w: ${INIT_QW}}}}}" \
    --qos-reliability best_effort --qos-durability volatile >/dev/null 2>&1 || true
}

publish_initialpose_once() {
  timeout 3 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, pose: {pose: {position: {x: ${INIT_X}, y: ${INIT_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${INIT_QZ}, w: ${INIT_QW}}}}}" \
    --qos-reliability best_effort --qos-durability volatile >/dev/null 2>&1 || true
}

if [[ "${LAUNCH_BRINGUP}" == "1" ]]; then
  if ros2 node list 2>/dev/null | grep -q .; then
    echo "[ERROR] ROS graph is not clean while LAUNCH_BRINGUP=1." >&2
    ros2 node list 2>/dev/null | sed 's/^/  /' >&2
    exit 2
  fi
  echo "[1/5] Launch bringup (single entrypoint)"
  launch_cmd=(ros2 launch rm_nav_bringup bringup.launch.py "localization:=${LOCALIZATION}" "nav_rviz:=${NAV_RVIZ}")
  if [[ -n "${CONTROLLER}" ]]; then
    launch_cmd+=("controller:=${CONTROLLER}")
  fi
  if [[ -n "${NAV_START_DELAY}" ]]; then
    launch_cmd+=("nav_start_delay:=${NAV_START_DELAY}")
  fi
  "${launch_cmd[@]}" > "${ARTIFACT_DIR}/bringup.log" 2>&1 &
  BRINGUP_PID=$!
fi

echo "[2/5] Publish /initialpose (BEST_EFFORT + VOLATILE)"
publish_initialpose
if wait_for_amcl_pose 8; then
  echo "[INFO] /amcl_pose detected."
else
  echo "[WARN] /amcl_pose not observed yet; continue waiting for Nav2 action server." >&2
fi

echo "[3/5] Wait for /navigate_to_pose action server"
if ! wait_for_action "${ACTION_WAIT_TIMEOUT}"; then
  if [[ "${LOCALIZATION}" == "icp" || "${LOCALIZATION}" == "small_gicp" ]]; then
    echo "[WARN] Action server not ready; re-publish /initialpose burst and retry..." >&2
    publish_initialpose
    if ! wait_for_action 60; then
      echo "[ERROR] /navigate_to_pose not available after initialpose retry" >&2
      exit 3
    fi
  else
    echo "[ERROR] /navigate_to_pose not available" >&2
    exit 3
  fi
fi
echo "[INFO] Action server: ${ACTION_NAME}"
# Some localization nodes (e.g. icp/small_gicp) are launched with delay.
# Re-publish here so delayed subscribers can still receive it.
if [[ "${LOCALIZATION}" == "icp" || "${LOCALIZATION}" == "small_gicp" ]]; then
  echo "[INFO] Re-publish /initialpose once for delayed localization nodes"
  publish_initialpose_once
fi
if [[ "${STARTUP_SETTLE_SEC}" != "0" ]]; then
  echo "[INFO] Settling ${STARTUP_SETTLE_SEC}s for lifecycle stabilization..."
  sleep "${STARTUP_SETTLE_SEC}"
fi
if [[ "${LOCALIZATION}" == "icp" || "${LOCALIZATION}" == "small_gicp" ]]; then
  echo "[INFO] Wait for map TF (frame_id=map), timeout=${MAP_TF_WAIT_TIMEOUT}s"
  if wait_for_map_tf "${MAP_TF_WAIT_TIMEOUT}"; then
    echo "[INFO] map TF detected."
  else
    echo "[WARN] map TF not detected before timeout; continuing to send goals." >&2
  fi
fi

IFS=';' read -r -a WAYPOINT_ARRAY <<< "${WAYPOINTS}"
if [[ "${#WAYPOINT_ARRAY[@]}" -eq 0 ]]; then
  echo "[ERROR] WAYPOINTS is empty" >&2
  exit 5
fi

echo "[4/5] Start stress run"
echo "  rounds=${ROUNDS}"
echo "  goal_timeout=${GOAL_TIMEOUT}s"
echo "  between_goals=${BETWEEN_GOALS_SEC}s"
echo "  artifacts=${ARTIFACT_DIR}"

total=0
succeeded=0
aborted=0
timeout_cnt=0
other_fail=0
duration_sum=0

for ((round = 1; round <= ROUNDS; round++)); do
  echo "== Round ${round}/${ROUNDS} =="
  for idx in "${!WAYPOINT_ARRAY[@]}"; do
    wp_raw="${WAYPOINT_ARRAY[$idx]}"
    wp="$(echo "${wp_raw}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
    if [[ -z "${wp}" ]]; then
      continue
    fi

    IFS=',' read -r gx gy gqz gqw <<< "${wp}"
    gqz="${gqz:-0.0}"
    gqw="${gqw:-1.0}"

    total=$((total + 1))
    goal_log="${ARTIFACT_DIR}/goal_r${round}_p$((idx + 1)).log"
    start_ns="$(date +%s%N)"

    set +e
    timeout --signal=TERM --kill-after=5 "${GOAL_TIMEOUT}" ros2 action send_goal "${ACTION_NAME}" nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: ${gx}, y: ${gy}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${gqz}, w: ${gqw}}}}}" \
      > "${goal_log}" 2>&1
    rc=$?
    set -e

    end_ns="$(date +%s%N)"
    duration_sec="$(awk "BEGIN{printf \"%.3f\", (${end_ns}-${start_ns})/1000000000}")"
    duration_sum="$(awk "BEGIN{printf \"%.3f\", ${duration_sum}+${duration_sec}}")"

    status_line="$(grep -E 'Goal finished with status:' "${goal_log}" | tail -n 1 || true)"
    if [[ "${rc}" -eq 124 ]]; then
      timeout_cnt=$((timeout_cnt + 1))
      result="TIMEOUT"
    elif [[ "${status_line}" == *"SUCCEEDED"* ]]; then
      succeeded=$((succeeded + 1))
      result="SUCCEEDED"
    elif [[ "${status_line}" == *"ABORTED"* ]]; then
      aborted=$((aborted + 1))
      result="ABORTED"
    else
      other_fail=$((other_fail + 1))
      result="FAILED"
    fi

    echo "[goal ${total}] (${gx}, ${gy}) -> ${result} (${duration_sec}s)"
    sleep "${BETWEEN_GOALS_SEC}"
  done
done

avg_duration="$(awk "BEGIN{if (${total}>0) printf \"%.3f\", ${duration_sum}/${total}; else print \"0.000\"}")"
success_rate="$(awk "BEGIN{if (${total}>0) printf \"%.1f\", 100.0*${succeeded}/${total}; else print \"0.0\"}")"

summary_file="${ARTIFACT_DIR}/summary.txt"
cat > "${summary_file}" <<EOF
total_goals=${total}
succeeded=${succeeded}
aborted=${aborted}
timeout=${timeout_cnt}
other_fail=${other_fail}
success_rate_percent=${success_rate}
avg_duration_sec=${avg_duration}
EOF

echo "[5/5] Summary"
cat "${summary_file}"
echo "Logs: ${ARTIFACT_DIR}"

if [[ "${succeeded}" -ne "${total}" ]]; then
  exit 10
fi
