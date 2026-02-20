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
LIDAR_NOISE_STDDEV="${LIDAR_NOISE_STDDEV:-}"
STARTUP_SETTLE_SEC="${STARTUP_SETTLE_SEC:-3}"
ACTION_WAIT_TIMEOUT="${ACTION_WAIT_TIMEOUT:-90}"
AMCL_POSE_WAIT_TIMEOUT="${AMCL_POSE_WAIT_TIMEOUT:-25}"
ODOM_WAIT_TIMEOUT="${ODOM_WAIT_TIMEOUT:-35}"
ODOM_TOPIC="${ODOM_TOPIC:-auto}"  # auto => probe /Odometry -> /ground_truth/odom -> /odom
ODOM_TOPIC_CANDIDATES="${ODOM_TOPIC_CANDIDATES:-/Odometry /ground_truth/odom /odom}"
MAP_TF_WAIT_TIMEOUT="${MAP_TF_WAIT_TIMEOUT:-45}"
MAP_TF_STABLE_SAMPLES="${MAP_TF_STABLE_SAMPLES:-3}"
MAP_TF_REQUIRED="${MAP_TF_REQUIRED:-}"
BRINGUP_WORLD="${BRINGUP_WORLD:-}"
BRINGUP_MAP="${BRINGUP_MAP:-}"
BRINGUP_LIO="${BRINGUP_LIO:-}"
CLEAR_COSTMAP_BEFORE_GOAL="${CLEAR_COSTMAP_BEFORE_GOAL:-0}"
GOAL_OCCUPANCY_POLICY="${GOAL_OCCUPANCY_POLICY:-reject}"  # off|warn|reject|snap
GOAL_NEAREST_RADIUS="${GOAL_NEAREST_RADIUS:-1.5}"
MAP_YAML_PATH="${MAP_YAML_PATH:-}"

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
  echo "  world=${BRINGUP_WORLD:-<config default>}"
  echo "  map=${BRINGUP_MAP:-<config/world default>}"
  echo "  lio=${BRINGUP_LIO:-<config default>}"
  echo "  clear_costmap_before_goal=${CLEAR_COSTMAP_BEFORE_GOAL}"
  echo "  goal_occupancy_policy=${GOAL_OCCUPANCY_POLICY}"
  echo "  goal_nearest_radius=${GOAL_NEAREST_RADIUS}m"
  echo "  map_yaml_path=${MAP_YAML_PATH:-<auto>}"
  echo "  controller=${CONTROLLER:-<default>}"
  echo "  nav_start_delay=${NAV_START_DELAY:-<auto>}"
  echo "  lidar_noise_stddev=${LIDAR_NOISE_STDDEV:-<config default>}"
  echo "  action_wait_timeout=${ACTION_WAIT_TIMEOUT}s"
  echo "  amcl_pose_wait_timeout=${AMCL_POSE_WAIT_TIMEOUT}s"
  echo "  odom_wait_timeout=${ODOM_WAIT_TIMEOUT}s"
  echo "  odom_topic=${ODOM_TOPIC} (candidates: ${ODOM_TOPIC_CANDIDATES})"
  echo "  map_tf_wait_timeout=${MAP_TF_WAIT_TIMEOUT}s"
  echo "  map_tf_stable_samples=${MAP_TF_STABLE_SAMPLES}"
  echo "  map_tf_required=${MAP_TF_REQUIRED:-<auto>}"
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

if [[ -z "${MAP_TF_REQUIRED}" ]]; then
  if [[ "${LOCALIZATION}" == "icp" || "${LOCALIZATION}" == "small_gicp" ]]; then
    MAP_TF_REQUIRED=1
  else
    MAP_TF_REQUIRED=0
  fi
fi

# Validate occupancy policy early.
case "${GOAL_OCCUPANCY_POLICY}" in
  off|warn|reject|snap) ;;
  *)
    echo "[ERROR] invalid GOAL_OCCUPANCY_POLICY=${GOAL_OCCUPANCY_POLICY}, expected one of: off|warn|reject|snap" >&2
    exit 7
    ;;
esac

# Refresh ros2 daemon to avoid stale graph cache during rapid relaunch loops.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

BRINGUP_PID=""

cleanup() {
  if [[ -n "${BRINGUP_PID}" ]] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
    echo "[cleanup] stopping bringup process group pgid=${BRINGUP_PID}" >&2
    # bringup is launched with setsid, so killing by negative pid cleans all children.
    kill -- "-${BRINGUP_PID}" 2>/dev/null || kill "${BRINGUP_PID}" 2>/dev/null || true
    sleep 1
    kill -9 -- "-${BRINGUP_PID}" 2>/dev/null || kill -9 "${BRINGUP_PID}" 2>/dev/null || true
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

wait_for_lifecycle_active() {
  local node="$1"
  local timeout_s="$2"
  local loops=$((timeout_s * 2))
  local i
  for ((i = 1; i <= loops; i++)); do
    if timeout 2 ros2 lifecycle get "${node}" 2>/dev/null | grep -qi "active"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

wait_for_topic_once() {
  local topic="$1"
  local timeout_s="$2"
  local loops=$((timeout_s * 2))
  local i
  for ((i = 1; i <= loops; i++)); do
    if timeout 1 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

topic_has_publisher_once() {
  local topic="$1"
  timeout 1 ros2 topic info "${topic}" -v 2>/dev/null \
    | awk '/Publisher count:/ {if ($3+0 > 0) found=1} END {exit(found?0:1)}'
}

wait_for_topic_publisher() {
  local topic="$1"
  local timeout_s="$2"
  local loops=$((timeout_s * 2))
  local i
  for ((i = 1; i <= loops; i++)); do
    if topic_has_publisher_once "${topic}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

resolve_odom_topic_auto() {
  local timeout_s="$1"
  local loops=$((timeout_s * 2))
  local i
  local cand
  for ((i = 1; i <= loops; i++)); do
    for cand in ${ODOM_TOPIC_CANDIDATES}; do
      if topic_has_publisher_once "${cand}"; then
        ODOM_TOPIC="${cand}"
        return 0
      fi
    done
    sleep 0.5
  done
  return 1
}

has_tf_pair_once() {
  local parent="$1"
  local child="$2"
  timeout 2 bash -lc "ros2 topic echo /tf 2>/dev/null | awk -v p='${parent}' -v c='${child}' '
    \$1==\"frame_id:\" {gsub(/\\\"/, \"\", \$2); frame=\$2}
    \$1==\"child_frame_id:\" {
      gsub(/\\\"/, \"\", \$2); ch=\$2
      if (frame==p && ch==c) {found=1; exit 0}
    }
    END {exit(found?0:1)}
  '"
}

has_odom_base_tf_once() {
  has_tf_pair_once "odom" "base_link" || has_tf_pair_once "odom" "base_link_fake"
}

wait_for_odom_base_tf_stable() {
  local timeout_s="$1"
  local stable_samples="$2"
  local loops=$((timeout_s * 2))
  local consecutive=0
  local i
  for ((i = 1; i <= loops; i++)); do
    if has_odom_base_tf_once; then
      consecutive=$((consecutive + 1))
      if [[ "${consecutive}" -ge "${stable_samples}" ]]; then
        return 0
      fi
    else
      consecutive=0
    fi
    sleep 0.5
  done
  return 1
}

has_map_odom_tf_once() {
  has_tf_pair_once "map" "odom"
}

wait_for_map_tf_stable() {
  local timeout_s="$1"
  local stable_samples="$2"
  local loops=$((timeout_s * 2))
  local consecutive=0
  local i
  for ((i = 1; i <= loops; i++)); do
    if has_map_odom_tf_once; then
      consecutive=$((consecutive + 1))
      if [[ "${consecutive}" -ge "${stable_samples}" ]]; then
        return 0
      fi
    else
      consecutive=0
    fi
    sleep 0.5
  done
  return 1
}

clear_costmaps() {
  # Clear costmaps to reduce stale obstacle influence between consecutive goals.
  local srv
  for srv in \
    "/local_costmap/clear_entirely_local_costmap" \
    "/global_costmap/clear_entirely_global_costmap"; do
    if ros2 service list 2>/dev/null | grep -qx "${srv}"; then
      timeout 3 ros2 service call "${srv}" nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1 || true
    fi
  done
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

get_param_value() {
  local node="$1"
  local param="$2"
  local raw
  raw="$(timeout 3 ros2 param get "${node}" "${param}" 2>/dev/null || true)"
  echo "${raw}" | sed -nE 's/^String value is: (.*)$/\1/p; s/^value: (.*)$/\1/p' | tail -n1 | xargs
}

resolve_map_yaml_path() {
  if [[ -n "${MAP_YAML_PATH}" && -f "${MAP_YAML_PATH}" ]]; then
    return 0
  fi
  if [[ -n "${BRINGUP_MAP}" ]]; then
    local p1="${WS_DIR}/src/rm_nav_bringup/map/${BRINGUP_MAP}.yaml"
    local p2="${WS_DIR}/install/rm_nav_bringup/share/rm_nav_bringup/map/${BRINGUP_MAP}.yaml"
    if [[ -f "${p1}" ]]; then
      MAP_YAML_PATH="${p1}"
      return 0
    fi
    if [[ -f "${p2}" ]]; then
      MAP_YAML_PATH="${p2}"
      return 0
    fi
  fi
  local from_param
  from_param="$(get_param_value /map_server yaml_filename)"
  if [[ -n "${from_param}" && -f "${from_param}" ]]; then
    MAP_YAML_PATH="${from_param}"
    return 0
  fi
  return 1
}

probe_goal_occupancy() {
  local x="$1"
  local y="$2"
  python3 "${WS_DIR}/tools/map_goal_probe.py" \
    --map-yaml "${MAP_YAML_PATH}" \
    --x "${x}" \
    --y "${y}" \
    --max-nearest-radius "${GOAL_NEAREST_RADIUS}"
}

if [[ "${LAUNCH_BRINGUP}" == "1" ]]; then
  if ros2 node list 2>/dev/null | grep -q .; then
    echo "[ERROR] ROS graph is not clean while LAUNCH_BRINGUP=1." >&2
    ros2 node list 2>/dev/null | sed 's/^/  /' >&2
    exit 2
  fi
  echo "[1/5] Launch bringup (single entrypoint)"
  launch_cmd=(ros2 launch rm_nav_bringup bringup.launch.py "localization:=${LOCALIZATION}" "nav_rviz:=${NAV_RVIZ}")
  if [[ -n "${BRINGUP_WORLD}" ]]; then
    launch_cmd+=("world:=${BRINGUP_WORLD}")
  fi
  if [[ -n "${BRINGUP_MAP}" ]]; then
    launch_cmd+=("map:=${BRINGUP_MAP}")
  fi
  if [[ -n "${BRINGUP_LIO}" ]]; then
    launch_cmd+=("lio:=${BRINGUP_LIO}")
  fi
  if [[ -n "${CONTROLLER}" ]]; then
    launch_cmd+=("controller:=${CONTROLLER}")
  fi
  if [[ -n "${NAV_START_DELAY}" ]]; then
    launch_cmd+=("nav_start_delay:=${NAV_START_DELAY}")
  fi
  if [[ -n "${LIDAR_NOISE_STDDEV}" ]]; then
    launch_cmd+=("lidar_noise_stddev:=${LIDAR_NOISE_STDDEV}")
  fi
  setsid "${launch_cmd[@]}" > "${ARTIFACT_DIR}/bringup.log" 2>&1 &
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

if [[ "${ODOM_TOPIC}" == "auto" ]]; then
  echo "[INFO] Auto-detect odometry topic (${ODOM_TOPIC_CANDIDATES}), timeout=${ODOM_WAIT_TIMEOUT}s"
  if resolve_odom_topic_auto "${ODOM_WAIT_TIMEOUT}"; then
    echo "[INFO] odometry topic detected: ${ODOM_TOPIC}"
  else
    echo "[ERROR] failed to detect any odometry topic with live messages from candidates: ${ODOM_TOPIC_CANDIDATES}" >&2
    exit 11
  fi
else
  echo "[INFO] Wait for odometry signal (${ODOM_TOPIC}), timeout=${ODOM_WAIT_TIMEOUT}s"
  if wait_for_topic_publisher "${ODOM_TOPIC}" "${ODOM_WAIT_TIMEOUT}"; then
    echo "[INFO] ${ODOM_TOPIC} publisher detected."
  else
    echo "[ERROR] ${ODOM_TOPIC} has no publisher; localization/odom chain not ready." >&2
    exit 11
  fi
fi

echo "[INFO] Wait for stable odom->base_link TF"
if wait_for_odom_base_tf_stable "${ODOM_WAIT_TIMEOUT}" 3; then
  echo "[INFO] stable odom->base_link TF detected."
else
  echo "[ERROR] odom->base_link TF not stable in time." >&2
  exit 12
fi

if [[ "${LOCALIZATION}" == "amcl" ]]; then
  if ! wait_for_amcl_pose "${AMCL_POSE_WAIT_TIMEOUT}"; then
    echo "[WARN] /amcl_pose still missing after ${AMCL_POSE_WAIT_TIMEOUT}s; re-publish /initialpose burst and retry..." >&2
    publish_initialpose
    if ! wait_for_amcl_pose "${AMCL_POSE_WAIT_TIMEOUT}"; then
      echo "[ERROR] /amcl_pose not available; localization not initialized." >&2
      exit 8
    fi
  fi
  echo "[INFO] /amcl_pose ready."

  echo "[INFO] Wait for Nav2 lifecycle nodes active (bt_navigator/controller_server)"
  if ! wait_for_lifecycle_active "/bt_navigator" 40; then
    echo "[ERROR] /bt_navigator not ACTIVE in time." >&2
    exit 9
  fi
  if ! wait_for_lifecycle_active "/controller_server" 40; then
    echo "[ERROR] /controller_server not ACTIVE in time." >&2
    exit 9
  fi
fi
if [[ "${STARTUP_SETTLE_SEC}" != "0" ]]; then
  echo "[INFO] Settling ${STARTUP_SETTLE_SEC}s for lifecycle stabilization..."
  sleep "${STARTUP_SETTLE_SEC}"
fi
if [[ "${LOCALIZATION}" == "icp" || "${LOCALIZATION}" == "small_gicp" ]]; then
  echo "[INFO] Wait for stable map->odom TF, timeout=${MAP_TF_WAIT_TIMEOUT}s, required_samples=${MAP_TF_STABLE_SAMPLES}"
  if wait_for_map_tf_stable "${MAP_TF_WAIT_TIMEOUT}" "${MAP_TF_STABLE_SAMPLES}"; then
    echo "[INFO] stable map->odom TF detected."
  else
    if [[ "${MAP_TF_REQUIRED}" == "1" ]]; then
      echo "[ERROR] stable map->odom TF not detected before timeout." >&2
      exit 4
    fi
    echo "[WARN] stable map->odom TF not detected before timeout; continuing by policy." >&2
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
echo "  goal_occupancy_policy=${GOAL_OCCUPANCY_POLICY}"
echo "  goal_nearest_radius=${GOAL_NEAREST_RADIUS}m"

invalid_goal_cnt=0
if [[ "${GOAL_OCCUPANCY_POLICY}" != "off" ]]; then
  if resolve_map_yaml_path; then
    echo "  map_yaml=${MAP_YAML_PATH}"
  else
    echo "[WARN] goal occupancy checking enabled but map yaml not resolved, disable check." >&2
    GOAL_OCCUPANCY_POLICY="off"
  fi
fi

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

    if [[ "${GOAL_OCCUPANCY_POLICY}" != "off" ]]; then
      probe_out="$(probe_goal_occupancy "${gx}" "${gy}" 2>&1 || true)"
      goal_status="$(echo "${probe_out}" | awk -F= '/^status=/{print $2}' | tail -n1)"
      nearest_x="$(echo "${probe_out}" | awk -F= '/^nearest_free_x=/{print $2}' | tail -n1)"
      nearest_y="$(echo "${probe_out}" | awk -F= '/^nearest_free_y=/{print $2}' | tail -n1)"
      nearest_d="$(echo "${probe_out}" | awk -F= '/^nearest_free_dist=/{print $2}' | tail -n1)"
      if [[ -z "${goal_status}" ]]; then
        echo "[WARN] failed to probe goal occupancy at (${gx}, ${gy}), continue as-is"
      elif [[ "${goal_status}" != "FREE" ]]; then
        case "${GOAL_OCCUPANCY_POLICY}" in
          warn)
            echo "[WARN] goal (${gx}, ${gy}) status=${goal_status}; nearest free=(${nearest_x}, ${nearest_y}) d=${nearest_d}m"
            ;;
          reject)
            invalid_goal_cnt=$((invalid_goal_cnt + 1))
            echo "[goal skip] (${gx}, ${gy}) status=${goal_status}; nearest free=(${nearest_x}, ${nearest_y}) d=${nearest_d}m"
            continue
            ;;
          snap)
            if [[ -n "${nearest_x}" && -n "${nearest_y}" ]]; then
              echo "[goal snap] (${gx}, ${gy}) status=${goal_status} -> (${nearest_x}, ${nearest_y}) d=${nearest_d}m"
              gx="${nearest_x}"
              gy="${nearest_y}"
            else
              invalid_goal_cnt=$((invalid_goal_cnt + 1))
              echo "[goal skip] (${gx}, ${gy}) status=${goal_status}; no nearest free found within ${GOAL_NEAREST_RADIUS}m"
              continue
            fi
            ;;
          *)
            echo "[WARN] unknown GOAL_OCCUPANCY_POLICY=${GOAL_OCCUPANCY_POLICY}, treat as warn"
            ;;
        esac
      fi
    fi

    if [[ "${CLEAR_COSTMAP_BEFORE_GOAL}" == "1" && "${total}" -gt 0 ]]; then
      clear_costmaps
    fi

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
invalid_goal_skipped=${invalid_goal_cnt}
success_rate_percent=${success_rate}
avg_duration_sec=${avg_duration}
EOF

echo "[5/5] Summary"
cat "${summary_file}"
echo "Logs: ${ARTIFACT_DIR}"

if [[ "${succeeded}" -ne "${total}" ]]; then
  exit 10
fi
