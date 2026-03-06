#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_ROOT="${LOG_ROOT:-${WS_DIR}/logs/nav_runs/${RUN_ID}}"
ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_ROOT}/ros}"
STDOUT_LOG="${LOG_ROOT}/bringup.stdout.log"
KEY_ERROR_LOG="${LOG_ROOT}/key_errors.log"
SNAPSHOT_DIR="${LOG_ROOT}/snapshots"
META_LOG="${LOG_ROOT}/run_meta.txt"

MODE="${MODE:-nav}"
LOCALIZATION="${LOCALIZATION:-amcl}"
CONTROLLER="${CONTROLLER:-dwb}"
NAV_RVIZ="${NAV_RVIZ:-false}"
ENABLE_NAV2="${ENABLE_NAV2:-true}"
USE_ROBOSTACK="${USE_ROBOSTACK:-0}"

BAG_RECORD="${BAG_RECORD:-0}"
BAG_OUTPUT="${BAG_OUTPUT:-${LOG_ROOT}/bag}"
BAG_RECORD_LOG="${LOG_ROOT}/rosbag_record.log"
BAG_TOPICS="${BAG_TOPICS:-/tf /tf_static /map /scan /segmentation/obstacle /Odometry /amcl_pose /cmd_vel /cmd_vel_chassis}"

BRINGUP_PID=""
BAG_PID=""

mkdir -p "${LOG_ROOT}" "${ROS_LOG_DIR}" "${SNAPSHOT_DIR}"
export ROS_LOG_DIR

setup_ros_env() {
  if [[ "${USE_ROBOSTACK}" == "1" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "${WS_DIR}/tools/activate_robostack.sh"
    set -u
    return
  fi

  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER
  unset PYTHONPATH PYTHONHOME
  export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "${WS_DIR}/install/setup.bash"
  set -u
}

snapshot_state() {
  local tag="$1"
  local ts
  ts="$(date +%Y%m%d_%H%M%S)"
  local prefix="${SNAPSHOT_DIR}/${ts}_${tag}"

  {
    echo "timestamp: $(date -Is)"
    echo "run_id: ${RUN_ID}"
    echo "mode: ${MODE}"
    echo "localization: ${LOCALIZATION}"
    echo "controller: ${CONTROLLER}"
    echo "nav_rviz: ${NAV_RVIZ}"
    echo "enable_nav2: ${ENABLE_NAV2}"
    echo "log_root: ${LOG_ROOT}"
    echo "ros_log_dir: ${ROS_LOG_DIR}"
    echo "bag_record: ${BAG_RECORD}"
    echo "bag_output: ${BAG_OUTPUT}"
    echo "extra_launch_args: $*"
  } > "${META_LOG}"

  ROS2CLI_NO_DAEMON=1 ros2 node list > "${prefix}_nodes.txt" 2>&1 || true
  ROS2CLI_NO_DAEMON=1 ros2 topic list > "${prefix}_topics.txt" 2>&1 || true
  ROS2CLI_NO_DAEMON=1 ros2 topic info /map -v > "${prefix}_map_info.txt" 2>&1 || true
  ROS2CLI_NO_DAEMON=1 ros2 topic info /scan -v > "${prefix}_scan_info.txt" 2>&1 || true
}

cleanup() {
  set +e
  if [[ -n "${BAG_PID}" ]]; then
    kill -INT "-${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${BRINGUP_PID}" ]]; then
    kill -INT "-${BRINGUP_PID}" 2>/dev/null || true
    wait "${BRINGUP_PID}" 2>/dev/null || true
  fi

  grep -E \
    "queue is full|earlier than all the data in the transform cache|Extrapolation Error|delay is high|dropping stale|transformPoseInTargetFrame|Failed to transform" \
    "${STDOUT_LOG}" > "${KEY_ERROR_LOG}" 2>/dev/null || true

  snapshot_state "final"

  echo
  echo "================== capture_nav_logs done =================="
  echo "log_root     : ${LOG_ROOT}"
  echo "stdout_log   : ${STDOUT_LOG}"
  echo "ros_log_dir  : ${ROS_LOG_DIR}"
  echo "key_errors   : ${KEY_ERROR_LOG}"
  if [[ "${BAG_RECORD}" == "1" ]]; then
    echo "bag_output   : ${BAG_OUTPUT}"
    echo "bag_log      : ${BAG_RECORD_LOG}"
  fi
  echo "==========================================================="
}

trap cleanup EXIT INT TERM

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] missing ${WS_DIR}/install/setup.bash (build workspace first)" >&2
  exit 1
fi

setup_ros_env

echo "run_id=${RUN_ID}"
echo "log_root=${LOG_ROOT}"
echo "ros_log_dir=${ROS_LOG_DIR}"
echo "mode=${MODE} localization=${LOCALIZATION} controller=${CONTROLLER} nav_rviz=${NAV_RVIZ}"
echo "extra launch args: $*"

snapshot_state "prelaunch"

LAUNCH_ARGS=(
  "mode:=${MODE}"
  "localization:=${LOCALIZATION}"
  "controller:=${CONTROLLER}"
  "nav_rviz:=${NAV_RVIZ}"
  "enable_nav2:=${ENABLE_NAV2}"
)

(
  cd "${WS_DIR}"
  setsid ros2 launch rm_nav_bringup bringup.launch.py "${LAUNCH_ARGS[@]}" "$@"
) 2>&1 | tee "${STDOUT_LOG}" &
BRINGUP_PID=$!

if [[ "${BAG_RECORD}" == "1" ]]; then
  (
    cd "${WS_DIR}"
    setsid ros2 bag record -o "${BAG_OUTPUT}" ${BAG_TOPICS}
  ) > "${BAG_RECORD_LOG}" 2>&1 &
  BAG_PID=$!
fi

wait "${BRINGUP_PID}"
