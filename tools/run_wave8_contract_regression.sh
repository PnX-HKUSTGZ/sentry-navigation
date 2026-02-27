#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

USE_ROBOSTACK="${USE_ROBOSTACK:-0}"
DRY_RUN="${DRY_RUN:-0}"

ARTIFACT_DIR="${ARTIFACT_DIR:-${WS_DIR}/artifacts/wave8_contract_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${ARTIFACT_DIR}"

# Wave-road trajectory contract defaults (proven lane in historical passing sample).
BRINGUP_WORLD="${BRINGUP_WORLD:-RMUL_26_WAVE}"
BRINGUP_MAP="${BRINGUP_MAP:-RMUL26_WAVE}"
LOCALIZATION="${LOCALIZATION:-amcl}"
CONTROLLER="${CONTROLLER:-dwb}"
RM_NAV_OBSTACLE_PROFILE="${RM_NAV_OBSTACLE_PROFILE:-wavepass}"
NAV_USE_STVL="${NAV_USE_STVL:-0}"
LIDAR_NOISE_STDDEV="${LIDAR_NOISE_STDDEV:-0.0}"

INIT_X="${INIT_X:--1.60}"
INIT_Y="${INIT_Y:-3.30}"
INIT_QZ="${INIT_QZ:-0.0}"
INIT_QW="${INIT_QW:-1.0}"
WAYPOINTS="${WAYPOINTS:-1.90,3.30,0,1}"

ROUNDS="${ROUNDS:-1}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-180}"
TIMEOUT_RETRY_COUNT="${TIMEOUT_RETRY_COUNT:-0}"
ABORT_RETRY_COUNT="${ABORT_RETRY_COUNT:-0}"
GOAL_OCCUPANCY_POLICY="${GOAL_OCCUPANCY_POLICY:-snap}"
GOAL_NEAREST_RADIUS="${GOAL_NEAREST_RADIUS:-2.0}"
AMCL_INIT_XY_TOL="${AMCL_INIT_XY_TOL:-0.45}"
BAG_TOPICS="${BAG_TOPICS:-/clock /tf /tf_static /amcl_pose /ground_truth/odom}"

# Contract thresholds
WAVE_X_MIN="${WAVE_X_MIN:--1.40}"
WAVE_X_MAX="${WAVE_X_MAX:-0.52}"
WAVE_Y_MIN="${WAVE_Y_MIN:-2.20}"
WAVE_Y_MAX="${WAVE_Y_MAX:-3.20}"
WAVE_WAVELENGTH="${WAVE_WAVELENGTH:-0.24}"
FIT_WINDOW_SEC="${FIT_WINDOW_SEC:-2.0}"
MIN_WAVES="${MIN_WAVES:-7.5}"
MIN_INSIDE_TIME="${MIN_INSIDE_TIME:-2.0}"
MAX_WAVE_ALIGNED_P95="${MAX_WAVE_ALIGNED_P95:-0.35}"
MAX_WAVE_ALIGNED_MAX="${MAX_WAVE_ALIGNED_MAX:-0.55}"
REQUIRE_ACTION_SUCCESS="${REQUIRE_ACTION_SUCCESS:-1}"

WORLD_FILE="${WORLD_FILE:-${WS_DIR}/src/rm_simulation/pb_rm_simulation/world/RMUL2026_world/RMUL2026_wave.world}"
STRESS_LOG="${ARTIFACT_DIR}/wave8_contract_stress.log"
CONTRACT_REPORT="${ARTIFACT_DIR}/contract_report.txt"

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

setup_ros_env

if [[ "${DRY_RUN}" == "1" ]]; then
  cat <<EOF
[DRY_RUN] run_wave8_contract_regression.sh
artifact_dir=${ARTIFACT_DIR}
world/map=${BRINGUP_WORLD}/${BRINGUP_MAP}
localization/controller=${LOCALIZATION}/${CONTROLLER}
init_pose=(${INIT_X}, ${INIT_Y}, ${INIT_QZ}, ${INIT_QW})
waypoints=${WAYPOINTS}
goal_timeout=${GOAL_TIMEOUT}s rounds=${ROUNDS}
contract: waves>=${MIN_WAVES}, inside_time>=${MIN_INSIDE_TIME}s,
          wave_aligned_p95<=${MAX_WAVE_ALIGNED_P95}m,
          wave_aligned_max<=${MAX_WAVE_ALIGNED_MAX}m,
          require_action_success=${REQUIRE_ACTION_SUCCESS}
EOF
  exit 0
fi

echo "[1/3] run stress_dynamic_nav for wave8 contract"
(
  cd "${WS_DIR}"
  RM_NAV_OBSTACLE_PROFILE="${RM_NAV_OBSTACLE_PROFILE}" \
  LAUNCH_BRINGUP=1 \
  LOCALIZATION="${LOCALIZATION}" \
  CONTROLLER="${CONTROLLER}" \
  BRINGUP_WORLD="${BRINGUP_WORLD}" \
  BRINGUP_MAP="${BRINGUP_MAP}" \
  NAV_USE_STVL="${NAV_USE_STVL}" \
  LIDAR_NOISE_STDDEV="${LIDAR_NOISE_STDDEV}" \
  INIT_X="${INIT_X}" INIT_Y="${INIT_Y}" INIT_QZ="${INIT_QZ}" INIT_QW="${INIT_QW}" \
  WAYPOINTS="${WAYPOINTS}" \
  ROUNDS="${ROUNDS}" \
  GOAL_TIMEOUT="${GOAL_TIMEOUT}" \
  TIMEOUT_RETRY_COUNT="${TIMEOUT_RETRY_COUNT}" \
  ABORT_RETRY_COUNT="${ABORT_RETRY_COUNT}" \
  GOAL_OCCUPANCY_POLICY="${GOAL_OCCUPANCY_POLICY}" \
  GOAL_NEAREST_RADIUS="${GOAL_NEAREST_RADIUS}" \
  AMCL_INIT_XY_TOL="${AMCL_INIT_XY_TOL}" \
  BAG_RECORD=1 \
  BAG_TOPICS="${BAG_TOPICS}" \
  ARTIFACT_DIR="${ARTIFACT_DIR}" \
  USE_ROBOSTACK="${USE_ROBOSTACK}" \
  bash tools/stress_dynamic_nav.sh
) | tee "${STRESS_LOG}"

if [[ ! -f "${ARTIFACT_DIR}/rosbag_dataset/rosbag_dataset_0.db3" ]]; then
  echo "[ERROR] rosbag missing: ${ARTIFACT_DIR}/rosbag_dataset/rosbag_dataset_0.db3" >&2
  exit 3
fi

echo "[2/3] analyze drift dataset"
(
  cd "${WS_DIR}"
  /usr/bin/python3 tools/analyze_amcl_drift_from_bag.py \
    --artifact "${ARTIFACT_DIR}" \
    --world "${WORLD_FILE}" \
    --source auto
) >/dev/null

CSV_PATH="${ARTIFACT_DIR}/analysis_drift/loc_vs_ground_truth.csv"
if [[ ! -f "${CSV_PATH}" ]]; then
  echo "[ERROR] missing drift csv: ${CSV_PATH}" >&2
  exit 4
fi

echo "[3/3] evaluate wave8 trajectory contract"
(
  cd "${WS_DIR}"
  /usr/bin/python3 tools/evaluate_wave8_contract.py \
    --csv "${CSV_PATH}" \
    --summary "${ARTIFACT_DIR}/summary.txt" \
    --wave-x-min "${WAVE_X_MIN}" \
    --wave-x-max "${WAVE_X_MAX}" \
    --wave-y-min "${WAVE_Y_MIN}" \
    --wave-y-max "${WAVE_Y_MAX}" \
    --wavelength "${WAVE_WAVELENGTH}" \
    --fit-window-sec "${FIT_WINDOW_SEC}" \
    --min-waves "${MIN_WAVES}" \
    --min-inside-time "${MIN_INSIDE_TIME}" \
    --max-wave-aligned-p95 "${MAX_WAVE_ALIGNED_P95}" \
    --max-wave-aligned-max "${MAX_WAVE_ALIGNED_MAX}" \
    --require-action-success "${REQUIRE_ACTION_SUCCESS}"
) | tee "${CONTRACT_REPORT}"

echo "report=${CONTRACT_REPORT}"
echo "artifact=${ARTIFACT_DIR}"
