#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ACTIVATE_SCRIPT="${WS_DIR}/tools/activate_robostack.sh"
ENV_NAME="${ROBOSTACK_ENV_NAME:-sentry-rs-humble}"
ENV_PREFIX_DEFAULT="${WS_DIR}/.conda/${ENV_NAME}"
ENV_PREFIX="${ROBOSTACK_ENV_PREFIX:-${ENV_PREFIX_DEFAULT}}"

FAILED=0
WARNED=0
ACTIVE_OK=0

pass() {
  echo "[PASS] $*"
}

warn() {
  echo "[WARN] $*"
  WARNED=$((WARNED + 1))
}

fail() {
  echo "[FAIL] $*"
  FAILED=$((FAILED + 1))
}

echo "[INFO] Checking RoboStack environment at: ${ENV_PREFIX}"

if [ ! -f "${ACTIVATE_SCRIPT}" ]; then
  fail "Missing activate script: ${ACTIVATE_SCRIPT}"
fi

if [ ! -d "${ENV_PREFIX}" ]; then
  fail "Missing conda env dir: ${ENV_PREFIX}"
fi

if ! command -v conda >/dev/null 2>&1; then
  fail "conda command not found"
fi

if [ "${FAILED}" -eq 0 ]; then
  # shellcheck source=/dev/null
  if source "${ACTIVATE_SCRIPT}" >/dev/null 2>&1; then
    pass "activate_robostack.sh executed successfully"
    ACTIVE_OK=1
  else
    fail "Failed to source ${ACTIVATE_SCRIPT}"
  fi
fi

if [ "${ACTIVE_OK}" -eq 1 ]; then
  if command -v ros2 >/dev/null 2>&1; then
    pass "ros2 is available: $(command -v ros2)"
  else
    fail "ros2 command not found after activation"
  fi

  if python -c "import rclpy" >/dev/null 2>&1; then
    pass "python can import rclpy"
  else
    fail "python cannot import rclpy"
  fi

  if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    pass "foxglove_bridge package is available"
  else
    fail "foxglove_bridge package not found"
  fi

  if [ "${ROS_DISTRO:-}" = "humble" ]; then
    pass "ROS_DISTRO is humble"
  else
    fail "ROS_DISTRO is '${ROS_DISTRO:-unset}', expected 'humble'"
  fi

  if [[ ":${AMENT_PREFIX_PATH:-}:" == *":${CONDA_PREFIX:-}:"* ]]; then
    pass "AMENT_PREFIX_PATH includes conda env"
  else
    fail "AMENT_PREFIX_PATH does not include conda env"
  fi

  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    if [[ ":${AMENT_PREFIX_PATH:-}:" == *":${WS_DIR}/install:"* ]]; then
      pass "workspace overlay is in AMENT_PREFIX_PATH"
    else
      warn "workspace overlay exists but is not in AMENT_PREFIX_PATH"
    fi
  else
    warn "workspace install/setup.bash not found (build may be skipped)"
  fi

  if [[ "${CMAKE_PREFIX_PATH:-}" == *"/opt/ros/"* ]] || [[ "${AMENT_PREFIX_PATH:-}" == *"/opt/ros/"* ]]; then
    warn "Detected /opt/ros path in prefix variables; mixed environment risk"
  else
    pass "no /opt/ros path detected in active prefix variables"
  fi
else
  warn "Skipped detailed checks because activation failed."
fi

echo
echo "[INFO] Summary: failed=${FAILED}, warned=${WARNED}"
if [ "${FAILED}" -ne 0 ]; then
  echo "[INFO] Fix suggestion: bash tools/setup_robostack_env.sh && source tools/activate_robostack.sh"
  exit 1
fi

echo "[DONE] RoboStack environment check passed."
