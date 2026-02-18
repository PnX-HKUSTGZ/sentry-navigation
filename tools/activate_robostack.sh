#!/usr/bin/env bash
set -euo pipefail

_is_sourced=0
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
  _is_sourced=1
fi

_die() {
  echo "[ERROR] $*" >&2
  if [ "${_is_sourced}" = "1" ]; then
    return 1
  fi
  exit 1
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_NAME="${ROBOSTACK_ENV_NAME:-sentry-rs-humble}"
ENV_PREFIX_DEFAULT="${WS_DIR}/.conda/${ENV_NAME}"
ENV_PREFIX="${ROBOSTACK_ENV_PREFIX:-${ENV_PREFIX_DEFAULT}}"

if ! command -v conda >/dev/null 2>&1; then
  _die "conda not found. Please install Miniconda/Anaconda first."
fi

if [ ! -d "${ENV_PREFIX}" ]; then
  _die "RoboStack env not found at ${ENV_PREFIX}. Run tools/setup_robostack_env.sh first."
fi

eval "$(conda shell.bash hook)"
conda activate "${ENV_PREFIX}"

# Keep this shell clean from system ROS overlays before sourcing RoboStack.
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset ROS_ETC_DIR
unset ROS_VERSION
unset ROS_PYTHON_VERSION
unset ROS_DISTRO
unset RMW_IMPLEMENTATION

if [ -f "${CONDA_PREFIX}/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${CONDA_PREFIX}/setup.bash"
else
  _die "Missing setup.bash in ${CONDA_PREFIX}. Environment may be incomplete."
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${WS_DIR}/install/setup.bash"
fi

echo "[INFO] Active ROS distro: ${ROS_DISTRO:-unknown}"
echo "[INFO] Active conda env : ${CONDA_PREFIX}"
echo "[INFO] Workspace overlay: ${WS_DIR}/install/setup.bash"

