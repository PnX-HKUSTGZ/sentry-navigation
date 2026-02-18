#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_NAME="${ROBOSTACK_ENV_NAME:-sentry-rs-humble}"
ENV_PREFIX_DEFAULT="${WS_DIR}/.conda/${ENV_NAME}"
ENV_PREFIX="${ROBOSTACK_ENV_PREFIX:-${ENV_PREFIX_DEFAULT}}"
ENV_FILE="${WS_DIR}/environment.robostack.yml"
RUN_ROSDEP="${RUN_ROSDEP:-0}"
RUN_COLCON_BUILD="${RUN_COLCON_BUILD:-1}"
ROBOSTACK_OFFLINE="${ROBOSTACK_OFFLINE:-0}"
ROBOSTACK_SOLVER="${ROBOSTACK_SOLVER:-libmamba}"

if ! command -v conda >/dev/null 2>&1; then
  echo "[ERROR] conda not found. Please install Miniconda/Anaconda first."
  exit 1
fi
CONDA_BIN="$(command -v conda)"

if [ ! -f "${ENV_FILE}" ]; then
  echo "[ERROR] Missing env file: ${ENV_FILE}"
  exit 1
fi

if [ -d "${ENV_PREFIX}" ]; then
  echo "[INFO] Reusing existing RoboStack env: ${ENV_PREFIX}"
else
  echo "[INFO] Creating RoboStack env: ${ENV_PREFIX}"
  echo "[INFO] Conda solver: ${ROBOSTACK_SOLVER}"
  mkdir -p "$(dirname "${ENV_PREFIX}")"
  CREATE_ARGS=(env create --solver "${ROBOSTACK_SOLVER}" --prefix "${ENV_PREFIX}" --file "${ENV_FILE}")
  if [ "${ROBOSTACK_OFFLINE}" = "1" ]; then
    CREATE_ARGS+=(--offline)
  fi
  if ! CONDA_NO_PLUGINS=true "${CONDA_BIN}" --no-plugins "${CREATE_ARGS[@]}"; then
    if [ "${ROBOSTACK_SOLVER}" != "classic" ]; then
      echo "[WARN] Failed with solver=${ROBOSTACK_SOLVER}, retry with solver=classic"
      CREATE_ARGS=(env create --solver classic --prefix "${ENV_PREFIX}" --file "${ENV_FILE}")
      if [ "${ROBOSTACK_OFFLINE}" = "1" ]; then
        CREATE_ARGS+=(--offline)
      fi
      if ! CONDA_NO_PLUGINS=true "${CONDA_BIN}" --no-plugins "${CREATE_ARGS[@]}"; then
        echo "[ERROR] Failed to create conda env."
        echo "        If network is restricted, try:"
        echo "        ROBOSTACK_OFFLINE=1 RUN_ROSDEP=0 RUN_COLCON_BUILD=0 bash tools/setup_robostack_env.sh"
        echo "        (requires all packages to be present in local conda cache)"
        exit 1
      fi
    else
      echo "[ERROR] Failed to create conda env."
      echo "        If network is restricted, try:"
      echo "        ROBOSTACK_OFFLINE=1 RUN_ROSDEP=0 RUN_COLCON_BUILD=0 bash tools/setup_robostack_env.sh"
      echo "        (requires all packages to be present in local conda cache)"
      exit 1
    fi
  fi
fi

eval "$("${CONDA_BIN}" shell.bash hook)"
conda activate "${ENV_PREFIX}"

if [ "${RUN_ROSDEP}" = "1" ]; then
  echo "[INFO] Updating rosdep index"
  rosdep update --rosdistro humble
  echo "[INFO] Installing rosdep dependencies"
  rosdep install -r --from-paths "${WS_DIR}/src" --ignore-src --rosdistro humble -y
else
  echo "[INFO] Skip rosdep (RUN_ROSDEP=${RUN_ROSDEP})"
fi

if [ "${RUN_COLCON_BUILD}" = "1" ]; then
  echo "[INFO] Building workspace with colcon"
  cd "${WS_DIR}"
  colcon build --symlink-install
else
  echo "[INFO] Skip build (RUN_COLCON_BUILD=${RUN_COLCON_BUILD})"
fi

echo "[DONE] RoboStack environment is ready."
echo "       Activate with: source ${WS_DIR}/tools/activate_robostack.sh"
