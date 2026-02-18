#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/activate_robostack.sh"

FOXGLOVE_ADDRESS="${FOXGLOVE_ADDRESS:-0.0.0.0}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

echo "[INFO] Starting foxglove_bridge at ws://${FOXGLOVE_ADDRESS}:${FOXGLOVE_PORT}"
exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p address:="${FOXGLOVE_ADDRESS}" \
  -p port:="${FOXGLOVE_PORT}"

