#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

BAG_PATH="${BAG_PATH:-}"
PCD_PATH="${PCD_PATH:-}"
PARAM_FILE="${PARAM_FILE:-${WS_DIR}/src/rm_nav_bringup/config/simulation/small_gicp_registration.yaml}"
PLAY_RATE="${PLAY_RATE:-1.0}"
INIT_X="${INIT_X:--5.0}"
INIT_Y="${INIT_Y:-3.0}"
INIT_QZ="${INIT_QZ:-0.0}"
INIT_QW="${INIT_QW:-1.0}"
OUT_DIR="${OUT_DIR:-${WS_DIR}/artifacts/small_gicp_offline_$(date +%Y%m%d_%H%M%S)}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/livox/lidar/pointcloud}"

if [[ -z "${BAG_PATH}" || -z "${PCD_PATH}" ]]; then
  echo "[ERROR] usage: BAG_PATH=<bag_dir> PCD_PATH=<map.pcd> $0" >&2
  exit 2
fi

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "[ERROR] BAG_PATH not found: ${BAG_PATH}" >&2
  exit 2
fi

if [[ ! -f "${PCD_PATH}" ]]; then
  echo "[ERROR] PCD_PATH not found: ${PCD_PATH}" >&2
  exit 2
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] Missing ${WS_DIR}/install/setup.bash. Build workspace first." >&2
  exit 3
fi

mkdir -p "${OUT_DIR}"
LOG_SMALL_GICP="${OUT_DIR}/small_gicp.log"
LOG_BAG_PLAY="${OUT_DIR}/bag_play.log"
SUMMARY="${OUT_DIR}/summary.txt"
CSV="${OUT_DIR}/iterations.csv"

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

export ROS2CLI_NO_DAEMON=1

SMALL_GICP_PID=""
BAG_PLAY_PID=""

cleanup() {
  if [[ -n "${BAG_PLAY_PID}" ]] && kill -0 "${BAG_PLAY_PID}" 2>/dev/null; then
    kill -- "-${BAG_PLAY_PID}" 2>/dev/null || kill "${BAG_PLAY_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SMALL_GICP_PID}" ]] && kill -0 "${SMALL_GICP_PID}" 2>/dev/null; then
    kill -- "-${SMALL_GICP_PID}" 2>/dev/null || kill "${SMALL_GICP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[1/4] launch small_gicp_registration (offline)"
setsid ros2 run small_gicp_registration small_gicp_registration_node --ros-args \
  --params-file "${PARAM_FILE}" \
  -p use_sim_time:=true \
  -p pcd_path:="${PCD_PATH}" \
  -p pointcloud_topic:="${POINTCLOUD_TOPIC}" \
  > "${LOG_SMALL_GICP}" 2>&1 &
SMALL_GICP_PID=$!

sleep 2

echo "[2/4] replay bag: ${BAG_PATH}"
setsid ros2 bag play "${BAG_PATH}" --clock --rate "${PLAY_RATE}" \
  > "${LOG_BAG_PLAY}" 2>&1 &
BAG_PLAY_PID=$!

sleep 1

echo "[3/4] publish /initialpose"
timeout 6 ros2 topic pub -r 8 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, pose: {pose: {position: {x: ${INIT_X}, y: ${INIT_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${INIT_QZ}, w: ${INIT_QW}}}}}" \
  --qos-reliability best_effort --qos-durability volatile >/dev/null 2>&1 || true

set +e
wait "${BAG_PLAY_PID}"
BAG_RC=$?
set -e

sleep 2

if kill -0 "${SMALL_GICP_PID}" 2>/dev/null; then
  kill -- "-${SMALL_GICP_PID}" 2>/dev/null || kill "${SMALL_GICP_PID}" 2>/dev/null || true
  sleep 1
  kill -9 -- "-${SMALL_GICP_PID}" 2>/dev/null || kill -9 "${SMALL_GICP_PID}" 2>/dev/null || true
fi

echo "[4/4] parse iteration logs"
python3 - "${LOG_SMALL_GICP}" "${CSV}" "${SUMMARY}" <<'PY'
import csv
import re
import statistics
import sys
from pathlib import Path

log_path = Path(sys.argv[1])
csv_path = Path(sys.argv[2])
summary_path = Path(sys.argv[3])
pat = re.compile(
    r"small-gicp iter: src=(\d+) ds=(\d+) candidates=(\d+) success=(\d) score=([\-0-9\.]+) elapsed=(\d+)ms"
)

rows = []
for line in log_path.read_text(encoding="utf-8", errors="ignore").splitlines():
    m = pat.search(line)
    if not m:
        continue
    src, ds, cand, suc, score, elapsed = m.groups()
    rows.append(
        {
            "src": int(src),
            "ds": int(ds),
            "candidates": int(cand),
            "success": int(suc),
            "score": float(score),
            "elapsed_ms": int(elapsed),
        }
    )

csv_path.parent.mkdir(parents=True, exist_ok=True)
with csv_path.open("w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(
        f,
        fieldnames=["src", "ds", "candidates", "success", "score", "elapsed_ms"],
    )
    w.writeheader()
    w.writerows(rows)

n = len(rows)
succ = [r for r in rows if r["success"] == 1]
succ_n = len(succ)
succ_rate = (100.0 * succ_n / n) if n else 0.0
succ_scores = [r["score"] for r in succ if r["score"] >= 0]
all_elapsed = [r["elapsed_ms"] for r in rows]

lines = [
    f"iterations_total={n}",
    f"iterations_success={succ_n}",
    f"success_rate_percent={succ_rate:.2f}",
    f"score_mean_success={statistics.fmean(succ_scores):.5f}" if succ_scores else "score_mean_success=nan",
    f"score_p95_success={sorted(succ_scores)[max(0, int(len(succ_scores)*0.95)-1)]:.5f}" if succ_scores else "score_p95_success=nan",
    f"elapsed_mean_ms={statistics.fmean(all_elapsed):.2f}" if all_elapsed else "elapsed_mean_ms=nan",
    f"elapsed_p95_ms={sorted(all_elapsed)[max(0, int(len(all_elapsed)*0.95)-1)]}" if all_elapsed else "elapsed_p95_ms=nan",
]
summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print("\n".join(lines))
PY

if [[ "${BAG_RC}" -ne 0 ]]; then
  echo "[WARN] ros2 bag play exit code=${BAG_RC}" >&2
fi

echo "artifacts=${OUT_DIR}"
