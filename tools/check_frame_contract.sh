#!/usr/bin/env bash
set -euo pipefail

echo "[check] ROS env"
set +u
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u

# Align DDS transport settings with tools/launch_nav_safe.sh when present.
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f /tmp/sentry_fastdds_profile.xml ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/sentry_fastdds_profile.xml
fi
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] missing command: $1"
    exit 1
  fi
}

need_cmd ros2

# Refresh daemon to avoid stale graph cache between relaunches.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

check_param_eq() {
  local node="$1"
  local param="$2"
  local expected="$3"
  local raw
  local got

  raw="$(ros2 param get "$node" "$param" 2>&1 || true)"
  if echo "${raw}" | grep -qiE "node not found|parameter not set"; then
    echo "[fail] cannot read $node/$param"
    return 1
  fi
  got="$(echo "${raw}" | sed -nE 's/^String value is: (.*)$/\1/p; s/^value: (.*)$/\1/p' | tail -n 1 | xargs)"
  if [[ -z "${got}" ]]; then
    echo "[fail] cannot parse value for $node/$param"
    echo "  raw: ${raw}"
    return 1
  fi
  if [[ "$got" == "$expected" ]]; then
    echo "[pass] $node/$param = $got"
    return 0
  fi
  echo "[fail] $node/$param = $got (expected: $expected)"
  return 1
}

check_topic_exists() {
  local topic="$1"
  if ros2 topic list | grep -qx "$topic"; then
    echo "[pass] topic exists: $topic"
    return 0
  fi
  echo "[fail] topic missing: $topic"
  return 1
}

check_tf_echo_once() {
  local parent="$1"
  local child="$2"
  timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" >/tmp/tf_echo_check.log 2>&1 || true
  if grep -q "At time" /tmp/tf_echo_check.log; then
    echo "[pass] tf reachable: $parent -> $child"
    return 0
  fi
  echo "[fail] tf unreachable: $parent -> $child"
  echo "------ tf2_echo output ------"
  sed -n '1,40p' /tmp/tf_echo_check.log || true
  echo "-----------------------------"
  return 1
}

rc=0

echo "[check] required nodes"
for n in /amcl /controller_server /bt_navigator /fake_vel_transform; do
  if ros2 node list | grep -qx "$n"; then
    echo "[pass] node exists: $n"
  else
    echo "[fail] node missing: $n"
    rc=1
  fi
done

echo "[check] key params"
check_param_eq /amcl base_frame_id base_link || rc=1
check_param_eq /controller_server robot_base_frame base_link_fake || rc=1
check_param_eq /bt_navigator robot_base_frame base_link_fake || rc=1
check_param_eq /fake_vel_transform base_frame base_link || rc=1
check_param_eq /fake_vel_transform fake_base_frame base_link_fake || rc=1

echo "[check] tf and topics"
check_topic_exists /tf || rc=1
check_tf_echo_once base_link base_link_fake || rc=1

if [[ "$rc" -eq 0 ]]; then
  echo "[result] frame contract check PASSED"
else
  echo "[result] frame contract check FAILED"
fi

exit "$rc"
