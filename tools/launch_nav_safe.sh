#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ROS_LOG_DIR_DEFAULT="/tmp/roslog_sentry_nav_$(date +%Y%m%d_%H%M%S)"
ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_LOG_DIR_DEFAULT}}"
FASTDDS_PROFILE_FILE="${FASTDDS_PROFILE_FILE:-/tmp/sentry_fastdds_profile.xml}"
NAV_RVIZ="${NAV_RVIZ:-false}"
DRY_RUN="${DRY_RUN:-0}"

if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
  echo "[ERROR] Missing ${WS_DIR}/install/setup.bash. Build the workspace first."
  exit 1
fi

mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

# Prevent system ROS2 python nodes (e.g. gazebo_ros/spawn_entity.py) from
# accidentally importing Anaconda Python 3.12 modules.
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER
unset PYTHONPATH PYTHONHOME
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Disable FastDDS SHM to avoid permission issues in constrained environments.
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

echo "==================================================================="
echo "Sentry Navigation Safe Launcher"
echo "Workspace       : ${WS_DIR}"
echo "ROS log dir     : ${ROS_LOG_DIR}"
echo "FastDDS profile : ${FASTDDS_PROFILE_FILE}"
echo "Python          : $(command -v python3) ($(python3 -V 2>&1))"
echo "ros2            : $(command -v ros2)"
echo "==================================================================="

if [ "${DRY_RUN}" = "1" ]; then
  echo "[INFO] DRY_RUN=1, skip launching navigation."
  exit 0
fi

exec ros2 launch rm_nav_bringup bringup.launch.py "nav_rviz:=${NAV_RVIZ}" "$@"
