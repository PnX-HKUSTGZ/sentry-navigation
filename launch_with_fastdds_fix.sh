#!/bin/bash
# Sentry Navigation Launch Wrapper
# This script sets up FastDDS to avoid SHM errors that prevent gazebo ROS plugins from working

# Create FastDDS profile to disable SHM transport
FASTDDS_PROFILE="/tmp/sentry_fastdds_profile.xml"
cat > "$FASTDDS_PROFILE" << 'EOF'
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

# Set environment variables for FastDDS
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
export RMW_FASTRTPS_USE_SHM=0

# Source ROS workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

# Launch with all arguments passed through
echo "==================================================================="
echo "Sentry Navigation Launch with FastDDS Fix"
echo "FastDDS Profile: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo "==================================================================="
exec ros2 launch rm_nav_bringup bringup.launch.py "$@"
