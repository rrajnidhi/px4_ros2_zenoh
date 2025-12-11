#!/bin/bash
set -e

printf "Entering px4_ros2_humble container\n"
sleep 1

# ----------------------------
# Basic tmux config
# ----------------------------
if [ ! -f /root/.tmux.conf ]; then
    echo "bind e kill-session" > /root/.tmux.conf
fi

# ----------------------------
# Environment
# ----------------------------
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-3}
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export GZ_SIM_RESOURCE_PATH="/app/PX4-Autopilot/Tools/simulation/gz:/root/.gz/models:${GZ_SIM_RESOURCE_PATH}"
export PX4_GZ_MODEL_PATH="/app/PX4-Autopilot/Tools/simulation/gz/models:/root/.gz/models"
export GZ_MODEL_PATH="$PX4_GZ_MODEL_PATH"

# ----------------------------
# Source ROS and workspace
# ----------------------------
source /opt/ros/humble/setup.bash
[ -f /app/ros2_ws/install/setup.bash ] && source /app/ros2_ws/install/setup.bash

# ----------------------------
# Stop previous ROS 2 daemons
# ----------------------------
if pgrep -f ros >/dev/null 2>&1; then
    echo "Stopping previous ROS 2 daemons..."
    pkill -9 -f ros || true
    ros2 daemon stop || true
fi

# ----------------------------
# Default shell / command
# ----------------------------
if [ "$#" -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi

