#!/bin/bash
# PX4 + ROS2 Humble + Zenoh SITL tmux launcher

SESSION_NAME="sitl_px4_ros2_zenoh"

# ----------------------------
# Environment variables
# ----------------------------
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-3}
PX4_ZENOH_NS=${PX4_ZENOH_NS:-cam_drone}
PX4_UAV_MODEL=${PX4_UAV_MODEL:-gz_x500}
PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:-"0,0,2,0,0,0"}  # default safe spawn pose

RMW_IMPLEMENTATION=rmw_zenoh_cpp

# ----------------------------
# Kill existing session
# ----------------------------
tmux kill-session -t $SESSION_NAME 2>/dev/null

# ----------------------------
# Start new tmux session (detached)
# ----------------------------
tmux new-session -d -s $SESSION_NAME

# ----------------------------
# Pane 0: Zenoh router first
# ----------------------------
tmux send-keys -t $SESSION_NAME "
echo 'Starting Zenoh router...'
ros2 run rmw_zenoh_cpp rmw_zenohd
" C-m

# ----------------------------
# Pane 1: PX4 SITL + Gazebo headless
# Wait a few seconds to allow Zenoh router to initialize
# ----------------------------
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.1 "
sleep 5
cd /app/PX4-Autopilot/
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION
export PX4_ZENOH_NS=$PX4_ZENOH_NS
export PX4_GZ_MODEL_POSE=\"$PX4_GZ_MODEL_POSE\"
make px4_sitl_zenoh $PX4_UAV_MODEL HEADLESS=1
" C-m

# ----------------------------
# Pane 2: ROS2 image bridge
# Starts after PX4 is running
# ----------------------------
tmux split-window -v -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME:0.2 "
bash -c '
source /opt/ros/humble/setup.bash
source /app/ros2_ws/install/setup.bash
# Wait until PX4 SITL is running
until pgrep -f px4_sitl > /dev/null; do sleep 1; done
echo \"Starting ROS2 image bridge...\"
ros2 run ros_gz_image image_bridge /camera
'" C-m

# ----------------------------
# New window: ROS2 tools (topic echo, monitoring)
# ----------------------------
tmux new-window -t $SESSION_NAME -n "ros2_tools"
tmux send-keys -t $SESSION_NAME:1 "
bash -c '
source /opt/ros/humble/setup.bash
source /app/ros2_ws/install/setup.bash
echo \"Listing all ROS2 topics...\"
ros2 topic list
'" C-m

# ----------------------------
# Focus on first window and first pane
# ----------------------------
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0

# ----------------------------
# Attach to session
# ----------------------------
tmux attach-session -t $SESSION_NAME

