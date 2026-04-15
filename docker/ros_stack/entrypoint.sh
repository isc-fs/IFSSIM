#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros_stack_ws/install/setup.bash

# ament_cmake packages (fs_msgs, ifssim_bridge) are not added to AMENT_PREFIX_PATH
# by the colcon-generated setup scripts — add them explicitly.
export AMENT_PREFIX_PATH="/ros_stack_ws/install/fs_msgs:/ros_stack_ws/install/ifssim_bridge:$AMENT_PREFIX_PATH"

echo "IFSSIM ROS stack starting..."
echo "  Simulator: $IFSSIM_HOST:$IFSSIM_PORT"
echo "  Mission:   $MISSION_NAME / track $TRACK_NAME"

exec ros2 launch /ros_stack_ws/pipeline.launch.py \
    host:=$IFSSIM_HOST \
    port:=$IFSSIM_PORT \
    mission_name:=$MISSION_NAME \
    track_name:=$TRACK_NAME
