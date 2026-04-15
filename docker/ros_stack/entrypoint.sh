#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros_stack_ws/install/setup.bash

# ament_cmake packages (fs_msgs, ifssim_bridge) are not added to AMENT_PREFIX_PATH
# by the colcon-generated setup scripts — add them explicitly.
export AMENT_PREFIX_PATH="/ros_stack_ws/install/fs_msgs:/ros_stack_ws/install/ifssim_bridge:$AMENT_PREFIX_PATH"


if [ "${PIPELINE_ENABLED:-false}" = "true" ]; then
    LAUNCH_FILE=/ros_stack_ws/pipeline.launch.py
    echo "IFSSIM ROS stack starting (bridge + pipeline)..."
else
    LAUNCH_FILE=/ros_stack_ws/bridge.launch.py
    echo "IFSSIM ROS stack starting (bridge only)..."
fi

echo "  Simulator: $IFSSIM_HOST:$IFSSIM_PORT"
echo "  Mission:   $MISSION_NAME / track $TRACK_NAME"

# Start rosboard web visualizer in the background (accessible at http://localhost:8888)
echo "  Starting rosboard at :8888"
rosboard_node &

exec ros2 launch $LAUNCH_FILE \
    host:=$IFSSIM_HOST \
    port:=$IFSSIM_PORT \
    mission_name:=$MISSION_NAME \
    track_name:=$TRACK_NAME
