#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros_stack_ws/install/setup.bash

echo "IFSSIM ROS stack starting..."
echo "  Simulator: $IFSSIM_HOST:$IFSSIM_PORT"
echo "  Mission:   $MISSION_NAME / track $TRACK_NAME"

exec ros2 launch /ros_stack_ws/pipeline.launch.py \
    host:=$IFSSIM_HOST \
    port:=$IFSSIM_PORT \
    mission_name:=$MISSION_NAME \
    track_name:=$TRACK_NAME
