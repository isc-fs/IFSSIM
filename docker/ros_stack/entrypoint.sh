#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros_stack_ws/install/setup.bash

# ament_cmake packages (fs_msgs, ifssim_bridge) are not added to AMENT_PREFIX_PATH
# by the colcon-generated setup scripts — add them explicitly.
export AMENT_PREFIX_PATH="/ros_stack_ws/install/fs_msgs:/ros_stack_ws/install/ifssim_bridge:$AMENT_PREFIX_PATH"

PIPELINE_CTL=/pipeline_ctrl/enable
PIPELINE_PID=""

echo "IFSSIM ROS stack starting (bridge + foxglove)..."
echo "  Simulator: $IFSSIM_HOST:$IFSSIM_PORT"
echo "  Mission:   $MISSION_NAME / track $TRACK_NAME"

# Always start the bridge (background so we can monitor pipeline flag)
ros2 launch /ros_stack_ws/bridge.launch.py \
    host:=$IFSSIM_HOST \
    port:=$IFSSIM_PORT \
    mission_name:=$MISSION_NAME \
    track_name:=$TRACK_NAME &
BRIDGE_PID=$!

# If PIPELINE_ENABLED=true at container start, pre-create the flag file
if [ "${PIPELINE_ENABLED:-false}" = "true" ]; then
    mkdir -p /pipeline_ctrl
    touch $PIPELINE_CTL
    echo "Pipeline auto-enabled via PIPELINE_ENABLED=true"
fi

# Monitor flag file and start/stop pipeline accordingly
while kill -0 $BRIDGE_PID 2>/dev/null; do
    if [ -f "$PIPELINE_CTL" ] && [ -z "$PIPELINE_PID" ]; then
        echo "Pipeline start signal — launching pipeline nodes..."
        ros2 launch /ros_stack_ws/pipeline_only.launch.py \
            host:=$IFSSIM_HOST \
            port:=$IFSSIM_PORT \
            mission_name:=$MISSION_NAME \
            track_name:=$TRACK_NAME &
        PIPELINE_PID=$!
        echo "Pipeline PID: $PIPELINE_PID"

    elif [ ! -f "$PIPELINE_CTL" ] && [ -n "$PIPELINE_PID" ]; then
        echo "Pipeline stop signal — killing pipeline nodes..."
        kill $PIPELINE_PID 2>/dev/null || true
        wait $PIPELINE_PID 2>/dev/null || true
        PIPELINE_PID=""
        echo "Pipeline stopped."

    elif [ -n "$PIPELINE_PID" ] && ! kill -0 $PIPELINE_PID 2>/dev/null; then
        echo "Pipeline process exited unexpectedly."
        PIPELINE_PID=""
    fi

    sleep 1
done

wait $BRIDGE_PID
