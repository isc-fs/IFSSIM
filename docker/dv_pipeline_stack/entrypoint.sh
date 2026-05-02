#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Optional rebuild step. With docker-compose bind-mounting the host's
# pipeline/ and ros2/src/ packages over the image's COPY'd baseline,
# Python edits go live via --symlink-install without any rebuild. But
# changes that need re-running colcon — new packages, setup.py edits,
# .msg regeneration, C++ source changes — require this. Off by default
# because it adds ~3–10s to startup; opt in via DV_REBUILD_ON_STARTUP=true
# in compose / shell env.
if [ "${DV_REBUILD_ON_STARTUP:-false}" = "true" ]; then
    echo "DV_REBUILD_ON_STARTUP=true — running colcon build before startup..."
    cd /dv_pipeline_stack_ws
    colcon build --symlink-install
    cd - >/dev/null
fi

source /dv_pipeline_stack_ws/install/setup.bash

# ament_cmake packages (fs_msgs, ifssim_bridge) are not added to AMENT_PREFIX_PATH
# by the colcon-generated setup scripts — add them explicitly.
export AMENT_PREFIX_PATH="/dv_pipeline_stack_ws/install/fs_msgs:/dv_pipeline_stack_ws/install/ifssim_bridge:$AMENT_PREFIX_PATH"

PIPELINE_CTL=/pipeline_ctrl/enable
PIPELINE_PID=""

echo "IFSSIM ROS stack starting (bridge + foxglove)..."
echo "  Simulator: $IFSSIM_HOST:$IFSSIM_PORT"
echo "  Mission:   $MISSION_NAME / track $TRACK_NAME"

# Always clear stale pipeline flag on startup — pipeline must be explicitly started
mkdir -p /pipeline_ctrl
rm -f $PIPELINE_CTL

# Always start the bridge (background so we can monitor pipeline flag)
ros2 launch /dv_pipeline_stack_ws/bridge.launch.py \
    host:=$IFSSIM_HOST \
    port:=$IFSSIM_PORT \
    mission_name:=$MISSION_NAME \
    track_name:=$TRACK_NAME &
BRIDGE_PID=$!

# Monitor flag file and start/stop pipeline accordingly
while kill -0 $BRIDGE_PID 2>/dev/null; do
    if [ -f "$PIPELINE_CTL" ] && [ -z "$PIPELINE_PID" ]; then
        echo "Pipeline start signal — launching pipeline nodes..."
        # setsid runs ros2 launch in a new process group so the whole tree
        # (launcher + every ROS node spawned under it) can be killed as one
        # unit when the stop flag clears. Without setsid, `kill $PID` only
        # hits the launcher — the node children keep running, saturating
        # CPU and blocking future restarts.
        setsid ros2 launch /dv_pipeline_stack_ws/pipeline_only.launch.py \
            host:=$IFSSIM_HOST \
            port:=$IFSSIM_PORT \
            mission_name:=$MISSION_NAME \
            track_name:=$TRACK_NAME &
        PIPELINE_PID=$!
        echo "Pipeline PID (PGID): $PIPELINE_PID"

    elif [ ! -f "$PIPELINE_CTL" ] && [ -n "$PIPELINE_PID" ]; then
        echo "Pipeline stop signal — killing pipeline nodes..."
        # Negative PID targets the whole process group, reaching every ROS
        # node under the launcher. Fall back to a plain TERM if the group
        # kill fails (e.g. setsid not available).
        kill -TERM -$PIPELINE_PID 2>/dev/null || kill -TERM $PIPELINE_PID 2>/dev/null || true
        # Give nodes up to 5 s to exit cleanly, then force-kill the group.
        for _ in $(seq 1 5); do
            kill -0 -$PIPELINE_PID 2>/dev/null || break
            sleep 1
        done
        kill -KILL -$PIPELINE_PID 2>/dev/null || true
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
