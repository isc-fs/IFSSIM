#!/usr/bin/env bash
# Replay a recorded sensor bag through cone_graph_slam and stream a
# pose comparison vs ground truth (/testing_only/odom).
#
# Usage:
#   tools/replay.sh <bag-name> [duration-seconds]
#
#   bag-name    — folder under tools/bags/ (recorded by tools/record_bag.sh)
#   duration    — wall-seconds the comparator runs (default 80)
#
# Mechanics:
#   Spins up an EPHEMERAL container off the same ifssim-dv_pipeline_stack
#   image, isolated on ROS_DOMAIN_ID=42 so a live dv_pipeline_stack
#   on domain 0 is unaffected. Inside it: cone_graph_slam (consumes
#   /imu, /Conos_raw, /motor_rpm — all from the bag — and publishes
#   /cone_slam/state), and ros2 bag play. The bag fixture already
#   contains /Conos_raw recorded from Cone_Detection during the
#   original drive, so we don't spawn Cone_Detection here — doing so
#   would publish /Conos_raw twice per scan (bag + node), leading to
#   doubled cone callbacks with identical header stamps. Each duplicate
#   makes the IMU preintegrator's integrate_to find an empty window
#   (last_integration_t already equals t_end from the previous call)
#   and raises "skip scan: no IMU samples to integrate", which we used
#   to see by the thousand in replay logs. Single-publisher fixes it.
#
#   When the comparator finishes (after `duration` wall-seconds) the
#   container exits and removes itself. The pose-comparison table is
#   tee'd to tools/bags/<bag-name>/replay_pose_cmp_cone_slam.txt.
#
# Pre-conditions:
#   - The ifssim-dv_pipeline_stack image is built (docker compose build).
#   - The bag was recorded with ≥ 3 seconds of car-stationary at the
#     start (cone_graph_slam's IMU calibration window).

set -euo pipefail

BAG_NAME="${1:-}"
DURATION="${2:-80}"
if [ -z "$BAG_NAME" ]; then
    echo "Usage: $0 <bag-name> [duration-seconds]" >&2
    exit 1
fi
SLAM_TOPIC="/cone_slam/state"

HOST_BAG_DIR="$(realpath "tools/bags/$BAG_NAME")"
if [ ! -d "$HOST_BAG_DIR" ]; then
    echo "Bag dir not found: $HOST_BAG_DIR" >&2
    exit 1
fi

REPO="$(realpath ".")"
IMAGE="ifssim-dv_pipeline_stack:latest"

echo "==> Replaying $BAG_NAME for ${DURATION}s in an ephemeral container"
echo "    (live dv_pipeline_stack — if running — stays untouched on ROS_DOMAIN_ID=0)"
echo

# Git Bash on Windows mangles unix-style absolute paths (e.g. /bin/bash
# gets rewritten to C:/Program Files/Git/usr/bin/bash). Disable that for
# the docker invocation so the container's own /bin/bash is used.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

docker run --rm \
    --name "ifssim-replay-$$" \
    -v "$HOST_BAG_DIR:/replay/bag:ro" \
    -v "$REPO/tools/pose_cmp.py:/replay/pose_cmp.py:ro" \
    -e ROS_DOMAIN_ID=42 \
    -e "SLAM_TOPIC=$SLAM_TOPIC" \
    -e "DURATION=$DURATION" \
    --entrypoint=/bin/bash \
    "$IMAGE" \
    -c "
        set -e
        source /opt/ros/humble/setup.bash
        source /dv_pipeline_stack_ws/install/setup.bash
        export AMENT_PREFIX_PATH=/dv_pipeline_stack_ws/install/fs_msgs:/dv_pipeline_stack_ws/install/ifssim_bridge:\$AMENT_PREFIX_PATH

        echo '==> Starting cone_graph_slam'
        ros2 run cone_slam cone_graph_slam \\
            > /tmp/slam.log 2>&1 &
        SLAM_PID=\$!

        # Give the SLAM node a moment to subscribe before the bag starts
        # publishing — otherwise the very first IMU samples can race
        # ahead of the subscription and miss the calibration window.
        sleep 2

        echo '==> Starting bag play (no Cone_Detection — /Conos_raw is in the bag)'
        ros2 bag play /replay/bag --disable-keyboard-controls \\
            > /tmp/bag.log 2>&1 &
        BAG_PID=\$!

        # Tiny grace period so the comparator's first spin sees both
        # publishers already advertising.
        sleep 1

        echo '==> Streaming pose comparison (${DURATION}s) — cone_slam'
        python3 /replay/pose_cmp.py $DURATION --slam-topic $SLAM_TOPIC

        echo '==> SLAM log tail (last 30 lines):'
        tail -30 /tmp/slam.log || true

        kill \$SLAM_PID \$BAG_PID 2>/dev/null || true
    " | tee "$HOST_BAG_DIR/replay_pose_cmp_cone_slam.txt"

echo
echo "==> Done. Comparison saved to $HOST_BAG_DIR/replay_pose_cmp_cone_slam.txt"
