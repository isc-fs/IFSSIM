#!/usr/bin/env bash
# Replay a recorded bag through the live cone-SLAM node in an
# ephemeral container, isolated on its own ROS_DOMAIN_ID.
#
# Usage:
#   tools/replay.sh <bag-name> [duration-seconds] [--mode <m>]
#
#   bag-name    — folder under bags/ (recorded by the
#                 Mission Control session UX or `ros2 bag record`).
#                 Both sqlite3 and mcap formats are accepted —
#                 storage backend is auto-detected from
#                 <bag-name>/metadata.yaml.
#   duration    — wall-seconds before the comparator exits (default 80).
#   --mode m    — mission strategy passed to slam_node's `~/setup`
#                 service. One of trackdrive / autocross / accel /
#                 skidpad / scruti. Default autocross.
#
# Mechanics:
#   Spins up an EPHEMERAL container off the same ifssim-dv_pipeline_stack
#   image, isolated on ROS_DOMAIN_ID=42 so a live dv_pipeline_stack on
#   domain 0 is unaffected. Inside it:
#     1. Launches slam_node (post-PR-518 lifecycle node).
#     2. Calls /slam_node/setup, then `ros2 lifecycle set` to drive
#        configure → activate (the bringup path mode_manager would
#        normally run; we bypass mode_manager here so the test is
#        slam-only and doesn't need the rest of the pipeline alive).
#     3. Plays the bag with bag-clock semantics so slam_node sees
#        the recorded sensor timestamps.
#     4. Records /slam/pose, /Conos, /tf, /cone_slam/gt_aligned,
#        /cone_slam/gt_error_m into a fresh mcap next to the input
#        bag for Lichtblick inspection.
#
#   When the duration elapses the container exits and removes itself.
#
# Pre-conditions:
#   - The ifssim-dv_pipeline_stack image is built (docker compose build).
#   - The bag contains /imu, /motor_rpm, /Conos_raw, /testing_only/odom.
#     (Phase 1 SLAM rewrite will allow /odom to be optional; today's
#     cone_graph_slam reads /odom too — the bag should have it.)
#
# Phase 0 scope (this rewrite): bring up the SLAM node only — not
# cone_detection / path_planning / control. The bag already contains
# /Conos_raw recorded from a live cone_detection run, so running
# cone_detection here would double-publish. Future Phase 0 work may
# add a flag to run cone_detection live (testing fresh perception
# code against a recorded LiDAR stream).

set -euo pipefail

BAG_NAME="${1:-}"
DURATION="${2:-80}"
MODE="autocross"

# Parse optional --mode after the positional args.
shift 2 2>/dev/null || true
while [ $# -gt 0 ]; do
    case "$1" in
        --mode) MODE="$2"; shift 2 ;;
        *)      echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

if [ -z "$BAG_NAME" ]; then
    echo "Usage: $0 <bag-name> [duration-seconds] [--mode <m>]" >&2
    echo "  Valid modes: trackdrive autocross accel skidpad scruti" >&2
    exit 1
fi

# Bag location: prefer bags/ (the post-#498 host landing zone) but
# fall back to tools/bags/ for legacy bag layouts.
if   [ -d "bags/$BAG_NAME" ];           then HOST_BAG_DIR="$(realpath "bags/$BAG_NAME")"
elif [ -d "tools/bags/$BAG_NAME" ];     then HOST_BAG_DIR="$(realpath "tools/bags/$BAG_NAME")"
else
    echo "Bag dir not found in bags/ or tools/bags/: $BAG_NAME" >&2
    exit 1
fi

IMAGE="ifssim-dv_pipeline_stack:latest"
echo "==> Replaying $BAG_NAME ($HOST_BAG_DIR) for ${DURATION}s, mode=$MODE"
echo "    (live dv_pipeline_stack — if running — stays untouched on ROS_DOMAIN_ID=0)"
echo

# Git Bash / msys2 path-mangling guard: keep absolute container-side
# paths from being rewritten when running on Windows.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

# Recording from this replay lands here (host-side). Wipe any previous
# replay's output first since `ros2 bag record` refuses to overwrite.
HOST_REC_DIR="$HOST_BAG_DIR/replay_slam_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$HOST_REC_DIR"

docker run --rm \
    --name "ifssim-replay-$$" \
    -v "$HOST_BAG_DIR:/replay/bag:ro" \
    -v "$HOST_REC_DIR:/replay/out" \
    -e ROS_DOMAIN_ID=42 \
    -e "DURATION=$DURATION" \
    -e "MODE=$MODE" \
    --entrypoint=/bin/bash \
    "$IMAGE" \
    -c '
        set -e
        source /opt/ros/humble/setup.bash
        source /dv_pipeline_stack_ws/install/setup.bash

        echo "==> Launching slam_node (lifecycle: unconfigured)"
        ros2 run cone_slam slam_node > /replay/out/slam.log 2>&1 &
        SLAM_PID=$!
        # Wait for lifecycle service to come up.
        for i in $(seq 1 20); do
            if ros2 service list 2>/dev/null | grep -q "^/slam_node/change_state$"; then
                echo "    slam_node alive (lifecycle services advertised after ${i}s)"
                break
            fi
            sleep 1
        done
        if ! ros2 service list 2>/dev/null | grep -q "^/slam_node/change_state$"; then
            echo "==> ERROR: slam_node never advertised /slam_node/change_state. Log tail:" >&2
            tail -40 /replay/out/slam.log >&2
            exit 1
        fi

        echo "==> Driving lifecycle: ~/setup($MODE) → configure → activate"
        # Setup tells the node which mode/behavior to instantiate.
        ros2 service call /slam_node/setup dv_msgs/srv/Setup \
            "{mode_name: $MODE, behavior: $MODE}" >/dev/null

        # Configure transition (id=1). The cone_graph_slam_node does
        # most of its heavy init here (preintegrator, factor graph,
        # publishers).
        ros2 lifecycle set /slam_node configure >/dev/null

        # Activate transition (id=3). After this the IMU / RPM / cones
        # callbacks become live.
        ros2 lifecycle set /slam_node activate >/dev/null
        echo "    slam_node ACTIVE"

        echo "==> Recording SLAM outputs to /replay/out/replay_slam.mcap"
        ros2 bag record -s mcap -o /replay/out/replay_slam \
            /tf /tf_static /slam/pose /Conos /Path \
            /cone_slam/gt_aligned /cone_slam/gt_error_m \
            > /replay/out/recorder.log 2>&1 &
        REC_PID=$!
        sleep 1

        echo "==> Starting bag play"
        ros2 bag play /replay/bag --disable-keyboard-controls \
            > /replay/out/bag.log 2>&1 &
        BAG_PID=$!

        # Run for DURATION wall-seconds, then teardown.
        sleep "$DURATION"

        echo "==> Done; tearing down"
        # SIGINT recorder first so mcap chunk index flushes cleanly
        # (SIGTERM truncates trailing chunks and Lichtblick refuses to
        # open the file).
        kill -INT $REC_PID 2>/dev/null || true
        wait $REC_PID 2>/dev/null || true

        kill $BAG_PID $SLAM_PID 2>/dev/null || true

        echo
        echo "==> slam_node log tail:"
        tail -20 /replay/out/slam.log || true
        echo
        echo "==> Output: /replay/out/replay_slam (host: '"$HOST_REC_DIR"')"
        ls -la /replay/out/
    '

echo
echo "==> Done. Recorded SLAM outputs at:"
echo "    $HOST_REC_DIR/"
if [ -d "$HOST_REC_DIR" ]; then
    MCAP=$(ls -1 "$HOST_REC_DIR/replay_slam/"*.mcap 2>/dev/null | head -1 || true)
    if [ -n "$MCAP" ]; then
        echo "==> Lichtblick: open $MCAP"
        echo "    (http://localhost:8080 → Open file → drag the .mcap in)"
    fi
fi
