#!/usr/bin/env bash
# Record a ROS 2 bag of the sensor + GT + cone topics that tools/replay.sh
# expects. Output goes to tools/bags/<name>/ and is the canonical fixture
# format consumed by tools/replay.sh and tools/pose_cmp.py.
#
# Topics captured:
#   /imu                  — bridge IMU stream (~400 Hz BMI088 model)
#   /lidar/Lidar1         — bridge LiDAR stream (~10 Hz Hesai ATX 128 ch)
#   /gps                  — bridge GPS NavSatFix (~10 Hz)
#   /motor_rpm            — bridge motor RPM Float32 (100 Hz, real-IFS-08
#                            CAN-parity; new in this branch)
#   /testing_only/odom    — bridge ground-truth Odometry (sim-only,
#                            consumed by pose_cmp)
#   /Conos_raw            — Cone_Detection MarkerArray (10 Hz). Recording
#                            this lets cone_slam replays skip the
#                            numba-JIT warmup of Cone_Detection.
#
# Usage (from the repo root):
#   tools/record_bag.sh <bag-name> [duration-seconds]
#
#   bag-name         folder under tools/bags/. Convention is
#                    <description>_<duration>s, e.g. clean_drive_75s.
#   duration         optional wall-seconds; default 90. ros2 bag record
#                    runs for this long then exits cleanly.
#
# Pre-conditions:
#   - dv_pipeline_stack container is running and connected to UE5.
#   - PIE is in the desired starting state (3 s standstill before driving
#     is what fast_LIMO and cone_slam want for their calibration windows).
#
# Pairs naturally with tools/track_driver.py: start the recorder, kick
# off the driver, wait for the driver to finish, recorder exits on its
# own duration timer.

set -euo pipefail

BAG_NAME="${1:-}"
DURATION="${2:-90}"
if [ -z "$BAG_NAME" ]; then
    echo "Usage: $0 <bag-name> [duration-seconds]" >&2
    exit 1
fi

REPO="$(realpath ".")"
BAG_DIR="$REPO/tools/bags/$BAG_NAME"
if [ -d "$BAG_DIR" ]; then
    echo "Bag dir already exists: $BAG_DIR" >&2
    echo "Refusing to overwrite. Pick a fresh name or `rm -rf` it first." >&2
    exit 2
fi
mkdir -p "$REPO/tools/bags"

echo "==> Recording '$BAG_NAME' for ${DURATION}s into $BAG_DIR"

# Run inside the live dv_pipeline_stack container so we share its DDS
# domain (ROS_DOMAIN_ID=0 by default). Bind-mount the bag dir into the
# container so the bag is written directly to the host. ros2 bag record's
# --max-bag-duration exits cleanly after the wall-time; pkill after a
# small grace window catches the rare case where it lingers.
docker exec -i ifssim-dv_pipeline_stack-1 bash -lc "
    set -e
    source /opt/ros/humble/setup.bash
    source /dv_pipeline_stack_ws/install/setup.bash
    mkdir -p /tmp/recording
    cd /tmp/recording
    rm -rf '$BAG_NAME'
    ros2 bag record \
        --output '$BAG_NAME' \
        --max-bag-duration '$DURATION' \
        /imu /lidar/Lidar1 /gps /motor_rpm /testing_only/odom /Conos_raw \
        &
    REC_PID=\$!
    sleep $((DURATION + 2))
    kill -INT \$REC_PID 2>/dev/null || true
    wait \$REC_PID 2>/dev/null || true
"

# Pull the recorded bag out of the container onto the host.
docker cp "ifssim-dv_pipeline_stack-1:/tmp/recording/$BAG_NAME" "$BAG_DIR"
docker exec ifssim-dv_pipeline_stack-1 rm -rf "/tmp/recording/$BAG_NAME"

echo "==> Bag saved to $BAG_DIR"
ls -lh "$BAG_DIR" | tail -n +2
