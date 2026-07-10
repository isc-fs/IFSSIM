#!/usr/bin/env bash
# Record a ROS 2 bag from the sim. Two topic sets:
#
# DEV (default) — the canonical dev fixture consumed by tools/replay.sh and
# tools/dev/pose_cmp.py:
#   /imu                  — bridge IMU stream (~400 Hz BMI088 model)
#   /lidar/Lidar1         — bridge LiDAR stream (~10 Hz Hesai ATX 128 ch)
#   /gps                  — bridge GPS NavSatFix (~10 Hz)
#   /motor_rpm            — bridge motor RPM Float32 (100 Hz)
#   /testing_only/odom    — bridge ground-truth Odometry (SIM-ONLY)
#   /Conos_raw            — Cone_Detection MarkerArray (10 Hz; lets replays
#                            skip the numba-JIT warmup)
#
# CAR-PARITY (--car-parity) — a bag that LIFTS onto the real car
# (isc-fs/IFS08-DV-uDV). Records exactly the sensor stream the real uDV +
# Hesai driver put on the wire, so replaying it on the car looks like real
# hardware to the pipeline. See tools/lift_to_car.sh for the replay recipe.
#   /imu                  — frame_id imu_link (renamed to match firmware)
#   /lidar/Lidar1         — frame_id hesai_lidar (renamed to match driver);
#                            remapped to /lidar_points at replay time
#   /motor_rpm            — Float32 (mechanical shaft rpm; see rpm-scaling
#                            note in tools/lift_to_car.sh)
#   /steering_angle       — Float32 rad, pipeline odometry_filter input
#   /tf_static            — base_link->imu_link / ->hesai_lidar so the car's
#                            cone_detection TF lookup resolves
#   NB: /gps, /testing_only/*, /Conos_raw are DELIBERATELY EXCLUDED — on the
#   car /testing_only/* would activate slam_node's dead GT subscription and
#   inject ground truth; /Conos_raw would double the car's own cone_detection.
#
# Usage (from the repo root):
#   tools/record_bag.sh <bag-name> [duration-seconds] [--car-parity]
#
#   bag-name         folder under tools/bags/.
#   duration         optional wall-seconds; default 90.
#   --car-parity     record the car-liftable sensor set instead of the dev set.
#
# Pre-conditions:
#   - dv_pipeline_stack container is running and connected to UE5.
#   - PIE is in the desired starting state (3 s standstill before driving
#     is what cone_slam wants for its calibration window).

set -euo pipefail

CAR_PARITY=0
POSITIONAL=()
for arg in "$@"; do
    case "$arg" in
        --car-parity) CAR_PARITY=1 ;;
        *) POSITIONAL+=("$arg") ;;
    esac
done
BAG_NAME="${POSITIONAL[0]:-}"
DURATION="${POSITIONAL[1]:-90}"
if [ -z "$BAG_NAME" ]; then
    echo "Usage: $0 <bag-name> [duration-seconds] [--car-parity]" >&2
    exit 1
fi

if [ "$CAR_PARITY" -eq 1 ]; then
    TOPICS="/imu /lidar/Lidar1 /motor_rpm /steering_angle /tf_static"
    echo "==> mode: CAR-PARITY (uDV bag-lift set) — $TOPICS"
else
    TOPICS="/imu /lidar/Lidar1 /gps /motor_rpm /testing_only/odom /Conos_raw"
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
        $TOPICS \
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
