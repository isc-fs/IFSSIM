#!/usr/bin/env bash
# Lift a sim-recorded rosbag onto the REAL CAR (isc-fs/IFS08-DV-uDV) — replay
# a car-parity sim bag into the live car pipeline so the whole autonomy stack
# can be tested on real hardware before ground testing.
#
# Record the bag with:  tools/record_bag.sh <name> <dur> --car-parity
# (that set is /imu /lidar/Lidar1 /motor_rpm /steering_angle /tf_static —
#  already free of the sim-only topics that must never reach the car).
#
# This script plays that bag with the transforms the car pipeline expects:
#   - /imu          -> NO REMAP. IMU is canonical /imu on BOTH sides: the uDV
#                      firmware publishes /imu and odometry_filter/slam
#                      subscribe to /imu in code (settled 2026-07-04, pipeline
#                      dev/v1.0.0 — the old /imu/data_raw car remap was
#                      dropped). Do NOT remap /imu.
#   - /lidar/Lidar1 -> /lidar_points   (REMAP_LIDAR_CAR — the only sensor remap
#                      the car needs; Hesai driver publishes /lidar_points)
#   - /tf_static replayed RELIABLE + TRANSIENT_LOCAL so late-joining TF
#     listeners (cone_detection) latch base_link->imu_link / ->hesai_lidar.
#
# It does NOT:
#   - pass --clock / use_sim_time (the sim bridge stamps on WALL CLOCK; the
#     car runs use_sim_time=false — correct as-is, no /clock in the bag).
#   - replay any command / safety / GT topics. /ctrl/cmd (Twist) is produced
#     LIVE by mission_control on the car; a sim /control_command has no car
#     subscriber. /testing_only/* would inject ground truth into slam_node.
#     The --exclude below defends against lifting a legacy (non-parity) bag.
#
# Prereqs on the car: ROS 2 Humble sourced, the DV pipeline (car profile)
# already running, and the uDV firmware up (it publishes the live state the
# bag deliberately omits).
#
# Usage:
#   tools/lift_to_car.sh <bag-path> [extra ros2 bag play args...]

set -euo pipefail

BAG="${1:-}"
if [ -z "$BAG" ] || [ ! -e "$BAG" ]; then
    echo "Usage: $0 <bag-path> [extra ros2 bag play args...]" >&2
    echo "  bag-path: a car-parity bag (tools/record_bag.sh ... --car-parity)" >&2
    exit 1
fi
shift || true

# QoS override so /tf_static latches for late TF subscribers.
QOS_YAML="$(mktemp -t tf_static_qos.XXXXXX.yaml)"
trap 'rm -f "$QOS_YAML"' EXIT
cat > "$QOS_YAML" <<'YAML'
/tf_static:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1
YAML

echo "==> Lifting $BAG onto the car pipeline"
echo "    remap: /lidar/Lidar1->/lidar_points  (/imu stays /imu — no remap)"
echo "    /tf_static: reliable+transient_local"
echo "    playing only the car-parity sensor set (sim-only/command/GT topics dropped)"

# --topics WHITELISTS exactly the car-parity sensor set. A --car-parity bag
# already contains only these (no-op filter); for a legacy full bag this is
# the belt-and-braces that keeps sim-only topics OFF the wire — /testing_only/*
# would inject GT into slam_node on the car, /Conos_raw would double the car's
# own cone_detection, /gps + command topics have no car use.
# NB: Humble `ros2 bag play` has NO --exclude flag; --topics (whitelist) is the
# portable way to filter. The /lidar/Lidar1 remap still applies to the
# whitelisted topic. Verified on a live parity bag (2026-07-10): /lidar_points
# + /imu receive and base_link->hesai_lidar / ->imu_link TF both resolve.
ros2 bag play "$BAG" \
    --remap /lidar/Lidar1:=/lidar_points \
    --qos-profile-overrides-path "$QOS_YAML" \
    --topics /imu /lidar/Lidar1 /motor_rpm /steering_angle /tf_static \
    "$@"

echo "==> Playback finished."
echo "    Validate: ros2 topic hz /imu ; ros2 topic hz /lidar_points"
echo "    and confirm cone_detection emits cones with no TF-lookup errors."
