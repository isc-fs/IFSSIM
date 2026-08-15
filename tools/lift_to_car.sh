#!/usr/bin/env bash
# Lift a sim-recorded rosbag onto the REAL CAR (isc-fs/IFS08-DV-uDV) — replay
# a car-parity sim bag into the live car pipeline so the whole autonomy stack
# can be tested on real hardware with the car UP ON STANDS before ground testing.
#
# Record the bag with:  tools/record_bag.sh <name> <dur> --car-parity
# (that set is exactly /imu + /lidar/Lidar1 — see below for why only those two).
#
# WHY ONLY /imu + /lidar (the "car up on stands, thinking it's moving" model):
#   The uDV firmware is the ROS<->CAN bridge. On a live car it PUBLISHES its own
#   /imu (BMI088), /steering_angle (steering-board CAN 0x528) and /motor_rpm
#   (ECU CAN 0x506), and SUBSCRIBES /ctrl/cmd (Twist) which it relays over CAN to
#   the ECU (motor/inverter) + steering board. So on the bench:
#     - /lidar : the Hesai sees the garage (no cones) -> useless -> REPLAY it.
#     - /imu   : the car is on stands = physically stationary, so the real IMU
#                reads zero motion and odometry/SLAM would collapse at the first
#                corner -> REPLAY the sim IMU to supply the yaw/accel that match
#                the replayed LiDAR.
#     - /motor_rpm, /steering_angle : LIVE from the uDV. The wheels + steering
#                actually move (up on stands) in response to the pipeline's live
#                /ctrl/cmd, and their real feedback is exactly what we want to
#                validate. DO NOT replay them.
#     - /tf_static : LIVE from the car's URDF/robot_state_publisher
#                (base_link->imu_link / ->hesai_lidar). DO NOT replay it.
#     - /ctrl/cmd, /control_command : produced LIVE by the pipeline; they drive
#                the real actuators. NEVER replay (stale commands to real motors).
#
# BENCH SETUP CONSEQUENCE (must do this or you double-publish):
#   Because /imu and /lidar_points are replayed here, the LIVE sources of those
#   two topics MUST be silenced during the test:
#     - disable the uDV's /imu publisher (bench/replay mode), and
#     - do NOT launch the Hesai driver (it would also publish /lidar_points).
#   Keep the uDV's /motor_rpm + /steering_angle publishers and its /ctrl/cmd
#   subscription LIVE — those are the actuator loop under test.
#
# Transforms applied on replay:
#   - /imu          -> NO REMAP. IMU is canonical /imu on BOTH sides (uDV
#                      publishes /imu; odometry_filter/slam subscribe /imu —
#                      settled 2026-07-04, pipeline dev/v1.0.0).
#   - /lidar/Lidar1 -> /lidar_points   (REMAP_LIDAR_CAR — Hesai driver topic)
#
# It does NOT pass --clock / use_sim_time: the sim bridge stamps IMU/LiDAR on
# wall clock and the car runs use_sim_time=false, so replayed stamps flow
# straight through. (feat/501's sim-capture stamping only matters inside the sim.)
#
# Prereqs on the car: ROS 2 Humble sourced, the DV pipeline (car profile)
# running, the uDV firmware up in bench/replay mode (live /motor_rpm +
# /steering_angle, /imu publisher OFF), and the Hesai driver NOT running.
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

echo "==> Lifting $BAG onto the car pipeline (car on stands)"
echo "    replaying ONLY /imu + /lidar/Lidar1  (remap /lidar/Lidar1->/lidar_points)"
echo "    /motor_rpm, /steering_angle, /tf_static come LIVE from the car — not replayed"
echo "    reminder: uDV /imu publisher OFF and Hesai driver NOT running on the bench"

# --topics WHITELISTS exactly the two replayed sensors. A --car-parity bag
# already contains only these (no-op filter); for a legacy full bag this keeps
# every other topic OFF the wire — /steering_angle + /motor_rpm from an old bag
# would fight the live uDV feedback, /testing_only/* would inject GT into
# slam_node, /Conos_raw would double the car's cone_detection.
# NB: Humble `ros2 bag play` has NO --exclude flag; --topics (whitelist) is the
# portable filter. The /lidar/Lidar1 remap still applies to the whitelisted topic.
ros2 bag play "$BAG" \
    --remap /lidar/Lidar1:=/lidar_points \
    --topics /imu /lidar/Lidar1 \
    "$@"

echo "==> Playback finished."
echo "    Validate: ros2 topic hz /imu ; ros2 topic hz /lidar_points"
echo "    and confirm cone_detection emits cones (car's live TF resolves the frames)."
