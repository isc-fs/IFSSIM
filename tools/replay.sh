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

# Recorded SLAM-side outputs land here; Lichtblick opens the directory
# directly (it understands rosbag2 storage). We blow away any previous
# replay's recording first since `ros2 bag record` refuses to overwrite.
# Storage is sqlite3 (the default) because the dv_pipeline_stack image
# doesn't carry the mcap rosbag2 plugin yet — once an image rebuild
# picks up Dockerfile's ros-humble-rosbag2-storage-mcap line, switch
# the -s flag below to mcap to write a single .mcap file instead.
HOST_REC_DIR="$HOST_BAG_DIR/replay_cone_slam"
rm -rf "$HOST_REC_DIR"

docker run --rm \
    --name "ifssim-replay-$$" \
    -v "$HOST_BAG_DIR:/replay/bag:ro" \
    -v "$HOST_BAG_DIR:/replay/out" \
    -v "$REPO/tools/dev/pose_cmp.py:/replay/pose_cmp.py:ro" \
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

        echo '==> Starting Cone_Detection (LIVE — picks up algorithm changes since the bag was recorded)'
        ros2 run slam Cone_Detection \\
            --ros-args -r /fsds/lidar/Lidar1:=/lidar/Lidar1 \\
            > /tmp/cone_detection.log 2>&1 &
        CONE_PID=\$!
        sleep 3   # numba JIT warmup

        echo '==> Starting cone_graph_slam'
        ros2 run cone_slam cone_graph_slam \\
            > /tmp/slam.log 2>&1 &
        SLAM_PID=\$!

        echo '==> Starting Plan_Path (path_planning)'
        ros2 run path_planning Plan_Path \\
            > /tmp/path.log 2>&1 &
        PATH_PID=\$!

        echo '==> Starting Control'
        # Same /control_command remap as the live launch; control no
        # longer needs GSS/odom subscriptions (it reads /cone_slam/state).
        # /Conos_Orange isn't published in replay (Cone_Detection only
        # emits /Conos_raw), so Control's orange-stop branch is naturally
        # disabled here — orange_callback just never receives.
        ros2 run control Control \\
            --ros-args \\
              -r /fsds/control_command:=/control_command \\
            > /tmp/control.log 2>&1 &
        CTRL_PID=\$!

        # Give SLAM + Plan_Path + Control a moment to subscribe before
        # the bag starts publishing — otherwise the very first IMU
        # samples can race ahead of the SLAM subscription and miss the
        # calibration window, and the downstream planners can miss the
        # first few publications.
        sleep 2

        echo '==> Starting MCAP recorder for SLAM outputs'
        # Record the SLAM-side topics into an mcap so the user can open
        # the run in Lichtblick afterward. We deliberately skip raw
        # sensor topics (already in the source bag) to keep the file
        # small. /tf_static is captured because Lichtblick needs it for
        # the 3D panel even though it rarely changes.
        ros2 bag record \\
            -o /replay/out/replay_cone_slam \\
            /tf /tf_static /cone_slam/state /Conos /Conos_raw /Path \\
            /control_command \\
            > /tmp/recorder.log 2>&1 &
        REC_PID=\$!
        sleep 1

        echo '==> Starting bag play (remap bag /Conos_raw to /dev/null so Cone_Detection owns the topic)'
        # The bag was recorded with cone_detection running live. Remap
        # the recorded /Conos_raw to a dead topic so the live
        # Cone_Detection's output (with current algorithm fixes —
        # cluster-centroid fallback + lower c bound) is the one
        # cone_slam consumes. Without this remap we'd get two
        # publishers with identical timestamps and integrate_to would
        # see the same t_end twice, raising 'skip scan' errors.
        ros2 bag play /replay/bag --disable-keyboard-controls \\
            --remap /Conos_raw:=/Conos_raw_recorded_unused \\
            > /tmp/bag.log 2>&1 &
        BAG_PID=\$!

        # Tiny grace period so the comparator's first spin sees both
        # publishers already advertising.
        sleep 1

        echo '==> Streaming pose comparison (${DURATION}s) — cone_slam'
        python3 /replay/pose_cmp.py $DURATION --slam-topic $SLAM_TOPIC

        echo '==> SLAM log tail (last 30 lines):'
        tail -30 /tmp/slam.log || true

        echo '==> Path planning log tail (PATH_RATE lines + last 5):'
        grep PATH_RATE /tmp/path.log | tail -10 || true
        echo '   --- last 5 lines of path.log ---'
        tail -5 /tmp/path.log || true

        echo '==> Control log tail (DIAG lines + last 5):'
        grep DIAG /tmp/control.log | tail -10 || true
        echo '   --- last 5 lines of control.log ---'
        tail -5 /tmp/control.log || true

        # Persist the full control log next to the recording so the
        # smoke test (and humans triaging) can scan the entire run, not
        # just the tail.
        cp /tmp/control.log /replay/out/replay_control.log 2>/dev/null || true

        echo '==> Recorder log tail (last 20 lines):'
        tail -20 /tmp/recorder.log || true

        # Stop the recorder cleanly first so its db3 closes properly.
        # SIGINT lets ros2 bag flush; SIGTERM would truncate the trailing
        # message_index/footer chunks and Lichtblick refuses to open the
        # file.
        kill -INT \$REC_PID 2>/dev/null || true
        wait \$REC_PID 2>/dev/null || true

        kill \$SLAM_PID \$BAG_PID \$CONE_PID \$PATH_PID \$CTRL_PID 2>/dev/null || true

        # Convert the sqlite3 recording to a single .mcap file for
        # Lichtblick. Once the image rebuild picks up the
        # ros-humble-rosbag2-storage-mcap line in Dockerfile, this is a
        # no-op apt-get; until then we install on the fly here so the
        # conversion works against today's image.
        if ! dpkg -s ros-humble-rosbag2-storage-mcap >/dev/null 2>&1; then
            echo '==> Installing rosbag2-storage-mcap (one-shot)'
            apt-get update -qq >/dev/null 2>&1 || true
            apt-get install -y ros-humble-rosbag2-storage-mcap >/dev/null 2>&1 || true
        fi
        if dpkg -s ros-humble-rosbag2-storage-mcap >/dev/null 2>&1; then
            echo '==> Converting recording to MCAP'
            cat > /tmp/convert.yaml <<YAML
output_bags:
  - uri: /replay/out/replay_cone_slam_mcap
    storage_id: mcap
    all: true
YAML
            rm -rf /replay/out/replay_cone_slam_mcap
            ros2 bag convert -i /replay/out/replay_cone_slam -o /tmp/convert.yaml \\
                > /tmp/convert.log 2>&1 || tail -20 /tmp/convert.log
            # Flatten: move the inner .mcap up one level and drop the dir.
            INNER_MCAP=\$(ls -1 /replay/out/replay_cone_slam_mcap/*.mcap 2>/dev/null | head -1 || true)
            if [ -n \"\$INNER_MCAP\" ]; then
                mv \"\$INNER_MCAP\" /replay/out/replay_cone_slam.mcap
                rm -rf /replay/out/replay_cone_slam_mcap
                echo \"==> MCAP at /replay/out/replay_cone_slam.mcap (\$(du -h /replay/out/replay_cone_slam.mcap | cut -f1))\"
            else
                echo '==> WARN: mcap conversion produced no file; sqlite3 dir is still available'
            fi
        else
            echo '==> mcap plugin unavailable; keeping sqlite3 recording'
        fi
    " | tee "$HOST_BAG_DIR/replay_pose_cmp_cone_slam.txt"

echo
echo "==> Done. Comparison saved to $HOST_BAG_DIR/replay_pose_cmp_cone_slam.txt"
HOST_MCAP_PATH="$HOST_BAG_DIR/replay_cone_slam.mcap"
if [ -f "$HOST_MCAP_PATH" ]; then
    echo "==> Lichtblick: open $HOST_MCAP_PATH"
    echo "    (http://localhost:8080 → Open file → drag the .mcap in)"
elif [ -d "$HOST_REC_DIR" ]; then
    echo "==> Lichtblick: open the directory $HOST_REC_DIR"
    echo "    (Lichtblick at http://localhost:8080 → Open data source → ROS 2 bag → pick the dir)"
fi
