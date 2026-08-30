#!/usr/bin/env bash
# Single-shot Godot → ROS-bridge wire compatibility check.
#
# Launches the Godot sim headless, waits for the publisher to come
# up, runs verify_sensor_frame.py against UDP 41452, then tears the
# sim down. Exits 0 only if the verifier saw >0 packets and 0 bad
# magic (i.e. the bytes Godot emits parse as FFSDSSensorFrame).
#
# Phase 5.2 caught a real bug this way: a 12-byte stale-padding tail
# left the publisher emitting 184-byte frames when sizeof(struct) is
# 172, so the bridge's strict `n == sizeof(frame)` check silently
# dropped every packet. Re-run this any time the wire layout changes.
#
# Mission Control integration steps (manual, run from a SEPARATE
# shell after this script passes):
#
#   1. tools/refresh-bridge.sh
#        — relays Docker Desktop's UDP backend so port bindings on
#          41452 / 41453 actually deliver into the container.
#   2. ros2 launch ifssim_bridge ifssim_bridge.launch.py
#        — bridge starts publishing /imu, /gss, /gps/fix, /odom,
#          /testing_only/odom, /fsds/lidar.
#   3. ros2 topic hz /imu
#        — should read ~400 Hz (matches Godot publisher physics tick).
#   4. ros2 topic echo /odom --once
#        — pose should match the Godot HUD's body v/a readouts.
#   5. Open Mission Control; sim selector → 'godot' → enable pipeline.
#        (As per project memory: this step is OPERATOR-RUN, not
#         automated, because it triggers the FS-DV lifecycle.)
#
# Bag recording: pass `-s mcap` (project memory: bags are mcap, not
# db3, so they open natively in Lichtblick / Foxglove):
#
#   ros2 bag record -s mcap -o godot_smoke -a
#
# Usage:
#   tools/verify_godot_pipeline.sh [--seconds N]
#
# Returns:
#   0  — verifier saw packets, all magic OK
#   1  — verifier failed or Godot didn't come up

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GODOT_DIR="${REPO_ROOT}/godot"
VERIFY="${GODOT_DIR}/sensors/verify_sensor_frame.py"

# Default: 8 s of capture (long enough for the 2 s referee settle +
# 5 s of sensor frames). Bump with --seconds for longer captures.
SECONDS_TO_RUN=8
while [[ $# -gt 0 ]]; do
    case "$1" in
        --seconds) SECONDS_TO_RUN="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

if ! command -v godot >/dev/null 2>&1; then
    echo "[verify] godot not on PATH" >&2
    exit 1
fi

LOG="$(mktemp -t godot_verify.XXXXXX)"
trap 'rm -f "${LOG}"' EXIT

# --quit-after is counted in frames; project ticks at 60 fps headless
# so multiply seconds by 60 with a 20-frame margin for startup.
QUIT_FRAMES=$(( (SECONDS_TO_RUN + 5) * 60 ))

echo "[verify] launching Godot headless (${SECONDS_TO_RUN}s capture window)..." >&2
(
    cd "${GODOT_DIR}"
    godot --headless --quit-after "${QUIT_FRAMES}" >"${LOG}" 2>&1 &
    echo $! > "${LOG}.pid"
)

GODOT_PID="$(cat "${LOG}.pid")"
trap 'kill "${GODOT_PID}" 2>/dev/null || true ; rm -f "${LOG}" "${LOG}.pid"' EXIT

# Give the publisher a moment to bind the socket.
sleep 5

echo "[verify] running verify_sensor_frame.py..." >&2
if python3 "${VERIFY}" --count 3 --timeout "${SECONDS_TO_RUN}" --every 80; then
    echo "[verify] PASS — Godot wire format is bridge-compatible." >&2
    exit 0
else
    echo "[verify] FAIL — see ${LOG} for Godot stdout." >&2
    tail -20 "${LOG}" >&2 || true
    exit 1
fi
