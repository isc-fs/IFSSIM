#!/usr/bin/env bash
# tools/measure_gpu_lidar.sh — Phase 0 viability measurement for #223.
#
# Samples UE5's process CPU% every 5 s for a fixed window, reports avg/min/max.
# Pair with `stat unit` / `stat gpu` in PIE for the GPU + GT delta.
#
# Procedure:
#   1. Restart UE5 (so the spike dylib is loaded).
#   2. Open editor console (`~` key), set CVar:
#        fsds.LidarGPUSpike.Enable 1
#   3. Press Play, drive normally. In PIE console run:
#        stat unit
#        stat gpu
#   4. From a separate terminal run this script:
#        ./tools/measure_gpu_lidar.sh 30        # 30 s window (default)
#   5. Stop PIE, set CVar back to 0, repeat for the baseline.
#   6. Diff the two avg CPU% numbers.
#
# Phase-0 decision gate:
#   - avg CaptureScene() GT cost (from UE log)            < 1.0 ms
#   - SceneCaptures GPU pass cost (from `stat gpu`)        < 2.0 ms
#   - Δ overall UE5 process CPU% vs spike-off baseline     < 30 %
# All three pass → proceed Phase 1. Any one fails → diagnose first.

set -eo pipefail

WINDOW_SECONDS="${1:-30}"
SAMPLE_INTERVAL=5

PID="$(pgrep -f 'UnrealEditor.*IFSSIM.uproject' | head -1 || true)"
if [[ -z "$PID" ]]; then
  echo "error: UE5 editor not running with IFSSIM.uproject" >&2
  exit 1
fi
echo "→ UE5 PID: $PID"
echo "→ Sampling for ${WINDOW_SECONDS}s (every ${SAMPLE_INTERVAL}s)..."

SAMPLES=$(( WINDOW_SECONDS / SAMPLE_INTERVAL ))
N=$(( SAMPLES + 1 ))
RAW="$(top -pid "$PID" -l "$N" -s "$SAMPLE_INTERVAL" -stats pid,cpu 2>/dev/null \
       | awk -v pid="$PID" '$1 == pid { print $2 }')"

if [[ -z "$RAW" ]]; then
  echo "error: no samples captured (process may have exited)" >&2
  exit 1
fi

echo "→ Samples (CPU%):"
echo "$RAW" | awk '{ printf "    %s\n", $1 }'
echo "$RAW" | awk '
  { v=$1+0; sum+=v; n++; if (n==1 || v<min) min=v; if (n==1 || v>max) max=v }
  END { printf "→ avg=%.1f%%  min=%.1f%%  max=%.1f%%  (n=%d)\n", sum/n, min, max, n }'

echo
echo "Now check UE5 log for: 'FSDS LiDAR GPU Spike: avg CaptureScene() game-thread = X.XXX ms'"
echo "And in PIE console: stat unit / stat gpu (look for SceneCaptures row in stat gpu)"
