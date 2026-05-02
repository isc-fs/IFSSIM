#!/bin/bash
# Refresh the dv_pipeline_stack container after editing bridge code, the
# launch file, the entrypoint, or the Fast DDS profile.
#
# `docker compose restart` is NOT enough — Docker Desktop on macOS
# accumulates UDP proxy state and Fast DDS leaves SHM segments around;
# both eventually wedge in ways that make /lidar/Lidar1 disappear from
# external subscribers even when the bridge process is publishing
# internally. Only a full container teardown + recreate consistently
# recovers (verified by an afternoon of chasing this).
#
# What this script does, in order:
#   1. `docker compose down dv_pipeline_stack` — destroys the container
#      (and with it: the host-side UDP proxy, /dev/shm tmpfs, any wedged
#      Fast DDS daemon state, the rw fs layer).
#   2. `docker compose up -d dv_pipeline_stack` — recreates from image,
#      remounts /dev/shm, rebinds host UDP proxies for 41452 and 51453.
#      LIDAR_TRANSPORT defaults to udp here; pass it as an env var to
#      override (e.g. `LIDAR_TRANSPORT=tcp ./refresh-bridge.sh`).
#   3. Wait for the container to report healthy.
#   4. Copy the host's current launch.py and entrypoint.sh into the
#      container — they're COPY'd into the image at build time, not
#      bind-mounted, so any edits since the last image build only
#      reach a fresh container if we paste them in.
#   5. `colcon build --packages-select ifssim_bridge --symlink-install`
#      from the bind-mounted source so any C++ edits land.
#   6. `docker compose restart dv_pipeline_stack` so the bridge process
#      picks up the freshly built install/ tree + the new launch +
#      entrypoint /dev/shm cleanup.
#
# Anything not under ifssim_bridge (e.g. cone_slam, control,
# path_planning) is Python and lives via --symlink-install — those
# pick up host edits without needing this script. Run this script
# only when the bridge or its container plumbing has changed.

set -euo pipefail

cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

LIDAR_TRANSPORT="${LIDAR_TRANSPORT:-udp}"
export LIDAR_TRANSPORT

echo "→ docker compose down dv_pipeline_stack"
docker compose down dv_pipeline_stack 2>&1 | tail -3

echo "→ docker compose up -d dv_pipeline_stack  (LIDAR_TRANSPORT=$LIDAR_TRANSPORT)"
docker compose up -d dv_pipeline_stack 2>&1 | tail -3

echo "→ waiting for healthy..."
for _ in $(seq 1 30); do
    status=$(docker compose ps --format '{{.Name}} {{.Status}}' 2>/dev/null \
             | awk '/dv_pipeline_stack-1/ {for (i=2;i<=NF;i++) printf "%s ", $i; print ""}' \
             | tr -d '()')
    if [[ "$status" == *"healthy"* ]]; then
        echo "  ready: $status"
        break
    fi
    sleep 1
done

echo "→ rebuilding ifssim_bridge from host source"
docker compose exec -T dv_pipeline_stack bash -lc \
    'cd /dv_pipeline_stack_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select ifssim_bridge --symlink-install' \
    2>&1 | tail -3

echo "→ copying launch + entrypoint into container"
docker compose cp docker/dv_pipeline_stack/bridge.launch.py        dv_pipeline_stack:/dv_pipeline_stack_ws/bridge.launch.py        >/dev/null
docker compose cp docker/dv_pipeline_stack/pipeline.launch.py      dv_pipeline_stack:/dv_pipeline_stack_ws/pipeline.launch.py      >/dev/null
docker compose cp docker/dv_pipeline_stack/pipeline_only.launch.py dv_pipeline_stack:/dv_pipeline_stack_ws/pipeline_only.launch.py >/dev/null
docker compose cp docker/dv_pipeline_stack/entrypoint.sh           dv_pipeline_stack:/entrypoint.sh                                >/dev/null
docker exec ifssim-dv_pipeline_stack-1 chmod +x /entrypoint.sh

echo "→ docker compose restart dv_pipeline_stack"
docker compose restart dv_pipeline_stack 2>&1 | tail -2

echo
echo "✓ bridge refreshed. Tail logs with:"
echo "    docker compose logs -f dv_pipeline_stack"
