#!/bin/bash
# Refresh the dv_pipeline_stack container after editing bridge code,
# any pipeline ROS source, the launch files, the entrypoint, or the
# Fast DDS profile.
#
# `docker compose restart` is NOT enough — Docker Desktop on macOS
# accumulates UDP proxy state and Fast DDS leaves SHM segments around;
# both eventually wedge in ways that make /lidar/Lidar1 disappear from
# external subscribers even when the bridge process is publishing
# internally. Only a full container teardown + recreate consistently
# recovers (verified by an afternoon of chasing this).
#
# #490 — pipeline source is no longer bind-mounted. We rebuild the
# image so the new source is baked in, then recreate the container.
# A repo-wide .dockerignore keeps the build context small (~200 MB
# vs. the unbounded ~33 GB it used to be) so the rebuild stays fast
# even on Windows + WSL2 with the repo on /mnt/c.
#
# What this script does, in order:
#   1. `docker compose build dv_pipeline_stack` — rebuild the image
#      against the current host source. BuildKit's layer cache makes
#      this fast for incremental Python edits (only the final COPY
#      + colcon build layers re-run).
#   2. `docker compose up -d --force-recreate dv_pipeline_stack` —
#      destroys + recreates the container, drops any wedged DDS SHM
#      / UDP proxy state, mounts the new image.
#   3. Wait for the container to report healthy.
#
# Run this any time you edit:
#   - pipeline/* ROS Python or C++ source
#   - ros2/src/* (fs_msgs / ifssim_bridge)
#   - docker/dv_pipeline_stack/{bridge,pipeline,pipeline_only}.launch.py
#   - docker/dv_pipeline_stack/entrypoint.sh
#   - docker/dv_pipeline_stack/fastdds_profile.xml

set -euo pipefail

# Git Bash on Windows rewrites Linux-looking paths like /entrypoint.sh
# before native Windows executables see them. Docker commands need those
# paths to reach the Linux container unchanged.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

echo "→ docker compose build dv_pipeline_stack"
docker compose build dv_pipeline_stack 2>&1 | tail -3

echo "→ docker compose up -d --force-recreate dv_pipeline_stack"
docker compose up -d --force-recreate dv_pipeline_stack 2>&1 | tail -3

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

echo
echo "✓ bridge refreshed. Tail logs with:"
echo "    docker compose logs -f dv_pipeline_stack"
