#!/usr/bin/env bash
# List bags currently sitting in the dv_pipeline_stack container's
# bag volume (#465 v3).
#
# Background: bag recordings used to land in a host-bind-mounted
# `bags/` directory automatically. On Windows (WSL2 9p) + macOS
# (virtiofs) the cross-filesystem move of multi-GB bags blocked the
# StopBag service for ~30 s, visibly stalling Mission Control. We
# moved bags to a docker-managed named volume (single-fs writes,
# <1 s move) and added explicit pull/list helpers to keep the
# "operator finds bags on the host" UX without paying the perf cost
# on every session-stop.
#
# Usage:
#   tools/list-bags.sh           # one bag per line, name + size + mtime
#   tools/list-bags.sh --paths   # absolute container paths only
#
set -euo pipefail

CONTAINER="${IFSSIM_DV_CONTAINER:-ifssim-dv_pipeline_stack-1}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "error: container '$CONTAINER' is not running" >&2
    echo "       (start with 'docker compose up -d dv_pipeline_stack' first)" >&2
    exit 1
fi

case "${1:-}" in
    --paths)
        # Just the absolute paths — useful for piping into pull-bag.sh
        # or scripting (`xargs -I{} basename {}` etc.).
        docker exec "$CONTAINER" bash -lc \
            'find /bags -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort'
        ;;
    "" )
        # Human-readable: name (with " — " separator) size, modified.
        # `du -sh` on each dir is cheap (the volume's local ext4).
        docker exec "$CONTAINER" bash -lc '
            shopt -s nullglob
            cd /bags
            entries=( */ )
            if (( ${#entries[@]} == 0 )); then
                echo "(no bags in volume)" >&2
                exit 0
            fi
            for d in "${entries[@]}"; do
                name="${d%/}"
                size=$(du -sh "$d" 2>/dev/null | cut -f1)
                # mtime of the .mcap inside; falls back to dir mtime.
                mcap=$(ls -1 "$d"*.mcap 2>/dev/null | head -1)
                mtime=$(stat -c "%y" "${mcap:-$d}" 2>/dev/null | cut -d. -f1)
                printf "%-60s  %6s  %s\n" "$name" "$size" "$mtime"
            done
        '
        ;;
    -h|--help)
        sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
        ;;
    *)
        echo "unknown arg: $1" >&2
        echo "usage: $0 [--paths]" >&2
        exit 2
        ;;
esac
