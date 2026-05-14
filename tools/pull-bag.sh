#!/usr/bin/env bash
# Pull a finalised bag from the dv_pipeline_stack container's bag
# volume onto the host's `bags/` directory (#465 v3).
#
# Why: live recording lands in a docker named volume to avoid the
# virtiofs/9p cross-fs move overhead that used to stall Mission
# Control's session-stop click on macOS + Windows (see comment in
# docker-compose.yml on the `ifssim_bags` volume). This helper does
# the explicit `docker cp` step for an operator who actually wants
# to inspect / replay the bag on the host.
#
# Usage:
#   tools/pull-bag.sh <bag_name>
#   tools/pull-bag.sh trackdrive_TrainingMap_20260514_173022
#
# Output:
#   ./bags/<bag_name>/                — full bag directory on host
#   ./bags/<bag_name>/<bag_name>_0.mcap
#   ./bags/<bag_name>/metadata.yaml
#
# Idempotent: refuses to overwrite an existing host dir; pass --force
# to override (rm -rf then re-cp).
#
# To get the list of available bags first:  tools/list-bags.sh
set -euo pipefail

CONTAINER="${IFSSIM_DV_CONTAINER:-ifssim-dv_pipeline_stack-1}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
HOST_BAGS_DIR="${IFSSIM_HOST_BAGS_DIR:-$REPO/bags}"

FORCE=false
BAG_NAME=""
while [ $# -gt 0 ]; do
    case "$1" in
        --force) FORCE=true; shift ;;
        -h|--help)
            sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        --) shift; break ;;
        -*)
            echo "unknown flag: $1" >&2
            exit 2
            ;;
        *)
            if [ -z "$BAG_NAME" ]; then
                BAG_NAME="$1"; shift
            else
                echo "extra arg: $1" >&2
                exit 2
            fi
            ;;
    esac
done

if [ -z "$BAG_NAME" ]; then
    echo "usage: $0 <bag_name> [--force]" >&2
    echo "       (run 'tools/list-bags.sh' to see what's available)" >&2
    exit 2
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "error: container '$CONTAINER' is not running" >&2
    exit 1
fi

# Verify the bag exists in the volume before we touch the host side.
if ! docker exec "$CONTAINER" test -d "/bags/$BAG_NAME"; then
    echo "error: /bags/$BAG_NAME doesn't exist in the container" >&2
    echo "       (run 'tools/list-bags.sh' for the right name)" >&2
    exit 1
fi

DEST="$HOST_BAGS_DIR/$BAG_NAME"

if [ -e "$DEST" ]; then
    if [ "$FORCE" = "true" ]; then
        echo "==> removing existing $DEST"
        rm -rf "$DEST"
    else
        echo "error: $DEST already exists — pass --force to overwrite" >&2
        exit 1
    fi
fi

mkdir -p "$HOST_BAGS_DIR"

echo "==> docker cp $CONTAINER:/bags/$BAG_NAME -> $DEST"
# `docker cp` between a container and the host is the fastest path
# Docker Desktop has for this — it uses the same vmcompute pipe as
# `docker run`'s stdio, NOT the bind-mount virtiofs/9p layer. For
# multi-GB bags this is ~10x faster than `cp` across a bind mount.
docker cp "$CONTAINER:/bags/$BAG_NAME" "$DEST"

# Summarise so the operator knows what they got.
echo
echo "==> done"
du -sh "$DEST" | awk '{print "    size: " $1}'
ls -1 "$DEST" | awk '{print "    file: " $0}'
echo
echo "Replay with:  tools/replay.sh $BAG_NAME"
echo "Open in Lichtblick at http://localhost:8080 (mcap file picker)"
