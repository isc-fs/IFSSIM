"""Track-centering utilities — dependency-free (stdlib only).

A track CSV is one `type,x,y[,...]` row per line (metres, ENU). The
simulator's track area (the `floor` plane in customMap) is ~200 m × 250 m
centred on the origin, so a track whose cones sit mostly to one side of the
origin spills off the floor — cones beyond the plane miss the spawner's
ground line-trace and fall through.

The random-track-generator pins each track's start gate at (0, 0) and lets the
loop sprawl off to one side, so a track whose start line is near one end of
the loop overruns the floor. This module recentres such tracks at *load* time,
in code we own and deploy (the backend image) — the generator lives in a
third-party git submodule that's baked into the image, so we don't touch it.

"Centred" means the bounding box of the **blue/yellow** boundary cones is
centred on the origin. The orange start gate is *not* part of the bbox; it
rides along with whatever shift the boundary needs, so the car still spawns on
the start line (the sim derives the spawn pose from the orange cones, not the
world origin — see FSDSConeSpawner::ComputeStartGatePose).
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

# Cones whose bbox defines "centred" — the track boundary only.
_BBOX_TYPES = frozenset({"blue", "yellow"})
# Cone rows that get translated when recentring — the whole track, gate included.
_SHIFT_TYPES = frozenset({"blue", "yellow", "big_orange", "small_orange", "orange"})

# A track whose boundary-bbox centre is within this distance of the origin is
# treated as already centred and left untouched. Gate-at-origin tracks are off
# by tens of metres, so this only needs to absorb authoring noise.
CENTERED_TOL_M = 0.5


def _cone_xy(parts: List[str]) -> Optional[Tuple[float, float]]:
    """(x, y) for a cone row, or None if the row isn't a parseable cone."""
    if len(parts) < 3:
        return None
    if parts[0].strip().lower() not in _SHIFT_TYPES:
        return None
    try:
        return float(parts[1]), float(parts[2])
    except ValueError:
        return None


def bbox_center(path: str) -> Optional[Tuple[float, float]]:
    """Centre of the blue/yellow bounding box, or None if there are none.

    Cheap: a single pass tracking min/max — no per-cone translation and no
    file write. This is the "is it centred?" workhorse.
    """
    xmin = ymin = float("inf")
    xmax = ymax = float("-inf")
    found = False
    with open(path, "r") as f:
        for line in f:
            parts = line.split(",")
            if parts[0].strip().lower() not in _BBOX_TYPES:
                continue
            xy = _cone_xy(parts)
            if xy is None:
                continue
            x, y = xy
            xmin, xmax = min(xmin, x), max(xmax, x)
            ymin, ymax = min(ymin, y), max(ymax, y)
            found = True
    if not found:
        return None
    return (xmin + xmax) / 2.0, (ymin + ymax) / 2.0


def is_centered(path: str, tol: float = CENTERED_TOL_M) -> bool:
    """True if the track's boundary bbox is already centred on the origin.

    A track with no blue/yellow cones is reported centred (nothing to do).
    """
    center = bbox_center(path)
    if center is None:
        return True
    cx, cy = center
    return max(abs(cx), abs(cy)) <= tol


def ensure_centered(path: str, tol: float = CENTERED_TOL_M) -> bool:
    """Recentre `path` in place if it isn't already centred.

    Returns True if the file was rewritten, False if it was left untouched.
    Only the x/y of cone rows are shifted; trailing columns, header lines,
    comments and blank lines are preserved verbatim.
    """
    center = bbox_center(path)
    if center is None:
        return False
    cx, cy = center
    if max(abs(cx), abs(cy)) <= tol:
        return False

    with open(path, "r") as f:
        lines = f.readlines()

    out: List[str] = []
    for line in lines:
        parts = line.rstrip("\n").split(",")
        xy = _cone_xy(parts)
        if xy is None:
            out.append(line)  # header / comment / blank / non-cone — verbatim
            continue
        x, y = xy
        parts[1] = repr(x - cx)
        parts[2] = repr(y - cy)
        out.append(",".join(parts) + ("\n" if line.endswith("\n") else ""))

    # temp file + replace so an interrupted write can't leave a half-rewritten
    # track on disk.
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.writelines(out)
    os.replace(tmp, path)
    return True
