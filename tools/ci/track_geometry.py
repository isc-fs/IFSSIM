"""Track geometry for the sim check: the centreline of a cone track and how
tight its corners are. Pure Python (the GPU box has no numpy).

Shared by sim_report.py (the "tightest corner" column and the cone-hit table)
and gen_tracks.py (reject tracks the car cannot physically drive).

The car's turning circle is ~4.95 m (1.627 m wheelbase, 18.2 deg lock), so a
centreline corner tighter than that cannot be driven, whatever the controller.

Cones are (x, y, colour) with colour BLUE = 1 / YELLOW = 0, the referee's
numbering (cones.csv). Blue is the LEFT boundary and yellow the RIGHT one,
in the driving direction (the FS rules' convention).
"""

from __future__ import annotations

import math

YELLOW, BLUE = 0, 1
PAIR_MAX_M = 6.0         # a cone's partner is the nearest of the other colour within this
MIN_SEGMENT_M = 0.5      # shorter centreline segments are noise, not geometry
CLOSED_GAP_M = 7.0       # first and last midpoint closer than this: a closed loop
CAR_MIN_RADIUS_M = 4.95  # the car's turning circle at full lock
DEFAULT_FLOOR_M = 5.0    # default generator floor, just above it
SIDE = {BLUE: "blue", YELLOW: "yellow"}


def circumradius(a, b, c) -> float:
    """Radius of the circle through three points (inf if collinear)."""
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    return math.inf if cross < 1e-9 else ab * bc * ca / (2 * cross)


def midpoints(cones: list[tuple[float, float, int]]) -> list[tuple[float, float]]:
    """Centreline points: each blue cone with its nearest yellow (< PAIR_MAX_M),
    midway. The referee and the track CSVs list each colour in track order, so
    the blue cones' order is the centreline's."""
    blue = [(x, y) for x, y, c in cones if c == BLUE]
    yellow = [(x, y) for x, y, c in cones if c == YELLOW]
    out = []
    for b in blue:
        if not yellow:
            break
        y = min(yellow, key=lambda q: math.dist(q, b))
        if math.dist(y, b) < PAIR_MAX_M:
            out.append(((b[0] + y[0]) / 2, (b[1] + y[1]) / 2))
    return out


def is_closed(pts: list[tuple[float, float]]) -> bool:
    return len(pts) > 3 and math.dist(pts[0], pts[-1]) < CLOSED_GAP_M


def radii(pts: list[tuple[float, float]]) -> list[float]:
    """Circumradius at each interior point (every point on a closed loop);
    a triple with a segment under MIN_SEGMENT_M gives inf (skipped)."""
    n = len(pts)
    if n < 3:
        return []
    idx = range(n) if is_closed(pts) else range(1, n - 1)
    out = []
    for i in idx:
        a, b, c = pts[(i - 1) % n], pts[i], pts[(i + 1) % n]
        if math.dist(a, b) < MIN_SEGMENT_M or math.dist(b, c) < MIN_SEGMENT_M:
            out.append(math.inf)
        else:
            out.append(circumradius(a, b, c))
    return out


def corner_stats(cones: list[tuple[float, float, int]], threshold_m: float = CAR_MIN_RADIUS_M) -> dict:
    """{'min_radius_m': float|None, 'n_tight': int}. n_tight counts separate
    corners (runs of consecutive points) below threshold_m."""
    rs = radii(midpoints(cones))
    finite = [r for r in rs if math.isfinite(r)]
    if not finite:
        return {"min_radius_m": None, "n_tight": 0}
    n_tight, prev = 0, False
    for r in rs:
        tight = r < threshold_m
        n_tight += tight and not prev
        prev = tight
    return {"min_radius_m": round(min(finite), 2), "n_tight": n_tight}


def min_radius(cones: list[tuple[float, float, int]]) -> float | None:
    return corner_stats(cones)["min_radius_m"]


def radius_near(cones, x: float, y: float, span: int = 3) -> float | None:
    """The tightest radius within +-span centreline points of the one nearest (x, y)."""
    pts = midpoints(cones)
    if len(pts) < 3:
        return None
    rs = radii(pts)
    closed = is_closed(pts)
    off = 0 if closed else 1  # rs[k] is the radius at pts[k + off]
    i = min(range(len(pts)), key=lambda k: math.dist(pts[k], (x, y)))
    near = []
    for d in range(-span, span + 1):
        j = (i + d) % len(pts) if closed else i + d
        k = j - off
        if 0 <= k < len(rs) and math.isfinite(rs[k]):
            near.append(rs[k])
    return min(near) if near else None
