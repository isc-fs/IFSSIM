"""A random track as control points for the track editor.

The random generator (tools/random-track-generator) only writes cones: a ring
of blue cones and a ring of yellow ones, each in order. The editor draws with
control points that envgen turns into cones, so this takes the centreline back
out of the cones (each blue cone's midpoint with its nearest yellow one) and
thins it to the points that carry its shape: dense in corners, few on
straights. Saved from the editor, it is then an ordinary drawn track.
"""

from typing import List, Sequence, Tuple

from shapely.geometry import LinearRing

Point = Tuple[float, float]

# How far the thinned path may stray from the cone centreline. The generator's
# tightest corner has a 3.75 m radius; 0.8 m keeps its corners and leaves
# 35-55 points on a 350-500 m track, few enough to edit by hand.
TOLERANCE_M = 0.8


def _nearest(p: Point, pts: Sequence[Point]) -> Point:
    return min(pts, key=lambda q: (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2)


def centreline(blue: Sequence[Point], yellow: Sequence[Point]) -> List[Point]:
    """Midpoints of each blue cone with its nearest yellow cone, in blue's order."""
    return [((b[0] + y[0]) / 2.0, (b[1] + y[1]) / 2.0) for b in blue for y in [_nearest(b, yellow)]]


def control_points(blue: Sequence[Point], yellow: Sequence[Point],
                   tolerance: float = TOLERANCE_M) -> List[Point]:
    """The editor's control points for a closed track with these cones.

    In driving order (blue on the left, as envgen colours them), starting at
    the point nearest the origin, where the generator put the start.
    """
    if len(blue) < 3 or len(yellow) < 3:
        raise ValueError("not enough cones for a track")
    mid = centreline(blue, yellow)
    pts = list(LinearRing(mid).simplify(tolerance).coords)[:-1]   # the ring repeats its first point
    if len(pts) < 3:
        pts = mid

    # Driving order: blue cones on the left of the direction of travel.
    left = 0
    for i, a in enumerate(pts):
        b = pts[(i + 1) % len(pts)]
        c = _nearest(a, blue)
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        left += 1 if cross > 0 else -1
    if left < 0:
        pts.reverse()

    # Start where the generator's start was: at the origin.
    k = min(range(len(pts)), key=lambda i: pts[i][0] ** 2 + pts[i][1] ** 2)
    pts = pts[k:] + pts[:k]
    return [(round(x, 3), round(y, 3)) for x, y in pts]
