"""Where on the track a sim check run went wrong.

For a run that didn't finish, finds the first moment the car left the track
(or, if it never did, where it stopped) and describes that place: how far
round the lap it is, and the corner there. The sim check reports these, and
the report issue keeps them, so failures can be compared across tracks.

Inputs are the run's cones.csv (the referee's cones, in track order per
colour) and telemetry.csv, both in ENU metres. Pure Python: it runs on the
GPU box's own interpreter, which has no numpy.
"""

from __future__ import annotations

import math

YELLOW, BLUE = 0, 1
# The car is off the track once its centre is this far past the cone line.
OFF_MARGIN_M = 1.0
# Corner radius is measured over this much centreline either side.
RADIUS_SPAN_M = 4.0
STRAIGHT_R_M = 60.0
# The corner a failure is put down to: the tightest within this window (m).
CORNER_BEHIND_M, CORNER_AHEAD_M = 3, 15


def centreline(cones: list[dict]) -> list[tuple[float, float, float]]:
    """(x, y, half width) along the track: each blue cone with its nearest
    yellow. The referee lists each colour in track order, so the blue cones'
    order is the centreline's."""
    blue = [(c["x"], c["y"]) for c in cones if int(c.get("color", -1)) == BLUE]
    yellow = [(c["x"], c["y"]) for c in cones if int(c.get("color", -1)) == YELLOW]
    if len(blue) < 5 or len(yellow) < 5:
        return []
    out = []
    for b in blue:
        y = min(yellow, key=lambda q: (q[0] - b[0]) ** 2 + (q[1] - b[1]) ** 2)
        out.append(((b[0] + y[0]) / 2, (b[1] + y[1]) / 2, math.dist(b, y) / 2))
    return out


class Track:
    """A closed centreline with arc length, for projecting the car onto."""

    def __init__(self, line: list[tuple[float, float, float]]):
        self.p = line
        self.n = len(line)
        self.s = [0.0]
        for i in range(self.n):
            a, b = line[i], line[(i + 1) % self.n]
            self.s.append(self.s[-1] + math.dist(a[:2], b[:2]))
        self.length = self.s[-1]

    def project(self, x: float, y: float, near: int | None = None, window: int = 25):
        """(arc length, distance from the centreline, half width there, segment)."""
        idx = range(self.n) if near is None else (
            (near + k) % self.n for k in range(-window, window + 1))
        best = None
        for i in idx:
            a, b = self.p[i], self.p[(i + 1) % self.n]
            dx, dy = b[0] - a[0], b[1] - a[1]
            l2 = dx * dx + dy * dy
            t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / l2))
            px, py = a[0] + t * dx, a[1] + t * dy
            d = math.hypot(x - px, y - py)
            if best is None or d < best[1]:
                best = (self.s[i] + t * math.sqrt(l2), d, a[2] + t * (b[2] - a[2]), i)
        return best

    def point(self, s: float) -> tuple[float, float]:
        s %= self.length
        i = max(0, min(self.n - 1, _bisect(self.s, s) - 1))
        a, b = self.p[i], self.p[(i + 1) % self.n]
        seg = self.s[i + 1] - self.s[i]
        t = 0.0 if seg < 1e-9 else (s - self.s[i]) / seg
        return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])

    def curvature(self, s: float, span: float = RADIUS_SPAN_M) -> float:
        """Signed curvature at s (1/m, + = turning left in the direction of
        increasing s), from the heading change over ±span."""
        p0, p1, p2 = self.point(s - span), self.point(s), self.point(s + span)
        h1 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        h2 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        dh = (h2 - h1 + math.pi) % (2 * math.pi) - math.pi
        return dh / span


def _bisect(a: list[float], v: float) -> int:
    lo, hi = 0, len(a)
    while lo < hi:
        mid = (lo + hi) // 2
        if a[mid] <= v:
            lo = mid + 1
        else:
            hi = mid
    return lo


def locate(cones: list[dict], tel: list[dict]) -> dict | None:
    """Where a run that didn't finish went wrong, or None if that can't be
    told (no cones or no telemetry)."""
    line = centreline(cones)
    pts = [p for p in tel if "x" in p and "y" in p]
    if not line or len(pts) < 2:
        return None
    tr = Track(line)
    if tr.length < 20:
        return None

    s0, _, _, seg = tr.project(pts[0]["x"], pts[0]["y"])
    # Which way round the car drives: the sign of its first few metres.
    direction = 1
    for p in pts[1:]:
        if math.dist((p["x"], p["y"]), (pts[0]["x"], pts[0]["y"])) > 5.0:
            s1 = tr.project(p["x"], p["y"])[0]
            ds = (s1 - s0 + tr.length / 2) % tr.length - tr.length / 2
            direction = 1 if ds >= 0 else -1
            break

    travelled, prev_s = 0.0, s0
    found, kind = None, None
    last_moving = pts[0]
    last_moving_travelled = 0.0
    for p in pts:
        s, d, half, near = tr.project(p["x"], p["y"], near=seg)
        if d > half + OFF_MARGIN_M:
            # Progress is where it last was on the track: off it, the nearest
            # centreline point can be anywhere.
            found, kind = (p, prev_s, d, half, travelled), "left the track"
            break
        seg = near
        ds = (s - prev_s + tr.length / 2) % tr.length - tr.length / 2
        travelled += direction * ds
        prev_s = s
        if p.get("speed", 1.0) > 0.3:
            last_moving, last_moving_travelled = p, travelled
    if found is None:
        p = last_moving
        s, d, half, _ = tr.project(p["x"], p["y"])
        moved = any(q.get("speed", 0.0) > 0.3 for q in pts)
        found, kind = (p, s, d, half, last_moving_travelled), "stopped" if moved else "never moved"

    p, s, d, half, travelled = found
    # The corner the car was in or about to take: the tightest point from a
    # few metres back to CORNER_AHEAD_M ahead (+ = it turns left as driven).
    ks = [tr.curvature(s + direction * a) * direction
          for a in range(-CORNER_BEHIND_M, CORNER_AHEAD_M + 1)]
    k = max(ks, key=abs)
    radius = abs(1 / k) if abs(k) > 1e-6 else math.inf
    if radius >= STRAIGHT_R_M:
        corner = "straight"
    else:
        corner = f"{'left' if k > 0 else 'right'} R {radius:.0f} m"
    progress = max(0.0, travelled)
    return {
        "kind": kind,
        "x": round(p["x"], 2),
        "y": round(p["y"], 2),
        "t_s": round(p.get("t", 0.0) - pts[0].get("t", 0.0), 1),
        "progress_m": round(progress, 1),
        "lap_m": round(tr.length, 1),
        "progress_pct": round(100 * min(progress, tr.length) / tr.length),
        "radius_m": None if math.isinf(radius) else round(radius, 1),
        "corner": corner,
        "off_by_m": round(max(0.0, d - half), 2),
        "speed": round(p["speed"], 2) if "speed" in p else None,
    }
