"""From a drawn path to cones: the corridor.

    spec points --Catmull-Rom--> smooth centreline --offset +-w/2--> boundaries
    --spacing by curvature--> blue (left) and yellow (right) cones, a start gate,
    and the spec's sections applied on top.

Conventions, all in the track CSV's frame (metres, right-handed, +y to the left
of +x; docs/environment_sidecar.md):
- the car drives from points[0] toward points[1];
- blue is the LEFT boundary and yellow the RIGHT, as the FS rules and the
  pipeline's planner (left chain = blue) expect;
- the start gate is four big orange cones around points[0], 2.6 m deep and
  wider than the track, the layout ComputeStartGatePose reads.

THE START ZONE. On a closed loop the sim decides which way the car faces from
the four track cones nearest the gate (startpose.py). Cones placed only by
spacing can put two behind the gate and two ahead, and then the direction is
decided by centimetres. So the start zone is laid out by hand: two cone pairs
just ahead of the gate (2.0 m and 3.5 m) and nothing within 5 m behind it.
The four nearest are then always ahead. Sections never touch the start zone.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

from .spec import Section, TrackSpec

DS = 0.25                  # centreline sample spacing [m]
CORNER_K = 0.35            # corner spacing = CORNER_K x boundary radius (~20 deg a cone)
GATE_DEPTH = 2.6           # along-track depth of a start/finish gate [m]
GATE_MARGIN = 0.5          # gate cones sit this far outside the cone lines [m]
GATE_HALF_MIN = 2.2        # FSG gate is 4.4 m across
AHEAD_FIRST = 2.0          # first cone pair ahead of the start gate [m]
AHEAD_SECOND = 3.5         # second cone pair ahead of the start gate [m]
GAP_BEHIND = 5.0           # no cones this close behind the start gate [m]
FINISH_CLEAR = 2.0         # last cone pair before an open track's finish gate [m]
CENTER_FLOOR = True        # recentre on the origin, as Mission Control would on load

Cone = Tuple[str, float, float]
XY = Tuple[float, float]


@dataclass
class Track:
    spec: TrackSpec
    cones: List[Cone]                 # CSV order: track cones by distance, then gates
    centreline: List[XY]
    left: List[XY]                    # blue boundary line
    right: List[XY]                   # yellow boundary line
    s: List[float]                    # centreline arc length per sample
    kappa: List[float]                # signed curvature per sample (+ = left turn)
    width: List[float]                # width per sample
    length: float                     # centreline length (a lap, if closed)
    offset: XY                        # translation applied to centre the track
    cone_s: List[float] = field(default_factory=list)   # centreline s of each track cone
    cone_side: List[str] = field(default_factory=list)  # left/right/gate_left/gate_right
    issues: List[Dict] = field(default_factory=list)    # filled by rules.check


# ---------------------------------------------------------------------------
# centreline
# ---------------------------------------------------------------------------

def _catmull_rom(points: Sequence[XY], closed: bool) -> List[Tuple[float, float, float]]:
    """Dense centripetal Catmull-Rom through the points: (x, y, u) with u the
    control parameter (segment i from points[i] to points[i+1] spans u in [i, i+1])."""
    n = len(points)
    pts = list(points)
    segs = n if closed else n - 1

    def P(i: int) -> XY:
        if closed:
            return pts[i % n]
        if i < 0:
            return (2 * pts[0][0] - pts[1][0], 2 * pts[0][1] - pts[1][1])
        if i >= n:
            return (2 * pts[-1][0] - pts[-2][0], 2 * pts[-1][1] - pts[-2][1])
        return pts[i]

    out: List[Tuple[float, float, float]] = []
    for i in range(segs):
        p0, p1, p2, p3 = P(i - 1), P(i), P(i + 1), P(i + 2)

        def knot(t: float, a: XY, b: XY) -> float:
            return t + max(math.hypot(b[0] - a[0], b[1] - a[1]), 1e-6) ** 0.5

        t0 = 0.0
        t1 = knot(t0, p0, p1)
        t2 = knot(t1, p1, p2)
        t3 = knot(t2, p2, p3)
        chord = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        m = max(8, int(math.ceil(chord / 0.05)))
        for j in range(m):
            f = j / m
            t = t1 + f * (t2 - t1)

            def lerp(a: XY, b: XY, ta: float, tb: float) -> XY:
                w = (t - ta) / (tb - ta)
                return (a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]))

            a1 = lerp(p0, p1, t0, t1)
            a2 = lerp(p1, p2, t1, t2)
            a3 = lerp(p2, p3, t2, t3)
            b1 = lerp(a1, a2, t0, t2)
            b2 = lerp(a2, a3, t1, t3)
            c = lerp(b1, b2, t1, t2)
            out.append((c[0], c[1], i + f))
    if not closed:
        out.append((pts[-1][0], pts[-1][1], float(n - 1)))
    return out


def _resample(dense: List[Tuple[float, float, float]], closed: bool, n_ctrl: int):
    """Uniform arc-length resampling. Returns (xs, ys, us, length)."""
    pts = list(dense)
    if closed:
        pts.append((pts[0][0], pts[0][1], float(n_ctrl)))
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    length = cum[-1]
    count = max(4, int(round(length / DS)))
    ds = length / count
    targets = [k * ds for k in range(count if closed else count + 1)]
    xs, ys, us = [], [], []
    j = 0
    for t in targets:
        while j < len(cum) - 2 and cum[j + 1] < t:
            j += 1
        seg = cum[j + 1] - cum[j]
        w = 0.0 if seg <= 0 else (t - cum[j]) / seg
        a, b = pts[j], pts[j + 1]
        xs.append(a[0] + w * (b[0] - a[0]))
        ys.append(a[1] + w * (b[1] - a[1]))
        us.append(a[2] + w * (b[2] - a[2]))
    return xs, ys, us, length


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _headings_curvature(xs, ys, closed, ds):
    n = len(xs)
    th = []
    for i in range(n):
        if closed:
            a, b = (i - 1) % n, (i + 1) % n
        else:
            a, b = max(0, i - 1), min(n - 1, i + 1)
        th.append(math.atan2(ys[b] - ys[a], xs[b] - xs[a]))
    k = []
    for i in range(n):
        if closed:
            a, b = (i - 1) % n, (i + 1) % n
            span = 2 * ds
        else:
            a, b = max(0, i - 1), min(n - 1, i + 1)
            span = (b - a) * ds
        k.append(_wrap(th[b] - th[a]) / span if span > 0 else 0.0)
    # light smoothing over about +-1 m: the spline is smooth, this only removes
    # the resampling ripple so corner spacing doesn't flicker cone to cone.
    w = 4
    ks = []
    for i in range(n):
        idx = [(i + d) % n if closed else min(n - 1, max(0, i + d)) for d in range(-w, w + 1)]
        ks.append(sum(k[j] for j in idx) / len(idx))
    return th, ks


# ---------------------------------------------------------------------------
# sections
# ---------------------------------------------------------------------------

def _in_section(u: float, sec: Section, n_ctrl: int, closed: bool) -> bool:
    if not closed:
        return sec.start <= u <= sec.end
    span = sec.end - sec.start if sec.end > sec.start else sec.end - sec.start + n_ctrl
    return ((u - sec.start) % n_ctrl) < span


def _sections_at(u: float, spec: TrackSpec) -> List[Section]:
    return [s for s in spec.sections
            if _in_section(u, s, len(spec.points), spec.closed)]


def _value_at(u: float, spec: TrackSpec, attr: str, default: float) -> float:
    v = default
    for s in _sections_at(u, spec):          # later sections win
        if getattr(s, attr) is not None:
            v = getattr(s, attr)
    return v


# ---------------------------------------------------------------------------
# cones
# ---------------------------------------------------------------------------

def _place_side(bx, by, bs_cum, spacing, i0, i1):
    """Indices-free cone placement on one boundary between centreline samples
    i0..i1 (inclusive, increasing): returns fractional sample positions."""
    # integral of 1/spacing along this boundary
    U = [0.0]
    for i in range(i0, i1):
        d = bs_cum[i + 1] - bs_cum[i]
        U.append(U[-1] + d / max(1e-6, 0.5 * (spacing[i] + spacing[i + 1])))
    total = U[-1]
    n = max(1, int(math.ceil(total - 1e-9)))
    out = []
    j = 0
    for k in range(n + 1):
        t = total * k / n
        while j < len(U) - 2 and U[j + 1] < t:
            j += 1
        seg = U[j + 1] - U[j]
        w = 0.0 if seg <= 0 else (t - U[j]) / seg
        out.append(i0 + j + w)
    return out


def build_track(spec: TrackSpec) -> Track:
    n_ctrl = len(spec.points)
    dense = _catmull_rom(spec.points, spec.closed)
    xs, ys, us, length = _resample(dense, spec.closed, n_ctrl)
    n = len(xs)
    ds = length / (n if spec.closed else n - 1)
    th, kap = _headings_curvature(xs, ys, spec.closed, ds)
    s = [i * ds for i in range(n)]
    width = [_value_at(u, spec, "width", spec.width) for u in us]
    straight = [_value_at(u, spec, "spacing", spec.spacing_straight) for u in us]

    # Boundaries: left (blue) = +w/2 along the left normal, right (yellow) = -w/2.
    lx, ly, rx, ry = [], [], [], []
    for i in range(n):
        nx, ny = -math.sin(th[i]), math.cos(th[i])
        h = width[i] / 2
        lx.append(xs[i] + nx * h); ly.append(ys[i] + ny * h)
        rx.append(xs[i] - nx * h); ry.append(ys[i] - ny * h)

    def spacing_for(side_sign: float) -> List[float]:
        out = []
        for i in range(n):
            d = side_sign * width[i] / 2      # offset along the left normal
            denom = 1.0 - d * kap[i]
            kb = abs(kap[i] / denom) if denom > 1e-6 else float("inf")
            r = 1.0 / kb if kb > 1e-9 else float("inf")
            out.append(min(straight[i], max(spec.spacing_corner_min, CORNER_K * r)))
        return out

    def cum(bx, by):
        c = [0.0]
        for i in range(1, n):
            c.append(c[-1] + math.hypot(bx[i] - bx[i - 1], by[i] - by[i - 1]))
        if spec.closed:
            c.append(c[-1] + math.hypot(bx[0] - bx[-1], by[0] - by[-1]))
        return c

    # Extend arrays by one wrap sample for closed loops so indices i0..i1 work.
    def ext(a):
        return a + [a[0]] if spec.closed else a

    i_first = int(round(AHEAD_FIRST / ds))
    i_second = int(round(AHEAD_SECOND / ds))
    if spec.closed:
        i_end = int(round((length - GAP_BEHIND) / ds))
    else:
        i_end = int(round((length - FINISH_CLEAR) / ds))
    i_end = max(i_end, i_second + 1)

    def interp(a, f):
        i = int(math.floor(f))
        w = f - i
        aa = ext(a)
        if i >= len(aa) - 1:
            return aa[-1]
        return aa[i] + w * (aa[i + 1] - aa[i])

    track_cones: List[Tuple[str, float, float, float, str, float]] = []  # type,x,y,s,side,u
    for side, bx, by, sign, colour in (("left", lx, ly, +1.0, "blue"),
                                       ("right", rx, ry, -1.0, "yellow")):
        sp = spacing_for(sign)
        bcum = cum(bx, by)
        fr = [float(i_first)] + _place_side(ext(bx), ext(by), bcum, ext(sp), i_second, i_end)
        for f in fr:
            track_cones.append((colour, interp(bx, f), interp(by, f), f * ds, side, interp(us, f)))

    # Sections: drop, jitter, recolor. One fixed set of draws per cone, so
    # editing one section never reshuffles the cones of another.
    rng = random.Random(spec.seed)
    protected_to = AHEAD_SECOND + 1e-6
    finish_from = length - FINISH_CLEAR - 1e-6
    out_cones = []
    for colour, x, y, sc, side, u in sorted(track_cones, key=lambda c: (c[3], c[4])):
        r_drop, jx, jy = rng.random(), rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0)
        if sc <= protected_to or (not spec.closed and sc >= finish_from):
            out_cones.append((colour, x, y, sc, side))
            continue
        keep = True
        for sec in _sections_at(u, spec):
            if sec.side not in ("both", side):
                continue
            if sec.drop and r_drop < sec.drop:
                keep = False
            if sec.jitter:
                x += jx * sec.jitter
                y += jy * sec.jitter
            if sec.recolor == "swap":
                colour = "yellow" if colour == "blue" else "blue" if colour == "yellow" else colour
            elif sec.recolor:
                colour = sec.recolor
        if keep:
            out_cones.append((colour, x, y, sc, side))

    # Gates: start around s=0; an open track also gets a finish gate at its end.
    def gate(i: int):
        """Four big orange cones around sample i: (cone, s, side)."""
        half = max(GATE_HALF_MIN, width[i] / 2 + GATE_MARGIN)
        tx, ty = math.cos(th[i]), math.sin(th[i])
        nx, ny = -ty, tx
        g = []
        for a in (-GATE_DEPTH / 2, GATE_DEPTH / 2):
            for b, side in ((half, "gate_left"), (-half, "gate_right")):
                g.append((("big_orange", xs[i] + tx * a + nx * b, ys[i] + ty * a + ny * b),
                          (s[i] + a) % length if spec.closed else s[i] + a, side))
        return g

    gates = gate(0)
    if not spec.closed:
        gates += gate(n - 1)

    # Centre the boundary bbox on the origin, the way Mission Control's
    # ensure_centered would on load, so the file isn't rewritten there.
    ox = oy = 0.0
    by_cones = [(x, y) for c, x, y, _, _ in out_cones if c in ("blue", "yellow")]
    if CENTER_FLOOR and by_cones:
        ox = -(min(p[0] for p in by_cones) + max(p[0] for p in by_cones)) / 2
        oy = -(min(p[1] for p in by_cones) + max(p[1] for p in by_cones)) / 2

    def sh(p: XY) -> XY:
        return (p[0] + ox, p[1] + oy)

    cones: List[Cone] = [(c, x + ox, y + oy) for c, x, y, _, _ in out_cones]
    cones += [(c, x + ox, y + oy) for (c, x, y), _, _ in gates]
    return Track(
        spec=spec, cones=cones,
        centreline=[sh(p) for p in zip(xs, ys)],
        left=[sh(p) for p in zip(lx, ly)], right=[sh(p) for p in zip(rx, ry)],
        s=s, kappa=kap, width=width, length=length, offset=(ox, oy),
        cone_s=[c[3] for c in out_cones] + [g[1] for g in gates],
        cone_side=[c[4] for c in out_cones] + [g[2] for g in gates],
    )


def to_csv(track: Track) -> str:
    """The track CSV, in the format the sim and every tool read: type,x,y plus
    the four trailing columns the generated tracks carry (ignored on load)."""
    return "".join(f"{t},{x:.4f},{y:.4f},0,0.01,0.01,0\n" for t, x, y in track.cones)
