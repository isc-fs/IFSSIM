"""Stress tracks: parametric shapes the track creator can't be drawn by hand.

A drawn track is good for "a track like this one"; it is bad for "a circle of
exactly 9 m" or "a corner whose radius tightens from 20 m to 6 m". These are
the shapes that isolate one thing the car has to get right, so they are built
here from geometry and handed to the same corridor builder as a drawn track:

    straight + arc + spiral  --turtle-->  control points  -->  TrackSpec

Every stress track is a normal spec, so it gets the same checks, the same
start gate and the same CSV + `<name>.track.json` as a drawn one, and it can
be reopened and tweaked in the Draw Track editor.

    cd tools
    python3 -m envgen stress --list
    python3 -m envgen stress -o ../Content/tracks            # the whole library
    python3 -m envgen stress circle_r9 -o ../Content/tracks  # one of them

Shapes are dense in control points (a point every <= 15 deg and <= 5 m of
arc, and every <= 5 m of straight) so the Catmull-Rom centreline follows the
geometry closely: a circle comes out a circle, not a rounded polygon.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from .spec import FORMAT, TrackSpec, parse_spec

XY = Tuple[float, float]

ARC_STEP_RAD = math.radians(15.0)
STRAIGHT_STEP_M = 5.0
WIDTH_M = 3.5


class Turtle:
    """Walks a centreline and drops control points along it.

    Starts at the origin heading +x, where the start gate goes. Positive
    angles turn left (counter-clockwise), as the CSV frame has +y on the left.
    """

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.h = 0.0
        self.points: List[XY] = [(0.0, 0.0)]

    def _drop(self) -> None:
        self.points.append((self.x, self.y))

    def mark(self) -> int:
        """Index of the current control point, for a section's from/to."""
        return len(self.points) - 1

    def straight(self, length: float) -> "Turtle":
        n = max(1, math.ceil(length / STRAIGHT_STEP_M))
        for _ in range(n):
            self.x += math.cos(self.h) * length / n
            self.y += math.sin(self.h) * length / n
            self._drop()
        return self

    def arc(self, radius: float, angle_deg: float) -> "Turtle":
        return self.spiral(radius, radius, angle_deg)

    def spiral(self, r_start: float, r_end: float, angle_deg: float) -> "Turtle":
        """Turn by angle_deg with the radius changing linearly with the angle.

        Integrated in fine steps, and a control point dropped every
        ARC_STEP_RAD, so a spiral is a spiral and not a chain of arcs."""
        total = math.radians(angle_deg)
        sub = 40                                  # integration steps per point
        mean_r = (abs(r_start) + abs(r_end)) / 2
        n = max(1, math.ceil(abs(total) / ARC_STEP_RAD),
                math.ceil(abs(total) * mean_r / STRAIGHT_STEP_M))
        steps = n * sub
        dth = total / steps
        for k in range(steps):
            f = (k + 0.5) / steps
            r = r_start + (r_end - r_start) * f
            ds = r * abs(dth)
            hm = self.h + dth / 2                  # midpoint heading
            self.x += math.cos(hm) * ds
            self.y += math.sin(hm) * ds
            self.h += dth
            if (k + 1) % sub == 0:
                self._drop()
        return self

    def closed_points(self, tol: float = 0.05) -> List[XY]:
        """The loop's points, without the last one if it lands on the start."""
        last = self.points[-1]
        if math.hypot(last[0], last[1]) > tol:
            raise ValueError(f"loop does not close: ends at ({last[0]:.3f}, {last[1]:.3f})")
        return self.points[:-1]


def _round(points: List[XY]) -> List[List[float]]:
    return [[round(x, 3), round(y, 3)] for x, y in points]


def _spec(name: str, points: List[XY], closed: bool, **kw) -> TrackSpec:
    d = {"format": FORMAT, "name": name, "closed": closed,
         "points": _round(points), "width": WIDTH_M}
    d.update(kw)
    return parse_spec(d)


# ---------------------------------------------------------------------------
# shapes
# ---------------------------------------------------------------------------

def circle(radius: float, clockwise: bool = False, **kw) -> TrackSpec:
    """Constant radius, lap after lap: steady-state cornering, the planner's
    and the controller's view of a single curvature."""
    t = Turtle().arc(radius, -360.0 if clockwise else 360.0)
    name = f"circle_r{radius:g}{'_cw' if clockwise else ''}"
    return _spec(name, t.closed_points(), True, **kw)


def brake_hairpin(straight: float = 75.0, radius: float = 5.0, **kw) -> TrackSpec:
    """A paperclip: the longest legal straight into the tightest legal
    hairpin, twice a lap. Braking point, top speed, hairpin entry."""
    t = (Turtle().straight(straight / 2).arc(radius, 180).straight(straight)
         .arc(radius, 180).straight(straight / 2))
    return _spec(f"brake_hairpin_{straight:g}m", t.closed_points(), True, **kw)


def snail(r_wide: float = 20.0, r_tight: float = 6.0, opening: bool = False,
          lead: float = 20.0, **kw) -> TrackSpec:
    """A loop whose main corner is a spiral: 180 deg with the radius going
    from r_wide down to r_tight (or, driven the other way, opening up).
    Tests a controller that must keep adding steer through one corner, and
    a planner whose curvature estimate lags."""
    t = Turtle().straight(lead).spiral(r_wide, r_tight, 180.0)
    # Heading is now -x, some distance d above the start line. Drive back to
    # x = 0 and close with a constant 180 deg turn of radius d/2.
    t.straight(t.x)
    t.arc(t.y / 2, 180.0)
    pts = t.closed_points()
    if opening:
        # Same loop, driven the other way: the gate stays on points[0].
        pts = [pts[0]] + pts[:0:-1]
    name = f"snail_{'opening' if opening else 'tightening'}_r{r_wide:g}_{r_tight:g}"
    return _spec(name, pts, True, **kw)


def slalom(amplitude: float, wavelength: float, periods: int = 4,
           lead: float = 20.0, **kw) -> TrackSpec:
    """An open track weaving side to side: fast direction changes, the
    curvature sign flipping every half period."""
    pts: List[XY] = [(0.0, 0.0)]
    x = 0.0
    n = max(1, math.ceil(lead / STRAIGHT_STEP_M))
    for _ in range(n):
        x += lead / n
        pts.append((x, 0.0))
    x0 = x
    per = 12                                       # points per period
    for k in range(1, periods * per + 1):
        u = k / per
        pts.append((x0 + u * wavelength, amplitude * math.sin(2 * math.pi * u)))
    x = pts[-1][0]
    for _ in range(n):
        x += lead / n
        pts.append((x, 0.0))
    return _spec(f"slalom_a{amplitude:g}_l{wavelength:g}", pts, False, **kw)


def hairpins(count: int = 5, radius: float = 4.5, leg: float = 20.0,
             lead: float = 20.0, **kw) -> TrackSpec:
    """An open track of back-to-back hairpins, alternating left and right,
    with short legs between them: low-speed, full-lock, and the other leg's
    cones a few metres away (association across the corridor)."""
    if count < 1 or count % 2 == 0:
        raise ValueError("hairpins: count must be odd, so the track exits the way it came in")
    t = Turtle().straight(lead).arc(radius, 90)
    for i in range(count):
        t.straight(leg).arc(radius, -180 if i % 2 == 0 else 180)
    t.straight(leg).arc(radius, 90).straight(lead)
    return _spec(f"hairpins_{count}x_r{radius:g}", t.points, False, **kw)


# ---------------------------------------------------------------------------
# the library
# ---------------------------------------------------------------------------

@dataclass
class Stress:
    name: str
    tests: str
    build: Callable[[], TrackSpec]


def _paperclip_with_gaps() -> TrackSpec:
    """brake_hairpin with half the blue cones missing on the braking straight."""
    t = Turtle().straight(37.5).arc(5.0, 180)
    a = t.mark()
    t.straight(75.0)
    b = t.mark()
    t.arc(5.0, 180).straight(37.5)
    return _spec("brake_hairpin_gaps", t.closed_points(), True, seed=3, sections=[
        {"from": a, "to": b, "drop": 0.5, "side": "left",
         "note": "half the blue cones missing on the braking straight"}])


def _circle_jitter() -> TrackSpec:
    """circle_r15 with every cone up to ~0.3 m off its line."""
    s = circle(15.0)
    d = s.to_dict()
    d["name"] = "circle_r15_jitter"
    d["seed"] = 5
    d["sections"] = [{"from": 1, "to": len(d["points"]) - 1, "jitter": 0.3,
                      "note": "every cone misplaced by ~0.3 m"}]
    return parse_spec(d)


LIBRARY: List[Stress] = [
    Stress("circle_r5", "near full steering lock, constant radius",
           lambda: circle(5.0)),
    Stress("circle_r9", "skidpad radius, constant, counter-clockwise",
           lambda: circle(9.0)),
    Stress("circle_r9_cw", "skidpad radius, constant, clockwise",
           lambda: circle(9.0, clockwise=True)),
    Stress("circle_r15", "medium constant radius",
           lambda: circle(15.0)),
    Stress("circle_r25", "fast constant radius",
           lambda: circle(25.0)),
    Stress("snail_tightening_r20_6", "a corner that keeps tightening, 20 m to 6 m",
           lambda: snail(20.0, 6.0)),
    Stress("snail_opening_r20_6", "a corner that keeps opening, 6 m to 20 m",
           lambda: snail(20.0, 6.0, opening=True)),
    Stress("slalom_a2.5_l30", "wide slalom: smooth, fast direction changes",
           lambda: slalom(2.5, 30.0)),
    Stress("slalom_a1.5_l20", "tight slalom: quick, sharp direction changes (~6.8 m turns)",
           lambda: slalom(1.5, 20.0)),
    Stress("hairpins_5x_r4.5", "five back-to-back hairpins, legs 20 m apart in time, 9 m in space",
           lambda: hairpins(5, 4.5)),
    Stress("brake_hairpin_75m", "75 m straight into a hairpin, both ends",
           lambda: brake_hairpin(75.0, 5.0)),
    Stress("brake_hairpin_gaps", "the brake hairpin with half the blue cones missing on one straight",
           _paperclip_with_gaps),
    Stress("circle_r15_jitter", "the 15 m circle with every cone ~0.3 m off its line",
           _circle_jitter),
]

FILE_PREFIX = "stress_"


def by_name(name: str) -> Optional[Stress]:
    for s in LIBRARY:
        if s.name == name or FILE_PREFIX + s.name == name:
            return s
    return None


def build_all() -> Dict[str, TrackSpec]:
    return {s.name: s.build() for s in LIBRARY}
