"""The track spec: what a person draws, as data.

A spec is the editable source of a track. The CSV the sim loads is built from
it (`build_track`) and never edited by hand; the spec is saved next to the CSV
as `<track>.track.json` so a drawn track can be reopened and changed.

Format v1:

    {
      "format": "ifssim-trackspec/1",
      "closed": true,
      "points": [[0, 0], [40, 0], [55, 20], ...],
      "width": 3.5,
      "spacing": {"straight": 5.0, "corner_min": 1.5},
      "seed": 1,
      "sections": [
        {"from": 3, "to": 5, "width": 3.0},
        {"from": 6, "to": 7, "drop": 0.5, "side": "left"},
        {"from": 8, "to": 9, "recolor": "swap"},
        {"from": 9, "to": 10, "jitter": 0.3}
      ]
    }

- `points` are the drawn control points, in metres, in the track CSV's frame
  (right-handed, +y to the left of +x; see docs/environment_sidecar.md). The
  car drives from the first point toward the second. A closed track is a loop
  through every point and back to the first; the start gate sits on the first.
- `width` is the distance between the blue and yellow cone lines.
- `spacing.straight` is the cone-to-cone distance on a straight;
  `spacing.corner_min` is the tightest it gets in the tightest corner.
- `sections` change one stretch of track, from control point `from` to
  control point `to` (indices into `points`; on a closed track `to` may wrap
  past the last point). Each one sets any of:
    - `width`, `spacing`: override the track's own;
    - `drop`: the fraction of cones removed, 0 to 1 (missing cones);
    - `side`: which cone line `drop`, `recolor` and `jitter` touch:
      "left" (blue), "right" (yellow) or "both" (the default);
    - `recolor`: "swap" (blue <-> yellow), or every cone becomes "blue",
      "yellow" or "small_orange" (wrong-colour cones);
    - `jitter`: standard deviation, in metres, of a random displacement
      (misplaced cones).
  Big orange is not offered: an extra big-orange cluster changes where the
  sim spawns the car (FSDSConeSpawner::ComputeStartGatePose).
- `seed` makes `drop` and `jitter` reproducible: the same spec always builds
  the same cones.

Strict on purpose, like the environment sidecar: unknown keys and wrong types
are errors, so a misspelt `widht` doesn't silently fall back to the default.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

FORMAT = "ifssim-trackspec/1"

DEFAULT_WIDTH_M = 3.5
DEFAULT_SPACING_STRAIGHT_M = 5.0
DEFAULT_SPACING_CORNER_MIN_M = 1.5
MAX_COORD_M = 500.0          # same sanity bound as tools/validate_tracks.py
MAX_POINTS = 500

SIDES = ("left", "right", "both")
RECOLORS = ("swap", "blue", "yellow", "small_orange")


class SpecError(ValueError):
    """The spec is malformed: it cannot be built at all."""


@dataclass
class Section:
    start: int                       # control-point index `from`
    end: int                         # control-point index `to`
    width: Optional[float] = None
    spacing: Optional[float] = None
    drop: float = 0.0
    side: str = "both"
    recolor: Optional[str] = None
    jitter: float = 0.0


@dataclass
class TrackSpec:
    points: List[Tuple[float, float]]
    closed: bool = True
    width: float = DEFAULT_WIDTH_M
    spacing_straight: float = DEFAULT_SPACING_STRAIGHT_M
    spacing_corner_min: float = DEFAULT_SPACING_CORNER_MIN_M
    seed: int = 1
    sections: List[Section] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            "format": FORMAT,
            "closed": self.closed,
            "points": [[x, y] for x, y in self.points],
            "width": self.width,
            "spacing": {"straight": self.spacing_straight,
                        "corner_min": self.spacing_corner_min},
            "seed": self.seed,
        }
        if self.sections:
            d["sections"] = []
            for s in self.sections:
                sd: Dict[str, Any] = {"from": s.start, "to": s.end}
                if s.width is not None:
                    sd["width"] = s.width
                if s.spacing is not None:
                    sd["spacing"] = s.spacing
                if s.drop:
                    sd["drop"] = s.drop
                if s.side != "both":
                    sd["side"] = s.side
                if s.recolor is not None:
                    sd["recolor"] = s.recolor
                if s.jitter:
                    sd["jitter"] = s.jitter
                d["sections"].append(sd)
        return d


def _num(v: Any, where: str) -> float:
    # bool is an int subclass; `true` is not a number here.
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise SpecError(f"{where} must be a number, not {v!r}")
    return float(v)


def _int(v: Any, where: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise SpecError(f"{where} must be a whole number, not {v!r}")
    return v


def _keys(d: Any, where: str, allowed: Tuple[str, ...]) -> Dict[str, Any]:
    if not isinstance(d, dict):
        raise SpecError(f"{where} must be an object")
    extra = sorted(set(d) - set(allowed))
    if extra:
        raise SpecError(f"{where}: unknown key(s) {extra} (allowed: {list(allowed)})")
    return d


def _positive(v: float, where: str, lo: float, hi: float) -> float:
    if not (lo <= v <= hi):
        raise SpecError(f"{where} must be in [{lo}, {hi}], not {v}")
    return v


def parse_spec(d: Any) -> TrackSpec:
    """Parse and check a spec dict (e.g. from JSON). Raises SpecError."""
    d = _keys(d, "spec", ("format", "closed", "points", "width", "spacing",
                          "seed", "sections", "name"))
    if d.get("format") != FORMAT:
        raise SpecError(f'spec.format must be "{FORMAT}", not {d.get("format")!r}')

    closed = d.get("closed", True)
    if not isinstance(closed, bool):
        raise SpecError("spec.closed must be true or false")

    pts_raw = d.get("points")
    if not isinstance(pts_raw, list):
        raise SpecError("spec.points must be a list of [x, y] pairs")
    if len(pts_raw) > MAX_POINTS:
        raise SpecError(f"spec.points: {len(pts_raw)} points, at most {MAX_POINTS}")
    points: List[Tuple[float, float]] = []
    for i, p in enumerate(pts_raw):
        if not (isinstance(p, list) and len(p) == 2):
            raise SpecError(f"spec.points[{i}] must be [x, y]")
        x, y = _num(p[0], f"spec.points[{i}][0]"), _num(p[1], f"spec.points[{i}][1]")
        if abs(x) > MAX_COORD_M or abs(y) > MAX_COORD_M:
            raise SpecError(f"spec.points[{i}] = ({x}, {y}) is beyond ±{MAX_COORD_M} m "
                            f"(metres, not centimetres?)")
        points.append((x, y))
    need = 3 if closed else 2
    if len(points) < need:
        raise SpecError(f"a {'closed' if closed else 'open'} track needs at least "
                        f"{need} points, got {len(points)}")
    for i in range(len(points) - (0 if closed else 1)):
        a, b = points[i], points[(i + 1) % len(points)]
        if abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6:
            raise SpecError(f"spec.points[{i}] and the next point are the same point")

    width = _positive(_num(d.get("width", DEFAULT_WIDTH_M), "spec.width"),
                      "spec.width", 1.0, 20.0)

    sp = _keys(d.get("spacing", {}), "spec.spacing", ("straight", "corner_min"))
    s_str = _positive(_num(sp.get("straight", DEFAULT_SPACING_STRAIGHT_M),
                           "spec.spacing.straight"), "spec.spacing.straight", 0.5, 20.0)
    s_min = _positive(_num(sp.get("corner_min", DEFAULT_SPACING_CORNER_MIN_M),
                           "spec.spacing.corner_min"), "spec.spacing.corner_min", 0.5, 20.0)
    if s_min > s_str:
        raise SpecError("spec.spacing.corner_min must not exceed spec.spacing.straight")

    seed = _int(d.get("seed", 1), "spec.seed")
    if seed < 0:
        raise SpecError("spec.seed must be non-negative")

    n = len(points)
    sections: List[Section] = []
    secs_raw = d.get("sections", [])
    if not isinstance(secs_raw, list):
        raise SpecError("spec.sections must be a list")
    for k, sr in enumerate(secs_raw):
        w = f"spec.sections[{k}]"
        sr = _keys(sr, w, ("from", "to", "width", "spacing", "drop", "side",
                           "recolor", "jitter", "note"))
        a, b = _int(sr.get("from"), f"{w}.from"), _int(sr.get("to"), f"{w}.to")
        last = n - 1 if not closed else n
        if not (0 <= a < n) or not (0 <= b <= last) or (not closed and b <= a) \
                or (closed and a == b):
            raise SpecError(f"{w}: from={a} to={b} is not a stretch of this "
                            f"{n}-point {'loop' if closed else 'track'}")
        sec = Section(start=a, end=b)
        if "width" in sr:
            sec.width = _positive(_num(sr["width"], f"{w}.width"), f"{w}.width", 1.0, 20.0)
        if "spacing" in sr:
            sec.spacing = _positive(_num(sr["spacing"], f"{w}.spacing"), f"{w}.spacing", 0.5, 20.0)
        if "drop" in sr:
            sec.drop = _positive(_num(sr["drop"], f"{w}.drop"), f"{w}.drop", 0.0, 1.0)
        if "side" in sr:
            if sr["side"] not in SIDES:
                raise SpecError(f"{w}.side must be one of {list(SIDES)}")
            sec.side = sr["side"]
        if "recolor" in sr:
            if sr["recolor"] not in RECOLORS:
                raise SpecError(f"{w}.recolor must be one of {list(RECOLORS)} "
                                f"(big_orange would move the car's start pose)")
            sec.recolor = sr["recolor"]
        if "jitter" in sr:
            sec.jitter = _positive(_num(sr["jitter"], f"{w}.jitter"), f"{w}.jitter", 0.0, 2.0)
        sections.append(sec)

    return TrackSpec(points=points, closed=closed, width=width,
                     spacing_straight=s_str, spacing_corner_min=s_min,
                     seed=seed, sections=sections)
