"""Seed + profile + track CSV -> the track's environment sidecar (format v1).

Deterministic: the same seed, profile and CSV give the same bytes on every
platform (docs/environment_sidecar.md, roadmap rule 3). Every prop is placed by
rejection sampling against the track's geometry:

- its footprint stays out of the track area (the cone corridor), within its
  class's distance band from it;
- it keeps 0.5 m from every cone and 0.3 m from every other prop;
- it stays on the sim's floor.

A prop that finds no valid spot in MAX_TRIES draws is dropped, so a cramped
track gets fewer props, never a prop in the wrong place.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.geometry.base import BaseGeometry

from . import track as trackmod
from .catalogue import CLASSES, PROFILES, PropClass
from .seed import Stream

FORMAT = "ifssim-env/1"
MAX_TRIES = 60
CONE_CLEARANCE_M = 0.5
PROP_CLEARANCE_M = 0.3

# The sim's floor (customMap) is about 200 m x 250 m around the origin, and the
# backend centres a track's boundary on the origin when it loads it
# (mission_control/backend/track_centering.py), shifting the sidecar with it.
# So props are checked against the floor in centred coordinates; 95 m keeps a
# margin on both axes, whichever way the floor's long side runs.
FLOOR_HALF_M = 95.0
# Timing-line clusters: big orange cones within this distance belong together.
LINE_CLUSTER_M = 6.0


def footprint(cls: PropClass, x: float, y: float, yaw_deg: float) -> BaseGeometry:
    if cls.length_m:
        rect = box(-cls.length_m / 2, -cls.width_m / 2, cls.length_m / 2, cls.width_m / 2)
        return affinity.translate(affinity.rotate(rect, yaw_deg, origin=(0, 0)), x, y)
    return Point(x, y).buffer(cls.radius_m, 16)


def _boundary_centre(tr: trackmod.Track) -> Tuple[float, float]:
    """Where the backend's centring moves the origin to (track_centering.bbox_center)."""
    pts = [(x, y) for t, x, y in tr.cones if t in ("blue", "yellow")] or \
          [(x, y) for _, x, y in tr.cones]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0


class _Placer:
    def __init__(self, tr: trackmod.Track, allowed: Optional[BaseGeometry] = None):
        self.tr = tr
        cx, cy = _boundary_centre(tr)
        self.floor = box(cx - FLOOR_HALF_M, cy - FLOOR_HALF_M, cx + FLOOR_HALF_M, cy + FLOOR_HALF_M)
        # Where props may stand at all, e.g. a venue's lot; None: anywhere on the floor.
        self.allowed = allowed
        self.placed: List[BaseGeometry] = []

    def ok(self, cls: PropClass, fp: BaseGeometry) -> bool:
        tr = self.tr
        if not self.floor.contains(fp) or fp.intersects(tr.area):
            return False
        if self.allowed is not None and not self.allowed.contains(fp):
            return False
        gap = tr.area.distance(fp)
        if not (cls.band_m[0] <= gap <= cls.band_m[1]):
            return False
        if tr._cone_points.distance(fp) < CONE_CLEARANCE_M:
            return False
        return all(fp.distance(o) >= PROP_CLEARANCE_M for o in self.placed)

    def try_place(self, cls: PropClass, x: float, y: float, yaw: float) -> Optional[Dict]:
        fp = footprint(cls, x, y, yaw)
        if not self.ok(cls, fp):
            return None
        self.placed.append(fp)
        return {"class": cls.name, "x": round(x, 3), "y": round(y, 3), "yaw_deg": round(yaw % 360.0, 1)}


def _outward(tr: trackmod.Track, x: float, y: float, tangent_deg: float) -> float:
    """Normal direction (deg) from a boundary point that leads out of the track area."""
    for sign in (90.0, -90.0):
        a = math.radians(tangent_deg + sign)
        if tr.distance_to_area(x + 0.5 * math.cos(a), y + 0.5 * math.sin(a)) > 0.0:
            return tangent_deg + sign
    return tangent_deg + 90.0


def _place_along_boundary(pl: _Placer, cls: PropClass, rng: Stream) -> Optional[Dict]:
    tr = pl.tr
    edge = tr.boundary
    for _ in range(MAX_TRIES):
        s = rng.uniform(0.0, edge.length)
        anchor = edge.interpolate(s)
        tangent = tr.tangent_deg(anchor.x, anchor.y)
        out = math.radians(_outward(tr, anchor.x, anchor.y, tangent))
        reach = cls.width_m / 2 if cls.length_m else cls.radius_m
        d = rng.uniform(*cls.band_m) + reach
        x, y = anchor.x + d * math.cos(out), anchor.y + d * math.sin(out)
        yaw = tangent if cls.along_boundary else rng.uniform(0.0, 360.0)
        prop = pl.try_place(cls, x, y, yaw)
        if prop:
            return prop
    return None


def _timing_lines(tr: trackmod.Track) -> List[List[Tuple[float, float]]]:
    lines: List[List[Tuple[float, float]]] = []
    for p in tr.oranges:
        for line in lines:
            if any(math.dist(p, q) <= LINE_CLUSTER_M for q in line):
                line.append(p)
                break
        else:
            lines.append([p])
    return lines[:2]


def _place_tripods(pl: _Placer, per_line: int, rng: Stream) -> List[Dict]:
    """Light barriers stand level with a timing line, one on each side of the track."""
    cls = CLASSES["tripod"]
    tr = pl.tr
    out: List[Dict] = []
    for line in _timing_lines(tr):
        cx = sum(p[0] for p in line) / len(line)
        cy = sum(p[1] for p in line) / len(line)
        tangent = tr.heading_deg(cx, cy)
        for side in (90.0, -90.0)[:per_line]:
            a = math.radians(tangent + side)
            # Walk out along the line until clear of the track area.
            t = 0.0
            while t < 30.0 and tr.distance_to_area(cx + t * math.cos(a), cy + t * math.sin(a)) == 0.0:
                t += 0.1
            for _ in range(MAX_TRIES):
                d = t + rng.uniform(*cls.band_m) + cls.radius_m
                along = rng.uniform(-0.5, 0.5)  # a tripod is set up by hand
                x = cx + d * math.cos(a) - along * math.sin(a)
                y = cy + d * math.sin(a) + along * math.cos(a)
                prop = pl.try_place(cls, x, y, rng.uniform(0.0, 120.0))
                if prop:
                    out.append(prop)
                    break
    return out


def generate(track_csv: str | Path, seed: int, profile: str) -> Dict:
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; one of {', '.join(PROFILES)}")
    if not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a non-negative integer")
    tr = trackmod.load(track_csv)
    props = place(_Placer(tr), PROFILES[profile], seed)
    return {"format": FORMAT, "seed": seed, "profile": profile, "props": props}


def place(pl: _Placer, prof, seed: int) -> List[Dict]:
    """A profile's props around pl's track, in the order generate writes them."""
    tr = pl.tr
    props: List[Dict] = []
    if prof.tripods_per_line:
        props += _place_tripods(pl, prof.tripods_per_line, Stream(seed, "Env.Props.tripod"))
    length_100m = tr.boundary_length() / 100.0
    # Walls first: they need the most room.
    for name in sorted(prof.per_100m, key=lambda n: (not CLASSES[n].along_boundary, n)):
        cls, rng = CLASSES[name], Stream(seed, f"Env.Props.{name}")
        for _ in range(int(round(prof.per_100m[name] * length_100m))):
            prop = _place_along_boundary(pl, cls, rng)
            if prop:
                props.append(prop)
    return props


def dumps(env: Dict) -> str:
    """The sidecar's bytes: stable key order and rounding, so a seed is a file hash."""
    return json.dumps(env, indent=2) + "\n"


def sha256(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def sidecar_path(track_csv: str | Path) -> Path:
    p = Path(track_csv)
    return p.with_name(p.stem + ".env.json")
