"""A track CSV as geometry: its cones, the area the car drives in, its boundary.

Props are placed against this. The track area is what nothing may overlap (the
cone corridor); its boundary is where props are anchored.

- **Closed track** (autocross, trackdrive): the blue and yellow cones each form
  a loop, in file order. The track area is the outer loop's polygon minus the
  inner one's.
- **Open layout** (acceleration, skidpad), or a closed track whose loops don't
  make clean polygons: the convex hull of every cone. That is conservative: no
  prop lands inside a skidpad circle or a self-crossing loop.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

from shapely.geometry import LineString, MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry

Xy = Tuple[float, float]

# A colour's cone sequence is a loop if its ends are this close (m). FS cone
# spacing is at most 5 m, so a closing gap is well under this.
LOOP_CLOSE_M = 8.0
MIN_LOOP_CONES = 6


@dataclass
class Track:
    path: Path
    cones: List[Tuple[str, float, float]]
    area: BaseGeometry                 # where the car drives: props stay out
    closed: bool
    _cone_points: BaseGeometry = field(init=False, repr=False)

    def __post_init__(self):
        self._cone_points = MultiPoint([(x, y) for _, x, y in self.cones])

    @property
    def boundary(self) -> BaseGeometry:
        return self.area.boundary

    @property
    def oranges(self) -> List[Xy]:
        return [(x, y) for t, x, y in self.cones if t == "big_orange"]

    def distance_to_area(self, x: float, y: float) -> float:
        return self.area.distance(Point(x, y))

    def distance_to_cones(self, x: float, y: float) -> float:
        return self._cone_points.distance(Point(x, y))

    def boundary_length(self) -> float:
        return self.boundary.length

    def heading_deg(self, x: float, y: float, radius_m: float = 12.0) -> float:
        """Direction the track runs near (x, y): the main axis of the nearby
        cones of one boundary colour. Unlike tangent_deg, this follows the cone
        lines even where the track area is a hull."""
        best: List[Xy] = []
        for colour in ("blue", "yellow"):
            pts = [(cx, cy) for t, cx, cy in self.cones
                   if t == colour and math.dist((cx, cy), (x, y)) <= radius_m]
            if len(pts) > len(best):
                best = pts
        if len(best) < 2:
            return self.tangent_deg(x, y)
        mx = sum(p[0] for p in best) / len(best)
        my = sum(p[1] for p in best) / len(best)
        sxx = sum((p[0] - mx) ** 2 for p in best)
        syy = sum((p[1] - my) ** 2 for p in best)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in best)
        return math.degrees(0.5 * math.atan2(2 * sxy, sxx - syy))

    def tangent_deg(self, x: float, y: float) -> float:
        """Direction of the boundary nearest (x, y), counter-clockwise from +x."""
        line = _nearest_line(self.boundary, Point(x, y))
        s = line.project(Point(x, y))
        a = line.interpolate(max(0.0, s - 0.5))
        b = line.interpolate(min(line.length, s + 0.5))
        return math.degrees(math.atan2(b.y - a.y, b.x - a.x))


def _nearest_line(geom: BaseGeometry, p: Point) -> LineString:
    parts = list(getattr(geom, "geoms", [geom]))
    return min(parts, key=lambda g: g.distance(p))


def read_cones(path: Path) -> List[Tuple[str, float, float]]:
    cones = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) < 3:
                continue
            kind = row[0].strip().lower()
            try:
                x, y = float(row[1]), float(row[2])
            except ValueError:
                continue  # a header or comment line
            cones.append((kind, x, y))
    if not cones:
        raise ValueError(f"{path}: no cones")
    return cones


def _loop(points: List[Xy]) -> Polygon | None:
    if len(points) < MIN_LOOP_CONES:
        return None
    if math.dist(points[0], points[-1]) > LOOP_CLOSE_M:
        return None
    poly = Polygon(points)
    return poly if poly.is_valid and poly.area > 0 else None


def load(path: str | Path) -> Track:
    path = Path(path)
    cones = read_cones(path)
    blue = [(x, y) for t, x, y in cones if t == "blue"]
    yellow = [(x, y) for t, x, y in cones if t == "yellow"]
    a, b = _loop(blue), _loop(yellow)
    if a is not None and b is not None:
        outer, inner = (a, b) if a.area >= b.area else (b, a)
        if outer.contains(inner):
            return Track(path, cones, outer.difference(inner), closed=True)
    hull = MultiPoint([(x, y) for _, x, y in cones]).convex_hull
    if hull.geom_type != "Polygon":
        raise ValueError(f"{path}: cones don't span an area")
    return Track(path, cones, hull, closed=False)
