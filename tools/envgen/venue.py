"""Venues: a fixed place, such as a parking lot, that several tracks share.

A venue file (`Content/tracks/venues/<venue>.venue.json`, format
"ifssim-venue/1") describes the place once, in its own frame (metres, the track
CSV's axes): the `lot`, the open asphalt where tracks are drawn, and the props
around it. The tracks drawn there are named `<venue>__<track>.csv`. Each one
gets an environment sidecar (`<track>.env.json`, docs/environment_sidecar.md)
holding the venue's props, so the plugin needs nothing new: it builds the
sidecar as it builds any other.

A track's CSV is recentred on its own cones (corridor.CENTER_FLOOR, and Mission
Control on load), so the venue moves with it: the sidecar's props are the
venue's, shifted by the same offset. The offset comes from rebuilding the
track's drawn spec (`<track>.track.json`), which is why a venue track must be
drawn, in the track editor or as a spec.

See docs/venues.md for the plan this is the first step of.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from .catalogue import CLASSES
from .generate import FLOOR_HALF_M, FORMAT as ENV_FORMAT, dumps, footprint
from .spec import SpecError, parse_spec

FORMAT = "ifssim-venue/1"
SEP = "__"                 # <venue>__<track>.csv
MAX_COORD_M = 500.0

# Props that may overlap others: a sidewalk is a flat slab under the 0.30 m
# rule (LiDAR-only), so a lamp's arm over it or a bench on its edge is fine.
FLAT_CLASSES = ("sidewalk",)


class VenueError(ValueError):
    """The venue file is malformed."""


@dataclass
class Venue:
    name: str
    title: str
    lot: Polygon
    margin_m: float
    props: List[Dict[str, Any]] = field(default_factory=list)   # venue frame, rows expanded

    def extent(self) -> Tuple[float, float, float, float]:
        shapes = [self.lot] + [footprint(CLASSES[p["class"]], p["x"], p["y"], p.get("yaw_deg", 0.0))
                               for p in self.props if p["class"] in CLASSES]
        return unary_union(shapes).bounds


# ---------------------------------------------------------------- parsing
def _num(v: Any, where: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise VenueError(f"{where} must be a number, not {v!r}")
    if abs(v) > MAX_COORD_M:
        raise VenueError(f"{where}: {v} is beyond +-{MAX_COORD_M:.0f} m (cm for m?)")
    return float(v)


def _xy(v: Any, where: str) -> Tuple[float, float]:
    if not (isinstance(v, list) and len(v) == 2):
        raise VenueError(f"{where} must be [x, y]")
    return _num(v[0], f"{where}[0]"), _num(v[1], f"{where}[1]")


def _keys(d: Any, where: str, allowed: Tuple[str, ...]) -> Dict[str, Any]:
    if not isinstance(d, dict):
        raise VenueError(f"{where} must be an object")
    extra = sorted(set(d) - set(allowed))
    if extra:
        raise VenueError(f"{where}: unknown key(s) {extra} (allowed: {list(allowed)})")
    return d


def _prop(cls: str, x: float, y: float, yaw: float) -> Dict[str, Any]:
    return {"class": cls, "x": round(x, 3), "y": round(y, 3), "yaw_deg": round(yaw, 2)}


def parse_venue(d: Any) -> Venue:
    """Parse and check a venue dict. Strict, like the sidecar: unknown keys are errors."""
    d = _keys(d, "venue", ("format", "name", "title", "lot", "margin_m", "props", "rows"))
    if d.get("format") != FORMAT:
        raise VenueError(f'venue.format must be "{FORMAT}", not {d.get("format")!r}')
    name = d.get("name")
    if not isinstance(name, str) or not name or SEP in name or not name.replace("_", "").isalnum():
        raise VenueError("venue.name must be letters, digits and single underscores")
    title = d.get("title", name)
    if not isinstance(title, str):
        raise VenueError("venue.title must be a string")

    lot_raw = d.get("lot")
    if not isinstance(lot_raw, list) or len(lot_raw) < 3:
        raise VenueError("venue.lot must be a polygon: at least 3 [x, y] points")
    lot = Polygon([_xy(p, f"venue.lot[{i}]") for i, p in enumerate(lot_raw)])
    if not lot.is_valid or lot.area <= 0:
        raise VenueError("venue.lot is not a simple polygon")
    margin = _num(d.get("margin_m", 2.0), "venue.margin_m")
    if margin < 0:
        raise VenueError("venue.margin_m must not be negative")

    props: List[Dict[str, Any]] = []
    for i, p in enumerate(d.get("props", [])):
        where = f"venue.props[{i}]"
        p = _keys(p, where, ("class", "x", "y", "yaw_deg"))
        cls = p.get("class")
        if cls not in CLASSES:
            raise VenueError(f"{where}: unknown class {cls!r} (known: {sorted(CLASSES)})")
        props.append(_prop(cls, _num(p.get("x"), f"{where}.x"), _num(p.get("y"), f"{where}.y"),
                           _num(p.get("yaw_deg", 0.0), f"{where}.yaw_deg")))

    # A row: props of one class from `from` to `to`, `spacing` apart, facing
    # along the row unless `yaw_deg` says otherwise. Sidewalks, tree lines,
    # lamp posts, a column of parked cars.
    for i, r in enumerate(d.get("rows", [])):
        where = f"venue.rows[{i}]"
        r = _keys(r, where, ("class", "from", "to", "spacing", "yaw_deg"))
        cls = r.get("class")
        if cls not in CLASSES:
            raise VenueError(f"{where}: unknown class {cls!r} (known: {sorted(CLASSES)})")
        (x0, y0), (x1, y1) = _xy(r.get("from"), f"{where}.from"), _xy(r.get("to"), f"{where}.to")
        spacing = _num(r.get("spacing"), f"{where}.spacing")
        if spacing <= 0:
            raise VenueError(f"{where}.spacing must be positive")
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = _num(r["yaw_deg"], f"{where}.yaw_deg") if "yaw_deg" in r else \
            math.degrees(math.atan2(y1 - y0, x1 - x0))
        n = int(math.floor(length / spacing + 1e-9)) + 1
        for k in range(n):
            t = 0.0 if n == 1 else k * spacing / length
            props.append(_prop(cls, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))

    return Venue(name=name, title=title, lot=lot, margin_m=margin, props=props)


def load_venue(path: str | Path) -> Venue:
    try:
        return parse_venue(json.loads(Path(path).read_text()))
    except json.JSONDecodeError as e:
        raise VenueError(f"{path}: not JSON: {e}") from e


# ---------------------------------------------------------------- checks
def check_venue(v: Venue) -> List[str]:
    """What's wrong with the place itself: props on the lot or on each other."""
    errors: List[str] = []
    lot_inner = v.lot.buffer(-0.01)
    shapes = []
    for i, p in enumerate(v.props):
        fp = footprint(CLASSES[p["class"]], p["x"], p["y"], p.get("yaw_deg", 0.0))
        if fp.intersects(lot_inner):
            errors.append(f"prop {i} ({p['class']} at {p['x']}, {p['y']}) stands on the lot, "
                          f"where tracks go")
        shapes.append((i, p, fp))
    solid = [(i, p, fp) for i, p, fp in shapes if p["class"] not in FLAT_CLASSES]
    for a in range(len(solid)):
        for b in range(a + 1, len(solid)):
            ia, pa, fa = solid[a]
            ib, pb, fb = solid[b]
            if fa.buffer(-0.01).intersects(fb.buffer(-0.01)):
                errors.append(f"props {ia} ({pa['class']}) and {ib} ({pb['class']}) overlap")
    return errors


def track_offset(spec: Dict[str, Any]) -> Tuple[Tuple[float, float], Any]:
    """Build a drawn spec: the shift that recentred it, and the built track."""
    from . import create_track
    created = create_track(parse_spec(spec))
    return created.track.offset, created


def check_track(v: Venue, spec: Dict[str, Any]) -> List[str]:
    """A track must stay on the lot, and the venue on the sim's floor once moved with it."""
    errors: List[str] = []
    (ox, oy), created = track_offset(spec)
    inner = v.lot.buffer(-v.margin_m)
    outside = [(t, x - ox, y - oy) for t, x, y in created.track.cones
               if not inner.contains(Point(x - ox, y - oy))]
    if outside:
        t, x, y = outside[0]
        errors.append(f"{len(outside)} cone(s) off the lot (keep {v.margin_m:.1f} m inside it), "
                      f"e.g. a {t} cone at ({x:.1f}, {y:.1f})")
    x0, y0, x1, y1 = v.extent()
    floor = box(-FLOOR_HALF_M, -FLOOR_HALF_M, FLOOR_HALF_M, FLOOR_HALF_M)
    if not floor.contains(box(x0 + ox, y0 + oy, x1 + ox, y1 + oy)):
        errors.append(f"with this track the venue doesn't fit on the sim's floor (+-{FLOOR_HALF_M:.0f} m)")
    return errors


# ---------------------------------------------------------------- output
def sidecar(v: Venue, offset: Tuple[float, float]) -> Dict[str, Any]:
    """The env sidecar for one of the venue's tracks: its props, moved with the track."""
    ox, oy = offset
    x0, y0, x1, y1 = v.extent()
    return {
        "format": ENV_FORMAT,
        "seed": 0,                       # nothing random: the venue is fixed
        "profile": f"venue_{v.name}",
        "ground": {"extent": {"x_min": round(x0 + ox, 3), "y_min": round(y0 + oy, 3),
                              "x_max": round(x1 + ox, 3), "y_max": round(y1 + oy, 3)}},
        "props": [_prop(p["class"], p["x"] + ox, p["y"] + oy, p["yaw_deg"]) for p in v.props],
    }


def venue_tracks(v: Venue, tracks_dir: str | Path) -> List[Path]:
    return sorted(Path(tracks_dir).glob(f"{v.name}{SEP}*.csv"))


def build(venue_path: str | Path, tracks_dir: Optional[str | Path] = None, check_only: bool = False,
          log=print) -> int:
    """Check the venue and its tracks, and write each track's sidecar. Returns an exit code."""
    venue_path = Path(venue_path)
    try:
        v = load_venue(venue_path)
    except (OSError, VenueError) as e:
        log(f"envgen: {e}")
        return 2
    tracks_dir = Path(tracks_dir) if tracks_dir else venue_path.parent.parent
    bad = 0
    for e in check_venue(v):
        log(f"  [error] {e}")
        bad += 1
    log(f"{v.name}: {len(v.props)} props, lot {v.lot.area:.0f} m2")
    for csv in venue_tracks(v, tracks_dir):
        spec_path = csv.with_suffix(".track.json")
        if not spec_path.exists():
            log(f"  {csv.name}: [error] no {spec_path.name}: a venue track must be drawn (its spec "
                f"places it in the venue)")
            bad += 1
            continue
        try:
            spec = json.loads(spec_path.read_text())
            errors = check_track(v, spec)
            offset, _ = track_offset(spec)
        except (OSError, json.JSONDecodeError, SpecError) as e:
            log(f"  {csv.name}: [error] {e}")
            bad += 1
            continue
        for e in errors:
            log(f"  {csv.name}: [error] {e}")
        bad += len(errors)
        if errors or check_only:
            continue
        out = csv.with_name(csv.stem + ".env.json")
        out.write_text(dumps(sidecar(v, offset)))
        log(f"  wrote {out.name}")
    return 1 if bad else 0
