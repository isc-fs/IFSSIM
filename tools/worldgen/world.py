"""The world format (`ifssim-world/1`): a real place the sim builds around a track.

A world lives in `Content/tracks/worlds/<name>/`:

    world.json    what to build (this module checks it)
    terrain.i16   the ground: int16 heights, row-major, little-endian

A track uses it from its environment sidecar:

    "world": { "name": "montmelo_straight", "x": 0.0, "y": 0.0 }

where (x, y) is where the world's origin sits in the track CSV's frame. Mission
Control's recentring moves it with the cones.

World frame: metres, +x along the place's main axis (for the Montmeló straight,
the race direction from the start line), +y to the left of +x, +z up, origin
on the ground at a named reference point. It maps into the track frame by a
translation only, so a world's x/y are the CSV's x/y once (x, y) is added.

Standard library only: validate_tracks.py runs it in CI.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

FORMAT = "ifssim-world/1"
TERRAIN_FILE = "terrain.i16"
MAX_COORD_M = 2000.0        # anything beyond is a unit mistake, not a place
MAX_TERRAIN_CELLS = 2_000_000

SURFACE_MATERIALS = ("asphalt", "asphalt_light", "concrete", "grass")
LINE_COLOURS = ("white", "yellow")
WALL_MATERIALS = ("concrete",)
BUILDING_MATERIALS = ("concrete", "glass")


class WorldError(ValueError):
    """A world file that the plugin would refuse."""


# --- strict reading helpers ---------------------------------------------------

def _obj(v: Any, where: str, required: Sequence[str], optional: Sequence[str] = ()) -> Dict[str, Any]:
    if not isinstance(v, dict):
        raise WorldError(f"{where}: expected an object")
    unknown = set(v) - set(required) - set(optional)
    if unknown:
        raise WorldError(f"{where}: unknown key(s) {sorted(unknown)}")
    missing = [k for k in required if k not in v]
    if missing:
        raise WorldError(f"{where}: missing key(s) {missing}")
    return v


def _num(v: Any, where: str, lo: float = -MAX_COORD_M, hi: float = MAX_COORD_M) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise WorldError(f"{where}: expected a number")
    if not lo <= v <= hi:
        raise WorldError(f"{where}: {v} is outside [{lo}, {hi}]")
    return float(v)


def _str(v: Any, where: str, choices: Sequence[str] = ()) -> str:
    if not isinstance(v, str) or not v:
        raise WorldError(f"{where}: expected a non-empty string")
    if choices and v not in choices:
        raise WorldError(f"{where}: {v!r} is not one of {list(choices)}")
    return v


def _int(v: Any, where: str, lo: int, hi: int) -> int:
    if isinstance(v, bool) or not isinstance(v, int) or not lo <= v <= hi:
        raise WorldError(f"{where}: expected an integer in [{lo}, {hi}]")
    return v


def _box(e: Dict[str, Any], where: str) -> None:
    for a, b in (("x_min", "x_max"), ("y_min", "y_max")):
        if _num(e[a], f"{where}.{a}") >= _num(e[b], f"{where}.{b}"):
            raise WorldError(f"{where}: {a} must be below {b}")


# --- the format ---------------------------------------------------------------

def validate(world: Any) -> Dict[str, Any]:
    """Check a parsed world.json the way the plugin does. Returns it unchanged."""
    w = _obj(world, "world", ["format", "name", "title", "attribution", "frame", "terrain"],
             ["detail", "surfaces", "lines", "walls", "buildings", "grandstands"])
    if w["format"] != FORMAT:
        raise WorldError(f"world.format: expected {FORMAT!r}")
    _str(w["name"], "world.name")
    _str(w["title"], "world.title")
    if not isinstance(w["attribution"], list) or not w["attribution"]:
        raise WorldError("world.attribution: list every data source and its licence")
    for i, a in enumerate(w["attribution"]):
        _str(a, f"world.attribution[{i}]")

    f = _obj(w["frame"], "world.frame", ["crs", "origin_e", "origin_n", "origin_z", "x_bearing_deg"])
    _str(f["crs"], "world.frame.crs")
    for k in ("origin_e", "origin_n", "origin_z"):
        _num(f[k], f"world.frame.{k}", -1e7, 1e7)
    _num(f["x_bearing_deg"], "world.frame.x_bearing_deg", 0.0, 360.0)

    t = _obj(w["terrain"], "world.terrain", ["file", "x0", "y0", "cell_m", "nx", "ny", "z_unit_m"])
    if t["file"] != TERRAIN_FILE:
        raise WorldError(f"world.terrain.file: expected {TERRAIN_FILE!r}")
    _num(t["x0"], "world.terrain.x0"); _num(t["y0"], "world.terrain.y0")
    _num(t["cell_m"], "world.terrain.cell_m", 0.1, 10.0)
    nx = _int(t["nx"], "world.terrain.nx", 2, 20000)
    ny = _int(t["ny"], "world.terrain.ny", 2, 20000)
    if nx * ny > MAX_TERRAIN_CELLS:
        raise WorldError(f"world.terrain: {nx}x{ny} is over {MAX_TERRAIN_CELLS} cells")
    _num(t["z_unit_m"], "world.terrain.z_unit_m", 1e-4, 1.0)

    if "detail" in w:
        d = _obj(w["detail"], "world.detail", ["x_min", "x_max"])
        if _num(d["x_min"], "world.detail.x_min") >= _num(d["x_max"], "world.detail.x_max"):
            raise WorldError("world.detail: x_min must be below x_max")

    for i, s in enumerate(w.get("surfaces", [])):
        where = f"world.surfaces[{i}]"
        _obj(s, where, ["material", "x_min", "x_max", "y_min", "y_max"])
        _str(s["material"], f"{where}.material", SURFACE_MATERIALS)
        _box(s, where)

    for i, ln in enumerate(w.get("lines", [])):
        where = f"world.lines[{i}]"
        _obj(ln, where, ["x0", "y0", "x1", "y1", "width_m", "colour"])
        for k in ("x0", "y0", "x1", "y1"):
            _num(ln[k], f"{where}.{k}")
        _num(ln["width_m"], f"{where}.width_m", 0.02, 5.0)
        _str(ln["colour"], f"{where}.colour", LINE_COLOURS)
        if (ln["x0"], ln["y0"]) == (ln["x1"], ln["y1"]):
            raise WorldError(f"{where}: the two ends are the same point")

    for i, wl in enumerate(w.get("walls", [])):
        where = f"world.walls[{i}]"
        _obj(wl, where, ["material", "x0", "y0", "x1", "y1", "height_m", "thickness_m"], ["fence_height_m"])
        _str(wl["material"], f"{where}.material", WALL_MATERIALS)
        for k in ("x0", "y0", "x1", "y1"):
            _num(wl[k], f"{where}.{k}")
        h = _num(wl["height_m"], f"{where}.height_m", 0.1, 20.0)
        _num(wl["thickness_m"], f"{where}.thickness_m", 0.05, 5.0)
        if "fence_height_m" in wl and _num(wl["fence_height_m"], f"{where}.fence_height_m", 0.0, 20.0) <= h:
            raise WorldError(f"{where}: fence_height_m must be above height_m")

    for i, b in enumerate(w.get("buildings", [])):
        where = f"world.buildings[{i}]"
        _obj(b, where, ["material", "x_min", "x_max", "y_min", "y_max", "top_z"])
        _str(b["material"], f"{where}.material", BUILDING_MATERIALS)
        _box(b, where)
        _num(b["top_z"], f"{where}.top_z", -50.0, 200.0)

    for i, g in enumerate(w.get("grandstands", [])):
        where = f"world.grandstands[{i}]"
        _obj(g, where, ["x_min", "x_max", "front_y", "back_y", "front_z", "back_z",
                        "roof_front_y", "roof_back_y", "roof_front_z", "roof_back_z"])
        if _num(g["x_min"], f"{where}.x_min") >= _num(g["x_max"], f"{where}.x_max"):
            raise WorldError(f"{where}: x_min must be below x_max")
        for k in ("front_y", "back_y", "roof_front_y", "roof_back_y"):
            _num(g[k], f"{where}.{k}")
        for k in ("front_z", "back_z", "roof_front_z", "roof_back_z"):
            _num(g[k], f"{where}.{k}", -50.0, 200.0)
        if abs(g["back_y"]) <= abs(g["front_y"]):
            raise WorldError(f"{where}: back_y must be further from the track (|y|) than front_y")
        if g["back_z"] <= g["front_z"]:
            raise WorldError(f"{where}: the seating must rise from front to back")
    return w


def load(world_dir: Path) -> Tuple[Dict[str, Any], List[int]]:
    """Read and check a world directory: (world.json, terrain heights)."""
    world = validate(json.loads((Path(world_dir) / "world.json").read_text()))
    t = world["terrain"]
    raw = (Path(world_dir) / t["file"]).read_bytes()
    n = t["nx"] * t["ny"]
    if len(raw) != 2 * n:
        raise WorldError(f"{t['file']}: {len(raw)} bytes, expected {2 * n} for {t['nx']}x{t['ny']} int16")
    return world, list(struct.unpack(f"<{n}h", raw))


def write_terrain(path: Path, heights_units: Sequence[int]) -> None:
    """Write int16 heights, little-endian, index = j * nx + i (see height_at)."""
    Path(path).write_bytes(struct.pack(f"<{len(heights_units)}h", *heights_units))


def height_at(world: Dict[str, Any], heights: Sequence[int], x: float, y: float) -> float:
    """Ground height (m, world frame) at (x, y), bilinear; clamped at the edges.

    Layout: index = j * nx + i, with i along x and j along y.
    """
    t = world["terrain"]
    nx, ny, c, u = t["nx"], t["ny"], t["cell_m"], t["z_unit_m"]
    fx = min(max((x - t["x0"]) / c, 0.0), nx - 1.0)
    fy = min(max((y - t["y0"]) / c, 0.0), ny - 1.0)
    i0, j0 = min(int(fx), nx - 2), min(int(fy), ny - 2)
    ax, ay = fx - i0, fy - j0
    h = lambda i, j: heights[j * nx + i] * u
    return ((1 - ax) * (1 - ay) * h(i0, j0) + ax * (1 - ay) * h(i0 + 1, j0)
            + (1 - ax) * ay * h(i0, j0 + 1) + ax * ay * h(i0 + 1, j0 + 1))
