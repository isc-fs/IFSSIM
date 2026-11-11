"""The world format (`ifssim-world/1`): a real place the sim builds around a track.

A world lives in `Content/tracks/worlds/<name>/`:

    world.json    what to build (this module checks it)
    terrain.i16   the ground: int16 heights, row-major, little-endian
    wear.u8       optional: asphalt wear, uint8 row-major (128 = as scanned)

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
WEAR_FILE = "wear.u8"
MAX_COORD_M = 2000.0        # anything beyond is a unit mistake, not a place
MAX_TERRAIN_CELLS = 2_000_000

SURFACE_MATERIALS = ("asphalt", "asphalt_light", "concrete", "grass")
LINE_COLOURS = ("white", "yellow", "red", "green")
MAX_LINE_WIDTH_M = 10.0     # a painted area (a pit-exit wedge) is a wide line
MAX_ROOF_PANELS = 500
WALL_MATERIALS = ("concrete",)
BUILDING_MATERIALS = ("concrete", "glass")
# Props a world places by name (docs/world_assets.md); the plugin places the
# rest itself, from the structures they belong to.
PROP_MESHES = ("StartGantry", "MarshalPost", "SignBoard")
TREE_KINDS = ("pine", "broadleaf")
MAX_TREES = 20_000
ROOF_KEYS = ("roof_front_y", "roof_back_y", "roof_front_z", "roof_back_z")


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


def _panels(v: Any, where: str, x_rng: Tuple[float, float], d_rng: Tuple[float, float]) -> None:
    """Roof panels: boxes inside x_rng, and inside d_rng in |y| (distance from the axis)."""
    if not isinstance(v, list) or not v or len(v) > MAX_ROOF_PANELS:
        raise WorldError(f"{where}: expected a list of 1 to {MAX_ROOF_PANELS} boxes")
    for i, p in enumerate(v):
        w = f"{where}[{i}]"
        _obj(p, w, ["x_min", "x_max", "y_min", "y_max"])
        _box(p, w)
        d = sorted((abs(p["y_min"]), abs(p["y_max"])))
        if (p["y_min"] < 0.0 < p["y_max"] or p["x_min"] < x_rng[0] or p["x_max"] > x_rng[1]
                or d[0] < d_rng[0] - 1e-6 or d[1] > d_rng[1] + 1e-6):
            raise WorldError(f"{w}: outside the roof")


# --- the format ---------------------------------------------------------------

def validate(world: Any) -> Dict[str, Any]:
    """Check a parsed world.json the way the plugin does. Returns it unchanged."""
    w = _obj(world, "world", ["format", "name", "title", "attribution", "frame", "terrain"],
             ["detail", "wear", "surfaces", "lines", "walls", "buildings", "grandstands", "props", "trees"])
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

    if "wear" in w:
        r = _obj(w["wear"], "world.wear", ["file", "x0", "y0", "dx_m", "dy_m", "nx", "ny"])
        if r["file"] != WEAR_FILE:
            raise WorldError(f"world.wear.file: expected {WEAR_FILE!r}")
        _num(r["x0"], "world.wear.x0"); _num(r["y0"], "world.wear.y0")
        _num(r["dx_m"], "world.wear.dx_m", 0.05, 10.0)
        _num(r["dy_m"], "world.wear.dy_m", 0.05, 10.0)
        wx = _int(r["nx"], "world.wear.nx", 2, 20000)
        wy = _int(r["ny"], "world.wear.ny", 2, 20000)
        if wx * wy > MAX_TERRAIN_CELLS:
            raise WorldError(f"world.wear: {wx}x{wy} is over {MAX_TERRAIN_CELLS} cells")

    for i, s in enumerate(w.get("surfaces", [])):
        where = f"world.surfaces[{i}]"
        _obj(s, where, ["material", "x_min", "x_max", "y_min", "y_max"])
        _str(s["material"], f"{where}.material", SURFACE_MATERIALS)
        _box(s, where)

    for i, ln in enumerate(w.get("lines", [])):
        where = f"world.lines[{i}]"
        _obj(ln, where, ["x0", "y0", "x1", "y1", "width_m", "colour"], ["end_width_m"])
        for k in ("x0", "y0", "x1", "y1"):
            _num(ln[k], f"{where}.{k}")
        _num(ln["width_m"], f"{where}.width_m", 0.02, MAX_LINE_WIDTH_M)
        if "end_width_m" in ln:
            _num(ln["end_width_m"], f"{where}.end_width_m", 0.02, MAX_LINE_WIDTH_M)
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
        _obj(b, where, ["material", "x_min", "x_max", "y_min", "y_max", "top_z"],
             ["yaw_deg", "base_z", "garages", "canopy_m", "roof_panels"])
        _str(b["material"], f"{where}.material", BUILDING_MATERIALS)
        _box(b, where)
        top = _num(b["top_z"], f"{where}.top_z", -50.0, 200.0)
        if "yaw_deg" in b:
            _num(b["yaw_deg"], f"{where}.yaw_deg", -360.0, 360.0)
        if "base_z" in b and _num(b["base_z"], f"{where}.base_z", -50.0, 200.0) >= top:
            raise WorldError(f"{where}: base_z must be below top_z")
        if "garages" in b and not isinstance(b["garages"], bool):
            raise WorldError(f"{where}.garages: expected true or false")
        if "canopy_m" in b:
            _num(b["canopy_m"], f"{where}.canopy_m", 0.5, 10.0)
            if b.get("garages") is not True or b.get("yaw_deg", 0.0) != 0.0:
                raise WorldError(f"{where}: a canopy shades garages, on a building with yaw_deg 0")
        if "roof_panels" in b:
            if b.get("yaw_deg", 0.0) != 0.0 or "base_z" in b:
                raise WorldError(f"{where}: roof panels need a building with yaw_deg 0 and no base_z")
            if b["y_min"] < 0.0 < b["y_max"]:
                raise WorldError(f"{where}: roof panels need a building on one side of the axis")
            near, far = sorted((abs(b["y_min"]), abs(b["y_max"])))
            _panels(b["roof_panels"], f"{where}.roof_panels", (b["x_min"], b["x_max"]), (near, far))

    for i, g in enumerate(w.get("grandstands", [])):
        where = f"world.grandstands[{i}]"
        _obj(g, where, ["x_min", "x_max", "front_y", "back_y", "front_z", "back_z"], ROOF_KEYS + ("roof_panels",))
        roof = [k for k in ROOF_KEYS if k in g]
        if roof and len(roof) != len(ROOF_KEYS):
            raise WorldError(f"{where}: a roof needs all of {list(ROOF_KEYS)}, or none of them")
        if _num(g["x_min"], f"{where}.x_min") >= _num(g["x_max"], f"{where}.x_max"):
            raise WorldError(f"{where}: x_min must be below x_max")
        for k in ("front_y", "back_y", "roof_front_y", "roof_back_y"):
            if k in g:
                _num(g[k], f"{where}.{k}")
        for k in ("front_z", "back_z", "roof_front_z", "roof_back_z"):
            if k in g:
                _num(g[k], f"{where}.{k}", -50.0, 200.0)
        if abs(g["back_y"]) <= abs(g["front_y"]):
            raise WorldError(f"{where}: back_y must be further from the track (|y|) than front_y")
        if g["back_z"] <= g["front_z"]:
            raise WorldError(f"{where}: the seating must rise from front to back")
        if "roof_panels" in g:
            if not roof:
                raise WorldError(f"{where}: roof panels need a roof")
            if g["roof_front_y"] * g["roof_back_y"] <= 0.0:
                raise WorldError(f"{where}: roof panels need a roof on one side of the axis")
            near, far = sorted((abs(g["roof_front_y"]), abs(g["roof_back_y"])))
            _panels(g["roof_panels"], f"{where}.roof_panels", (g["x_min"], g["x_max"]), (near, far))

    for i, p in enumerate(w.get("props", [])):
        where = f"world.props[{i}]"
        _obj(p, where, ["mesh", "x", "y"], ["yaw_deg"])
        _str(p["mesh"], f"{where}.mesh", PROP_MESHES)
        _num(p["x"], f"{where}.x")
        _num(p["y"], f"{where}.y")
        if "yaw_deg" in p:
            _num(p["yaw_deg"], f"{where}.yaw_deg", -360.0, 360.0)

    trees = w.get("trees", [])
    if not isinstance(trees, list) or len(trees) > MAX_TREES:
        raise WorldError(f"world.trees: expected a list of at most {MAX_TREES}")
    for i, t in enumerate(trees):
        where = f"world.trees[{i}]"
        _obj(t, where, ["x", "y", "h", "r", "kind"])
        _num(t["x"], f"{where}.x")
        _num(t["y"], f"{where}.y")
        _num(t["h"], f"{where}.h", 1.0, 60.0)
        _num(t["r"], f"{where}.r", 0.3, 20.0)
        _str(t["kind"], f"{where}.kind", TREE_KINDS)
    return w


def load(world_dir: Path) -> Tuple[Dict[str, Any], List[int]]:
    """Read and check a world directory: (world.json, terrain heights). The
    wear map, if the world has one, must be there and the right size."""
    world = validate(json.loads((Path(world_dir) / "world.json").read_text()))
    t = world["terrain"]
    raw = (Path(world_dir) / t["file"]).read_bytes()
    n = t["nx"] * t["ny"]
    if len(raw) != 2 * n:
        raise WorldError(f"{t['file']}: {len(raw)} bytes, expected {2 * n} for {t['nx']}x{t['ny']} int16")
    if "wear" in world:
        r = world["wear"]
        size = (Path(world_dir) / r["file"]).stat().st_size
        if size != r["nx"] * r["ny"]:
            raise WorldError(f"{r['file']}: {size} bytes, expected {r['nx'] * r['ny']} for {r['nx']}x{r['ny']} uint8")
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
