"""Build the `montmelo_straight` world: the main straight of the Circuit de
Barcelona-Catalunya, where Formula Student Spain runs its acceleration event.

Sources (both open, both credited in world.json and ATTRIBUTION.md):
- ICGC Territorial LiDAR v3r1 (2021-2023), CC BY 4.0: the ground, and the
  heights of the grandstand roof and the pit building. Tiles 438601 and
  438602, about 553 MB, are not in the repo; pass --cache to where they are.
- OpenStreetMap (ODbL): the straight's axis, the start line node, and where
  the pit lane, pit building and main grandstand start and end.

Frame (docs/worlds.md): x along the race direction from the painted start line,
y to the left, z from the ground at the start line. The straight runs from
x = -365.3 (its north-east end) to x = +683.7 (Turn 1).

Needs numpy and laspy[lazrs] (`pip install numpy "laspy[lazrs]"`); nothing
else in worldgen does.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

from . import world as W

NAME = "montmelo_straight"
TILES = ("438601", "438602")
TILE_FILE = "lidar-territorial-v3r1-full1km{}-2021-2023.laz"

# The straight's axis, EPSG:25831 (ETRS89 / UTM 31N), from OSM way 831804327:
# its longest straight run, 1049 m, NE end A to SW end B (race direction).
A = (438603.0, 4602608.0)
B = (438036.0, 4601726.0)
START_S = 365.3      # the painted start line, metres from A. LiDAR intensity
                     # shows it at 365.3; OSM's raceway=start node is at 365.2.
FINISH_S = 222.5     # the painted finish line (LiDAR); OSM's node is at 238.

# Terrain grid around the straight, world frame.
X0, X1 = -425.0, 745.0
Y0, Y1 = -160.0, 160.0
CELL = 1.0
DETAIL = (-100.0, 300.0)   # the 400 m modelled in full detail

GROUND_CLASSES = (2, 8)    # ICGC: ground, and ground model key points


def s_to_x(s: float) -> float:
    return s - START_S


def _axis() -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    L = math.dist(A, B)
    u = ((B[0] - A[0]) / L, (B[1] - A[1]) / L)
    n = (-u[1], u[0])                      # left of the race direction
    return u, n, L


def bearing_deg() -> float:
    u, _, _ = _axis()
    return math.degrees(math.atan2(u[0], u[1])) % 360.0


def load_points(cache: Path):
    """Every LiDAR point near the straight, in the world frame (x, y, z_abs, cls)."""
    import numpy as np
    import laspy
    u, n, _ = _axis()
    ox, oy = A[0] + START_S * u[0], A[1] + START_S * u[1]
    xs, ys, zs, cs = [], [], [], []
    for t in TILES:
        path = Path(cache) / TILE_FILE.format(t)
        if not path.is_file():
            raise FileNotFoundError(f"{path}: download it from ICGC (see docs/worlds.md)")
        las = laspy.read(str(path))
        dx, dy = np.asarray(las.x) - ox, np.asarray(las.y) - oy
        x = dx * u[0] + dy * u[1]
        y = dx * n[0] + dy * n[1]
        keep = (x >= X0 - 2) & (x <= X1 + 2) & (y >= Y0 - 2) & (y <= Y1 + 2)
        xs.append(x[keep]); ys.append(y[keep])
        zs.append(np.asarray(las.z)[keep]); cs.append(np.asarray(las.classification)[keep])
    return (np.concatenate(xs), np.concatenate(ys), np.concatenate(zs), np.concatenate(cs))


def ground_grid(x, y, z, c, z0: float):
    """Mean ground height per cell (m, relative to z0); holes filled from their
    neighbours (under buildings, where the scanner saw no ground)."""
    import numpy as np
    nx = int(round((X1 - X0) / CELL)) + 1
    ny = int(round((Y1 - Y0) / CELL)) + 1
    g = np.isin(c, GROUND_CLASSES)
    i = np.clip(np.rint((x[g] - X0) / CELL).astype(int), 0, nx - 1)
    j = np.clip(np.rint((y[g] - Y0) / CELL).astype(int), 0, ny - 1)
    flat = j * nx + i
    sm = np.bincount(flat, weights=z[g] - z0, minlength=nx * ny)
    ct = np.bincount(flat, minlength=nx * ny)
    h = np.full(nx * ny, np.nan)
    h[ct > 0] = sm[ct > 0] / ct[ct > 0]
    h = h.reshape(ny, nx)
    # Fill holes: repeatedly average the known 4-neighbours into unknown cells.
    while np.isnan(h).any():
        p = np.pad(h, 1, constant_values=np.nan)
        nb = np.stack([p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])
        cnt = (~np.isnan(nb)).sum(0)
        avg = np.where(cnt > 0, np.nansum(nb, 0) / np.maximum(cnt, 1), np.nan)
        fill = np.isnan(h) & (cnt > 0)
        if not fill.any():
            raise RuntimeError("terrain has cells no ground point reaches")
        h[fill] = avg[fill]
    return h, nx, ny


def pct(x, y, z, c, x_rng, y_rng, q: float, z0: float, classes=None) -> float:
    """The q-th percentile height (relative to z0) of the points in a box."""
    import numpy as np
    m = (x >= x_rng[0]) & (x < x_rng[1]) & (y >= y_rng[0]) & (y < y_rng[1])
    if classes is not None:
        m &= np.isin(c, classes)
    if m.sum() < 50:
        raise RuntimeError(f"too few points in box x{x_rng} y{y_rng}")
    return float(np.percentile(z[m] - z0, q))


def build(cache: Path, out_dir: Path) -> Dict:
    """Measure, then write world.json and terrain.i16 into out_dir."""
    import numpy as np
    x, y, z, c = load_points(cache)
    gm = np.isin(c, GROUND_CLASSES)
    # Datum: the ground on the track centre at the start line.
    near = gm & (np.abs(x) < 5.0) & (np.abs(y) < 1.0)
    z0 = float(np.median(z[near]))
    h, nx, ny = ground_grid(x, y, z, c, z0)
    units = np.clip(np.rint(h / 0.01), -32768, 32767).astype("<i2")

    sx = s_to_x
    # Measured from the LiDAR (points above the stands, relative to z0).
    gs_x = (sx(209.0), sx(498.0))            # OSM: main grandstand extent along the straight
    roof_front = pct(x, y, z, c, gs_x, (19.0, 23.0), 50, z0)
    roof_back = pct(x, y, z, c, gs_x, (40.0, 42.0), 50, z0)
    pit_x = (sx(245.0), sx(546.0))           # OSM: pit building ("Boxes") extent
    pit_h = pct(x, y, z, c, pit_x, (-44.0, -30.0), 50, z0)

    pits = (sx(133.0), sx(538.0))            # OSM: pit lane along the straight
    track = (sx(0.0), sx(1049.0))
    stand_side = (sx(150.0), sx(560.0))      # barrier and run-off in front of the stands

    world = {
        "format": W.FORMAT,
        "name": NAME,
        "title": "Circuit de Barcelona-Catalunya, main straight",
        "attribution": [
            "Terrain and structure heights: Institut Cartogràfic i Geològic de Catalunya (ICGC), "
            "Territorial LiDAR v3r1 2021-2023, CC BY 4.0",
            "Layout: © OpenStreetMap contributors, ODbL 1.0",
        ],
        "frame": {"crs": "EPSG:25831",
                  "origin_e": round(A[0] + START_S * _axis()[0][0], 2),
                  "origin_n": round(A[1] + START_S * _axis()[0][1], 2),
                  "origin_z": round(z0, 2),
                  "x_bearing_deg": round(bearing_deg(), 2)},
        "terrain": {"file": W.TERRAIN_FILE, "x0": X0, "y0": Y0, "cell_m": CELL,
                    "nx": nx, "ny": ny, "z_unit_m": 0.01},
        "detail": {"x_min": DETAIL[0], "x_max": DETAIL[1]},
        # Cross-section measured at the acceleration zone (LiDAR ground and
        # intensity, docs/worlds.md): the track is 20 m of asphalt with a
        # light painted band along each edge; the pit lane is 13 m wide.
        "surfaces": [
            {"material": "asphalt", "x_min": track[0], "x_max": track[1], "y_min": -10.25, "y_max": 10.0},
            {"material": "asphalt_light", "x_min": track[0], "x_max": track[1], "y_min": -10.0, "y_max": -7.5},
            {"material": "asphalt_light", "x_min": track[0], "x_max": track[1], "y_min": 5.5, "y_max": 9.5},
            {"material": "asphalt", "x_min": pits[0], "x_max": pits[1], "y_min": -25.5, "y_max": -12.5},
            {"material": "concrete", "x_min": stand_side[0], "x_max": stand_side[1], "y_min": 11.0, "y_max": 15.5},
        ],
        "lines": [
            {"x0": 0.0, "y0": -10.25, "x1": 0.0, "y1": 10.0, "width_m": 0.6, "colour": "white"},
            {"x0": sx(FINISH_S), "y0": -10.25, "x1": sx(FINISH_S), "y1": 10.0, "width_m": 0.6, "colour": "white"},
            {"x0": pits[0], "y0": -17.75, "x1": pits[1], "y1": -17.75, "width_m": 0.3, "colour": "white"},
        ],
        "walls": [
            # Pit wall: about 1.1 m of concrete, catch fence above (LiDAR tops ~2.1 m).
            {"material": "concrete", "x0": pits[0], "y0": -11.2, "x1": pits[1], "y1": -11.2,
             "height_m": 1.1, "thickness_m": 0.8, "fence_height_m": 2.1},
            # Barrier on the grandstand side, fence to ~1.8 m.
            {"material": "concrete", "x0": stand_side[0], "y0": 10.5, "x1": stand_side[1], "y1": 10.5,
             "height_m": 1.0, "thickness_m": 0.5, "fence_height_m": 1.8},
            # The grandstand's front wall (~2.7 m).
            {"material": "concrete", "x0": gs_x[0], "y0": 16.0, "x1": gs_x[1], "y1": 16.0,
             "height_m": 2.7, "thickness_m": 0.4},
        ],
        "buildings": [
            {"material": "concrete", "x_min": pit_x[0], "x_max": pit_x[1], "y_min": -46.0, "y_max": -26.0,
             "top_z": round(pit_h, 2)},
        ],
        "grandstands": [
            # Roof measured; the seating under it is not visible from the air,
            # so its rake (3 m at the front to 20 m at the back) is an estimate.
            {"x_min": gs_x[0], "x_max": gs_x[1], "front_y": 18.5, "back_y": 42.5,
             "front_z": 3.0, "back_z": 20.0,
             "roof_front_y": 18.5, "roof_back_y": 44.0,
             "roof_front_z": round(roof_front, 2), "roof_back_z": round(roof_back, 2)},
        ],
        # Generic models, not surveyed: the start gantry over the painted start
        # line, and two marshal posts on the grandstand-side run-off, clear of
        # the acceleration zone.
        "props": [
            {"mesh": "StartGantry", "x": 0.0, "y": 0.0},
            {"mesh": "MarshalPost", "x": -80.0, "y": 13.25},
            {"mesh": "MarshalPost", "x": 170.0, "y": 13.25},
        ],
    }
    for k in ("surfaces", "walls", "lines", "buildings", "grandstands", "props"):
        for e in world[k]:
            for f, v in list(e.items()):
                if isinstance(v, float):
                    e[f] = round(v, 2)
    W.validate(world)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "world.json").write_text(json.dumps(world, indent=2, ensure_ascii=False) + "\n")
    (out_dir / W.TERRAIN_FILE).write_bytes(units.tobytes(order="C"))
    (out_dir / "ATTRIBUTION.md").write_text(ATTRIBUTION)
    return world


ATTRIBUTION = """# montmelo_straight: data sources

This world is generated by `tools/worldgen` (`python -m worldgen build montmelo_straight`).

- **Terrain and structure heights:** Institut Cartogràfic i Geològic de Catalunya (ICGC),
  Territorial LiDAR v3r1, 2021-2023, tiles 438601 and 438602. Licensed under
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Derived: a 1 m ground grid
  and a few measured heights.
- **Layout:** © OpenStreetMap contributors, [ODbL 1.0](https://opendatacommons.org/licenses/odbl/).
  Used: the straight's axis, the start line, and the extents of the pit lane, pit building
  and main grandstand.

Signage is generic on purpose: no circuit or sponsor logos.
"""


def acceleration_track() -> Tuple[str, Dict]:
    """The FS acceleration layout on this world: the standard one, its start
    gate on the painted start line, centred on the track. Returns (csv, sidecar)."""
    rows: List[str] = []
    for side, y in (("yellow", -1.5), ("blue", 1.5)):
        for k in range(1, 17):
            rows.append(f"{side},{5.0 * k:.1f},{y},0,0,0,0")
    for xg in (3.0, 7.0, 78.0, 82.0):
        for y in (-1.5, 1.5):
            rows.append(f"big_orange,{xg:.1f},{y},0,0,0,0")
    sidecar = {"format": "ifssim-env/1", "seed": 0, "profile": f"world_{NAME}",
               "world": {"name": NAME, "x": 0.0, "y": 0.0}}
    return "\n".join(rows) + "\n", sidecar
