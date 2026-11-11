"""Build the `montmelo_straight` world: the main straight of the Circuit de
Barcelona-Catalunya, where Formula Student Spain runs its acceleration event.

Sources (all open, all credited in world.json and ATTRIBUTION.md):
- ICGC Territorial LiDAR v3r1 (2021-2023), CC BY 4.0: the ground, and the
  heights of the grandstand roof and the pit building. Tiles 438601 and
  438602, about 553 MB, are not in the repo; pass --cache to where they are.
- ICGC territorial orthophoto (current colour, 25 cm), CC BY 4.0: the track
  width, the painted markings, the roof panels and the asphalt's brightness
  across the track, measured once by hand into the constants below. The
  photo itself is not in the repo, and the generator doesn't read it.
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

from . import lidar_features as LF
from . import wear as WEAR
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
# The painted finish line is at x = -127.0 (orthophoto); OSM's node is 11 m
# further on, and nothing is painted where the LiDAR intensity seemed to put it.
FINISH_X = -127.0

# Terrain grid around the straight, world frame.
X0, X1 = -425.0, 745.0
Y0, Y1 = -160.0, 160.0
CELL = 1.0
DETAIL = (-100.0, 300.0)   # the 400 m modelled in full detail

GROUND_CLASSES = (2, 8)    # ICGC: ground, and ground model key points


def s_to_x(s: float) -> float:
    return s - START_S


# --- measured from the orthophoto (world frame, m) ----------------------------
# Track: asphalt from y -7.3 to 6.05, white edge lines at -7.0 and 5.69. The
# 3-4 m beyond each edge line is dry grass, not asphalt.
TRACK_Y = (-7.3, 6.05)
EDGE_Y = (-7.0, 5.69)
# Brightness of the asphalt across the track (photo values, sunlit, x 180-295):
# the left half is darker where the rubber is.
WEAR_PROFILE = (
    (5.0, 178.0), (4.5, 184.8), (4.0, 179.1), (3.5, 173.2), (3.0, 169.2), (2.5, 168.4),
    (2.0, 172.6), (1.5, 177.2), (1.0, 179.2), (0.5, 179.9), (0.0, 179.5), (-0.5, 179.9),
    (-1.0, 180.7), (-1.5, 182.0), (-2.0, 183.1), (-2.5, 183.5), (-3.0, 183.4), (-3.5, 184.8),
    (-4.0, 186.6), (-4.5, 188.2), (-5.0, 190.8), (-5.5, 193.3), (-6.0, 190.1),
)
# Along the track the photo shows only +-1 % (10 m windows); the noise stays below it.
WEAR_NOISE = ((40.0, 4.0, 0.02), (9.0, 1.5, 0.015))
# Other full-width lines (their purpose isn't signed): x, width.
TRANSVERSE = ((-197.7, 0.3), (32.4, 0.3))
# Grid slots: the front bar of each, right lane (y -5.0..-2.4); the left lane's
# (y 1.1..3.7) are 7.38 m further on. Each slot is a white bracket opening
# backwards, 3.3 m long, with a yellow mark from its inner side to the middle.
GRID_RIGHT_X = (-208.06, -192.69, -177.31, -161.94, -146.5, -131.06, -115.75,
                -100.25, -84.88, -69.44, -54.62, -39.88, -25.19, -10.44)
GRID_STAGGER = 7.38
GRID_BOX = {"right": (-5.0, -2.4), "left": (1.1, 3.7)}
GRID_TICK = {"right": (-2.3, 0.05), "left": (1.0, -0.95)}
GRID_ARM_M, GRID_LINE_M = 3.3, 0.15
# The pit lane: a white line along the wall at -13.06, and a red band with
# white borders between the fast lane and the working lane.
PIT_EDGE_Y = -13.06
RED_BAND = (-134.1, 176.7, -17.5, 1.1)          # x0, x1, y, width
RED_BORDERS_Y = (-16.8, -18.2)
# Green-painted areas on the verge between the track and the pit wall.
VERGE_GREEN = ((-138.1, -132.2), (-10.4, -1.1))
# The pit exit: the wall ends at 224; the exit lane runs between a green
# wedge (track side) and a white line, and joins the track at 316.
PIT_WALL_END_X = 224.0
EXIT_JOIN_X = 316.0
EXIT_WEDGE = ((226.0, -13.5), (292.0, -10.3), (EXIT_JOIN_X, -7.32))   # its lower edge
EXIT_LINE = ((230.0, -20.6), (332.0, -13.0))
EXIT_AREA = (172.7, 330.0, -23.0)                # x0, x1, y of its far edge
# Pit building roof: blocks of panel rows (6 per block, 1.46 m deep), the
# panels 3 m in from each block's ends. Rows mapped from the photo onto the
# building (relief-displaced there).
PIT_BLOCKS = ((-123.9, -83.6), (-83.1, -35.4), (-28.1, 12.7), (13.1, 60.8), (61.2, 108.8), (109.2, 157.1))
PIT_ROWS_Y = (-27.74, -30.39, -33.13, -38.01, -40.75, -43.54)
PIT_ROW_M = 1.46
PIT_CANOPY_M = 2.0
# Main grandstand roof: two bands of solar panels in ~20 m sections (y mapped
# onto the roof, which runs 18.5..44).
GS_PANEL_BANDS = {
    (23.94, 30.73): ((-128.8, -113.6), (-112.9, -93.4), (-92.7, -73.3), (-72.6, -53.3), (-52.6, -33.3),
                     (-32.7, -19.7), (-18.9, 0.3), (0.9, 20.6), (21.3, 40.7), (41.3, 60.6), (61.2, 80.7),
                     (81.3, 94.3)),
    (32.6, 37.0): ((-128.6, -113.6), (-112.8, -93.4), (-92.7, -73.3), (-72.6, -53.3), (-52.6, -33.2),
                   (-28.2, -19.7), (-18.9, 0.4), (1.2, 20.6), (21.3, 40.7), (41.3, 60.7), (61.4, 80.8),
                   (81.6, 94.3)),
}


def _line(x0, y0, x1, y1, w, colour, w1=None) -> Dict:
    ln = {"x0": x0, "y0": y0, "x1": x1, "y1": y1, "width_m": w, "colour": colour}
    if w1 is not None:
        ln["end_width_m"] = w1
    return ln


def markings(track: Tuple[float, float], pits: Tuple[float, float]) -> List[Dict]:
    """Every painted line and area, from the orthophoto."""
    lines = []
    y0, y1 = TRACK_Y
    # Start, finish and the other full-width lines.
    lines.append(_line(0.0, y0, 0.0, y1, 0.6, "white"))
    lines.append(_line(FINISH_X, y0, FINISH_X, y1, 0.6, "white"))
    for x, w in TRANSVERSE:
        lines.append(_line(x, y0, x, y1, w, "white"))
    # Edge lines; the right one stops where the pit exit joins.
    lines.append(_line(track[0], EDGE_Y[1], track[1], EDGE_Y[1], 0.3, "white"))
    lines.append(_line(track[0], EDGE_Y[0], PIT_WALL_END_X, EDGE_Y[0], 0.3, "white"))
    lines.append(_line(EXIT_JOIN_X, EDGE_Y[0], track[1], EDGE_Y[0], 0.3, "white"))
    # The grid.
    g = GRID_LINE_M
    for xr in GRID_RIGHT_X:
        for lane, xf in (("right", xr), ("left", round(xr + GRID_STAGGER, 2))):
            a, b = GRID_BOX[lane]
            lines.append(_line(xf, a, xf, b, g, "white"))
            for y in (a, b):
                lines.append(_line(xf - g / 2, y, xf - GRID_ARM_M, y, g, "white"))
            t0, t1 = GRID_TICK[lane]
            lines.append(_line(xf, t0, xf, t1, 0.12, "yellow"))
    # The pit lane.
    lines.append(_line(pits[0], PIT_EDGE_Y, PIT_WALL_END_X, PIT_EDGE_Y, 0.25, "white"))
    rx0, rx1, ry, rw = RED_BAND
    lines.append(_line(rx0, ry, rx1, ry, rw, "red"))
    for y in RED_BORDERS_Y:
        lines.append(_line(rx0, y, rx1, y, 0.25, "white"))
    # Green on the verge, from the asphalt's edge to the pit wall's face (-10.8).
    for xa, xb in VERGE_GREEN:
        lines.append(_line(xa, -9.05, xb, -9.05, 3.4, "green"))
    # The pit exit: the green wedge, its lower edge in two straight pieces and
    # its upper edge on the asphalt's edge, then the lane's white line.
    (xa, ya), (xb, yb), (xc, yc) = EXIT_WEDGE
    top = TRACK_Y[0] - 0.02
    for (p, q, r, t) in ((xa, ya, xb, yb), (xb, yb, xc, yc)):
        lines.append(_line(p, (q + top) / 2, r, (t + top) / 2, round(top - q, 2), "green",
                           max(0.02, round(top - t, 2))))
    (ea, eb), (ec, ed) = EXIT_LINE
    lines.append(_line(ea, eb, ec, ed, 0.25, "white"))
    return lines


def pit_roof_panels(pit_x: Tuple[float, float]) -> List[Dict]:
    out = []
    for a, b in PIT_BLOCKS:
        xa, xb = max(a + 3.0, pit_x[0] + 0.5), min(b - 3.0, pit_x[1] - 0.5)
        if xb - xa < 5.0:
            continue
        for y in PIT_ROWS_Y:
            out.append({"x_min": round(xa, 2), "x_max": round(xb, 2),
                        "y_min": round(y - PIT_ROW_M / 2, 2), "y_max": round(y + PIT_ROW_M / 2, 2)})
    return out


def grandstand_panels(gs_x: Tuple[float, float]) -> List[Dict]:
    return [{"x_min": max(a, gs_x[0]), "x_max": min(b, gs_x[1]), "y_min": lo, "y_max": hi}
            for (lo, hi), runs in GS_PANEL_BANDS.items() for a, b in runs]


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

    wear_entry, wear_bytes = WEAR.build(WEAR_PROFILE, track, (TRACK_Y[0] - 0.2, TRACK_Y[1] + 0.2),
                                        1.0, 0.25, WEAR_NOISE, seed=8)

    # Everything else the LiDAR shows: the open grandstands further along the
    # straight, the other buildings and footbridges, and the trees.
    assert CELL == 1.0, "lidar_features works on a 1 m grid"
    gi = np.clip(np.rint((x - X0) / CELL).astype(int), 0, nx - 1)
    gj = np.clip(np.rint((y - Y0) / CELL).astype(int), 0, ny - 1)
    zrel = z - z0
    agl = zrel - h[gj, gi]
    stands = LF.open_stands(x, y, zrel, c, (gs_x[1] + 20.0, X1), (12.0, 50.0))
    modelled = [(pit_x[0] - 1.0, pit_x[1] + 1.0, -47.0, -25.0),      # pit building
                (gs_x[0] - 2.0, gs_x[1] + 2.0, 15.0, 48.0)]          # main grandstand
    modelled += [(g["x_min"] - 2.0, g["x_max"] + 2.0, 12.0, 50.0) for g in stands]
    others = LF.buildings(x, y, zrel, agl, c, X0, Y0, (ny, nx), modelled)
    for b in others:
        b["material"] = "concrete"
    trees = [t for t in LF.trees(x, y, agl, c, X0, Y0, (ny, nx))
             if not any(e[0] <= t["x"] <= e[1] and e[2] <= t["y"] <= e[3] for e in modelled)]

    world = {
        "format": W.FORMAT,
        "name": NAME,
        "title": "Circuit de Barcelona-Catalunya, main straight",
        "attribution": [
            "Terrain and structure heights: Institut Cartogràfic i Geològic de Catalunya (ICGC), "
            "Territorial LiDAR v3r1 2021-2023, CC BY 4.0",
            "Markings, roof panels and asphalt wear: measured from the ICGC territorial orthophoto "
            "(current colour, 25 cm), CC BY 4.0",
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
        # Rubber on the racing line: the asphalt's brightness across the track.
        "wear": wear_entry,
        # Cross-section (orthophoto, docs/worlds.md): 13.35 m of asphalt with
        # dry grass beyond the edge lines; the pit lane is 13 m wide, and its
        # exit runs on to where it joins the track.
        "surfaces": [
            {"material": "asphalt", "x_min": track[0], "x_max": track[1], "y_min": TRACK_Y[0], "y_max": TRACK_Y[1]},
            {"material": "asphalt", "x_min": pits[0], "x_max": pits[1], "y_min": -25.5, "y_max": -12.5},
            {"material": "asphalt", "x_min": EXIT_AREA[0], "x_max": PIT_WALL_END_X, "y_min": -25.5, "y_max": -12.5},
            {"material": "asphalt", "x_min": PIT_WALL_END_X, "x_max": EXIT_AREA[1], "y_min": EXIT_AREA[2],
             "y_max": TRACK_Y[0]},
            {"material": "concrete", "x_min": stand_side[0], "x_max": stand_side[1], "y_min": 11.0, "y_max": 15.5},
        ],
        "lines": markings(track, pits),
        "walls": [
            # Pit wall: about 1.1 m of concrete, catch fence above (LiDAR tops ~2.1 m).
            {"material": "concrete", "x0": pits[0], "y0": -11.2, "x1": PIT_WALL_END_X, "y1": -11.2,
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
             "top_z": round(pit_h, 2), "garages": True, "canopy_m": PIT_CANOPY_M,
             "roof_panels": pit_roof_panels(pit_x)},
        ] + [dict(sorted(b.items())) for b in others],
        "grandstands": [
            # Roof measured; the seating under it is not visible from the air,
            # so its rake (3 m at the front to 20 m at the back) is an estimate.
            {"x_min": gs_x[0], "x_max": gs_x[1], "front_y": 18.5, "back_y": 42.5,
             "front_z": 3.0, "back_z": 20.0,
             "roof_front_y": 18.5, "roof_back_y": 44.0,
             "roof_front_z": round(roof_front, 2), "roof_back_z": round(roof_back, 2),
             "roof_panels": grandstand_panels(gs_x)},
        ] + stands,           # the rest along the straight are uncovered
        # Generic models, not surveyed: the start gantry over the painted start
        # line, and two marshal posts on the grandstand-side run-off, clear of
        # the acceleration zone.
        "props": [
            {"mesh": "StartGantry", "x": 0.0, "y": 0.0},
            {"mesh": "MarshalPost", "x": -80.0, "y": 13.25},
            {"mesh": "MarshalPost", "x": 170.0, "y": 13.25},
        ],
        # One per canopy peak in the LiDAR (lidar_features.trees).
        "trees": trees,
    }
    for k in ("surfaces", "walls", "lines", "buildings", "grandstands", "props", "trees"):
        for e in world[k]:
            for f, v in list(e.items()):
                if isinstance(v, float):
                    e[f] = round(v, 2)
    W.validate(world)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "world.json").write_text(json.dumps(world, indent=2, ensure_ascii=False) + "\n")
    (out_dir / W.TERRAIN_FILE).write_bytes(units.tobytes(order="C"))
    (out_dir / W.WEAR_FILE).write_bytes(wear_bytes)
    (out_dir / "ATTRIBUTION.md").write_text(ATTRIBUTION)
    return world


ATTRIBUTION = """# montmelo_straight: data sources

This world is generated by `tools/worldgen` (`python -m worldgen build montmelo_straight`).

- **Terrain and structure heights:** Institut Cartogràfic i Geològic de Catalunya (ICGC),
  Territorial LiDAR v3r1, 2021-2023, tiles 438601 and 438602. Licensed under
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Derived: a 1 m ground grid
  and a few measured heights.
- **Markings, roof panels and asphalt wear:** measured by hand from the ICGC territorial
  orthophoto (current colour, 25 cm/pixel), [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/):
  the track's width, every painted line and area, the panel rows on the pit and grandstand
  roofs, and the asphalt's brightness across the track (`wear.u8`). The photo is not included.
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
