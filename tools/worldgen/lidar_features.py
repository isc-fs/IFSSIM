"""Trees and buildings from classified airborne LiDAR, on a 1 m grid.

numpy only (no scipy): the grids here are a few hundred thousand cells, so
plain array passes are quick enough.

Every function takes heights relative to the world's z datum and positions in
the world frame (docs/worlds.md); `agl` is height above the measured ground.
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

VEGETATION = (4, 5)     # ICGC (ASPRS): medium and high vegetation
BUILDING = (6,)
GROUND = (2, 8)
DECK_M = 1.2            # a footbridge's depth, deck to soffit
DECK_CLEARANCE_M = 4.0  # lower than this, a narrow roof is a shed or a canopy


# --- grid helpers -------------------------------------------------------------

def rasterise(i, j, v, shape, how: str, fill: float) -> np.ndarray:
    """Per-cell max / min / p50 of v over points at grid indices (j, i)."""
    g = np.full(shape, fill, dtype=float)
    if how == "max":
        np.maximum.at(g, (j, i), v)
    elif how == "min":
        np.minimum.at(g, (j, i), v)
    else:
        raise ValueError(how)
    return g


def label(mask: np.ndarray) -> np.ndarray:
    """4-connected components of a boolean grid: each cell gets its component's
    smallest flat index, -1 outside. Min-label propagation with pointer
    jumping, so long thin shapes converge in a few dozen passes."""
    ny, nx = mask.shape
    big = mask.size + 1
    lab = np.where(mask, np.arange(mask.size).reshape(mask.shape), big)
    while True:
        p = np.pad(lab, 1, constant_values=big)
        m = np.minimum.reduce([p[1:-1, 1:-1], p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])
        m = np.where(mask, m, big)
        flat = m.ravel()
        inside = flat < big
        flat[inside] = flat[flat[inside]]          # jump to the root's label
        m = flat.reshape(mask.shape)
        if np.array_equal(m, lab):
            return np.where(mask, lab, -1)
        lab = m


def sliding_max(a: np.ndarray, r: int) -> np.ndarray:
    """Max over a (2r+1)^2 window (separable)."""
    out = a
    for axis in (0, 1):
        p = np.pad(out, [(r, r) if k == axis else (0, 0) for k in (0, 1)], constant_values=-np.inf)
        n = out.shape[axis]
        out = np.max(np.stack([np.take(p, range(k, k + n), axis=axis) for k in range(2 * r + 1)]), axis=0)
    return out


def box_blur(a: np.ndarray) -> np.ndarray:
    p = np.pad(a, 1, mode="edge")
    return sum(p[dj:dj + a.shape[0], di:di + a.shape[1]] for dj in range(3) for di in range(3)) / 9.0


# --- buildings ----------------------------------------------------------------

def rect_cover(mask: np.ndarray, min_cells: int) -> List[Tuple[int, int, int, int]]:
    """Cover a mask with axis-aligned rectangles (i0, i1, j0, j1, inclusive),
    largest first; leftovers under min_cells cells are dropped."""
    m = mask.copy()
    out = []
    while m.sum() >= min_cells:
        # Largest rectangle of ones: histogram method, row by row.
        best, hist = (0, None), np.zeros(m.shape[1], dtype=int)
        for j in range(m.shape[0]):
            hist = np.where(m[j], hist + 1, 0)
            stack: List[int] = []
            for i in range(len(hist) + 1):
                hv = hist[i] if i < len(hist) else 0
                while stack and hist[stack[-1]] >= hv:
                    top = stack.pop()
                    left = stack[-1] + 1 if stack else 0
                    area = hist[top] * (i - left)
                    if area > best[0]:
                        best = (area, (left, i - 1, j - hist[top] + 1, j))
                stack.append(i)
        if best[0] < min_cells:
            break
        i0, i1, j0, j1 = best[1]
        out.append(best[1])
        m[j0:j1 + 1, i0:i1 + 1] = False
    return out


def min_area_frame(p: np.ndarray) -> np.ndarray:
    """Axes (rows) of the smallest-area rectangle around centred points, the
    long axis first, tried in 1 degree steps."""
    best = None
    for deg in range(90):
        a = math.radians(deg)
        v = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        q = p @ v.T
        ext = q.max(0) - q.min(0) + 1.0
        if best is None or ext[0] * ext[1] < best[0]:
            best = (ext[0] * ext[1], v, ext)
    _, v, ext = best
    if ext[1] > ext[0]:
        v = np.array([v[1], -v[0]])
    return v


def buildings(x, y, z_rel, agl, cls, x0: float, y0: float, shape: Tuple[int, int],
              exclude: Sequence[Tuple[float, float, float, float]],
              min_cells: int = 20, fill_single: float = 0.6) -> List[Dict]:
    """Rotated boxes for every building blob the LiDAR shows outside `exclude`
    (x_min, x_max, y_min, y_max boxes: structures modelled some other way).

    A blob that fills most of its smallest rectangle is one box; an L, a T or
    a footbridge is cut into several boxes in that rectangle's frame. Roofs
    sit at the 90th percentile of the points under them. A narrow piece with
    ground seen under most of it is a raised deck (a footbridge), not a
    block: it gets a base_z."""
    ny, nx = shape
    gm = np.isin(cls, GROUND)
    ground_seen = np.zeros(shape, bool)
    ground_seen[np.clip(np.rint(y[gm] - y0).astype(int), 0, ny - 1),
                np.clip(np.rint(x[gm] - x0).astype(int), 0, nx - 1)] = True
    b = np.isin(cls, BUILDING) & (agl > 2.0)
    for ex in exclude:
        b &= ~((x >= ex[0]) & (x <= ex[1]) & (y >= ex[2]) & (y <= ex[3]))
    i = np.clip(np.rint(x[b] - x0).astype(int), 0, nx - 1)
    j = np.clip(np.rint(y[b] - y0).astype(int), 0, ny - 1)
    occ = np.zeros(shape, bool)
    occ[j, i] = True
    p = np.pad(occ, 1)
    occ |= (p[:-2, 1:-1] & p[2:, 1:-1]) | (p[1:-1, :-2] & p[1:-1, 2:])   # close 1-cell gaps
    lab = label(occ)
    cell_of_point = j * nx + i
    zb, ab = z_rel[b], agl[b]
    out = []
    ids, counts = np.unique(lab[lab >= 0], return_counts=True)
    for k, n in zip(ids, counts):
        if n < min_cells:
            continue
        jj, ii = np.nonzero(lab == k)
        pts = np.c_[ii + x0, jj + y0]
        mu = pts.mean(0)
        v = min_area_frame(pts - mu)
        q = (pts - mu) @ v.T
        lo, hi = q.min(0) - 0.5, q.max(0) + 0.5
        fill = n / float(np.prod(hi - lo))
        if fill >= fill_single:
            rects = [(lo[0], hi[0], lo[1], hi[1], np.ones(len(q), bool))]
        else:
            # Rasterise in the principal frame and cut into rectangles.
            gi = np.floor(q[:, 0] - lo[0]).astype(int)
            gj = np.floor(q[:, 1] - lo[1]).astype(int)
            g = np.zeros((gj.max() + 1, gi.max() + 1), bool)
            g[gj, gi] = True
            rects = []
            for ri0, ri1, rj0, rj1 in rect_cover(g, max(15, (3 * min_cells) // 4)):
                if min(ri1 - ri0, rj1 - rj0) < 1:     # under 2 m across: a sliver
                    continue
                sel = (gi >= ri0) & (gi <= ri1) & (gj >= rj0) & (gj <= rj1)
                rects.append((lo[0] + ri0, lo[0] + ri1 + 1, lo[1] + rj0, lo[1] + rj1 + 1, sel))
        flat_cells = jj * nx + ii
        for a0, a1, c0, c1, sel in rects:
            under = np.isin(cell_of_point, flat_cells[sel])
            if under.sum() < 10:
                continue
            cx, cy = mu + ((a0 + a1) / 2) * v[0] + ((c0 + c1) / 2) * v[1]
            hl, hw = (a1 - a0) / 2, (c1 - c0) / 2
            yaw = math.degrees(math.atan2(v[0, 1], v[0, 0]))
            top = float(np.percentile(zb[under], 90))
            e = {"x_min": cx - hl, "x_max": cx + hl, "y_min": cy - hw, "y_max": cy + hw,
                 "yaw_deg": yaw, "top_z": top}
            cells = flat_cells[sel]
            clear = float(np.percentile(ab[under], 50))   # height of the deck above the ground
            if min(hl, hw) <= 3.0 and clear >= DECK_CLEARANCE_M and ground_seen.ravel()[cells].mean() > 0.5:
                e["base_z"] = top - DECK_M
            out.append(e)
    return out


def open_stands(x, y, z_rel, cls, x_rng: Tuple[float, float], y_rng: Tuple[float, float],
                min_len: float = 40.0) -> List[Dict]:
    """Uncovered grandstands along x: each blob of building points in the
    band, as rows rising from front_y to back_y. Heights are the measured
    seating surface (the median per 1 m of depth), fitted with a line."""
    b = (np.isin(cls, BUILDING) & (x >= x_rng[0]) & (x <= x_rng[1])
         & (np.abs(y) >= y_rng[0]) & (np.abs(y) <= y_rng[1]))
    xs = np.unique(np.floor(x[b]).astype(int))
    if not len(xs):
        return []
    # Runs of occupied metres along x, gaps of up to 3 m bridged.
    runs, start = [], xs[0]
    for a, c in zip(xs[:-1], xs[1:]):
        if c - a > 3:
            runs.append((start, a + 1)); start = c
    runs.append((start, xs[-1] + 1))
    out = []
    for r0, r1 in runs:
        if r1 - r0 < min_len:
            continue
        m = b & (x >= r0) & (x < r1)
        side = 1.0 if np.median(y[m]) > 0 else -1.0
        d = np.abs(y[m])
        front, back = float(np.percentile(d, 2)), float(np.percentile(d, 98))
        bins = np.arange(math.floor(front), math.ceil(back))
        zs = [np.median(z_rel[m][(d >= k) & (d < k + 1)]) for k in bins if ((d >= k) & (d < k + 1)).sum() > 20]
        ks = [k + 0.5 for k in bins if ((d >= k) & (d < k + 1)).sum() > 20]
        if len(ks) < 3:
            continue
        slope, icpt = np.polyfit(ks, zs, 1)
        if slope <= 0:
            continue
        out.append({"x_min": float(r0), "x_max": float(r1), "front_y": side * front, "back_y": side * back,
                    "front_z": float(icpt + slope * front), "back_z": float(icpt + slope * back)})
    return out


# --- trees --------------------------------------------------------------------

def trees(x, y, agl, cls, x0: float, y0: float, shape: Tuple[int, int],
          min_h: float = 3.0, max_r: float = 9.0) -> List[Dict]:
    """One tree per canopy peak: position, height, crown radius and kind.

    Peaks are local maxima of the smoothed canopy height over a window that
    grows with the tree. The crown radius is the median distance, over eight
    directions, to where the canopy falls below half the tree's height. The
    kind follows the crown: a crown much wider than it is deep, high on a bare
    trunk, is a pine (Aleppo and stone pines line the circuit); the rest are
    broadleaf (holm oaks, planes)."""
    ny, nx = shape
    v = np.isin(cls, VEGETATION) & (agl > 0.5) & (agl < 40.0)
    i = np.clip(np.rint(x[v] - x0).astype(int), 0, nx - 1)
    j = np.clip(np.rint(y[v] - y0).astype(int), 0, ny - 1)
    chm = rasterise(i, j, agl[v], shape, "max", 0.0)
    low = rasterise(i, j, agl[v], shape, "min", 99.0)
    s = box_blur(chm)
    peak = np.zeros(shape, bool)
    for r, h_lo, h_hi in ((2, min_h, 7.0), (3, 7.0, 12.0), (4, 12.0, 99.0)):
        band = (s >= h_lo) & (s < h_hi)
        peak |= band & (s >= sliding_max(s, r) - 1e-6)
    jj, ii = np.nonzero(peak)
    out = []
    dirs = [(math.cos(a), math.sin(a)) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
    for pj, pi in zip(jj, ii):
        h = float(chm[pj, pi])
        if h < min_h:
            continue
        reach = []
        for dx, dy in dirs:
            d = 1.0
            while d <= max_r:
                qi, qj = int(round(pi + dx * d)), int(round(pj + dy * d))
                if not (0 <= qi < nx and 0 <= qj < ny) or chm[qj, qi] < 0.5 * h:
                    break
                d += 1.0
            reach.append(d - 0.5)
        # A low, wide blob is a hedge or a clump of trees, not one crown;
        # capped, it reads as a tree rather than a pancake.
        r = float(np.clip(np.median(reach), 1.0, min(max_r, 0.5 * h + 1.5)))
        # The lowest return near the trunk: the crown's base, where the
        # scanner saw through to it.
        win = low[max(0, pj - 1):pj + 2, max(0, pi - 1):pi + 2]
        base = float(win[win < 99].min()) if (win < 99).any() else 0.0
        pine = h >= 6.0 and base >= 0.45 * h and 2 * r >= 0.9 * (h - base)
        out.append({"x": float(pi + x0), "y": float(pj + y0), "h": h, "r": r,
                    "kind": "pine" if pine else "broadleaf"})
    return out
