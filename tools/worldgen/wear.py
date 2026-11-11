"""Asphalt wear maps (`wear.u8`, docs/worlds.md): how much lighter or darker
the asphalt is than its scanned texture, cell by cell.

A wear map is a grid of uint8 values over the track, row-major (index =
j * nx + i, i along x), 128 meaning "as scanned": the plugin multiplies the
asphalt's colour by value / 128. Rubber laid down on the racing line makes it
darker than 128; the unused edges stay lighter.

The map is built from a measured profile across the track (brightness against
y, from an aerial photo), turned into a linear-light ratio, plus gentle noise
so long straights don't look ruled. Nothing here invents patches or seams the
photo doesn't show.

Needs numpy.
"""

from __future__ import annotations

from typing import Dict, Sequence, Tuple

GAMMA = 2.2           # photo brightness to linear light
MAX_RATIO = 255 / 128


def profile_ratio(profile: Sequence[Tuple[float, float]], ys):
    """Linear-light ratio to the mean, at each y, from (y, photo brightness) samples."""
    import numpy as np
    p = sorted(profile)
    py = np.array([a for a, _ in p], float)
    pl = np.array([b for _, b in p], float)
    mean = float(np.mean(pl))
    return (np.interp(ys, py, pl) / mean) ** GAMMA


def value_noise(nx: int, ny: int, dx: float, dy: float, period: Tuple[float, float], rng):
    """Smooth noise in [-1, 1]: random values on a coarse grid, bilinear in between."""
    import numpy as np
    px, py = period
    gx = int(np.ceil(nx * dx / px)) + 2
    gy = int(np.ceil(ny * dy / py)) + 2
    g = rng.uniform(-1.0, 1.0, (gy, gx))
    fx = np.arange(nx) * dx / px
    fy = np.arange(ny) * dy / py
    i0 = np.floor(fx).astype(int); ax = fx - i0
    j0 = np.floor(fy).astype(int); ay = fy - j0
    # Smoothstep, so the grid doesn't show as creases.
    ax = ax * ax * (3 - 2 * ax); ay = ay * ay * (3 - 2 * ay)
    a = g[j0][:, i0] * (1 - ax) + g[j0][:, i0 + 1] * ax
    b = g[j0 + 1][:, i0] * (1 - ax) + g[j0 + 1][:, i0 + 1] * ax
    return a * (1 - ay)[:, None] + b * ay[:, None]


def build(profile: Sequence[Tuple[float, float]], x_range: Tuple[float, float], y_range: Tuple[float, float],
          dx: float, dy: float, noise: Sequence[Tuple[float, float, float]], seed: int) -> Tuple[Dict, bytes]:
    """The wear map over x_range x y_range (nodes dx, dy apart).

    noise: (period_x_m, period_y_m, amplitude) layers, amplitude as a
    fraction of the ratio. Returns (the world.json "wear" entry, the bytes).
    """
    import numpy as np
    nx = int(round((x_range[1] - x_range[0]) / dx)) + 1
    ny = int(round((y_range[1] - y_range[0]) / dy)) + 1
    ys = y_range[0] + np.arange(ny) * dy
    ratio = np.repeat(profile_ratio(profile, ys)[:, None], nx, axis=1)
    rng = np.random.default_rng(seed)
    for px, py, amp in noise:
        ratio *= 1.0 + amp * value_noise(nx, ny, dx, dy, (px, py), rng)
    v = np.clip(np.rint(ratio * 128.0), 1, 255).astype(np.uint8)
    entry = {"file": "wear.u8", "x0": round(x_range[0], 2), "y0": round(y_range[0], 2),
             "dx_m": dx, "dy_m": dy, "nx": nx, "ny": ny}
    return entry, v.tobytes(order="C")
