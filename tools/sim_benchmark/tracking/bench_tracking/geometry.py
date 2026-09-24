"""2-D trajectory helpers: interpolation, alignment, track-distance projection."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def path_length(x: np.ndarray, y: np.ndarray) -> float:
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    return float(np.hypot(np.diff(x), np.diff(y)).sum()) if x.size > 1 else 0.0


def interp(t_src: np.ndarray, v_src: np.ndarray, t_q: np.ndarray) -> np.ndarray:
    """Linear interpolation, NaN outside the source range."""
    t_src, v_src = np.asarray(t_src, float), np.asarray(v_src, float)
    ok = np.isfinite(t_src) & np.isfinite(v_src)
    if ok.sum() < 2:
        return np.full(np.shape(t_q), np.nan)
    out = np.interp(t_q, t_src[ok], v_src[ok])
    out[(t_q < t_src[ok][0]) | (t_q > t_src[ok][-1])] = np.nan
    return out


def interp_angle(t_src: np.ndarray, a_src: np.ndarray, t_q: np.ndarray) -> np.ndarray:
    return np.arctan2(
        interp(t_src, np.sin(a_src), t_q), interp(t_src, np.cos(a_src), t_q)
    )


def rigid(
    x: np.ndarray, y: np.ndarray, theta: float, tx: float, ty: float
) -> tuple[np.ndarray, np.ndarray]:
    c, s = math.cos(theta), math.sin(theta)
    return c * x - s * y + tx, s * x + c * y + ty


def align_start_pose(
    x: np.ndarray,
    y: np.ndarray,
    yaw0: float | None = None,
    heading_window_m: float = 3.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Move the first pose to the origin, heading +x.

    If ``yaw0`` isn't known it is estimated from the first ``heading_window_m``
    of travel. That's the only choice when a trajectory has no orientation.
    """
    ok = np.isfinite(x) & np.isfinite(y)
    if ok.sum() < 2:
        return x, y
    x0, y0 = x[ok][0], y[ok][0]
    if yaw0 is None:
        d = np.hypot(x[ok] - x0, y[ok] - y0)
        j = int(np.argmax(d > heading_window_m)) if (d > heading_window_m).any() else -1
        yaw0 = math.atan2(y[ok][j] - y0, x[ok][j] - x0)
    return rigid(x - x0, y - y0, -yaw0, 0.0, 0.0)


def umeyama_2d(src: np.ndarray, dst: np.ndarray) -> tuple[float, float, float]:
    """Least-squares rigid transform (theta, tx, ty) with dst ≈ R·src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mu_d - R @ mu_s
    return math.atan2(R[1, 0], R[0, 0]), float(t[0]), float(t[1])


def fit_cones(
    src: np.ndarray, dst: np.ndarray, iters: int = 20, gate_m: float = 3.0
) -> tuple[float, float, float, float]:
    """ICP with nearest-neighbour association. Returns (theta, tx, ty, rms_m)."""
    theta = tx = ty = 0.0
    rms = math.inf
    for _ in range(iters):
        xs, ys = rigid(src[:, 0], src[:, 1], theta, tx, ty)
        cur = np.c_[xs, ys]
        d = np.linalg.norm(cur[:, None, :] - dst[None, :, :], axis=2)
        j = d.argmin(1)
        keep = d[np.arange(len(cur)), j] < gate_m
        if keep.sum() < 3:
            break
        dth, dtx, dty = umeyama_2d(cur[keep], dst[j[keep]])
        xs2, ys2 = rigid(cur[:, 0], cur[:, 1], dth, dtx, dty)
        c, s = math.cos(dth), math.sin(dth)
        theta, tx, ty = theta + dth, c * tx - s * ty + dtx, s * tx + c * ty + dty
        rms = float(
            np.sqrt(np.mean(np.sum((np.c_[xs2, ys2][keep] - dst[j[keep]]) ** 2, 1)))
        )
        if abs(dth) < 1e-5 and abs(dtx) + abs(dty) < 1e-4:
            break
    return theta, tx, ty, rms


# ---------------------------------------------------------------- tracks
@dataclass
class Track:
    name: str
    blue: np.ndarray  # (N,2)
    yellow: np.ndarray
    orange: np.ndarray
    center: np.ndarray  # closed centerline (M,2), ~1 m spacing
    s: np.ndarray  # arc length at each centerline point
    length_m: float

    def project(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(s along centerline, signed lateral offset; +left) for each point."""
        P = np.c_[x, y]
        a, b = self.center[:-1], self.center[1:]
        ab = b - a
        L2 = (ab**2).sum(1)
        s_out = np.empty(len(P))
        e_out = np.empty(len(P))
        for i, p in enumerate(P):
            t = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
            q = a + ab * t[:, None]
            d = np.hypot(*(p - q).T)
            k = int(d.argmin())
            s_out[i] = self.s[k] + t[k] * math.sqrt(L2[k])
            cross = ab[k, 0] * (p[1] - a[k, 1]) - ab[k, 1] * (p[0] - a[k, 0])
            e_out[i] = math.copysign(d[k], cross)
        return s_out, e_out

    def curvature(self) -> np.ndarray:
        c = self.center
        d1 = np.gradient(c, self.s, axis=0)
        d2 = np.gradient(d1, self.s, axis=0)
        return (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(
            np.hypot(*d1.T) ** 3, 1e-9
        )


def load_track(path: Path, rotate_ccw_90: bool = True, spacing_m: float = 1.0) -> Track:
    """Track CSV (color,x,y,...) -> cones + resampled closed centerline.

    Same convention as ``common.load_csv_centerline`` (rotate CCW 90°), with
    the centerline built from blue/yellow midpoints ordered by walking
    nearest-neighbour from the first blue cone.
    """
    cones: dict[str, list[tuple[float, float]]] = {
        "blue": [],
        "yellow": [],
        "orange": [],
    }
    with path.open(newline="") as fh:
        for row in csv.reader(fh):
            if len(row) < 3:
                continue
            color = row[0].strip().lower()
            try:
                x, y = float(row[1]), float(row[2])
            except ValueError:
                continue
            if rotate_ccw_90:
                x, y = -y, x
            key = "orange" if "orange" in color else color
            if key in cones:
                cones[key].append((x, y))
    blue, yellow = np.array(cones["blue"]), np.array(cones["yellow"])
    orange = np.array(cones["orange"]) if cones["orange"] else np.zeros((0, 2))
    mids = []
    for b in blue:
        j = int(np.argmin(np.hypot(*(yellow - b).T)))
        mids.append((b + yellow[j]) / 2)
    mids = np.array(mids)
    # order by nearest-neighbour walk (CSV order is usually already sequential)
    order = [0]
    left = set(range(1, len(mids)))
    while left:
        last = mids[order[-1]]
        j = min(left, key=lambda k: float(np.hypot(*(mids[k] - last))))
        order.append(j)
        left.remove(j)
    mids = mids[order]
    closed = np.vstack([mids, mids[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.r_[0, np.cumsum(seg)]
    n = max(int(s[-1] / spacing_m), 8)
    sq = np.linspace(0, s[-1], n + 1)
    center = np.c_[np.interp(sq, s, closed[:, 0]), np.interp(sq, s, closed[:, 1])]
    # light smoothing (periodic) so curvature is usable
    k = 3
    ext = np.vstack([center[-k - 1 : -1], center, center[1 : k + 1]])
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    center = np.c_[
        np.convolve(ext[:, 0], ker, "valid"), np.convolve(ext[:, 1], ker, "valid")
    ]
    seg = np.hypot(*np.diff(center, axis=0).T)
    s = np.r_[0, np.cumsum(seg)]
    return Track(path.stem, blue, yellow, orange, center, s, float(s[-1]))
