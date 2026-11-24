"""Band RMS of the body's motion from /imu, by speed and by manoeuvre.

Pure functions over numpy arrays; the bag reading is in ``bag.py``. The unit
tests (tools/tests/test_body_motion.py) drive these with synthetic signals.

A recording is a dict of arrays:

* ``t``   /imu header stamps (s), used to split the recording into runs
* ``lt``  /imu log times (s), the clock shared with /motor_rpm
* ``az``  vertical acceleration (m/s^2); ``ax``, ``ay`` the other two
* ``gx``, ``gy``, ``gz`` roll, pitch and yaw rate (rad/s)
* ``rt``, ``rpm`` /motor_rpm log times (s) and values

Speed comes from the motor rpm (``attach_speed``), the same way the car's
odometry gets it.

Two views:

* ``by_speed``: 4 s windows at steady speed (spread under 3 m/s), every band
  from 0.5 to 15 Hz, by speed bin. Fine resolution, but a window mixes
  straights and corners.
* ``matched``: 2 s windows split by mean |a_y| as well, bands from 3 Hz up.
  Car and sim are only comparable at the same lateral acceleration: the car
  bag's fast windows are hard cornering and the sim's are mostly straight,
  and cornering adds body motion of its own.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.signal import welch

FS = 400.0                                    # /imu rate, car and sim
BANDS = [(0.5, 1.5), (1.5, 3.0), (3.0, 6.0), (6.0, 10.0), (10.0, 15.0)]
MATCHED_BANDS = BANDS[2:]                     # a 2 s window resolves 0.5 Hz
SPEEDS = [(3.0, 6.0), (6.0, 9.0), (9.0, 12.5)]
MATCHED_SPEEDS = [(3.0, 6.0), (9.0, 12.5)]
LATERAL = [(0.0, 2.5), (2.5, 4.0), (4.0, math.inf)]
MAX_SPEED_SPREAD = 3.0                        # m/s within one window
FREEZE_MIN_SAMPLES = 6                        # the car's accel holds (~110 ms+)


def rpm_to_ms(wheel_radius_m: float, gear_ratio: float) -> float:
    """Motor rpm to road speed (m/s): one wheel turn per ``gear_ratio`` motor turns."""
    return 2.0 * math.pi * wheel_radius_m / (60.0 * gear_ratio)


def attach_speed(d: Dict[str, np.ndarray], k_rpm: float) -> Dict[str, np.ndarray]:
    """Speed at each /imu sample, interpolated from /motor_rpm on the shared log clock."""
    o = np.argsort(d["rt"])
    d["v"] = np.interp(d["lt"], d["rt"][o], d["rpm"][o] * k_rpm)
    return d


def runs(t: np.ndarray) -> List[np.ndarray]:
    """Index arrays of contiguous runs: a stamp that goes back or skips over 0.1 s
    starts a new one (sim bags restart their clock every mission)."""
    dt = np.diff(t)
    cut = np.flatnonzero((dt <= 0) | (dt > 0.1)) + 1
    return np.split(np.arange(len(t)), cut)


def unfreeze(a: np.ndarray, min_len: int = FREEZE_MIN_SAMPLES) -> np.ndarray:
    """Bridge holds of ``min_len`` or more repeated samples linearly.

    The car's accelerometer runs at 100 Hz on a 400 Hz /imu (each value
    repeats 4 times, which this leaves alone) and freezes for 110 ms or more
    now and then. A freeze is a flat step that puts broadband power into every
    band, so it is replaced by a line between its neighbours."""
    a = np.asarray(a, dtype=float).copy()
    same = np.r_[False, np.diff(a) == 0]
    i, n = 0, len(a)
    while i < n:
        if not same[i]:
            i += 1
            continue
        j = i
        while j < n and same[j]:
            j += 1
        if j - i >= min_len and i >= 1 and j < n:
            a[i:j] = np.interp(np.arange(i, j), [i - 1, j], [a[i - 1], a[j]])
        i = j
    return a


def band_rms(x: np.ndarray, bands: Sequence[Tuple[float, float]] = BANDS, fs: float = FS) -> List[float]:
    """RMS of ``x`` in each band [lo, hi) Hz, after removing a linear trend (one Hann-windowed periodogram)."""
    x = np.asarray(x, dtype=float)
    k = np.arange(len(x))
    f, p = welch(x - np.polyval(np.polyfit(k, x, 1), k), fs=fs, nperseg=len(x), window="hann")
    df = f[1] - f[0]
    return [float(np.sqrt(p[(f >= lo) & (f < hi)].sum() * df)) for lo, hi in bands]


def _signals(d, idx, fix_freeze):
    az = unfreeze(d["az"][idx]) if fix_freeze else d["az"][idx]
    return az, np.degrees(d["gy"][idx]), np.degrees(d["gx"][idx])


def _windows(n_samples: int, n: int):
    return range(0, n_samples - n + 1, n // 2)


def _rms(rows):
    return np.sqrt(np.mean(np.square(np.asarray(rows)), axis=0)) if rows else None


def by_speed(d, fix_freeze: bool, win_s: float = 4.0):
    """{speed bin: (rms[3 quantities][bands] or None, n windows)}; quantities are
    a_z (m/s^2), pitch rate and roll rate (deg/s)."""
    n = int(win_s * FS)
    out = {s: [] for s in SPEEDS}
    for idx in runs(d["t"]):
        if len(idx) < n:
            continue
        az, pitch, roll = _signals(d, idx, fix_freeze)
        v_all = np.abs(d["v"][idx])
        for k in _windows(len(idx), n):
            v = v_all[k:k + n]
            if v.max() - v.min() > MAX_SPEED_SPREAD:
                continue
            for s in SPEEDS:
                if s[0] <= v.mean() < s[1]:
                    out[s].append([band_rms(q[k:k + n]) for q in (az, pitch, roll)])
    return {s: (_rms(rows), len(rows)) for s, rows in out.items()}


def matched(d, fix_freeze: bool, win_s: float = 2.0):
    """{(speed bin, |a_y| bin): (rms[3 quantities][MATCHED_BANDS] or None, n windows)}."""
    n = int(win_s * FS)
    out = {(s, a): [] for s in MATCHED_SPEEDS for a in LATERAL}
    for idx in runs(d["t"]):
        if len(idx) < n:
            continue
        az, pitch, roll = _signals(d, idx, fix_freeze)
        v_all, ay_all = np.abs(d["v"][idx]), np.abs(d["ay"][idx])
        for k in _windows(len(idx), n):
            v = v_all[k:k + n]
            if v.max() - v.min() > MAX_SPEED_SPREAD:
                continue
            ay = ay_all[k:k + n].mean()
            for s in MATCHED_SPEEDS:
                for a in LATERAL:
                    if s[0] <= v.mean() < s[1] and a[0] <= ay < a[1]:
                        out[(s, a)].append([band_rms(q[k:k + n], MATCHED_BANDS) for q in (az, pitch, roll)])
    return {key: (_rms(rows), len(rows)) for key, rows in out.items()}
