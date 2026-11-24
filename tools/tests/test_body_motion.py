"""Tests for the body-motion comparison (tools/body_motion) on synthetic /imu.

    python3 -m unittest discover -s tools/tests -v

Skipped where numpy or scipy is missing (the CI job that runs this folder is
plain Python; the tool itself needs both).
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import numpy as np
    import scipy  # noqa: F401
    from body_motion import analysis as A
except ImportError:  # pragma: no cover
    A = None


def recording(seconds, speed, ay=0.0, roll_hz=None, roll_deg_s=0.0, t0=0.0):
    """A steady recording: speed (m/s) from /motor_rpm, constant a_y, optional roll sine."""
    n = int(seconds * A.FS)
    t = t0 + np.arange(n) / A.FS
    gx = np.zeros(n)
    if roll_hz:
        gx = np.radians(roll_deg_s * math.sqrt(2) * np.sin(2 * math.pi * roll_hz * t))
    k = A.rpm_to_ms(0.202, 2.909)
    rt = t[::40]
    return dict(t=t, lt=t.copy(), ax=np.zeros(n), ay=np.full(n, ay), az=np.zeros(n),
                gx=gx, gy=np.zeros(n), gz=np.zeros(n), rt=rt, rpm=np.full(len(rt), speed / k)), k


def join(*parts):
    return {key: np.concatenate([p[key] for p in parts]) for key in parts[0]}


@unittest.skipIf(A is None, "numpy/scipy not installed")
class BodyMotionTests(unittest.TestCase):
    def test_rpm_to_ms_matches_the_odometry_constant(self):
        self.assertAlmostEqual(A.rpm_to_ms(0.202, 2.909), 0.00727, places=5)

    def test_band_rms_of_a_sine_lands_in_its_band(self):
        t = np.arange(int(4 * A.FS)) / A.FS
        x = math.sqrt(2) * np.sin(2 * math.pi * 4.0 * t)     # rms 1
        rms = A.band_rms(x)
        self.assertAlmostEqual(rms[2], 1.0, delta=0.03)       # 3-6 Hz
        for b in (0, 1, 3, 4):
            self.assertLess(rms[b], 0.05)

    def test_band_rms_ignores_a_linear_trend(self):
        x = np.linspace(0, 5, int(4 * A.FS))
        self.assertLess(max(A.band_rms(x)), 1e-6)

    def test_unfreeze_bridges_long_holds_only(self):
        a = np.arange(40, dtype=float)
        a[10:18] = a[9]          # 8 repeated: a freeze
        a[25:28] = a[24]         # 3 repeated: the 100 Hz accel on a 400 Hz /imu
        out = A.unfreeze(a)
        np.testing.assert_allclose(out[9:19], np.linspace(9, 18, 10))
        np.testing.assert_array_equal(out[25:28], a[25:28])

    def test_runs_split_on_clock_restart_and_gap(self):
        t = np.r_[np.arange(5) * 0.01, np.arange(5) * 0.01, 10 + np.arange(5) * 0.01]
        self.assertEqual([len(r) for r in A.runs(t)], [5, 5, 5])

    def test_by_speed_bins_steady_windows(self):
        d, k = recording(20, 4.5, roll_hz=4.0, roll_deg_s=1.5)
        res = A.by_speed(A.attach_speed(d, k), fix_freeze=False)
        rms, n = res[(3.0, 6.0)]
        self.assertGreater(n, 5)
        self.assertAlmostEqual(rms[2][2], 1.5, delta=0.05)    # roll, 3-6 Hz
        self.assertEqual(res[(9.0, 12.5)][1], 0)

    def test_by_speed_skips_windows_that_change_speed(self):
        d, k = recording(12, 4.0)
        d["rpm"] = np.linspace(0, 12.0, len(d["rpm"])) / k   # 0 -> 12 m/s
        res = A.by_speed(A.attach_speed(d, k), fix_freeze=False)
        self.assertEqual(sum(n for _, n in res.values()), 0)

    def test_matched_splits_by_lateral_acceleration(self):
        straight, k = recording(10, 10.0, ay=1.0)
        corner, _ = recording(10, 10.0, ay=3.0, roll_hz=8.0, roll_deg_s=2.0, t0=100.0)
        res = A.matched(A.attach_speed(join(straight, corner), k), fix_freeze=False)
        rms_s, n_s = res[((9.0, 12.5), (0.0, 2.5))]
        rms_c, n_c = res[((9.0, 12.5), (2.5, 4.0))]
        self.assertGreater(n_s, 3)
        self.assertGreater(n_c, 3)
        self.assertLess(rms_s[2][1], 0.05)                    # straight: no roll
        self.assertAlmostEqual(rms_c[2][1], 2.0, delta=0.1)   # corner: roll at 6-10 Hz


if __name__ == "__main__":
    unittest.main()
