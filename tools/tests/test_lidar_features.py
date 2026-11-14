"""Tests for tools/worldgen/lidar_features.py (docs/worlds.md): buildings and
trees found in synthetic LiDAR points.

    python -m unittest discover -s tools/tests -v

Skipped where numpy isn't installed.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

try:
    import numpy as np
except ImportError:  # the CSV-validator CI job has no numpy
    np = None

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))

SHAPE = (60, 80)   # ny, nx: a 80 x 60 m patch, 1 m cells, origin at (0, 0)


def cloud(parts):
    """Stack (x, y, height, class) arrays into the point columns."""
    x, y, h, c = (np.concatenate(a) for a in zip(*parts))
    return x, y, h, c


def ground():
    gx, gy = np.meshgrid(np.arange(0, 80, 0.5), np.arange(0, 60, 0.5))
    n = gx.size
    return gx.ravel(), gy.ravel(), np.zeros(n), np.full(n, 2)


@unittest.skipIf(np is None, "numpy not installed")
class LidarFeatures(unittest.TestCase):
    def setUp(self):
        from worldgen import lidar_features
        self.LF = lidar_features

    def test_building_box(self):
        # A 20 x 10 m block, roof at 9 m, centred at (30, 25).
        bx, by = np.meshgrid(np.arange(20.25, 40, 0.5), np.arange(20.25, 30, 0.5))
        roof = (bx.ravel(), by.ravel(), np.full(bx.size, 9.0), np.full(bx.size, 6))
        x, y, h, c = cloud([ground(), roof])
        out = self.LF.buildings(x, y, h, h, c, 0.0, 0.0, SHAPE, exclude=[])
        self.assertEqual(len(out), 1)
        b = out[0]
        self.assertNotIn("base_z", b)
        self.assertAlmostEqual(b["top_z"], 9.0, delta=0.1)
        cx, cy = (b["x_min"] + b["x_max"]) / 2, (b["y_min"] + b["y_max"]) / 2
        self.assertAlmostEqual(cx, 30.0, delta=1.0)
        self.assertAlmostEqual(cy, 25.0, delta=1.0)
        self.assertAlmostEqual(b["x_max"] - b["x_min"], 20.0, delta=1.5)
        self.assertAlmostEqual(b["y_max"] - b["y_min"], 10.0, delta=1.5)

    def test_excluded_building_dropped(self):
        bx, by = np.meshgrid(np.arange(20.25, 40, 0.5), np.arange(20.25, 30, 0.5))
        roof = (bx.ravel(), by.ravel(), np.full(bx.size, 9.0), np.full(bx.size, 6))
        x, y, h, c = cloud([ground(), roof])
        out = self.LF.buildings(x, y, h, h, c, 0.0, 0.0, SHAPE, exclude=[(15, 45, 15, 35)])
        self.assertEqual(out, [])

    def test_one_tree(self):
        # A dome crown 10 m high, 4 m radius, at (40, 30).
        tx, ty = np.meshgrid(np.arange(36, 44.01, 0.5), np.arange(26, 34.01, 0.5))
        d = np.hypot(tx - 40, ty - 30)
        keep = d <= 4.0
        crown_h = 10.0 - 4.0 * (d[keep] / 4.0) ** 2
        crown = (tx[keep], ty[keep], crown_h, np.full(keep.sum(), 5))
        x, y, h, c = cloud([ground(), crown])
        out = self.LF.trees(x, y, h, c, 0.0, 0.0, SHAPE)
        self.assertEqual(len(out), 1)
        t = out[0]
        self.assertAlmostEqual(t["x"], 40.0, delta=1.0)
        self.assertAlmostEqual(t["y"], 30.0, delta=1.0)
        self.assertGreater(t["h"], 8.0)
        self.assertLessEqual(t["r"], 0.5 * t["h"] + 1.5)
        self.assertIn(t["kind"], ("pine", "broadleaf"))


if __name__ == "__main__":
    unittest.main()
