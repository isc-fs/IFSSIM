"""perception_sanity: one scan through detect() with every stage recorded.

Needs the pipeline's cone_detection on the path (the dv_pipeline_stack container,
e.g. PYTHONPATH=pipeline/cone_detection); skipped where it is missing.
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import numpy as np

    from cone_detection.cone_detection import ConeDetectionConfig
    from cone_detection.cone_fit import _CONE_SMALL_C, _CONE_SMALL_D

    import perception_sanity as ps
except ImportError:  # host without the pipeline stack
    ps = None

SENSOR_H = 1.07
CONE = (7.0, 1.5)


def _scan(n_ground: int = 6000, seed: int = 7) -> "np.ndarray":
    """Flat ground 1.07 m below the sensor, one small cone's near face, a few no-return points."""
    rng = np.random.default_rng(seed)
    ground = np.column_stack([
        rng.uniform(0.5, 15.0, n_ground), rng.uniform(-5.0, 5.0, n_ground),
        rng.normal(-SENSOR_H, 0.005, n_ground),
    ])
    h = rng.uniform(0.06, 0.30, 60)
    r = (_CONE_SMALL_D - h) / _CONE_SMALL_C
    ux, uy = np.array(CONE) / np.hypot(*CONE)
    psi = rng.uniform(-1.0, 1.0, 60)
    cone = np.column_stack([
        CONE[0] - r * (np.cos(psi) * ux - np.sin(psi) * uy),
        CONE[1] - r * (np.cos(psi) * uy + np.sin(psi) * ux),
        h - SENSOR_H,
    ])
    return np.vstack([ground, cone, np.zeros((50, 3))]).astype(np.float32)


@unittest.skipIf(ps is None, "needs the pipeline's cone_detection (run in dv_pipeline_stack)")
class PerceptionSanityTest(unittest.TestCase):
    def test_record_describes_every_stage(self) -> None:
        cfg = ConeDetectionConfig()
        rec = ps.to_record(ps.capture(_scan(), cfg), cfg, meta={"index": 0}, max_ground_points=1000)
        s = rec["scan"]
        self.assertEqual(rec["version"], ps.FORMAT_VERSION)
        self.assertEqual(s["n_input"], 6060)  # the 50 no-return points never reach RANSAC
        self.assertAlmostEqual(s["sensor_height_m"], SENSOR_H, delta=0.02)
        self.assertEqual(s["n_accepted"], 1)

        roles = rec["points"]["role"]
        self.assertLessEqual(roles.count("ground"), 1000)  # thinned for display
        self.assertGreater(s["ground_stride"], 1)
        above = [z for z, r in zip(rec["points"]["z"], roles) if r == "above"]
        self.assertGreater(min(above), 0.0)  # z is height above the fitted ground

        (cone,) = [c for c in rec["clusters"] if c["accepted"]]
        self.assertTrue(cone["fitted"])
        self.assertLess(np.hypot(cone["a"] - CONE[0], cone["b"] - CONE[1]), 0.05)
        self.assertEqual(set(cone["templates"]), {"small", "big"})
        self.assertLess(cone["templates"]["small"]["rmse_mm"], cone["templates"]["big"]["rmse_mm"])

    def test_write_produces_plain_json(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            path = ps.write(Path(d), _scan(), ConeDetectionConfig(), meta={"index": 3})
            self.assertEqual(path.name, ps.FILENAME)
            rec = json.loads(path.read_text())  # no NaN / inf tokens
            self.assertEqual(rec["scan"]["index"], 3)
            n = len(rec["points"]["x"])
            self.assertTrue(all(len(v) == n for v in rec["points"].values()))

    def test_flat_cell_filter_drops_are_recorded(self) -> None:
        # A sheet 0.10 m above the ground: above the RANSAC threshold (not ground),
        # and flat. It lies inside 3 x 3 whole 0.3 m filter cells, ~44 points each,
        # so the filter drops every point of it.
        rng = np.random.default_rng(3)
        sheet = np.column_stack([
            rng.uniform(4.25, 5.05, 400), rng.uniform(-2.95, -2.15, 400),
            np.full(400, 0.10 - SENSOR_H),
        ]).astype(np.float32)
        cfg = ConeDetectionConfig(flat_cell_filter=True)
        rec = ps.to_record(ps.capture(np.vstack([_scan(), sheet]), cfg), cfg)
        s = rec["scan"]
        self.assertEqual(s["n_flat_dropped"], 400)
        self.assertEqual(rec["points"]["role"].count("flat"), 400)
        self.assertEqual(s["n_accepted"], 1)  # the cone is a spike, not a flat cell

    def test_detector_state_is_restored(self) -> None:
        import cone_detection.cone_detection as cd

        names = ("remove_ground", "_flat_cell_mask", "_tall_column_veto_mask",
                 "_bound_dbscan_input", "_fit_cluster", "clustering_separation_rt")
        before = [getattr(cd, n) for n in names]
        ps.capture(_scan())
        self.assertEqual(before, [getattr(cd, n) for n in names])


if __name__ == "__main__":
    unittest.main()
