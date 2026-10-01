from __future__ import annotations

import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception_metrics import (  # noqa: E402
    FS_CONE_ORANGE_BIG,
    FS_CONE_UNKNOWN,
    Cone2D,
    StopLatchReplay,
    aggregate_metrics,
    aggregate_scan_stats,
    classify_detections,
    evaluate_frame,
    reference_deltas,
    scan_stats,
)

BIG = FS_CONE_ORANGE_BIG
SMALL_BLUE = 0


def _obs(x: float, y: float, h: float) -> types.SimpleNamespace:
    return types.SimpleNamespace(x=x, y=y, height_m=h)


def _big(x: float, y: float) -> Cone2D:
    return Cone2D(x=x, y=y, color=BIG)


class ClassifyDetectionsTest(unittest.TestCase):
    def test_threshold_is_strictly_greater_like_the_node(self) -> None:
        out = classify_detections([_obs(1, 0, 0.45), _obs(2, 0, 0.4501), _obs(3, 0, 0.30)], 0.45)
        self.assertEqual([c.color for c in out], [FS_CONE_UNKNOWN, BIG, FS_CONE_UNKNOWN])
        self.assertEqual((out[1].x, out[1].y), (2.0, 0.0))


class BigOrangeScoringTest(unittest.TestCase):
    def test_counts_every_big_orange_outcome(self) -> None:
        gt = [
            Cone2D(x=5.0, y=1.0, color=SMALL_BLUE),  # small cone
            _big(10.0, 1.0),  # big orange, detected as big
            _big(10.0, -1.0),  # big orange, detected as small
            _big(15.0, 0.0),  # big orange, not detected at all
        ]
        pred = [
            Cone2D(x=5.1, y=1.0, color=BIG),  # big on a small cone -> false big (size error)
            Cone2D(x=10.1, y=1.0, color=BIG),  # TP
            Cone2D(x=10.1, y=-1.0, color=FS_CONE_UNKNOWN),  # matched, classed small -> missed big
            Cone2D(x=8.0, y=4.0, color=BIG),  # nothing there -> false big
        ]
        fm = evaluate_frame(t_s=0.0, latency_ms=0.0, n_points=0, pred=pred, gt=gt, gate_m=0.5)
        self.assertEqual(fm.n_pred_big, 3)
        self.assertEqual(fm.n_big_tp, 1)
        self.assertEqual(fm.n_false_big, 2)
        self.assertEqual(fm.n_missed_big, 2)
        # Position scoring is unchanged: three matches, one FP, one FN.
        self.assertEqual((fm.n_tp, fm.n_fp, fm.n_fn), (3, 1, 1))

    def test_aggregate_flags_frames_that_can_trip_the_latch(self) -> None:
        def frame(n_false: int):
            pred = [_big(20.0 + i, 5.0) for i in range(n_false)]
            return evaluate_frame(t_s=0.0, latency_ms=0.0, n_points=0, pred=pred, gt=[], gate_m=0.5)

        agg = aggregate_metrics([frame(0), frame(1), frame(2), frame(3)])
        self.assertEqual(agg["total_false_big"], 6)
        self.assertEqual(agg["frames_with_false_big"], 3)
        self.assertEqual(agg["frames_with_2plus_false_big"], 2)
        self.assertEqual(agg["max_false_big_per_frame"], 3)
        self.assertAlmostEqual(agg["false_big_per_frame"], 1.5)
        self.assertEqual(agg["big_orange_precision"], 0.0)

    def test_no_big_orange_in_view_has_no_precision_rather_than_zero(self) -> None:
        fm = evaluate_frame(
            t_s=0.0, latency_ms=0.0, n_points=0,
            pred=[Cone2D(x=1, y=1)], gt=[Cone2D(x=1, y=1, color=SMALL_BLUE)], gate_m=0.5,
        )
        agg = aggregate_metrics([fm])
        self.assertIsNone(agg["big_orange_precision"])
        self.assertIsNone(agg["big_orange_recall"])


class StopLatchReplayTest(unittest.TestCase):
    GATE = [(50.0, 2.0), (50.0, -2.0)]  # GT big-orange finish gate, world frame

    def _drive(self, replay: StopLatchReplay, *, to_x: float, cones=(), final_lap=True):
        """Drive along +x in 1 m steps; return the first latch event, if any."""
        event = None
        x = replay._last_xy[0] if replay._last_xy else 0.0
        while x <= to_x:
            e = replay.update(t_s=x, x=x, y=0.0, yaw=0.0,
                              big_cones_body=list(cones), final_lap=final_lap)
            event = event or e
            x += 1.0
        return event

    def test_no_latch_before_min_travel(self) -> None:
        r = StopLatchReplay(min_travel_m=30.0, gt_big_world=self.GATE)
        self.assertIsNone(self._drive(r, to_x=29.0, cones=[_big(5, 2), _big(5, -2)]))
        self.assertFalse(r.summary()["latched"])

    def test_one_cone_never_latches(self) -> None:
        r = StopLatchReplay(min_travel_m=30.0, gt_big_world=self.GATE)
        self.assertIsNone(self._drive(r, to_x=60.0, cones=[_big(5, 2)]))

    def test_final_lap_gate_blocks(self) -> None:
        r = StopLatchReplay(min_travel_m=30.0, gt_big_world=self.GATE)
        self.assertIsNone(self._drive(r, to_x=60.0, cones=[_big(5, 2), _big(5, -2)], final_lap=False))

    def test_latch_at_the_real_gate_is_not_premature(self) -> None:
        r = StopLatchReplay(min_travel_m=30.0, gt_big_world=self.GATE)
        self._drive(r, to_x=39.0)  # no cones yet
        # At x=40 the gate is 10 m ahead: body (10, ±2) -> world (50, ±2).
        e = r.update(t_s=40.0, x=40.0, y=0.0, yaw=0.0,
                     big_cones_body=[_big(10, 2), _big(10, -2)])
        self.assertIsNotNone(e)
        self.assertAlmostEqual(e.anchor_x, 50.0)
        self.assertAlmostEqual(e.anchor_y, 0.0)
        self.assertAlmostEqual(e.nearest_gt_big_m, 2.0)
        self.assertFalse(e.premature)

    def test_false_pair_mid_lap_is_premature_and_latches_for_good(self) -> None:
        r = StopLatchReplay(min_travel_m=30.0, gt_big_world=self.GATE, finish_tol_m=3.0)
        self._drive(r, to_x=31.0)
        e = r.update(t_s=32.0, x=32.0, y=0.0, yaw=0.0,
                     big_cones_body=[_big(3, 1.5), _big(3, -1.5)])
        self.assertTrue(e.premature)
        self.assertAlmostEqual(e.anchor_x, 35.0)
        # The real gate later does not replace the latch.
        self.assertIsNone(r.update(t_s=40.0, x=40.0, y=0.0, yaw=0.0,
                                   big_cones_body=[_big(10, 2), _big(10, -2)]))
        s = r.summary()
        self.assertTrue(s["latched"])
        self.assertTrue(s["premature"])
        self.assertAlmostEqual(s["event"]["anchor_x"], 35.0)

    def test_anchor_uses_heading(self) -> None:
        r = StopLatchReplay(min_travel_m=0.0, gt_big_world=[(0.0, 10.0)])
        e = r.update(t_s=0.0, x=0.0, y=0.0, yaw=math.pi / 2,
                     big_cones_body=[_big(10, 1), _big(10, -1)])
        self.assertAlmostEqual(e.anchor_x, 0.0, places=9)
        self.assertAlmostEqual(e.anchor_y, 10.0, places=9)
        self.assertFalse(e.premature)

    def test_without_ground_truth_premature_is_unknown(self) -> None:
        r = StopLatchReplay(min_travel_m=0.0, gt_big_world=[])
        e = r.update(t_s=0.0, x=0.0, y=0.0, yaw=0.0, big_cones_body=[_big(1, 1), _big(1, -1)])
        self.assertIsNone(e.premature)
        self.assertIsNone(e.nearest_gt_big_m)


class ScanStatsTest(unittest.TestCase):
    def test_from_detection_result(self) -> None:
        res = types.SimpleNamespace(
            rotated_xyz=[0] * 1000, outlier_xyz=[0] * 50, debug_counters={"n_clusters": 12}
        )
        s = scan_stats(res)
        self.assertEqual(s["n_clusters"], 12.0)
        self.assertAlmostEqual(s["ground_fraction"], 0.95)
        self.assertEqual(s["above_ground_points"], 50.0)

    def test_missing_fields_are_nan_and_skipped_in_aggregate(self) -> None:
        s = scan_stats(types.SimpleNamespace())
        self.assertTrue(math.isnan(s["n_clusters"]))
        self.assertTrue(math.isnan(s["ground_fraction"]))
        agg = aggregate_scan_stats([s, {"n_clusters": 10.0, "ground_fraction": 0.9,
                                        "above_ground_points": 100.0}])
        self.assertEqual(agg["mean_n_clusters"], 10.0)
        self.assertEqual(agg["mean_ground_fraction"], 0.9)


class StopLatchMinTravelTest(unittest.TestCase):
    def test_reads_this_checkouts_params_yaml(self) -> None:
        import run_perception_benchmark as rpb

        value, source = rpb._stop_latch_min_travel()
        params = Path(__file__).resolve().parents[3] / "pipeline/bringup/config/params.yaml"
        if not params.is_file():
            self.skipTest("pipeline submodule not checked out")
        self.assertEqual(value, 30.0)
        self.assertTrue(source.startswith("checkout params.yaml"), source)

    def test_shallow_container_path_does_not_raise(self) -> None:
        # In the benchmark container the script is /bench/<name>, with no
        # checkout above it; this used to raise IndexError on parents[2].
        import run_perception_benchmark as rpb

        value, source = rpb._stop_latch_min_travel(Path("/bench/run_perception_benchmark.py"))
        self.assertIsInstance(value, float)
        self.assertNotIn("checkout", source)


class ReferenceDeltasTest(unittest.TestCase):
    def test_numeric_delta_and_skips_missing(self) -> None:
        cur = {"gt_metrics": {"recall": 0.80, "big_orange_recall": None},
               "stop_latch": {"premature": True}}
        ref = {"gt_metrics": {"recall": 0.95, "big_orange_recall": 0.9},
               "stop_latch": {"premature": False}}
        d = reference_deltas(cur, ref)
        self.assertAlmostEqual(d["recall"]["delta"], -0.15)
        self.assertNotIn("big_orange_recall", d)
        # Booleans are compared, not subtracted.
        self.assertEqual(d["premature"], {"value": True, "reference": False})


if __name__ == "__main__":
    unittest.main()
