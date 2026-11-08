"""Tests for the track geometry shared by the sim report and the track generator
(tools/ci/track_geometry.py), the cone-hit rows of events.csv, and how the report
shows them.

    python -m unittest discover -s tools/tests -v
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "ci"))
import gen_tracks  # noqa: E402
import sim_report  # noqa: E402
import track_geometry as tg  # noqa: E402


def ring(r: float, half: float = 1.5, n: int = 48) -> list[tuple[float, float, int]]:
    """A round track of centreline radius r: blue outside, yellow inside."""
    out = []
    for color, rr in ((tg.BLUE, r + half), (tg.YELLOW, r - half)):
        for i in range(n):
            a = 2 * math.pi * i / n
            out.append((rr * math.cos(a), rr * math.sin(a), color))
    return out


def hairpin(r: float, half: float = 1.5, step_deg: float = 15.0) -> list[tuple[float, float, int]]:
    """Straight in, a 180 degree turn of centreline radius r, straight out (open)."""
    mid = [(float(x), 0.0) for x in range(-20, 0, 3)]
    for k in range(0, 181, int(step_deg)):
        a = math.radians(k - 90)
        mid.append((r * math.cos(a), r + r * math.sin(a)))
    mid += [(-float(x), 2 * r) for x in range(3, 21, 3)]
    out = []
    for color, sign in ((tg.BLUE, 1), (tg.YELLOW, -1)):
        for i, (x, y) in enumerate(mid):
            # offset along the normal of the local direction (left = +)
            j, k = max(i - 1, 0), min(i + 1, len(mid) - 1)
            dx, dy = mid[k][0] - mid[j][0], mid[k][1] - mid[j][1]
            n = math.hypot(dx, dy)
            out.append((x - sign * half * dy / n, y + sign * half * dx / n, color))
    return out


class Radius(unittest.TestCase):
    def test_circumradius_of_three_points_on_a_circle(self):
        pts = [(8 * math.cos(a), 8 * math.sin(a)) for a in (0.0, 0.4, 0.8)]
        self.assertAlmostEqual(tg.circumradius(*pts), 8.0)

    def test_collinear_is_infinite(self):
        self.assertTrue(math.isinf(tg.circumradius((0, 0), (1, 0), (2, 0))))

    def test_a_circle_of_8_m(self):
        stats = tg.corner_stats(ring(8.0))
        self.assertAlmostEqual(stats["min_radius_m"], 8.0, delta=0.1)
        self.assertEqual(stats["n_tight"], 0)

    def test_a_hairpin_is_found_and_counted_once(self):
        stats = tg.corner_stats(hairpin(3.5))
        self.assertAlmostEqual(stats["min_radius_m"], 3.5, delta=0.3)
        self.assertEqual(stats["n_tight"], 1)
        self.assertGreater(tg.min_radius(hairpin(6.0)), tg.DEFAULT_FLOOR_M)

    def test_short_segments_are_skipped(self):
        pts = [(0, 0), (0.1, 0), (0.1, 0.1), (5, 5), (10, 5.5)]
        self.assertTrue(math.isinf(tg.radii(pts)[0]))

    def test_radius_near_a_point(self):
        cones = hairpin(3.5)
        r_tip = tg.radius_near(cones, 3.5, 3.5)
        r_straight = tg.radius_near(cones, -18.0, 0.0)
        self.assertLess(r_tip, 4.5)
        self.assertIsNone(r_straight)  # nothing finite on a straight

    def test_no_cones(self):
        self.assertEqual(tg.corner_stats([]), {"min_radius_m": None, "n_tight": 0})


class GeneratorFloor(unittest.TestCase):
    def setUp(self):
        # the length and cone-count rules are not what these tests are about
        self._saved = (gen_tracks.MIN_CONES, gen_tracks.MIN_LENGTH_M)
        gen_tracks.MIN_CONES, gen_tracks.MIN_LENGTH_M = 0, 0.0

    def tearDown(self):
        gen_tracks.MIN_CONES, gen_tracks.MIN_LENGTH_M = self._saved

    @staticmethod
    def rows(cones):
        return [("blue" if c == tg.BLUE else "yellow", x, y) for x, y, c in cones]

    def test_a_track_under_the_floor_is_rejected(self):
        why = gen_tracks.problem(self.rows(ring(4.0, n=40)), 5.0)
        self.assertIn("corner R", why)

    def test_a_drivable_track_passes(self):
        self.assertIsNone(gen_tracks.problem(self.rows(ring(8.0, n=48)), 5.0))

    def test_floor_zero_turns_the_check_off(self):
        self.assertIsNone(gen_tracks.problem(self.rows(ring(4.0, n=40)), 0.0))


def doo_run(root: Path, hits: str):
    d = root / "headless" / "20261105_120000" / "mission" / "autocross_t" / "abc1234" / "seed1"
    d.mkdir(parents=True, exist_ok=True)
    (d / "results.json").write_text(json.dumps({"race/finished": 0, "race/n_doo": 2,
                                                "race/run_time_s": 40.0}))
    (d / "manifest.json").write_text(json.dumps({"outcome": "timeout",
        "scenario": {"event": "autocross", "track": {"csv": "t.csv"}}}))
    (d / "telemetry.csv").write_text("t,x,y,speed\n0,0,0,1\n")
    (d / "cones.csv").write_text("x,y,color\n" + "".join(
        f"{x},{y},{c}\n" for x, y, c in hairpin(3.5)))
    (d / "events.csv").write_text("kind,t,detail\n" + hits)
    (root / "headless" / "20261105_120000" / "run.json").write_text(json.dumps({"outcome": "finished"}))


class Report(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_corner_column_and_hit_table(self):
        doo_run(self.root,
                "dnf,40,timeout\n"
                "doo,12.5,cone=3 name=Cone_3 color=blue x=3.5 y=3.5 disp_m=0.4 tilt_deg=80.0 why=displaced "
                "car_x=3 car_y=2 car_speed=3.2\n"
                "doo,13.0,cone=4 color=yellow why=tipped\n")
        r = sim_report.build(self.root, {"required": [], "informational": ["autocross"]})
        run = r["runs"][0]
        self.assertEqual(run["dnf"], "timeout")  # a doo row doesn't disturb the dnf reader
        self.assertAlmostEqual(run["corner_min_m"], 3.5, delta=0.3)
        self.assertEqual(len(run["hits"]), 2)
        self.assertEqual(run["hits"][0]["color"], "blue")
        self.assertAlmostEqual(run["hits"][0]["speed"], 3.2)
        md = sim_report.tables(r, None)
        self.assertIn("Tightest corner", md)
        self.assertIn("**3.", md)  # under the car's turning circle: bold
        self.assertIn("<summary>Cone hits</summary>", md)
        self.assertIn("blue (left)", md)
        self.assertIn("yellow (right)", md)
        self.assertIn("3.2 m/s", md)

    def test_no_hits_no_table(self):
        doo_run(self.root, "dnf,40,timeout\n")
        r = sim_report.build(self.root, {"required": [], "informational": ["autocross"]})
        self.assertNotIn("Cone hits", sim_report.tables(r, None))

    def test_hit_rows_are_capped(self):
        rows = "".join(f"doo,{i},color=blue\n" for i in range(30))
        doo_run(self.root, rows)
        r = sim_report.build(self.root, {"required": [], "informational": ["autocross"]})
        md = sim_report.hits_table(r)
        self.assertIn("more", md)
        self.assertLessEqual(md.count("\n| autocross"), sim_report.HITS_TOTAL)


if __name__ == "__main__":
    unittest.main()
