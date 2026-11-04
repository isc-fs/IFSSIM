"""Tests for the random autocross tracks of the sim check: where a run failed
(tools/ci/sim_failures.py), how the report shows it (sim_report.py, sim_publish.py)
and the generator's track checks (gen_tracks.py). No network, no generator.

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
import sim_failures  # noqa: E402
import sim_publish  # noqa: E402
import sim_report  # noqa: E402

R, HALF = 20.0, 1.5  # a round track: centreline radius and half width


def circle_cones(n: int = 60) -> list[dict]:
    """Blue outside, yellow inside, both listed counter-clockwise from (R, 0)."""
    out = []
    for color, r in ((1, R + HALF), (0, R - HALF)):
        for i in range(n):
            a = 2 * math.pi * i / n
            out.append({"x": r * math.cos(a), "y": r * math.sin(a), "color": color})
    return out


def drive(arc_m: float, offset: float = 0.0, ccw: bool = True, step: float = 0.5) -> list[dict]:
    """Telemetry along the centreline (+ offset outward) for arc_m metres."""
    out, t = [], 0.0
    for k in range(int(arc_m / step) + 1):
        a = (k * step / R) * (1 if ccw else -1)
        r = R + offset
        out.append({"t": t, "x": r * math.cos(a), "y": r * math.sin(a), "speed": 3.0})
        t += step / 3.0
    return out


class Locate(unittest.TestCase):
    def test_leaving_the_track_is_found_with_its_progress(self):
        tel = drive(40.0) + [dict(p, x=p["x"] * (R + 4) / R, y=p["y"] * (R + 4) / R)
                             for p in drive(50.0)[-10:]]
        f = sim_failures.locate(circle_cones(), tel)
        self.assertEqual(f["kind"], "left the track")
        self.assertAlmostEqual(f["progress_m"], 40.0, delta=1.0)  # its last point on the track
        self.assertAlmostEqual(f["lap_m"], 2 * math.pi * R, delta=1.0)
        self.assertEqual(f["corner"], f"left R {R:.0f} m")

    def test_driving_the_other_way_is_a_right_hander(self):
        tel = drive(30.0, ccw=False) + [{"t": 99, "x": 0.0, "y": 0.0, "speed": 3.0}]
        f = sim_failures.locate(circle_cones(), tel)
        self.assertEqual(f["kind"], "left the track")
        self.assertTrue(f["corner"].startswith("right R "))
        self.assertAlmostEqual(f["progress_m"], 30.0, delta=1.5)

    def test_a_car_that_stays_on_track_stopped_where_it_last_moved(self):
        tel = drive(25.0) + [dict(p, speed=0.0, t=p["t"] + 20) for p in drive(25.0)[-1:]] * 5
        f = sim_failures.locate(circle_cones(), tel)
        self.assertEqual(f["kind"], "stopped")
        self.assertAlmostEqual(f["progress_m"], 25.0, delta=1.0)

    def test_nothing_to_go_on(self):
        self.assertIsNone(sim_failures.locate([], drive(10)))
        self.assertIsNone(sim_failures.locate(circle_cones(), []))


def write_run(root: Path, track: str, finished: bool, tel: list[dict], cones: list[dict]):
    stem = Path(track).stem
    d = root / "headless" / "20261104_120000" / "mission" / f"autocross_{stem}" / "abc1234" / "seed1"
    d.mkdir(parents=True, exist_ok=True)
    (d / "results.json").write_text(json.dumps({
        "race/finished": int(finished), "race/n_doo": 0, "race/n_off_track": int(not finished),
        "race/lap_time_best_s": 150.0 if finished else None,
        "race/run_time_s": 100.0, "race/run_sim_time_s": 99.0,
        "control/speed_mean_mps": 3.0, "control/speed_max_mps": 3.3,
    }))
    (d / "manifest.json").write_text(json.dumps({
        "outcome": "finished" if finished else "timeout",
        "scenario": {"event": "autocross", "track": {"csv": track}},
        "code": {"ifssim": {"sha": "1111111"}, "pipeline": {"sha": "2222222"}},
    }))
    (d / "events.csv").write_text("kind,t,detail\n" + ("" if finished else "dnf,300,timeout\n"))
    (d / "telemetry.csv").write_text("t,x,y,speed\n" + "".join(
        f"{p['t']},{p['x']},{p['y']},{p['speed']}\n" for p in tel))
    (d / "cones.csv").write_text("x,y,color\n" + "".join(
        f"{c['x']},{c['y']},{c['color']}\n" for c in cones))
    (root / "headless" / "20261104_120000" / "run.json").write_text(json.dumps({"outcome": "finished"}))


class Report(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        off = drive(40.0) + [{"t": 50, "x": 0.0, "y": 0.0, "speed": 3.0}]
        write_run(root, "track_a.csv", True, drive(120.0), circle_cones())
        write_run(root, "ci_rand_700.csv", True, drive(120.0), circle_cones())
        write_run(root, "ci_rand_720.csv", False, off, circle_cones())
        self.report = sim_report.build(root, {"required": [], "informational": ["autocross"]})

    def tearDown(self):
        self.tmp.cleanup()

    def test_only_the_unfinished_run_is_a_failure(self):
        fs = self.report["failures"]
        self.assertEqual(len(fs), 1)
        self.assertEqual((fs[0]["track"], fs[0]["random_seed"], fs[0]["dnf"]), ("ci_rand_720.csv", 720, "timeout"))
        self.assertEqual(fs[0]["kind"], "left the track")

    def test_random_tracks_share_one_summary_row(self):
        rows = sim_report.display_groups(self.report["groups"])
        self.assertEqual(len(rows), 2)
        rnd = next(g for g in rows if g.get("random"))
        self.assertEqual((rnd["tracks"], rnd["finished"], rnd["runs"], rnd["lap_mean"]), (2, 1, 2, None))
        table = sim_report.tables(self.report, None)
        self.assertIn("autocross · 2 random tracks *(info)* | **1/2**", table)
        self.assertIn("*autocross random* 1/2", sim_report.history_cell(self.report))

    def test_the_comment_says_where_it_failed(self):
        text = sim_report.failures_table(self.report, {"ci_rand_720": "https://x/ci_rand_720.csv"})
        self.assertIn("[`ci_rand_720`](https://x/ci_rand_720.csv)", text)
        self.assertIn("left the track", text)
        self.assertIn("(timeout)", text)

    def test_the_issue_keeps_failures_newest_first(self):
        rows = sim_publish.failure_rows(self.report, "PR #9", "2026-11-04 12:00")
        self.assertEqual(len(rows), 1)
        body = sim_publish.issue_body("", "", None, None, rows)
        body = sim_publish.issue_body(body, "", None, None, [rows[0].replace("PR #9", "dev")])
        failures = sim_publish.section(body, "failures").splitlines()
        self.assertIn("| dev |", failures[2])
        self.assertIn("| PR #9 |", failures[3])

    def test_an_old_issue_without_a_failures_section_gets_one(self):
        body = sim_publish.issue_body("", "| 2026 | x |", None)
        self.assertIn("<!-- failures -->", body)
        self.assertNotIn("| 2", sim_publish.section(body, "failures"))


class TrackChecks(unittest.TestCase):
    def cones(self, n: int, radius: float) -> list[tuple[str, float, float]]:
        out = []
        for kind, r in (("blue", radius + 1.5), ("yellow", radius - 1.5)):
            out += [(kind, r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]
        return out

    def test_a_drivable_track_passes(self):
        self.assertIsNone(gen_tracks.problem(self.cones(80, 50)))

    def test_too_short_too_long_or_too_few_cones(self):
        self.assertIn("m long", gen_tracks.problem(self.cones(40, 15)))
        self.assertIn("m long", gen_tracks.problem(self.cones(200, 150)))
        self.assertIn("cones", gen_tracks.problem(self.cones(20, 50)))

    def test_reads_the_generator_csv(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.csv"
            p.write_text("blue,1.0,2.0,0,0.01,0.01,0\nyellow,3.5,4.0,0,0.01,0.01,0\nbad line\n")
            self.assertEqual(gen_tracks.read_cones(p), [("blue", 1.0, 2.0), ("yellow", 3.5, 4.0)])


if __name__ == "__main__":
    unittest.main()
