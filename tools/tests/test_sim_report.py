"""Tests for the sim check's report (tools/ci/sim_report.py) and the report
issue's history (tools/ci/sim_publish.py). No network: only the pure parts.

    python -m unittest discover -s tools/tests -v
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "ci"))
import sim_publish  # noqa: E402
import sim_report  # noqa: E402

CONFIG = {"required": ["acceleration", "skidpad"], "informational": ["autocross"]}  # not the real one


def write_run(root: Path, scenario: str, event: str, seed: int, finished: bool, lap: float | None):
    d = root / "headless" / "20261103_120000" / "mission" / scenario / "abc1234" / f"seed{seed}"
    d.mkdir(parents=True, exist_ok=True)
    (d / "results.json").write_text(json.dumps({
        "race/finished": int(finished), "race/n_laps": int(finished), "race/n_doo": 1,
        "race/n_off_track": 0, "race/lap_time_best_s": lap,
        "race/run_time_s": 40.0, "race/run_sim_time_s": 38.0,
        "control/speed_mean_mps": 2.5, "control/speed_max_mps": 3.2,
    }))
    (d / "manifest.json").write_text(json.dumps({
        "outcome": "finished" if finished else "timeout",
        "scenario": {"event": event, "track": {"csv": f"{scenario.split('_', 1)[1]}.csv"}},
        "code": {"ifssim": {"sha": "1111111"}, "pipeline": {"sha": "2222222"}},
    }))
    (d / "telemetry.csv").write_text("t,x,y,speed,throttle,steering\n0,0,0,0,0.2,0.1\n1,1,0,2,0.2,-0.4\n")
    (root / "headless" / "20261103_120000" / "run.json").write_text(
        json.dumps({"outcome": "finished", "duration_s": 600}))


class Verdict(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_required_missions_finished_pass_even_if_informational_failed(self):
        for s in (1, 2, 3):
            write_run(self.root, "acceleration_acceleration", "acceleration", s, True, 30.0 + s)
        write_run(self.root, "skidpad_skidpad", "skidpad", 1, True, 80.0)
        write_run(self.root, "autocross_track_a", "autocross", 1, False, None)
        r = sim_report.build(self.root, CONFIG)
        self.assertEqual(r["verdict"], "pass")
        acc = next(g for g in r["groups"] if g["mission"] == "acceleration")
        self.assertEqual((acc["finished"], acc["runs"]), (3, 3))
        self.assertAlmostEqual(acc["lap_mean"], 32.0)
        self.assertAlmostEqual(acc["sim_speed"], 0.95)
        self.assertEqual((r["ifssim"], r["pipeline"]), ("1111111", "2222222"))
        auto = next(g for g in r["groups"] if g["mission"] == "autocross")
        self.assertFalse(auto["required"])

    def test_a_required_run_not_finished_fails(self):
        write_run(self.root, "acceleration_acceleration", "acceleration", 1, True, 30.0)
        write_run(self.root, "skidpad_skidpad", "skidpad", 1, False, None)
        self.assertEqual(sim_report.build(self.root, CONFIG)["verdict"], "fail")

    def test_a_required_mission_that_never_ran_fails(self):
        write_run(self.root, "acceleration_acceleration", "acceleration", 1, True, 30.0)
        r = sim_report.build(self.root, CONFIG)
        self.assertEqual((r["verdict"], r["missing"]), ("fail", ["skidpad"]))

    def test_no_run_is_an_error(self):
        self.assertEqual(sim_report.build(self.root, CONFIG)["verdict"], "error")

    def test_comment_compares_with_dev(self):
        write_run(self.root, "acceleration_acceleration", "acceleration", 1, True, 29.0)
        write_run(self.root, "skidpad_skidpad", "skidpad", 1, True, 80.0)
        r = sim_report.build(self.root, CONFIG)
        base = {"groups": [{"scenario": "acceleration_acceleration", "lap_mean": 30.0}]}
        body = sim_report.comment(r, base, {"acceleration_acceleration": "https://x/a.png"}, "https://run")
        self.assertTrue(body.startswith(sim_report.COMMENT_MARKER))
        self.assertIn("-1.0 s (dev 30.0)", body)
        self.assertIn('src="https://x/a.png"', body)
        self.assertIn("[Run, logs and telemetry](https://run)", body)


class IssueHistory(unittest.TestCase):
    def test_new_rows_go_first_and_the_latest_section_is_kept(self):
        body = sim_publish.issue_body("", "| 2026-11-01 00:00 | a |", "dev result 1")
        body = sim_publish.issue_body(body, "| 2026-11-02 00:00 | b |", None)
        self.assertTrue(body.startswith(sim_publish.ISSUE_MARKER))
        self.assertIn("dev result 1", body)
        rows = [ln for ln in body.splitlines() if ln.startswith("| 2")]
        self.assertEqual(rows, ["| 2026-11-02 00:00 | b |", "| 2026-11-01 00:00 | a |"])

    def test_history_is_capped(self):
        body = ""
        for i in range(sim_publish.HISTORY_ROWS + 5):
            body = sim_publish.issue_body(body, f"| 2026-11-03 {i:05d} |", None)
        rows = [ln for ln in body.splitlines() if ln.startswith("| 2")]
        self.assertEqual(len(rows), sim_publish.HISTORY_ROWS)
        self.assertEqual(rows[0], f"| 2026-11-03 {sim_publish.HISTORY_ROWS + 4:05d} |")

    def test_a_new_issue_has_no_empty_rows(self):
        body = sim_publish.issue_body("", "", None)
        self.assertNotIn("\n\n<!-- /history -->", body)
        self.assertIn("No dev run yet.", body)


if __name__ == "__main__":
    unittest.main()
