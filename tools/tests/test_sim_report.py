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

    def test_sim_speed_shows_the_machines_cpu_wait_when_recorded(self):
        write_run(self.root, "acceleration_acceleration", "acceleration", 1, True, 29.0)
        write_run(self.root, "skidpad_skidpad", "skidpad", 1, True, 80.0)
        res = self.root / "headless/20261103_120000/mission/skidpad_skidpad/abc1234/seed1/results.json"
        res.write_text(json.dumps({**json.loads(res.read_text()), "box/cpu_wait": 0.214}))
        r = sim_report.build(self.root, CONFIG)
        body = sim_report.comment(r, None, {}, "https://run")
        self.assertIn("0.95× · 21 % CPU wait", body)  # recorded: shown
        acc = next(g for g in r["groups"] if g["mission"] == "acceleration")
        self.assertIsNone(acc["cpu_wait"])  # an older run without it: just the speed
        self.assertEqual(sim_report.speed_cell(0.95, None), "0.95×")


class Characterisation(unittest.TestCase):
    """The car characterisation's result (tools/characterise) in the report."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.run = self.root / "headless" / "20261103_120000"
        for s in (1, 2):
            write_run(self.root, "acceleration_acceleration", "acceleration", s, True, 30.0)
        write_run(self.root, "skidpad_skidpad", "skidpad", 1, True, 80.0)

    def tearDown(self):
        self.tmp.cleanup()

    def write_result(self, status: str, failed: str = ""):
        d = self.run / "characterisation"
        d.mkdir(parents=True, exist_ok=True)
        checks = [{"id": "r", "name": "Full-lock radius, left", "hard": True,
                   "status": "fail" if failed else "pass"}]
        (d / "result.json").write_text(json.dumps({"status": status, "checks": checks}))
        (d / "summary.md").write_text(f"#### Car characterisation · {status}\n\n| Check |\n")

    def config(self, required: bool):
        return {**CONFIG, "characterisation": {"enabled": True, "required": required}}

    def test_a_failed_hard_check_fails_the_check_when_required(self):
        self.write_result("fail", failed="Full-lock radius, left")
        r = sim_report.build(self.root, self.config(True))
        self.assertEqual(r["verdict"], "fail")
        self.assertEqual(r["characterisation"]["failed"], ["Full-lock radius, left"])
        md = sim_report.summary(r, None)
        self.assertIn("#### Car characterisation · fail", md)
        self.assertIn("This fails the sim check.", md)
        self.assertIn("car fail", sim_report.history_cell(r))

    def test_report_only_when_not_required(self):
        self.write_result("fail", failed="Full-lock radius, left")
        r = sim_report.build(self.root, self.config(False))
        self.assertEqual(r["verdict"], "pass")
        self.assertIn("Reported only", sim_report.comment(r, None, {}, "https://run"))

    def test_a_characterisation_that_could_not_measure_fails_nothing(self):
        self.write_result("error")
        r = sim_report.build(self.root, self.config(True))
        self.assertEqual(r["verdict"], "pass")
        self.assertIn("Car characterisation · error", sim_report.comment(r, None, {}, ""))

    def test_one_that_died_without_a_result_is_shown(self):
        rec = json.loads((self.run / "run.json").read_text())
        rec["characterisation"] = {"status": "timeout"}
        (self.run / "run.json").write_text(json.dumps(rec))
        r = sim_report.build(self.root, self.config(True))
        self.assertEqual(r["verdict"], "pass")
        self.assertIn("couldn't run", sim_report.characterisation_section(r))
        self.assertIn("timeout", sim_report.characterisation_section(r))

    def test_no_characterisation_no_section(self):
        r = sim_report.build(self.root, self.config(True))
        self.assertIsNone(r["characterisation"])
        self.assertEqual(sim_report.characterisation_section(r), "")
        self.assertNotIn("car ", sim_report.history_cell(r))


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

    def test_the_latest_run_section_is_replaced_each_time(self):
        body = sim_publish.issue_body("", "| 2026-11-01 00:00 | a |", None, "run one")
        body = sim_publish.issue_body(body, "| 2026-11-02 00:00 | b |", None, "run two")
        self.assertIn("run two", body)
        self.assertNotIn("run one", body)
        self.assertIn("No dev run yet.", body)

    def test_old_rows_get_an_empty_maps_cell(self):
        old = sim_publish.issue_body("", "", None).replace(
            "<!-- /history -->", "| 2026-11-01 00:00 | #1 | `a` | `b` | ok | m | [run](u) |\n<!-- /history -->")
        rows = [ln for ln in sim_publish.issue_body(old, "", None).splitlines() if ln.startswith("| 2")]
        self.assertEqual(rows, ["| 2026-11-01 00:00 | #1 | `a` | `b` | ok | m | – | [run](u) |"])

    def test_history_rows_link_their_maps(self):
        report = {"groups": [{"mission": "skidpad", "scenario": "skidpad_skidpad"},
                             {"mission": "autocross", "scenario": "autocross_t"}],
                  "images": {"skidpad_skidpad": "https://x/s.png"}}
        self.assertEqual(sim_publish.map_links(report), "[skidpad](https://x/s.png)")

    def test_a_new_issue_has_no_empty_rows(self):
        body = sim_publish.issue_body("", "", None)
        self.assertNotIn("\n\n<!-- /history -->", body)
        self.assertIn("No dev run yet.", body)


if __name__ == "__main__":
    unittest.main()
