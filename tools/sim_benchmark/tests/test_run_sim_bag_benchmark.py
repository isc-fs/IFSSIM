from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run_sim_bag_benchmark as rs  # noqa: E402

GT_TOPICS = ("/testing_only/track", "/testing_only/odom", "/imu", "/lidar/Lidar1")


def fake_bag(root: Path, topics=GT_TOPICS) -> Path:
    bag = root / "bag"
    bag.mkdir(parents=True)
    lines = ["rosbag2_bagfile_information:", "  topics_with_message_count:"]
    for t in topics:
        lines += ["    - topic_metadata:", f"        name: {t}"]
    (bag / "metadata.yaml").write_text("\n".join(lines) + "\n")
    return bag


class SelectionTest(unittest.TestCase):
    def test_default_runs_everything(self) -> None:
        self.assertEqual(rs.selected(rs.parse_args(["b"])), ["perception", "slam"])

    def test_only_and_skip(self) -> None:
        self.assertEqual(rs.selected(rs.parse_args(["b", "--only", "slam"])), ["slam"])
        self.assertEqual(
            rs.selected(rs.parse_args(["b", "--skip", "slam"])), ["perception"]
        )
        self.assertEqual(
            rs.selected(rs.parse_args(["b", "--only", "slam", "--skip", "slam"])), []
        )


class CommandTest(unittest.TestCase):
    def cmd(self, name: str, *argv: str) -> list[str]:
        args = rs.parse_args(["bag", *argv])
        return rs.command(name, args, Path("/b"), Path("/s"))

    def test_gating_goes_to_every_benchmark(self) -> None:
        for n in ("perception", "slam"):
            c = self.cmd(n, "--gt-range-m", "25")
            self.assertEqual(c[c.index("--gt-range-m") + 1], "25.0")
            self.assertEqual(c[c.index("--results-root") + 1], "/s")

    def test_options_go_only_where_they_belong(self) -> None:
        p = self.cmd("perception", "--profile", "--motion-model", "imu")
        s = self.cmd("slam", "--profile", "--motion-model", "imu")
        self.assertIn("--profile", p)
        self.assertNotIn("--motion-model", p)
        self.assertIn("--motion-model", s)
        self.assertNotIn("--profile", s)

    def test_passthrough_args(self) -> None:
        c = self.cmd(
            "perception", "--perception-args", "--no-vfov-gate --bev-samples 4"
        )
        self.assertEqual(c[-3:], ["--no-vfov-gate", "--bev-samples", "4"])


class MainTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp())

    def run_main(self, *argv: str) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            rc = rs.main(list(argv))
        return rc, out.getvalue(), err.getvalue()

    def test_dry_run_prints_and_writes_nothing(self) -> None:
        bag = fake_bag(self.tmp)
        rc, out, _ = self.run_main(
            str(bag), "--results-root", str(self.tmp / "res"), "--dry-run"
        )
        self.assertEqual(rc, 0)
        self.assertIn("[perception]", out)
        self.assertIn("[slam]", out)
        self.assertFalse((self.tmp / "res").exists())

    def test_failed_benchmarks_make_a_failed_session(self) -> None:
        bag = fake_bag(self.tmp)
        with mock.patch.object(rs.subprocess, "call", return_value=1):
            rc, _, _ = self.run_main(str(bag), "--results-root", str(self.tmp / "res"))
        self.assertEqual(rc, 1)
        (session,) = (self.tmp / "res" / "sim_bag").iterdir()
        doc = json.loads((session / "session.json").read_text())
        self.assertEqual(doc["status"], "failed")
        self.assertEqual({b["status"] for b in doc["benchmarks"].values()}, {"failed"})

    def test_each_benchmark_gets_its_own_overrides(self) -> None:
        bag = fake_bag(self.tmp)
        o = self.tmp / "o.json"
        o.write_text(json.dumps({"slam_node": {"motion_model": "imu"}}))
        with mock.patch.object(rs.subprocess, "call", return_value=1) as call:
            self.run_main(
                str(bag),
                "--results-root",
                str(self.tmp / "res"),
                "--pipeline-overrides",
                str(o),
            )
        perception, slam = (c.args[0] for c in call.call_args_list)
        self.assertNotIn("--pipeline-overrides", perception)
        f = Path(slam[slam.index("--pipeline-overrides") + 1])
        self.assertEqual(
            json.loads(f.read_text()), {"slam_node": {"motion_model": "imu"}}
        )

    def test_overrides_no_selected_benchmark_uses_are_refused(self) -> None:
        bag = fake_bag(self.tmp)
        o = self.tmp / "o.json"
        o.write_text(json.dumps({"slam_node": {"motion_model": "imu"}}))
        rc, _, err = self.run_main(
            str(bag),
            "--only",
            "perception",
            "--pipeline-overrides",
            str(o),
            "--dry-run",
        )
        self.assertEqual(rc, 2)
        self.assertIn("does not use slam_node", err)

    def test_refuses_a_bag_without_ground_truth(self) -> None:
        bag = fake_bag(self.tmp, topics=("/imu", "/lidar/Lidar1"))
        rc, _, err = self.run_main(str(bag), "--dry-run")
        self.assertEqual(rc, 2)
        self.assertIn("no ground truth", err)


if __name__ == "__main__":
    unittest.main()
