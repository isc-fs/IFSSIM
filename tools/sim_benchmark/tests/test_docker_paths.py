from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import common  # noqa: E402


class DockerPathsTest(unittest.TestCase):
    def test_inside_the_checkout_nothing_changes(self) -> None:
        bag = common.results_dir() / "capture" / "bag"
        argv = [str(bag), "--results-root", str(common.results_dir() / "sim_bag" / "s")]
        paths = common.DockerPaths(argv)
        self.assertEqual(
            common._translate_argv(argv, paths),
            ["/results/capture/bag", "--results-root", "/results/sim_bag/s"],
        )
        self.assertEqual(paths.volumes(), [])

    def test_bags_and_results_outside_are_mounted(self) -> None:
        # the central machine: shared bag and results folders, a checkout per job
        t = Path(tempfile.mkdtemp()).resolve()
        session = t / "results" / "sim_bag" / "s1"
        argv = [
            str(t / "bags" / "b1"),
            "--results-root",
            str(session),
            "--pipeline-overrides",
            str(session / "overrides" / "slam.json"),
        ]
        paths = common.DockerPaths(argv)
        self.assertEqual(
            common._translate_argv(argv, paths),
            [
                "/ext/0/b1",
                "--results-root",
                "/results",
                "--pipeline-overrides",
                "/results/overrides/slam.json",
            ],
        )
        self.assertEqual(paths.results, session)
        self.assertEqual(paths.volumes(), ["-v", f"{t / 'bags'}:/ext/0:ro"])


if __name__ == "__main__":
    unittest.main()
