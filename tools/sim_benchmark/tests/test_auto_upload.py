from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import auto_upload as au  # noqa: E402


class EnabledTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = Path(tempfile.mkdtemp()) / "tracking.env"
        self.env = mock.patch.dict(
            os.environ, {"IFSSIM_BENCH_CONFIG": str(self.cfg)}, clear=False
        )
        self.env.start()
        for k in ("MLFLOW_TRACKING_URI", au.ENV_OFF, "IFSSIM_BENCHMARK_IN_DOCKER"):
            os.environ.pop(k, None)

    def tearDown(self) -> None:
        self.env.stop()

    def test_off_without_a_server(self) -> None:
        self.assertFalse(au.enabled())

    def test_on_with_a_server_in_the_config_file(self) -> None:
        self.cfg.write_text("# team server\nMLFLOW_TRACKING_URI=https://bench.x.ts.net\n")
        self.assertTrue(au.enabled())

    def test_can_be_turned_off(self) -> None:
        self.cfg.write_text("MLFLOW_TRACKING_URI=https://x\nIFSSIM_AUTO_UPLOAD=0\n")
        self.assertFalse(au.enabled())
        self.cfg.write_text("MLFLOW_TRACKING_URI=https://x\n")
        os.environ[au.ENV_OFF] = "0"
        self.assertFalse(au.enabled())

    def test_never_inside_the_container(self) -> None:
        os.environ["MLFLOW_TRACKING_URI"] = "https://x"
        os.environ["IFSSIM_BENCHMARK_IN_DOCKER"] = "1"
        self.assertFalse(au.enabled())

    def test_after_run_syncs_the_results_dir_and_never_raises(self) -> None:
        os.environ["MLFLOW_TRACKING_URI"] = "https://x"
        with (
            mock.patch.object(au.shutil, "which", return_value="/usr/bin/uv"),
            mock.patch.object(au.subprocess, "call", return_value=1) as call,
        ):
            au.after_run(Path("/r"))  # a failed upload only prints
        cmd = call.call_args[0][0]
        self.assertEqual(cmd[-3:], ["sync", "--results-root", "/r"])


if __name__ == "__main__":
    unittest.main()
