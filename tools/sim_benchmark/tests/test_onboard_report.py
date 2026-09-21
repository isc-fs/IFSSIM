from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from onboard_report import (  # noqa: E402
    empty_rate,
    interp_at,
    path_length_m,
    render_onboard_html,
    steer_residual_rad,
    summarize,
)
from run_onboard_replay import bag_duration_s, bag_topic_counts  # noqa: E402


def _samples() -> dict:
    return {
        "source_duration_s": 10.0,
        "source_counts": {"/imu": 4000, "/lidar_points": 100, "/motor_rpm": 800},
        "steering": [{"t_s": 0.0, "rad": 0.0}, {"t_s": 1.0, "rad": 0.2}],
        "conos_raw": [
            {"t_s": 0.0, "n": 8},
            {"t_s": 0.1, "n": 0},
            {"t_s": 0.2, "n": 6},
        ],
        "conos": [{"t_s": 0.2, "n": 12}],
        "odom": [
            {"t_s": 0.0, "x": 0.0, "y": 0.0},
            {"t_s": 1.0, "x": 3.0, "y": 4.0},
        ],
        "slam": [
            {"t_s": 0.0, "x": 0.0, "y": 0.0},
            {"t_s": 1.0, "x": 3.1, "y": 4.0},
        ],
        "path": [{"t_s": 1.0, "n": 2, "xs": [0.0, 3.0], "ys": [0.0, 4.0]}],
        "cmd": [
            {"t_s": 0.0, "throttle": 0.0, "steering": 0.0, "brake": 0.0},
            {"t_s": 1.0, "throttle": 0.4, "steering": 0.25, "brake": 0.0},
        ],
        "map_x": [1.0, 2.0],
        "map_y": [0.5, -0.5],
    }


class PathLengthTest(unittest.TestCase):
    def test_3_4_5(self) -> None:
        self.assertAlmostEqual(path_length_m([0.0, 3.0], [0.0, 4.0]), 5.0)

    def test_empty(self) -> None:
        self.assertEqual(path_length_m([], []), 0.0)


class EmptyRateTest(unittest.TestCase):
    def test_one_empty_of_three(self) -> None:
        self.assertAlmostEqual(empty_rate([8, 0, 6]), 1.0 / 3.0)

    def test_no_scans_is_all_empty(self) -> None:
        self.assertEqual(empty_rate([]), 1.0)


class InterpTest(unittest.TestCase):
    def test_midpoint(self) -> None:
        self.assertAlmostEqual(interp_at([0.0, 2.0], [0.0, 10.0], 1.0), 5.0)


class SummarizeTest(unittest.TestCase):
    def test_counts_and_gaps(self) -> None:
        summary = summarize(_samples())
        self.assertEqual(summary["module"], "onboard")
        self.assertFalse(summary["gt_eval"])
        self.assertEqual(summary["n_conos_raw"], 3)
        self.assertEqual(summary["max_conos_raw"], 8)
        self.assertAlmostEqual(summary["odom_path_length_m"], 5.0)
        self.assertAlmostEqual(summary["odom_slam_end_gap_m"], 0.1)
        self.assertEqual(summary["n_map_cones"], 2)
        residual = steer_residual_rad(
            _samples()["cmd"],
            _samples()["steering"],
        )
        self.assertAlmostEqual(residual[-1], abs(0.25 - 0.2))

    def test_html_mentions_no_gt(self) -> None:
        samples = _samples()
        html = render_onboard_html(summarize(samples), samples)
        self.assertIn("No ground truth", html)
        self.assertIn("Trajectory", html)
        self.assertIn("SLAM map cones", html)


class BagMetadataTest(unittest.TestCase):
    def test_counts_and_duration(self) -> None:
        text = """rosbag2_bagfile_information:
  version: 5
  storage_identifier: mcap
  duration:
    nanoseconds: 1500000000
  message_count: 12
  topics_with_message_count:
    - topic_metadata:
        name: /imu
        type: sensor_msgs/msg/Imu
      message_count: 10
    - topic_metadata:
        name: /lidar_points
        type: sensor_msgs/msg/PointCloud2
      message_count: 2
"""
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp)
            (bag / "metadata.yaml").write_text(text)
            self.assertAlmostEqual(bag_duration_s(bag), 1.5)
            self.assertEqual(bag_topic_counts(bag)["/imu"], 10)
            self.assertEqual(bag_topic_counts(bag)["/lidar_points"], 2)


if __name__ == "__main__":
    unittest.main()
