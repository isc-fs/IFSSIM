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
from run_onboard_replay import (  # noqa: E402
    bag_duration_s,
    bag_topic_counts,
    count_tcp_established,
    foxglove_client_connected,
    foxglove_params_yaml,
    live_docker_publish_args,
    wait_for_foxglove_client,
)


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


class LiveFoxgloveTest(unittest.TestCase):
    def test_params_yaml_has_lidar_points_and_port(self) -> None:
        yaml = foxglove_params_yaml(8766)
        self.assertIn("port: 8766", yaml)
        self.assertIn("/lidar_points", yaml)
        self.assertIn("/lidar_points/ground", yaml)
        self.assertIn("/lidar_points/above_ground", yaml)
        self.assertIn("/cone_detection/latency_ms", yaml)
        self.assertIn("/cone_detection/n_left", yaml)
        self.assertIn("/path_planning/n_waypoints", yaml)
        self.assertIn("/cone_slam/n_obs", yaml)
        self.assertIn("/slam/final_lap", yaml)
        self.assertIn("/cone_slam/hz", yaml)
        self.assertIn("/Conos_raw", yaml)
        self.assertIn("use_sim_time: true", yaml)

    def test_docker_publish_only_when_live(self) -> None:
        self.assertEqual(live_docker_publish_args(["--duration-s", "30"]), ["--shm-size=1g"])
        self.assertEqual(
            live_docker_publish_args(["--live"]),
            ["--shm-size=1g", "-p", "8766:8766"],
        )
        self.assertEqual(
            live_docker_publish_args(["--live", "--foxglove-port", "8765"]),
            ["--shm-size=1g", "-p", "8765:8765"],
        )

    def test_client_connected_matches_bridge_logs(self) -> None:
        self.assertTrue(
            foxglove_client_connected(
                '[INFO] [foxglove_bridge]: Client 0 connected from 172.17.0.1'
            )
        )
        self.assertTrue(
            foxglove_client_connected(
                "[2026-09-22T17:18:52Z INFO  foxglove::websocket::server] Connection opened"
            )
        )
        self.assertFalse(
            foxglove_client_connected(
                '[INFO] [foxglove_bridge]: Server listening on port 8766'
            )
        )
        self.assertFalse(
            foxglove_client_connected(
                '[INFO] [foxglove_bridge]: Advertising new channel 2 for topic "/tf"'
            )
        )

    def test_wait_returns_immediately_when_log_already_has_client(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "foxglove_bridge.log"
            log.write_text("[INFO] [foxglove_bridge]: Client 0 connected\n")
            self.assertTrue(wait_for_foxglove_client(log, timeout_s=5.0, poll_s=0.05))

    def test_wait_skipped_when_timeout_zero(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "missing.log"
            self.assertFalse(wait_for_foxglove_client(log, timeout_s=0.0))

    def test_tcp_established_ignores_listen(self) -> None:
        # 8766 = 0x223E. Listen (0A) must not count; ESTABLISHED (01) must.
        dump = (
            "  sl  local_address rem_address   st\n"
            "   0: 00000000:223E 00000000:0000 0A\n"
            "   1: 0100007F:223E 0100007F:C000 01\n"
            "   2: 00000000:2235 00000000:0000 0A\n"
        )
        self.assertEqual(count_tcp_established(8766, dump), 1)
        self.assertEqual(count_tcp_established(8765, dump), 0)


if __name__ == "__main__":
    unittest.main()
