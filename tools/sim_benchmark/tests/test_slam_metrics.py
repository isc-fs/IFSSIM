from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slam_metrics import (  # noqa: E402
    MapPoint,
    PoseStepSample,
    aggregate_map_match,
    aggregate_pose_steps,
    odom_frame_to_gt_aligned,
    pose_err_m,
    world_pose_to_aligned,
    yaw_err,
    _sample_latest_before,
    _state_to_pose3,
)


def _has_gtsam() -> bool:
    try:
        import gtsam  # noqa: F401

        return True
    except ImportError:
        return False


@unittest.skipUnless(_has_gtsam(), "gtsam not installed")
class OdomFrameAlignedTest(unittest.TestCase):
    def test_sync_is_origin(self) -> None:
        sync = _state_to_pose3(3.0, 4.0, 0.2)
        x, y, yaw = odom_frame_to_gt_aligned(sync, 3.0, 4.0, 0.2)
        self.assertAlmostEqual(x, 0.0, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)
        self.assertAlmostEqual(yaw, 0.0, places=5)


@unittest.skipUnless(_has_gtsam(), "gtsam not installed")
class WorldPoseAlignedTest(unittest.TestCase):
    def test_identity_at_origin(self) -> None:
        import gtsam
        import numpy as np

        gt0 = gtsam.Pose3(gtsam.Rot3.Yaw(0.1), np.array([5.0, 2.0, 0.0]))
        x, y, yaw = world_pose_to_aligned(gt0, 5.0, 2.0, 0.1)
        self.assertAlmostEqual(x, 0.0, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)
        self.assertAlmostEqual(yaw, 0.0, places=5)

    def test_offset_pose(self) -> None:
        import gtsam
        import numpy as np

        gt0 = gtsam.Pose3(gtsam.Rot3.Yaw(0.0), np.array([0.0, 0.0, 0.0]))
        x, y, _ = world_pose_to_aligned(gt0, 3.0, 4.0, 0.0)
        self.assertAlmostEqual(x, 3.0, places=5)
        self.assertAlmostEqual(y, 4.0, places=5)


class AggregateMapMatchTest(unittest.TestCase):
    def test_greedy_match(self) -> None:
        gt = [MapPoint(0, 0), MapPoint(5, 0)]
        slam = [MapPoint(0.1, 0.0), MapPoint(5.2, 0.0), MapPoint(99, 99)]
        stats = aggregate_map_match(gt, slam, gate_m=0.5)
        self.assertEqual(stats.matched, 2)
        self.assertEqual(stats.false_positive, 1)
        self.assertEqual(stats.false_negative, 0)


class SampleLatestBeforeTest(unittest.TestCase):
    def test_hold_last_value(self) -> None:
        series = [(100, 1.0), (200, 2.0), (300, 3.0)]
        self.assertIsNone(_sample_latest_before(series, 50))
        self.assertAlmostEqual(_sample_latest_before(series, 150), 1.0)
        self.assertAlmostEqual(_sample_latest_before(series, 300), 3.0)


class AggregatePoseStepsTest(unittest.TestCase):
    def test_imu_filter_stats(self) -> None:
        steps = [
            PoseStepSample(
                t_s=1.0,
                event="imu",
                gt_x=0.0,
                gt_y=0.0,
                gt_yaw=0.0,
                filter_err_m=0.1,
                filter_err_delta_m=0.05,
            ),
            PoseStepSample(
                t_s=2.0,
                event="imu",
                gt_x=1.0,
                gt_y=0.0,
                gt_yaw=0.0,
                filter_err_m=0.3,
                filter_err_delta_m=0.2,
            ),
            PoseStepSample(
                t_s=3.0,
                event="cone",
                gt_x=1.0,
                gt_y=0.0,
                gt_yaw=0.0,
                slam_err_m=0.5,
            ),
        ]
        stats = aggregate_pose_steps(steps, event="imu")
        self.assertEqual(stats["steps"], 2)
        self.assertAlmostEqual(stats["mean_err_m"], 0.2)
        self.assertAlmostEqual(stats["max_err_m"], 0.3)
        self.assertIn("mean_step_delta_m", stats)

    def test_imu_only_stats(self) -> None:
        steps = [
            PoseStepSample(
                t_s=1.0,
                event="imu",
                gt_x=0.0,
                gt_y=0.0,
                gt_yaw=0.0,
                imu_only_err_m=1.0,
                imu_only_err_delta_m=0.5,
            ),
            PoseStepSample(
                t_s=2.0,
                event="imu",
                gt_x=1.0,
                gt_y=0.0,
                gt_yaw=0.0,
                imu_only_err_m=3.0,
                imu_only_err_delta_m=2.0,
            ),
        ]
        stats = aggregate_pose_steps(steps, event="imu", err_field="imu_only_err_m")
        self.assertEqual(stats["steps"], 2)
        self.assertAlmostEqual(stats["mean_err_m"], 2.0)
        self.assertIn("mean_step_delta_m", stats)

    def test_prefix_specific_yaw_stats(self) -> None:
        steps = [
            PoseStepSample(
                t_s=1.0,
                event="imu",
                gt_x=0.0,
                gt_y=0.0,
                gt_yaw=0.0,
                wheel_err_m=1.0,
                wheel_yaw_err_rad=math.radians(5.0),
                supervisor_err_m=2.0,
                supervisor_yaw_err_rad=math.radians(7.0),
                slam_yaw_err_rad=math.radians(99.0),
            ),
        ]
        wheel = aggregate_pose_steps(steps, event="imu", err_field="wheel_err_m")
        supervisor = aggregate_pose_steps(
            steps,
            event="imu",
            err_field="supervisor_err_m",
        )
        self.assertAlmostEqual(wheel["mean_yaw_err_deg"], 5.0)
        self.assertAlmostEqual(supervisor["mean_yaw_err_deg"], 7.0)

    def test_pose_err_and_yaw(self) -> None:
        self.assertAlmostEqual(pose_err_m(0, 0, 3, 4), 5.0)
        self.assertAlmostEqual(abs(yaw_err(0.1, -0.1)), 0.2, places=5)
if __name__ == "__main__":
    unittest.main()
