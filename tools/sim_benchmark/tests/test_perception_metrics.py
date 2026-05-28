from __future__ import annotations

import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception_metrics import (
    Cone2D,
    WorldCone,
    cone_in_lidar_fov,
    filter_cones_in_fov,
    latch_track_layout,
    odom_at_time,
    odom_for_lidar_scan,
    track_at_or_before,
    track_cones_to_body,
    world_cones_to_body,
)


class _Quat:
    def __init__(self, w: float, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.w, self.x, self.y, self.z = w, x, y, z


class _Pose:
    def __init__(self, x: float, y: float, yaw: float):
        self.position = types.SimpleNamespace(x=x, y=y, z=0.0)
        self.orientation = _Quat(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


class _Odom:
    def __init__(self, x: float, y: float, yaw: float):
        self.pose = types.SimpleNamespace(pose=_Pose(x, y, yaw))


class _Cone:
    def __init__(self, x: float, y: float, color: int = 1):
        self.location = types.SimpleNamespace(x=x, y=y, z=0.0)
        self.color = color


class _Track:
    def __init__(self, cones: list[_Cone]):
        self.track = cones


class LatchTrackLayoutTest(unittest.TestCase):
    def test_uses_first_non_empty_track(self) -> None:
        t1 = _Track([_Cone(1.0, 0.0)])
        t2 = _Track([_Cone(99.0, 0.0)])
        msgs = [(5_000, t2), (1_000, t1)]
        layout = latch_track_layout(msgs)
        assert layout is not None
        self.assertEqual(len(layout), 1)
        self.assertAlmostEqual(layout[0].x, 1.0)

    def test_world_layout_matches_track_msg_transform(self) -> None:
        odom = _Odom(0.0, 0.0, 0.0)
        world = [WorldCone(5.0, 0.0)]
        from_msg = track_cones_to_body(_Track([_Cone(5.0, 0.0)]), odom, range_m=20.0)
        from_world = world_cones_to_body(world, odom, range_m=20.0)
        self.assertEqual(len(from_msg), len(from_world))
        self.assertAlmostEqual(from_msg[0].x, from_world[0].x)


class FilterConesInFovTest(unittest.TestCase):
    def test_drops_outside_fov(self) -> None:
        cones = [
            Cone2D(x=5.0, y=0.0),
            Cone2D(x=-2.0, y=0.0),
        ]
        kept = filter_cones_in_fov(cones, range_m=20.0, hfov_half_deg=60.0)
        self.assertEqual(len(kept), 1)
        self.assertAlmostEqual(kept[0].x, 5.0)


class TrackAtOrBeforeTest(unittest.TestCase):
    def test_picks_latest_latched_track_before_lidar_time(self) -> None:
        msgs = [
            (1_000, "t1"),
            (6_000_000_000, "t6s"),
            (11_000_000_000, "t11s"),
        ]
        self.assertIsNone(track_at_or_before(msgs, 500))
        self.assertEqual(track_at_or_before(msgs, 1_000), "t1")
        self.assertEqual(track_at_or_before(msgs, 5_000_000_000), "t1")
        self.assertEqual(track_at_or_before(msgs, 6_000_000_000), "t6s")
        self.assertEqual(track_at_or_before(msgs, 20_000_000_000), "t11s")


class TrackConesToBodyTest(unittest.TestCase):
    def test_range_gate_limits_gt_cones(self) -> None:
        odom = _Odom(0.0, 0.0, 0.0)
        track = _Track([_Cone(5.0, 0.0), _Cone(25.0, 0.0)])
        all_cones = track_cones_to_body(track, odom)
        gated = track_cones_to_body(track, odom, range_m=20.0)
        self.assertEqual(len(all_cones), 2)
        self.assertEqual(len(gated), 1)
        self.assertAlmostEqual(gated[0].x, 5.0)
        self.assertAlmostEqual(gated[0].y, 0.0)

    def test_body_transform_at_origin(self) -> None:
        odom = _Odom(10.0, 20.0, 0.0)
        track = _Track([_Cone(12.0, 20.0)])
        cones = track_cones_to_body(track, odom)
        self.assertAlmostEqual(cones[0].x, 2.0)
        self.assertAlmostEqual(cones[0].y, 0.0)

    def test_hfov_excludes_behind_and_wide_side_cones(self) -> None:
        odom = _Odom(0.0, 0.0, 0.0)
        track = _Track([_Cone(5.0, 0.0), _Cone(-3.0, 0.0), _Cone(5.0, 10.0)])
        cones = track_cones_to_body(track, odom, range_m=20.0, hfov_half_deg=60.0)
        self.assertEqual(len(cones), 1)
        self.assertAlmostEqual(cones[0].x, 5.0)
        self.assertFalse(cone_in_lidar_fov(-1.0, 0.0))
        self.assertFalse(cone_in_lidar_fov(5.0, 10.0))

    def test_min_range_excludes_blindspot_cones(self) -> None:
        odom = _Odom(0.0, 0.0, 0.0)
        track = _Track([_Cone(0.3, 0.0), _Cone(1.0, 0.0)])
        cones = track_cones_to_body(track, odom, range_m=20.0, min_range_m=0.5, hfov_half_deg=60.0)
        self.assertEqual(len(cones), 1)
        self.assertAlmostEqual(cones[0].x, 1.0)
        self.assertFalse(cone_in_lidar_fov(0.3, 0.0, min_range_m=0.5))
        self.assertTrue(cone_in_lidar_fov(1.0, 0.0, min_range_m=0.5))


class OdomForLidarScanTest(unittest.TestCase):
    def test_frac_zero_is_scan_start(self) -> None:
        msgs = [
            (0, _Odom(0.0, 0.0, 0.0)),
            (100_000_000, _Odom(10.0, 0.0, 0.0)),
        ]
        start = odom_for_lidar_scan(
            msgs, 0, scan_period_ns=100_000_000, center_fraction=0.0
        )
        assert start is not None
        self.assertAlmostEqual(start.pose.pose.position.x, 0.0)

    def test_frac_one_advances_to_scan_end(self) -> None:
        msgs = [
            (0, _Odom(0.0, 0.0, 0.0)),
            (100_000_000, _Odom(10.0, 0.0, 0.0)),
        ]
        end = odom_for_lidar_scan(
            msgs, 0, scan_period_ns=100_000_000, center_fraction=1.0
        )
        assert end is not None
        self.assertAlmostEqual(end.pose.pose.position.x, 10.0)


class OdomAtTimeTest(unittest.TestCase):
    def test_interpolates_position(self) -> None:
        msgs = [
            (0, _Odom(0.0, 0.0, 0.0)),
            (1_000_000_000, _Odom(10.0, 0.0, 0.0)),
        ]
        mid = odom_at_time(msgs, 500_000_000)
        assert mid is not None
        self.assertAlmostEqual(mid.pose.pose.position.x, 5.0)

    def test_interpolated_odom_has_header_stamp(self) -> None:
        msgs = [
            (0, _Odom(0.0, 0.0, 0.0)),
            (1_000_000_000, _Odom(10.0, 0.0, 0.0)),
        ]
        mid = odom_at_time(msgs, 500_000_000)
        assert mid is not None
        self.assertEqual(mid.header.stamp.sec, 0)
        self.assertEqual(mid.header.stamp.nanosec, 500_000_000)

if __name__ == "__main__":
    unittest.main()
