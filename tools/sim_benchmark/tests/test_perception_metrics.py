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
    cone_in_vertical_fov,
    evaluate_frame,
    filter_cones_in_fov,
    gt_cone_range_m,
    latch_track_layout,
    match_range_err_pairs,
    odom_at_time,
    odom_for_lidar_scan,
    summarize_error_by_range,
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


class ErrorByRangeTest(unittest.TestCase):
    def test_gt_cone_range(self) -> None:
        self.assertAlmostEqual(gt_cone_range_m(Cone2D(3.0, 4.0)), 5.0)

    def test_summarize_error_by_range(self) -> None:
        pairs = [(1.0, 0.1), (1.5, 0.3), (5.0, 0.5), (5.5, 0.7)]
        bins = summarize_error_by_range(pairs, bin_width_m=2.0, max_range_m=8.0)
        self.assertEqual(len(bins), 2)
        self.assertAlmostEqual(bins[0]["bin_lo_m"], 0.0)
        self.assertAlmostEqual(bins[0]["mean_err_m"], 0.2)
        self.assertAlmostEqual(bins[1]["bin_lo_m"], 4.0)
        self.assertAlmostEqual(bins[1]["mean_err_m"], 0.6)

    def test_match_range_err_pairs_from_frames(self) -> None:
        fm = evaluate_frame(
            t_s=0.0,
            latency_ms=0.0,
            n_points=100,
            pred=[Cone2D(5.05, 0.0), Cone2D(10.1, 0.0)],
            gt=[Cone2D(5.0, 0.0), Cone2D(10.0, 0.0)],
            gate_m=1.5,
        )
        pairs = match_range_err_pairs([fm])
        self.assertEqual(len(pairs), 2)
        self.assertAlmostEqual(pairs[0][0], 5.0)
        self.assertAlmostEqual(pairs[1][0], 10.0)


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

class VerticalFovGateTest(unittest.TestCase):
    # Hesai-like mount: h=1.1 m, vlower=-12.4° → small cone (0.35 m) blind below
    # ~3.4 m, visible beyond.
    H = 1.1
    VLO = -12.4

    def _vis(self, r: float, ch: float = 0.35) -> bool:
        return cone_in_vertical_fov(
            r, ch, lidar_height_m=self.H, vfov_lower_deg=self.VLO
        )

    def test_near_cone_below_lowest_beam_is_blind(self) -> None:
        self.assertFalse(self._vis(2.0))

    def test_far_cone_is_visible(self) -> None:
        self.assertTrue(self._vis(10.0))

    def test_blind_radius_matches_geometry(self) -> None:
        r_edge = (self.H - 0.35) / math.tan(math.radians(-self.VLO))
        self.assertFalse(self._vis(r_edge - 0.2))
        self.assertTrue(self._vis(r_edge + 0.2))

    def test_big_cone_sees_closer_than_small(self) -> None:
        # Taller cone clears the lowest beam at a shorter range.
        r = 2.7
        self.assertFalse(cone_in_vertical_fov(r, 0.35, lidar_height_m=self.H, vfov_lower_deg=self.VLO))
        self.assertTrue(cone_in_vertical_fov(r, 0.55, lidar_height_m=self.H, vfov_lower_deg=self.VLO))

    def test_gate_drops_near_cones_in_world_transform(self) -> None:
        odom = _Odom(0.0, 0.0, 0.0)
        cones = [WorldCone(2.0, 0.0, color=1), WorldCone(10.0, 0.0, color=1)]
        gated = world_cones_to_body(
            cones, odom, lidar_height_m=self.H, vfov_lower_deg=self.VLO
        )
        ungated = world_cones_to_body(cones, odom)
        self.assertEqual(len(ungated), 2)
        self.assertEqual([round(c.x) for c in gated], [10])


if __name__ == "__main__":
    unittest.main()
