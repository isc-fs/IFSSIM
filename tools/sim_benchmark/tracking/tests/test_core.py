"""Backend-free tests: log parsing, registry deltas, geometry, mock sim → aggregate → compare."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from bench_tracking import compare, registry
from bench_tracking.adapters.sim import aggregate, group_runs, load_sim_run
from bench_tracking.bundle import Series, Table
from bench_tracking.geometry import align_start_pose, fit_cones, path_length, rigid
from bench_tracking.logparse import parse_logs

REPO = Path(__file__).resolve().parents[4]
TRACKS = REPO / "Content" / "tracks"

SLAM_LOG = """\
[INFO] [1000.000000000] [slam_node]: Ready: mode=trackdrive behavior=trackdrive
[INFO] [1010.000000000] [slam_node]: Configured | mode: trackdrive | behavior: trackdrive
[INFO] [1011.000000000] [slam_node]: Activated in mode: trackdrive
[INFO] [1021.000000000] [slam_node]: SLAM_LAT proc=   2.0ms age=  300.0ms dt_pub= 100.0ms corr=(+0.00,+0.00|0.00m,+0.0deg)
[INFO] [1021.100000000] [slam_node]: SLAM_PROF[ok] total=   3.0ms pre=  0.5 assoc=  0.8 spawn=  0.2 commit=   0.4(upd=   0.3 est=   0.0 x1) db=  0.0 pub=  1.0 obs=11 map=13
[INFO] [1031.000000000] [slam_node]: SLAM_PROF[loc] total=   5.0ms pre=  0.5 assoc=  0.8 spawn=  nan commit=   0.4(upd=   0.3 est=   0.0 x1) db=  0.0 pub=  1.0 obs=11 map=20
[WARN] [1032.000000000] [slam_node]: localize pose-jump rejected: dev=(1.11 m, 4.2°) > (0.80 m, 17.2°) — holding motion prediction
[WARN] [1033.000000000] [slam_node]: skip cone factors: DA-failure spike [pct>60%] (obs=16 new=13 assoc=3) — EKF-odom-only update
[INFO] [1034.000000000] [slam_node]: SLAM_OBS (avg/scan over 4): obs=10.0 assoc= 8.0 new=0.5 vetoed=0.0 frozen=0.0 skip=0.0
Traceback (most recent call last):
  File "x.py", line 1, in main
rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the given context
"""
CONE_LOG = """\
[INFO] [1001.000000000] [cone_detection_node]: Ready: mode=trackdrive behavior=base
[INFO] [1012.000000000] [cone_detection_node]: Activated in mode: trackdrive
[INFO] [1025.000000000] [cone_detection_node]: CONE_FILTER (avg/scan over 3): pts=172811 clusters=42.7 -> >3pts=36.7 -> shape=16.7 -> residual=11.3 accepted=11.3 far_dropped=7.3 by-side: L= 8.7 R= 2.7 C=0.0 BO=0.0 dbscan_guard=3/3 hz= 2.4
[WARN] [1026.000000000] [cone_detection_node]: DBSCAN guard: above-ground cloud too large/dense, dropped 17361 pts before clustering (car off-track or ground plane mis-fit?)
Traceback (most recent call last):
  File "y.py", line 3, in cb
ValueError: real crash
"""
CTRL_LOG = """\
[INFO] [1002.000000000] [control_node]: Ready: mode=trackdrive behavior=pure_pursuit
[INFO] [1027.000000000] [control_node]: v=3.10 travelled=12.5m -> thr=+0.200 regen=+0.000 steer=+0.154 | path_n=24 path_len=11.6m stop_d=inf latched=False
[INFO] [1040.000000000] [control_node]: stop latched at odom=(-20.05, 4.77) from 2 big-orange cones
"""


@pytest.fixture()
def log_dir(tmp_path: Path) -> Path:
    d = tmp_path / "logs"
    d.mkdir()
    (d / "slam_node.log").write_text(SLAM_LOG)
    (d / "cone_detection_node.log").write_text(CONE_LOG)
    (d / "control_node.log").write_text(CTRL_LOG)
    (d / "play.log").write_text(
        "stdin is not a terminal device.[INFO] [1020.000000000] [rosbag2_player]: Set rate to 1"
    )
    return d


def test_parse_logs_time_base_and_series(log_dir: Path) -> None:
    g = parse_logs(log_dir)
    assert g.t0_source == "bag_play_start" and g.t0_wall == pytest.approx(1020.0)
    assert g.series["log_slam_latency"].step[0] == pytest.approx(1.0)
    assert g.series["log_perception_filter"].values["accepted"][0] == pytest.approx(
        11.3
    )
    assert g.series["log_control_status"].values["v_mps"][0] == pytest.approx(3.10)
    prof = g.series["log_slam_profile"]
    assert list(prof.values["mode"]) == [0.0, 1.0]
    assert math.isnan(prof.values["spawn_ms"][1])


def test_parse_logs_counts_and_crashes(log_dir: Path) -> None:
    s = parse_logs(log_dir).summary
    assert s["slam/n_pose_jump_rejected"] == 1
    assert s["slam/pose_jump_dev_max_m"] == pytest.approx(1.11)
    assert s["slam/n_da_failure_skips"] == 1
    assert s["perception/n_dbscan_guard_trips"] == 1
    assert s["control/stop_latched"] == 1
    assert s["slam/t_first_loc_s"] == pytest.approx(11.0)
    # shutdown noise is not a crash, the ValueError is
    assert s["run/n_crashes"] == 1
    assert s["slam/assoc_frac"] == pytest.approx(0.8)


def test_parse_logs_lifecycle(log_dir: Path) -> None:
    life = parse_logs(log_dir).lifecycle
    row = next(r for r in life.rows if r[0] == "slam_node")
    assert row[1] == pytest.approx(-20.0) and row[3] == pytest.approx(-9.0)


def test_registry_delta_direction() -> None:
    d = registry.delta("race/lap_time_mean_s", 30.0, 29.0)  # min is better, tol 0.3
    assert d["regression"] and not d["improvement"]
    d = registry.delta("perception/recall_frac", 0.70, 0.60)  # max is better
    assert d["improvement"] and not d["regression"]
    d = registry.delta("race/lap_time_mean_s", 29.1, 29.0)  # within tolerance
    assert not d["regression"] and not d["improvement"]
    assert registry.delta("perception/n_cones_mean", 1, 2)["regression"] is None


def test_registry_names_follow_convention() -> None:
    units = (
        "_m",
        "_s",
        "_ms",
        "_rad",
        "_deg",
        "_mps",
        "_hz",
        "_frac",
        "_radps",
        "_mps2",
    )
    for name, spec in registry.registry().items():
        dom, q = name.split("/", 1)
        assert dom in {
            "perception",
            "slam",
            "odom",
            "planning",
            "control",
            "race",
            "latency",
            "compute",
            "run",
            "compare",
        }, name
        assert spec.direction in ("min", "max", "none"), name
        if spec.unit not in ("count", "bool"):
            assert q.endswith(units) or spec.unit in ("frac",), name


def test_series_decimation_keeps_last() -> None:
    s = Series("f", "t/x_s", np.arange(10_000.0), {"v": np.arange(10_000.0)})
    d = s.decimated(100)
    assert len(d) <= 101 and d.step[-1] == 9999.0


def test_table_decimation_per_group() -> None:
    t = Table(
        "t",
        ["source", "x"],
        [["a", i] for i in range(50)] + [["b", i] for i in range(5)],
    )
    d = t.decimated(10, by="source")
    assert (
        sum(1 for r in d.rows if r[0] == "a") <= 10
        and sum(1 for r in d.rows if r[0] == "b") == 5
    )


def test_geometry_alignment_roundtrip() -> None:
    t = np.linspace(0, 2 * np.pi, 200)
    x, y = 10 * np.cos(t), 5 * np.sin(t)
    xr, yr = rigid(x, y, 0.7, 3.0, -2.0)
    assert path_length(x, y) == pytest.approx(path_length(xr, yr), rel=1e-9)
    ax, ay = align_start_pose(xr, yr)
    assert ax[0] == pytest.approx(0) and ay[0] == pytest.approx(0)
    th, tx, ty, rms = fit_cones(np.c_[xr, yr][::5], np.c_[x, y][::5])
    assert rms < 1e-6


@pytest.mark.skipif(
    not (TRACKS / "track_20260404_013723.csv").is_file(),
    reason="track CSVs not present",
)
def test_mock_sim_to_aggregate_and_compare(tmp_path: Path) -> None:
    from dataclasses import replace

    from bench_tracking import mock_sim as m

    sc = replace(m.SCENARIOS[0], laps=2)
    runs = []
    for cm in (m.BASELINE, m.CANDIDATE):
        for seed in (1, 2):
            sim = m.simulate(sc, cm, seed, TRACKS)
            tags = ["mock", "matrix"] + (["baseline"] if cm is m.BASELINE else [])
            runs.append(
                m.write_run(
                    tmp_path / cm.sha / f"s{seed}",
                    sc,
                    cm,
                    seed,
                    sim,
                    started_at=m.datetime(2026, 1, 1, tzinfo=m.timezone.utc),
                    tags=tags,
                )
            )
    seeds = [load_sim_run(r) for r in runs]
    aggs = [aggregate(v) for v in group_runs(seeds).values()]
    assert len(aggs) == 2
    assert {a.summary["run/n_seeds"] for a in aggs} == {2}
    base = compare.choose_baselines(aggs)
    compare.apply(aggs, base)
    cand = next(a for a in aggs if "baseline" not in a.tags)
    assert "baseline_comparison" in cand.tables
    assert "race/lap_time_mean_s.delta" in cand.summary
    assert "agg_track_profile_delta" in cand.series
    registry.check_names(cand.summary, context="test")


def test_bundle_store_roundtrip(tmp_path: Path) -> None:
    """The bundle/ artifact the viewer reads must give back the same run, NaNs and mixed columns included."""
    from datetime import datetime, timezone

    from bench_tracking.bundle import Event, RunBundle
    from bench_tracking.store import read_bundle, write_bundle

    b = RunBundle(
        "onboard_replay",
        "onboard/bag/run1",
        "sid@x",
        None,
        datetime(2026, 9, 21, tzinfo=timezone.utc),
        tags=["baseline"],
        config={"scenario": {"id": "sid", "bag": {"path": "/x"}}},
        summary={
            "slam/n_map_cones": 72,
            "slam/end_gap_vs_odom_m": float("nan"),
            "note": "text",
        },
    )
    b.add_series(
        Series(
            "replay_pose",
            "t/replay_s",
            np.array([0.0, 0.1, 0.2]),
            {"odom_x": np.array([0.0, np.nan, 1.0])},
        )
    )
    b.add_table(
        Table(
            "lifecycle",
            ["node", "ready_s"],
            [["slam_node", -3.5], ["control_node", None]],
        )
    )
    b.add_table(Table("mixed", ["v"], [[1], ["two"], [None]]))
    b.events = [
        Event(1.5, "slam_node", "pose_jump_rejected", "warn", "dev 2 m", None, 3.0, 4.0)
    ]
    r = read_bundle(write_bundle(b, tmp_path / "bundle"))
    assert (r.name, r.status, r.tags, r.config) == (b.name, b.status, b.tags, b.config)
    assert (
        r.summary["slam/n_map_cones"] == 72
        and r.summary["slam/end_gap_vs_odom_m"] is None
    )
    s = r.series["replay_pose"]
    assert (
        s.step_name == "t/replay_s"
        and s.step.tolist() == [0.0, 0.1, 0.2]
        and np.isnan(s.values["odom_x"][1])
    )
    assert r.tables["lifecycle"].rows == [["slam_node", -3.5], ["control_node", None]]
    assert r.tables["mixed"].column("v") == ["1", "two", None]
    assert r.events == b.events
