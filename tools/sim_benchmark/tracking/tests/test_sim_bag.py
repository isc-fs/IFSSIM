"""Simulator bag sessions -> bundles: sessions, legacy pairing, provenance and reruns."""

from __future__ import annotations

import json
from pathlib import Path

from bench_tracking import compare, registry
from bench_tracking.adapters import sim_bag

BAG = "/bags/trackdrive_test_bag"


def _perception(d: Path, ts: str, recall: float = 0.6) -> Path:
    rd = d / "perception" / f"base_{ts}"
    rd.mkdir(parents=True)
    (rd / "results.json").write_text(
        json.dumps(
            {
                "bag": BAG,
                "strategy": "base",
                "frames": 3,
                "gt_track_cones": 10,
                "gt_range_m": 20.0,
                "gt_hfov_deg": 60.0,
                "gt_min_range_m": 0.5,
                "gt_scan_period_ms": 100.0,
                "gt_metrics": {
                    "precision": 1.0,
                    "recall": recall,
                    "mean_match_err_m": 0.03,
                    "p95_match_err_m": 0.08,
                    "total_tp": 6,
                    "total_fp": 0,
                    "total_fn": 4,
                },
            }
        )
    )
    (rd / "results.csv").write_text(
        "t_s,latency_ms,n_points,n_cones,n_gt,n_tp,n_fp,n_fn,mean_match_err_m\n"
        "1.0,2.0,8000,2,4,2,0,2,0.02\n1.1,3.0,8000,2,3,2,0,1,0.03\n1.2,2.5,8000,2,3,2,0,1,0.04\n"
    )
    (rd / "frame_details.jsonl").write_text(
        json.dumps(
            {"t_s": 1.0, "match_errs": [0.01, 0.03], "match_ranges_m": [4.0, 9.0]}
        )
        + "\n"
    )
    return rd


def _slam(d: Path, ts: str) -> Path:
    rd = d / "slam" / f"trackdrive_{ts}"
    rd.mkdir(parents=True)
    (rd / "results.json").write_text(
        json.dumps(
            {
                "bag": BAG,
                "strategy": "trackdrive",
                "gt_track_cones": 10,
                "gt_range_m": 20.0,
                "gt_hfov_deg": 60.0,
                "gt_min_range_m": 0.5,
                "gt_scan_period_ms": 100.0,
                "slam_at_gt_rate": {
                    "mean_err_m": 0.1,
                    "p95_err_m": 0.2,
                    "max_err_m": 0.3,
                },
                "map": {
                    "gt_cones": 10,
                    "matched": 8,
                    "slam_landmarks": 9,
                    "false_positive": 1,
                },
                "filter_odom": {"mean_err_m": 0.5, "p95_err_m": 0.9},
            }
        )
    )
    (rd / "trajectory.csv").write_text(
        "t_s,cone_source,gt_x,gt_y,slam_x,slam_y,err_m\n"
        "1.0,gt,0,0,0,0,0.0\n1.5,gt,1,0,1.1,0,0.1\n2.0,gt,2,0,2.2,0,0.2\n"
    )
    (rd / "map_cones.csv").write_text(
        "source,x,y,landmark_id,n_obs,color\ngt,1,2,,,0\ngt,1,-2,,,1\nslam,1.1,2,3,5,\n"
    )
    return rd


def _session(root: Path, ts: str, prov: dict | None) -> Path:
    d = root / "sim_bag" / f"bag_{ts}"
    p, s = _perception(d, ts), _slam(d, ts)
    (d / "session.json").write_text(
        json.dumps(
            {
                "bag": BAG,
                "started_at": f"2026-09-25T{ts[-6:-4]}:{ts[-4:-2]}:00+00:00",
                "requested": ["perception", "slam"],
                "benchmarks": {
                    "perception": {"run_dir": p.relative_to(d).as_posix()},
                    "slam": {"run_dir": s.relative_to(d).as_posix()},
                },
            }
        )
    )
    if prov is not None:
        (d / "provenance.json").write_text(json.dumps(prov))
    return d


def _prov(cid: str, dirty: bool) -> dict:
    return {
        "captured": True,
        "code_id": cid,
        "label": f"a64350a-dirty.{cid}" if dirty else "a64350a",
        "code": {
            "pipeline": {"sha": "a64350a57", "dirty": dirty},
            "ifssim": {"sha": "6210d09", "dirty": False},
        },
    }


def test_session_becomes_one_run(tmp_path: Path) -> None:
    _session(tmp_path, "20260925_100000", _prov("1111", False))
    (b,) = sim_bag.load_all(tmp_path, full_bag_hash=False)
    assert b.job_type == "sim_bag" and b.status == "finished"
    assert b.config["code"]["label"] == "a64350a"
    assert b.group.endswith("@1111")
    assert b.summary["perception/recall_frac"] == 0.6
    assert b.summary["slam/map_recall_frac"] == 0.8
    assert {"gt_perception", "replay_pose"} <= set(b.series)
    # shares the replay time axis, so the viewer's playhead works
    assert b.series["replay_pose"].step_name == "t/replay_s"
    assert {r[3] for r in b.tables["map_cones"].rows if r[2] == "gt"} == {
        "blue",
        "yellow",
    }
    assert not registry.check_names(b.summary, context="test")
    assert any(p.name == "provenance.json" for p, _ in b.files)


def test_reruns_share_a_group_and_code_changes_do_not(tmp_path: Path) -> None:
    _session(tmp_path, "20260925_100000", _prov("1111", False))
    _session(tmp_path, "20260925_110000", _prov("1111", False))
    _session(tmp_path, "20260925_120000", _prov("2222", True))
    bs = sim_bag.load_all(tmp_path, full_bag_hash=False)
    groups = [b.group for b in sorted(bs, key=lambda b: b.started_at)]
    assert groups[0] == groups[1] != groups[2]
    assert len({b.run_key for b in bs}) == 3  # nothing overwrites anything
    assert "dirty" in bs[-1].tags and "dirty" not in bs[0].tags
    base = compare.choose_baselines(bs)
    (only,) = base.values()
    assert only.started_at == min(b.started_at for b in bs)


def test_legacy_runs_are_paired_by_bag_and_time(tmp_path: Path) -> None:
    _perception(tmp_path, "20260923_170906")
    _slam(tmp_path, "20260923_170908")
    _perception(tmp_path, "20260923_190000")  # an hour later: its own session
    ss = sim_bag.legacy_sessions(tmp_path)
    assert [sorted(s.parts) for s in ss] == [["perception", "slam"], ["perception"]]
    b = sim_bag.load_session(ss[0], full_bag_hash=False)
    assert b.config["code"]["label"] == "unknown" and "backfill" in b.tags


def test_missing_benchmark_marks_the_session_failed(tmp_path: Path) -> None:
    d = _session(tmp_path, "20260925_100000", None)
    doc = json.loads((d / "session.json").read_text())
    del doc["benchmarks"]["slam"]
    (d / "session.json").write_text(json.dumps(doc))
    (b,) = sim_bag.load_all(tmp_path, full_bag_hash=False)
    assert b.status == "failed" and "partial" in b.tags
    assert "perception/recall_frac" in b.summary


def test_time_starts_at_zero_whatever_the_simulator_clock(tmp_path: Path) -> None:
    d = _session(tmp_path, "20260925_100000", None)
    # this bag was recorded 12 950 s into the simulator's run
    for p in d.rglob("*.csv"):
        lines = p.read_text().splitlines()
        if lines[0].startswith("t_s,"):
            rows = [
                f"{float(r.split(',', 1)[0]) + 12950},{r.split(',', 1)[1]}"
                for r in lines[1:]
            ]
            p.write_text("\n".join([lines[0], *rows]) + "\n")
    (b,) = sim_bag.load_all(tmp_path, full_bag_hash=False)
    assert b.config["session"]["sim_time_origin_s"] == 12951.0
    assert b.series["gt_perception"].step[0] == 0.0
    # perception and SLAM keep their offset to each other
    assert b.series["replay_pose"].step[0] == 0.0
    assert b.series["replay_pose"].step[-1] == 1.0
