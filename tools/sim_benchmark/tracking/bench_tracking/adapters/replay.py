"""Onboard bag replay run dirs -> RunBundle.

Handles both kinds of dir that ``run_onboard_replay.py`` leaves in
``results/onboard/<mission>_<ts>/``:

* ``--report`` runs: results.json + perception/trajectory/control/map_cones
  CSVs + samples.json + report.html + logs/  -> job_type ``onboard_replay``
* ``--live`` runs without ``--report``: only logs/ (+ foxglove_params.yaml)
  -> job_type ``live_replay``. Everything comes from the node logs.

Both get the log-derived pipeline-health data (logparse). Nothing is re-run.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from .. import provenance
from ..bundle import SCHEMA_VERSION, RunBundle, Series, Table
from ..geometry import interp, interp_angle
from ..logparse import parse_logs

# Kept in sync with run_onboard_replay.MISSION_BEHAVIORS (feat/516). Duplicated
# on purpose: the tracker must not import the car submodule.
MISSION_BEHAVIORS = {
    "trackdrive": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "trackdrive",
        "path_planning_node": "trackdrive",
        "control_node": "pure_pursuit",
    },
    "autocross": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "autocross",
        "path_planning_node": "autocross",
        "control_node": "stanley",
    },
    "accel": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "accel",
        "path_planning_node": "accel",
        "control_node": "pure_pursuit",
    },
    "skidpad": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "skidpad",
        "path_planning_node": "skidpad",
        "control_node": "stanley",
    },
}

# results.json key -> canonical metric (see metrics.yaml)
RESULTS_MAP = {
    "mean_conos_raw": "perception/n_cones_mean",
    "median_conos_raw": "perception/n_cones_median",
    "max_conos_raw": "perception/n_cones_max",
    "empty_detection_rate": "perception/empty_frame_rate_frac",
    "n_conos_raw": "perception/n_scans",
    "odom_path_length_m": "odom/path_length_m",
    "slam_path_length_m": "slam/path_length_m",
    "odom_slam_end_gap_m": "slam/end_gap_vs_odom_m",
    "mean_abs_steer_residual_rad": "control/steer_residual_mean_abs_rad",
    "n_map_cones": "slam/n_map_cones",
    "last_path_poses": "planning/n_last_path_poses",
}


def _started_at(run_dir: Path) -> datetime:
    try:
        ts = run_dir.name.rsplit("_", 2)
        return datetime.strptime(f"{ts[-2]}_{ts[-1]}", "%Y%m%d_%H%M%S").replace(
            tzinfo=timezone.utc
        )
    except (ValueError, IndexError):
        return datetime.fromtimestamp(run_dir.stat().st_mtime, tz=timezone.utc)


def _host_bag(container_path: str | None, results_root: Path) -> Path | None:
    """results.json stores container paths (/results/...). Map back to the host."""
    if not container_path:
        return None
    p = Path(container_path)
    if p.parts[:2] == ("/", "results"):
        return results_root.joinpath(*p.parts[2:])
    return p if p.exists() else None


def _infer_bag(
    results_root: Path, replay_duration_s: float | None
) -> tuple[Path | None, str]:
    """Live runs don't record the bag. Pick it by duration if unambiguous."""
    caps = [
        d
        for d in (results_root / "capture").glob("*")
        if (d / "metadata.yaml").is_file()
    ]
    if not caps:
        return None, "no capture bags on disk"
    if len(caps) == 1:
        return caps[0], "only bag under results/capture"
    if replay_duration_s:
        best = min(
            caps,
            key=lambda d: abs(
                provenance.bag_metadata(d)["duration_s"] - replay_duration_s
            ),
        )
        return best, f"closest duration to replay ({replay_duration_s:.1f} s)"
    return None, "ambiguous"


def _arr(rows: list[dict], key: str) -> np.ndarray:
    return np.array(
        [r.get(key, math.nan) if r.get(key) is not None else math.nan for r in rows],
        dtype=float,
    )


def load_replay(
    run_dir: Path, results_root: Path, *, full_bag_hash: bool = True
) -> RunBundle:
    run_dir = Path(run_dir)
    results_json = run_dir / "results.json"
    has_report = results_json.is_file()
    res: dict[str, Any] = json.loads(results_json.read_text()) if has_report else {}
    mission = res.get("mission") or run_dir.name.rsplit("_", 2)[0]
    logs = parse_logs(run_dir / "logs") if (run_dir / "logs").is_dir() else None
    live = (run_dir / "foxglove_params.yaml").is_file()

    # ---------------------------------------------------------------- bag / scenario
    replay_dur = logs.summary.get("run/replay_duration_s") if logs else None
    if has_report:
        bag_dir, bag_how = _host_bag(res.get("bag"), results_root), "results.json"
    else:
        bag_dir, bag_how = _infer_bag(results_root, replay_dur)
    bag: dict[str, Any] = {"name": None, "id": None, "recorded_by": bag_how}
    if bag_dir and bag_dir.is_dir():
        meta = provenance.bag_metadata(bag_dir)
        bag.update(provenance.bag_identity(bag_dir, full_hash=full_bag_hash))
        bag.update(
            {
                "path": str(bag_dir),
                "duration_s": meta.get("duration_s"),
                "storage": meta.get("storage"),
                "n_topics": len(meta.get("topics", {})),
                "recorded_at": datetime.fromtimestamp(
                    meta["start_ns"] / 1e9, tz=timezone.utc
                ).isoformat()
                if meta.get("start_ns")
                else None,
                "topic_counts": {
                    k: v
                    for k, v in meta.get("topics", {}).items()
                    if k in ("/imu", "/lidar_points", "/motor_rpm", "/steering_angle")
                },
            }
        )
    bag["inferred"] = not has_report
    rate = float(res.get("rate", 1.0))
    clip_s = float(res.get("duration_s", 0.0))  # requested clip, 0 = full bag
    scen_fields = {
        "kind": "bag",
        "bag_id": bag.get("id"),
        "mission": mission,
        "rate": rate,
        "clip_s": clip_s,
    }
    sid = provenance.scenario_id(scen_fields)

    job_type = "onboard_replay" if has_report else "live_replay"
    played = bool(logs and logs.played)
    status = "finished" if played and (has_report or replay_dur) else "aborted"
    started = _started_at(run_dir)
    short_bag = (bag.get("name") or "unknown_bag").replace("_indexed", "")
    code = provenance.unknown_code()
    group = f"{sid}@{code['pipeline']['sha'] or 'unknown'}"

    tags = ["backfill", f"mission:{mission}"]
    if live:
        tags.append("live")
    if not has_report:
        tags.append("logs-only")
    if bag["inferred"]:
        tags.append("bag-inferred")
    if status != "finished":
        tags.append(status)

    b = RunBundle(
        job_type=job_type,
        name=f"{'onboard' if has_report else 'live'}/{short_bag}/{started:%m%dT%H%M%S}",
        group=group,
        source_dir=run_dir,
        started_at=started,
        status=status,
        tags=tags,
        notes=(
            "Imported from an existing run dir: code SHAs, params and image are unknown "
            "(recorded before provenance capture existed)."
        ),
    )
    b.config = {
        "schema_version": SCHEMA_VERSION,
        "job_type": job_type,
        "code": code,
        "scenario": {
            "id": sid,
            **scen_fields,
            "bag": bag,
            "behaviors": MISSION_BEHAVIORS.get(mission, {}),
            "live_viewer": live,
        },
        "params": {},
        "env": {
            "imported_by": provenance.env_info(),
            "started_at": started.isoformat(),
            "run_dir": run_dir.name,
            "log_time_base": logs.t0_source if logs else None,
        },
        "provenance": {
            "complete": False,
            "missing": ["code", "params", "image"]
            + ([] if has_report else ["bag (inferred)"]),
        },
    }

    S = b.summary
    if logs:
        S.update(logs.summary)
        for s in logs.series.values():
            b.add_series(s)
        b.events.extend(logs.events)
        if logs.lifecycle:
            b.add_table(logs.lifecycle)
        if logs.warn_catalogue:
            b.add_table(logs.warn_catalogue)
    S["run/completed"] = 1 if status == "finished" else 0
    if bag.get("duration_s") and replay_dur:
        S["run/replay_coverage_frac"] = min(1.0, replay_dur / bag["duration_s"])

    if has_report:
        for k, canon in RESULTS_MAP.items():
            if res.get(k) is not None:
                S[canon] = res[k]
        _add_report_data(b, run_dir, res)

    # files
    for p in sorted(run_dir.glob("*.csv")):
        b.files.append((p, "data"))
    if results_json.is_file():
        b.files.append((results_json, "data"))
    for p in (
        sorted((run_dir / "logs").glob("*.log")) if (run_dir / "logs").is_dir() else []
    ):
        b.files.append((p, "log"))
    if (run_dir / "foxglove_params.yaml").is_file():
        b.files.append((run_dir / "foxglove_params.yaml", "config"))
    if (run_dir / "report.html").is_file():
        b.html_report = run_dir / "report.html"
        b.files.append((run_dir / "report.html", "report"))
    return b


def _add_report_data(b: RunBundle, run_dir: Path, res: dict[str, Any]) -> None:
    """Series/tables from samples.json (superset of the CSVs, same t_s base)."""
    sp = run_dir / "samples.json"
    if not sp.is_file():
        return
    smp = json.loads(sp.read_text())
    S = b.summary
    duration = float(res.get("source_duration_s") or 0) or None

    # -- perception: raw and filtered cone counts (different stamps -> 2 families)
    raw = smp.get("conos_raw") or []
    if raw:
        t, n = _arr(raw, "t_s"), _arr(raw, "n")
        b.add_series(Series("replay_perception", "t/replay_s", t, {"n_cones_raw": n}))
        if len(t) > 1:
            S["perception/output_rate_hz"] = (len(t) - 1) / float(t[-1] - t[0])
    filt = smp.get("conos") or []
    if filt:
        b.add_series(
            Series(
                "replay_perception_filtered",
                "t/replay_s",
                _arr(filt, "t_s"),
                {"n_cones_filtered": _arr(filt, "n")},
            )
        )

    # -- poses
    odom, slam = smp.get("odom") or [], smp.get("slam") or []
    if odom:
        to, ox, oy, oyaw = (_arr(odom, k) for k in ("t_s", "x", "y", "yaw"))
        dt = np.gradient(to)
        speed = np.hypot(np.gradient(ox), np.gradient(oy)) / np.where(
            dt > 0, dt, np.nan
        )
        vals = {"odom_x": ox, "odom_y": oy, "odom_yaw": oyaw, "odom_speed_mps": speed}
        if slam:
            ts, sx, sy, syaw = (_arr(slam, k) for k in ("t_s", "x", "y", "yaw"))
            ix, iy = interp(ts, sx, to), interp(ts, sy, to)
            iyaw = interp_angle(ts, syaw, to)
            gap = np.hypot(ix - ox, iy - oy)
            dyaw = np.degrees(np.angle(np.exp(1j * (iyaw - oyaw))))
            vals.update(
                {
                    "slam_x": ix,
                    "slam_y": iy,
                    "slam_yaw": iyaw,
                    "slam_odom_gap_m": gap,
                    "slam_odom_dyaw_deg": dyaw,
                }
            )
            S["slam/odom_divergence_max_m"] = (
                float(np.nanmax(gap)) if np.isfinite(gap).any() else None
            )
        b.add_series(Series("replay_pose", "t/replay_s", to, vals))
        S["control/speed_mean_mps"] = float(np.nanmean(speed))
        S["control/speed_max_mps"] = float(np.nanpercentile(speed, 99))

        # trajectory table: long format (one row per point per source), for XY overlays
        rows = [
            [float(a), "odom", float(x), float(y), float(w)]
            for a, x, y, w in zip(to, ox, oy, oyaw)
        ]
        if slam:
            rows += [
                [float(a), "slam", float(x), float(y), float(w)]
                for a, x, y, w in zip(ts, sx, sy, syaw)
            ]
        b.add_table(
            Table(
                "trajectory",
                ["t_s", "source", "x", "y", "yaw"],
                rows,
                "Odom and SLAM poses in the odom/map frame (same-bag runs overlay directly)",
            )
        )

    # -- map
    mx, my = smp.get("map_x") or [], smp.get("map_y") or []
    if mx:
        b.add_table(
            Table(
                "map_cones",
                ["x", "y", "source", "color"],
                [[float(x), float(y), "slam", "unknown"] for x, y in zip(mx, my)],
                "Final SLAM landmarks",
            )
        )

    # -- control
    cmd = smp.get("cmd") or []
    if cmd:
        tc = _arr(cmd, "t_s")
        thr, st, br = _arr(cmd, "throttle"), _arr(cmd, "steering"), _arr(cmd, "brake")
        vals = {"throttle": thr, "steering_rad": st, "brake": br}
        pilot = smp.get("steering") or []
        if pilot:
            tp, rp = _arr(pilot, "t_s"), _arr(pilot, "rad")
            ip = interp(tp, rp, tc)
            vals["pilot_steering_rad"] = ip
            resid = st - ip
            vals["steer_residual_rad"] = resid
            S["control/steer_residual_p95_rad"] = float(
                np.nanpercentile(np.abs(resid), 95)
            )
        dt = np.diff(tc)
        rate = np.diff(st) / np.where(dt > 0, dt, np.nan)
        vals["steer_rate_radps"] = np.r_[0.0, rate]
        b.add_series(Series("replay_control", "t/replay_s", tc, vals))
        S["control/throttle_mean_frac"] = float(np.nanmean(thr))
        S["control/brake_mean_frac"] = float(np.nanmean(br))
        S["control/steer_rate_rms_radps"] = float(np.sqrt(np.nanmean(rate**2)))
        if len(tc) > 1:
            S["control/cmd_rate_hz"] = (len(tc) - 1) / float(tc[-1] - tc[0])

    # -- planning
    path = smp.get("path") or []
    if path:
        b.add_series(
            Series(
                "replay_planning",
                "t/replay_s",
                _arr(path, "t_s"),
                {"path_poses": _arr(path, "n")},
            )
        )
        S["planning/path_poses_mean"] = float(np.nanmean(_arr(path, "n")))

    # -- stop-latch event position comes from the logs; nothing else discrete here
    if duration:
        b.config["scenario"]["source_duration_s"] = duration
