"""Simulator bag benchmark sessions -> RunBundle (``job_type`` ``sim_bag``).

A session is what ``run_sim_bag_benchmark.py`` leaves in
``results/sim_bag/<bag>_<ts>/``: ``session.json``, ``provenance.json`` and one
sub-folder per benchmark (``perception/<strategy>_<ts>/``,
``slam/<strategy>_<ts>/``), each with the benchmark's usual ``results.json``,
CSVs and ``report.html``. One session becomes one run: perception and SLAM
of the same bag at the same code, scored against the simulator's ground truth.

Perception and SLAM runs made one at a time are imported too, each as a
session of its own. Runs from before the session runner (which recorded no
code) are paired instead: :func:`legacy_sessions` pairs a perception and a
SLAM run of the same bag started within a few minutes of each other.

Both benchmarks stamp their samples with the simulator's clock, which starts
wherever the simulator happened to be when the bag was recorded (35 s for one
bag, 12 952 s for another). Every session is shifted so its earliest sample is
at 0, which puts sessions of different bags on one axis. Perception and SLAM of
a session are shifted by the same amount, so they stay aligned with each other,
and their series share the bag replays' time axis (``t/replay_s``), so the
viewer's playhead works across them.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import numpy as np

from .. import provenance
from ..bundle import SCHEMA_VERSION, RunBundle, Series, Table
from ..geometry import interp

JOB = "sim_bag"
# fs_msgs/Cone colour codes
CONE_COLOUR = {0: "blue", 1: "yellow", 2: "big_orange", 3: "orange"}
PAIR_WINDOW = timedelta(minutes=5)


@dataclass
class Session:
    """Where the parts of one session live. ``root`` is None for a paired legacy session."""

    root: Path | None
    parts: dict[str, Path] = field(default_factory=dict)  # benchmark -> run dir
    doc: dict[str, Any] = field(default_factory=dict)  # session.json

    @property
    def key_dir(self) -> Path:
        return self.root or next(iter(self.parts.values()))


# ---------------------------------------------------------------- finding sessions
def _started(run_dir: Path) -> datetime:
    try:
        d, t = run_dir.name.rsplit("_", 2)[-2:]
        return datetime.strptime(f"{d}_{t}", "%Y%m%d_%H%M%S").replace(
            tzinfo=timezone.utc
        )
    except ValueError:
        return datetime.fromtimestamp(run_dir.stat().st_mtime, tz=timezone.utc)


def _results(run_dir: Path) -> dict[str, Any]:
    p = run_dir / "results.json"
    try:
        return json.loads(p.read_text()) if p.is_file() else {}
    except ValueError:
        return {}


def find_sessions(root: Path) -> list[Session]:
    """Every session under ``root``: the folder itself, or ``root/sim_bag/*`` / ``root/*``."""
    root = Path(root)
    cands = [root] if (root / "session.json").is_file() else []
    for d in (root / "sim_bag", root):
        if d.is_dir():
            cands += [p for p in sorted(d.iterdir()) if (p / "session.json").is_file()]
    out, seen = [], set()
    for d in cands:
        if d.resolve() in seen:
            continue
        seen.add(d.resolve())
        doc = json.loads((d / "session.json").read_text())
        if provenance.in_progress(d, finished=doc.get("status") != "running"):
            print(f"  skipping {d.name}: still running")
            continue
        parts = {}
        for name, info in (doc.get("benchmarks") or {}).items():
            if info.get("run_dir") and (d / info["run_dir"] / "results.json").is_file():
                parts[name] = d / info["run_dir"]
        out.append(Session(d, parts, doc))
    return out


def legacy_sessions(results_root: Path) -> list[Session]:
    """``results/perception/*`` and ``results/slam/*`` runs made one at a time, as sessions.

    Runs that recorded their code (``provenance.json``) are one session each. Older
    runs are paired: same bag, started within ``PAIR_WINDOW``. Unpaired runs become
    one-benchmark sessions."""
    runs: list[tuple[str, Path, str, datetime]] = []
    for name in ("perception", "slam"):
        d = Path(results_root) / name
        if not d.is_dir():
            continue
        for rd in sorted(p for p in d.iterdir() if p.is_dir()):
            res = _results(rd)
            if not res or not res.get("gt_track_cones"):
                continue  # not scored against ground truth
            runs.append((name, rd, Path(str(res.get("bag", ""))).name, _started(rd)))
    runs.sort(key=lambda r: r[3])
    out: list[Session] = []
    for name, rd, bag, t in runs:
        if (provenance.read_run(rd) or {}).get("captured"):
            out.append(
                Session(None, {name: rd}, {"bag_name": bag, "t": t, "single": True})
            )
            continue
        home = next(
            (
                s
                for s in reversed(out)
                if name not in s.parts
                and not s.doc.get("single")
                and s.doc["bag_name"] == bag
                and abs(t - s.doc["t"]) <= PAIR_WINDOW
            ),
            None,
        )
        if home is None:
            home = Session(None, {}, {"bag_name": bag, "t": t})
            out.append(home)
        home.parts[name] = rd
    return out


# ---------------------------------------------------------------- reading
def _csv(path: Path) -> dict[str, np.ndarray]:
    """Numeric columns of a CSV as float arrays (non-numbers -> NaN)."""
    if not path.is_file() or path.stat().st_size == 0:
        return {}
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return {}
    out = {}
    for k in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[k]))
            except (TypeError, ValueError):
                vals.append(math.nan)
        out[k] = np.array(vals)
    return out


def _col(path: Path, name: str) -> list[str]:
    if not path.is_file() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as fh:
        return [r.get(name, "") for r in csv.DictReader(fh)]


def _first_t(path: Path) -> float | None:
    """Earliest ``t_s`` in a benchmark CSV."""
    t = _csv(path).get("t_s")
    t = t[np.isfinite(t)] if t is not None else None
    return float(t.min()) if t is not None and t.size else None


def time_origin(parts: dict[str, Path]) -> float:
    """Simulator time of the session's earliest sample; subtracted from every ``t_s``."""
    ts = [
        t
        for d in parts.values()
        for name in ("results.csv", "trajectory.csv", "pose_steps.csv")
        if (t := _first_t(d / name)) is not None
    ]
    return min(ts) if ts else 0.0


def _get(d: dict, *keys: str) -> float | None:
    for k in keys:
        d = d.get(k) if isinstance(d, dict) else None
    return float(d) if isinstance(d, (int, float)) and math.isfinite(d) else None


def _host_path(p: str | None, results_root: Path | None) -> Path | None:
    """results.json holds container paths (/bags/..., /results/...)."""
    if not p:
        return None
    q = Path(p)
    if q.parts[:2] == ("/", "results") and results_root is not None:
        return results_root.joinpath(*q.parts[2:])
    return q if q.exists() else None


# ---------------------------------------------------------------- one session
def load_session(s: Session, *, full_bag_hash: bool = True) -> RunBundle:
    res = {name: _results(d) for name, d in s.parts.items()}
    first = next(iter(res.values()), {})
    bag_str = s.doc.get("bag") or first.get("bag") or ""
    bag_name = Path(bag_str).name.replace("_indexed", "") or "unknown_bag"

    # the session's own record, else the first benchmark's (both are written at run time)
    prov = (provenance.read_run(s.root) if s.root else None) or next(
        (provenance.read_run(d) for d in s.parts.values() if provenance.read_run(d)),
        None,
    )
    code = provenance.code_of(prov)

    gate = {
        k: first.get(k)
        for k in ("gt_range_m", "gt_hfov_deg", "gt_min_range_m", "gt_scan_period_ms")
    }
    strategies = {name: r.get("strategy") for name, r in res.items()}
    bag: dict[str, Any] = {"name": bag_name, "id": None, "path": bag_str}
    bag_dir = _host_path(bag_str, None)
    if bag_dir is not None and bag_dir.is_dir():
        bag.update(provenance.bag_identity(bag_dir, full_hash=full_bag_hash))
    scen_fields = {
        "kind": JOB,
        "bag": bag.get("id") or bag_name,
        **gate,
        **{f"strategy_{k}": v for k, v in sorted(strategies.items())},
    }
    sid = provenance.scenario_id(scen_fields)

    started = (
        datetime.fromisoformat(s.doc["started_at"])
        if s.doc.get("started_at")
        else min(_started(d) for d in s.parts.values())
    )
    requested = s.doc.get("requested") or sorted(s.parts)
    missing = [n for n in requested if n not in s.parts]
    # a session missing a benchmark it asked for is shown, but marked as failed
    status = "finished" if s.parts and not missing else "failed"

    tags = [f"bag:{bag_name}"] + sorted(f"benchmark:{n}" for n in s.parts)
    if not code["id"]:
        tags.append("backfill")
    if code["pipeline"].get("dirty") or code["ifssim"].get("dirty"):
        tags.append("dirty")
    if code["pipeline"].get("unpushed") or code["ifssim"].get("unpushed"):
        tags.append("unpushed")
    if s.root is None and not s.doc.get("single"):
        tags.append("paired")
    if missing:
        tags.append("partial")
    if code["spec_id"] != provenance.DEFAULT_SPEC:
        tags.append(f"spec:{code['spec_id']}")

    b = RunBundle(
        job_type=JOB,  # type: ignore[arg-type]
        name=f"simbag/{bag_name}/{code['label']}/{started:%m%dT%H%M%S}",
        group=f"{sid}@{provenance.variant_of(code)}",
        source_dir=s.key_dir,
        started_at=started,
        status=status,  # type: ignore[arg-type]
        tags=tags,
        notes=(
            f"Simulator bag {bag_name}: {', '.join(sorted(s.parts)) or 'nothing'} vs ground truth, "
            f"code {code['label']}."
            + (f" Missing: {', '.join(missing)}." if missing else "")
            + (
                " Paired from separate perception/SLAM runs (made before the session runner)."
                if s.root is None and not s.doc.get("single")
                else ""
            )
        ),
    )
    b.config = {
        "schema_version": SCHEMA_VERSION,
        "job_type": JOB,
        "code": code,
        "scenario": {"id": sid, "name": bag_name, **scen_fields, "bag": bag},
        "params": {
            name: {
                k: v
                for k, v in r.items()
                if not isinstance(v, (dict, list))
                and k not in ("bag", "csv", "report", "map_csv", "pose_steps_csv")
            }
            for name, r in res.items()
        },
        "session": {
            "dir": s.root.name if s.root else None,
            "requested": requested,
            "missing": missing,
            "benchmarks": {
                n: {
                    "run_dir": d.name,
                    **{
                        k: v
                        for k, v in (s.doc.get("benchmarks", {}).get(n) or {}).items()
                        if k != "command"
                    },
                }
                for n, d in s.parts.items()
            },
        },
        "provenance": {
            "complete": bool(code["id"]),
            "missing": [] if code["id"] else ["code"],
        },
        "launch": provenance.launch_of(s.root or next(iter(s.parts.values()), None)),
    }

    t0 = time_origin(s.parts)
    b.config["session"]["sim_time_origin_s"] = t0
    if "perception" in s.parts:
        _perception(b, s.parts["perception"], res["perception"], t0)
    if "slam" in s.parts:
        _slam(b, s.parts["slam"], res["slam"], t0)
    b.summary["run/n_benchmarks"] = len(s.parts)

    # files: every report and data file, plus the provenance record
    for name, d in s.parts.items():
        for p in sorted(d.iterdir()):
            if p.is_file() and p.suffix in (".html", ".json", ".csv", ".jsonl"):
                kind = "report" if p.suffix == ".html" else "data"
                b.files.append((p, kind))
        # the parameters in effect (pipeline_overrides.py)
        b.files += [(p, "params") for p in sorted((d / "params").glob("*.json"))]
    if s.root is not None:
        b.files.append((s.root / "session.json", "config"))
    prov_dir = s.root or next(iter(s.parts.values()), None)
    if prov_dir is not None:
        b.files += [(p, "provenance") for p in provenance.provenance_files(prov_dir)]
    return b


def _perception(b: RunBundle, d: Path, res: dict[str, Any], t0: float) -> None:
    S, g = b.summary, res.get("gt_metrics") or {}
    tp, fp, fn = (_get(g, k) for k in ("total_tp", "total_fp", "total_fn"))
    S["perception/precision_frac"] = _get(g, "precision")
    S["perception/recall_frac"] = _get(g, "recall")
    p, r = S["perception/precision_frac"], S["perception/recall_frac"]
    S["perception/f1_frac"] = 2 * p * r / (p + r) if p and r else None
    S["perception/pos_err_mean_m"] = _get(g, "mean_match_err_m")
    S["perception/pos_err_median_m"] = _get(g, "median_match_err_m")
    S["perception/pos_err_p95_m"] = _get(g, "p95_match_err_m")
    S["perception/pos_err_max_m"] = _get(g, "max_match_err_m")
    S["perception/bias_x_mean_m"] = _get(g, "mean_bias_x_m")
    S["perception/bias_y_mean_m"] = _get(g, "mean_bias_y_m")
    S["perception/n_cones_mean"] = _get(g, "mean_pred_per_frame") or _get(
        res, "mean_cones_per_frame"
    )
    S["perception/n_gt_cones_mean"] = _get(g, "mean_gt_per_frame")
    S["perception/n_false_pos"] = fp
    S["perception/n_missed"] = fn
    S["perception/n_scans"] = _get(res, "frames")

    rows = _csv(d / "results.csv")
    if rows:
        t = rows["t_s"] - t0
        with np.errstate(invalid="ignore", divide="ignore"):
            vals = {
                "latency_ms": rows.get("latency_ms"),
                "n_points": rows.get("n_points"),
                "n_cones": rows.get("n_cones"),
                "n_gt": rows.get("n_gt"),
                "n_tp": rows.get("n_tp"),
                "n_fp": rows.get("n_fp"),
                "n_fn": rows.get("n_fn"),
                "match_err_m": rows.get("mean_match_err_m"),
                "recall_frac": rows["n_tp"] / rows["n_gt"],
                "precision_frac": rows["n_tp"] / (rows["n_tp"] + rows["n_fp"]),
            }
        b.add_series(
            Series(
                "gt_perception",
                "t/replay_s",
                t,
                {k: v for k, v in vals.items() if v is not None},
            )
        )
        lat = rows.get("latency_ms")
        if lat is not None and np.isfinite(lat).any():
            S["perception/proc_mean_ms"] = float(np.nanmean(lat))
            S["perception/proc_p95_ms"] = float(np.nanpercentile(lat, 95))
            S["perception/proc_max_ms"] = float(np.nanmax(lat))

    # every matched cone: how far it was and how wrong
    fd = d / "frame_details.jsonl"
    if fd.is_file():
        mrows = []
        for line in fd.read_text().splitlines():
            try:
                f = json.loads(line)
            except ValueError:
                continue
            for e, rg in zip(f.get("match_errs") or [], f.get("match_ranges_m") or []):
                ft = f.get("t_s")
                mrows.append([ft - t0 if ft is not None else None, float(rg), float(e)])
        if mrows:
            b.add_table(
                Table(
                    "perception_matches",
                    ["t_s", "range_m", "err_m"],
                    mrows,
                    "Every detection matched to a true cone: its range and position error",
                )
            )
    st = d / "profile_stages.csv"
    if st.is_file() and st.stat().st_size:
        names, vals = _col(st, "stage"), _csv(st)
        b.add_table(
            Table(
                "perception_stages",
                ["stage", "mean_ms", "median_ms", "p95_ms", "max_ms"],
                [
                    [n]
                    + [
                        float(vals[c][i])
                        for c in ("mean_ms", "median_ms", "p95_ms", "max_ms")
                    ]
                    for i, n in enumerate(names)
                ],
                "Cone detection time per stage (--profile)",
            )
        )


ODOM_SOURCES = {
    "slam_at_gt_rate": "SLAM",
    "filter_odom": "EKF odometry",
    "wheel_odom": "wheel odometry",
    "imu_only_odom": "IMU only",
    "supervisor_odom": "bag /odom",
}


def _slam(b: RunBundle, d: Path, res: dict[str, Any], t0: float) -> None:
    S = b.summary
    at = res.get("slam_at_gt_rate") or {}
    S["slam/pos_err_mean_m"] = _get(at, "mean_err_m")
    S["slam/pos_err_p95_m"] = _get(at, "p95_err_m")
    S["slam/pos_err_max_m"] = _get(at, "max_err_m")
    S["slam/yaw_err_mean_deg"] = _get(at, "mean_yaw_err_deg")
    S["slam/pos_err_at_scans_mean_m"] = _get(res, "slam_at_cones", "mean_err_m")
    m = res.get("map") or {}
    gt_n, matched = _get(m, "gt_cones"), _get(m, "matched")
    S["slam/n_map_cones"] = _get(m, "slam_landmarks")
    S["slam/map_recall_frac"] = matched / gt_n if gt_n and matched is not None else None
    S["slam/n_map_false_pos"] = _get(m, "false_positive")
    S["slam/map_err_mean_m"] = _get(m, "mean_match_err_m")
    if gt_n and S["slam/n_map_cones"] is not None:
        S["slam/map_cones_vs_track_frac"] = S["slam/n_map_cones"] / gt_n
    S["odom/pos_err_mean_m"] = _get(res, "filter_odom", "mean_err_m")
    S["odom/pos_err_p95_m"] = _get(res, "filter_odom", "p95_err_m")
    S["odom/wheel_pos_err_mean_m"] = _get(res, "wheel_odom", "mean_err_m")
    S["odom/imu_only_pos_err_mean_m"] = _get(res, "imu_only_odom", "mean_err_m")
    b.add_table(
        Table(
            "pose_error_sources",
            [
                "source",
                "mean_err_m",
                "median_err_m",
                "p95_err_m",
                "max_err_m",
                "mean_yaw_err_deg",
            ],
            [
                [label]
                + [
                    _get(res, k, f)
                    for f in (
                        "mean_err_m",
                        "median_err_m",
                        "p95_err_m",
                        "max_err_m",
                        "mean_yaw_err_deg",
                    )
                ]
                for k, label in ODOM_SOURCES.items()
                if isinstance(res.get(k), dict) and res[k].get("mean_err_m") is not None
            ],
            "Position error against the true pose, per pose source",
        )
    )

    tr = _csv(d / "trajectory.csv")
    if tr and len(tr.get("t_s", [])):
        t = tr["t_s"] - t0
        vals = {
            "gt_x": tr["gt_x"],
            "gt_y": tr["gt_y"],
            "slam_x": tr["slam_x"],
            "slam_y": tr["slam_y"],
            "slam_err_m": tr["err_m"],
        }
        dt = np.gradient(t) if len(t) > 1 else np.ones_like(t)
        with np.errstate(invalid="ignore", divide="ignore"):
            vals["gt_speed_mps"] = np.hypot(
                np.gradient(tr["gt_x"]), np.gradient(tr["gt_y"])
            ) / np.where(dt > 0, dt, np.nan)
        ps = _csv(d / "pose_steps.csv")
        if ps and len(ps.get("t_s", [])):
            # the per-step file is ~7x denser; bring what the other pose sources got wrong onto these times
            for src, key in (
                ("filter_err_m", "ekf_err_m"),
                ("wheel_err_m", "wheel_err_m"),
                ("imu_only_err_m", "imu_only_err_m"),
            ):
                if src in ps:
                    vals[key] = interp(ps["t_s"] - t0, ps[src], t)
            if "slam_yaw_err_rad" in ps:
                vals["slam_yaw_err_deg"] = np.degrees(
                    interp(ps["t_s"] - t0, ps["slam_yaw_err_rad"], t)
                )
        b.add_series(Series("replay_pose", "t/replay_s", t, vals))
        e = tr["err_m"][np.isfinite(tr["err_m"])]
        if e.size:
            S["slam/pos_err_rms_m"] = float(np.sqrt(np.mean(e**2)))
        rows = [
            [float(a), "gt", float(x), float(y), None]
            for a, x, y in zip(t, tr["gt_x"], tr["gt_y"])
        ]
        rows += [
            [float(a), "slam", float(x), float(y), None]
            for a, x, y in zip(t, tr["slam_x"], tr["slam_y"])
        ]
        b.add_table(
            Table(
                "trajectory",
                ["t_s", "source", "x", "y", "yaw"],
                rows,
                "True pose and SLAM pose (same frame)",
            )
        )
    if ps := _csv(d / "pose_steps.csv"):
        fe = ps.get("filter_err_m")
        if fe is not None and np.isfinite(fe).any():
            S["odom/pos_err_rms_m"] = float(np.sqrt(np.nanmean(fe**2)))

    mc = d / "map_cones.csv"
    if mc.is_file():
        src, cols, xy = _col(mc, "source"), _col(mc, "color"), _csv(mc)
        b.add_table(
            Table(
                "map_cones",
                ["x", "y", "source", "color"],
                [
                    [
                        float(xy["x"][i]),
                        float(xy["y"][i]),
                        src[i],
                        CONE_COLOUR.get(int(float(cols[i])), "unknown")
                        if cols[i] not in ("", None)
                        else "unknown",
                    ]
                    for i in range(len(src))
                ],
                "True track cones (gt) and the final SLAM map (slam)",
            )
        )


def load_all(root: Path, *, full_bag_hash: bool = True) -> list[RunBundle]:
    """Sessions under ``root`` plus paired legacy perception/SLAM runs under it."""
    sessions = find_sessions(root) + legacy_sessions(root)
    return [load_session(s, full_bag_hash=full_bag_hash) for s in sessions if s.parts]
