"""MOCK simulator-in-the-loop benchmark results.

There is no closed-loop SIL runner yet (tools/scenario_runner doesn't drive
the car). This writes run dirs in the layout that runner is expected to
produce, so the tracking backends and dashboards can be built and judged
against realistic-looking data today:

    <out>/<scenario>/<commit>/seed<k>/
        manifest.json      provenance + scenario + params + referee state
        results.json       canonical summary metrics
        telemetry.csv      per ~1 m of travel: t, s, lap, pose, errors, commands
        laps.csv           per lap: time, penalties
        perception.csv     per LiDAR scan: GT/TP/FP/FN, errors
        perception_range.csv  recall / error per 2.5 m range bin
        latency.csv        per scan, per node latency
        events.csv         DOO, off-course, DNF, latency spikes, SLAM relocalisation
        map_cones.csv      GT + final SLAM landmarks

Everything is generated from a simple point-mass lap model on a real track
CSV. The *numbers are fake*; every run is tagged ``mock`` in every backend.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import dataclass, field, replace
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import numpy as np

from .geometry import Track, load_track

DOO_PENALTY_S = 2.0
OC_PENALTY_S = 10.0
CAR_HALF_WIDTH = 0.72
LIDAR_HZ = 10.0


@dataclass(frozen=True)
class Commit:
    sha: str
    branch: str
    message: str
    # behaviour knobs the mock maps a "code change" onto
    lat_acc_gain: float = 1.0  # >1: drives corners faster
    corner_cut: float = 0.16  # lateral error per unit of v²κ/alat
    track_noise_m: float = 0.07  # AR(1) lateral noise sigma
    perception_range_m: float = 20.0
    perception_fp_rate: float = 0.25  # FP per scan
    perception_ms: float = 28.0
    slam_ms: float = 5.0
    slam_drift: float = 0.004  # m per m during mapping lap
    speed_scale: float = 1.0  # <1: slower everywhere (e.g. a conservative regression)
    dnf_hazard: float = 0.0  # per-metre chance of a large excursion (likely DNF)


@dataclass(frozen=True)
class Scenario:
    name: str
    event: str
    track_csv: str
    laps: int
    noise: dict[str, float] = field(default_factory=dict)
    v_max: float = 14.0
    alat: float = 9.0
    a_acc: float = 4.5
    a_brk: float = 7.0
    v_explore: float = 6.0

    @property
    def noise_level(self) -> float:
        return float(self.noise.get("level", 1.0))


BASELINE = Commit("a64350a", "dev", "baseline: pure pursuit, fixed lookahead")
CANDIDATE = Commit(
    "c0ffee1",
    "feat/adaptive-lookahead",
    "adaptive lookahead + faster corner entry, wider perception range",
    lat_acc_gain=1.08,
    corner_cut=0.30,
    track_noise_m=0.085,
    perception_range_m=22.5,
    perception_fp_rate=0.32,
    perception_ms=31.0,
    dnf_hazard=0.00006,
)

SCENARIOS = [
    Scenario(
        "trackdrive_T23_nominal",
        "trackdrive",
        "track_20260404_013723.csv",
        10,
        {"level": 1.0, "gyro_std": 0.002, "accel_std": 0.05, "lidar_range_std": 0.02},
    ),
    Scenario(
        "trackdrive_T23_highnoise",
        "trackdrive",
        "track_20260404_013723.csv",
        10,
        {"level": 2.5, "gyro_std": 0.006, "accel_std": 0.15, "lidar_range_std": 0.05},
    ),
    Scenario(
        "autocross_T25",
        "autocross",
        "track_20260404_013725.csv",
        1,
        {"level": 1.0, "gyro_std": 0.002, "accel_std": 0.05, "lidar_range_std": 0.02},
        v_explore=8.0,
    ),
]


def _speed_profile(tr: Track, sc: Scenario, alat: float) -> np.ndarray:
    k = np.abs(tr.curvature())
    v = np.minimum(sc.v_max, np.sqrt(alat / np.maximum(k, 1e-4)))
    ds = np.diff(tr.s)
    for _ in range(2):  # periodic: two passes settle the wrap-around
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * sc.a_acc * ds[i - 1]))
        v[0] = min(v[0], v[-1])
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * sc.a_brk * ds[i]))
        v[-1] = min(v[-1], v[0])
    return v


def _normals(tr: Track) -> np.ndarray:
    d = np.gradient(tr.center, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return np.c_[-d[:, 1], d[:, 0]]


def simulate(
    sc: Scenario,
    cm: Commit,
    seed: int,
    tracks_dir: Path,
    *,
    params: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Run one mock seed. Returns in-memory tables; ``write_run`` persists them."""
    params = dict(params or {})
    key = json.dumps([sc.name, cm.sha, seed, sorted(params.items())]).encode()
    rng = np.random.default_rng(
        int.from_bytes(hashlib.sha1(key).digest()[:4], "little")
    )
    tr = load_track(tracks_dir / sc.track_csv)
    nl = sc.noise_level
    la_gain = params.get("control.lookahead_gain", 0.35)
    alat = params.get("control.max_lat_acc", sc.alat) * cm.lat_acc_gain
    # lookahead: small -> oscillation (noise), large -> corner cutting
    osc = 1.0 + 3.0 * max(0.0, 0.3 - la_gain) ** 1.2 * 5
    cut = cm.corner_cut * (0.6 + 1.3 * la_gain)
    v_race = _speed_profile(tr, sc, alat) * cm.speed_scale
    kappa = tr.curvature()
    nrm = _normals(tr)
    half_w = 1.55

    cones = np.vstack([tr.blue, tr.yellow])
    cone_color = ["blue"] * len(tr.blue) + ["yellow"] * len(tr.yellow)
    cs, ce = tr.project(cones[:, 0], cones[:, 1])

    tel: list[dict[str, float]] = []
    laps: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    t = 0.0
    e = 0.0
    slam_err = np.zeros(2)
    odom_err = np.zeros(2)
    dnf = False
    hit_cones: set[int] = set()
    n_seg = len(tr.s) - 1
    ds = np.diff(tr.s)
    for lap in range(1, sc.laps + 1):
        lap_t0 = t
        doo = oc = 0
        lap_scale = 1.0 + rng.normal(0, 0.01 * nl)
        mapping = lap == 1
        for i in range(n_seg):
            v = min(v_race[i] * lap_scale, sc.v_explore if mapping else 1e9)
            if mapping and i < 5:
                v = max(1.0, v * (i + 1) / 5)
            ss = tr.s[i] + (lap - 1) * tr.length_m
            # lateral error: corner cut (towards inside) + oscillation noise
            target = cut * v * v * kappa[i] / max(alat, 1e-3)
            e = (
                0.9 * e
                + 0.1 * target
                + rng.normal(0, cm.track_noise_m * osc * math.sqrt(nl) * 0.35)
            )
            # rare disturbances (slip / bad plan): larger with noise
            if rng.random() < 0.00025 * nl * cm.lat_acc_gain**8 * (alat / sc.alat) ** 4:
                e += rng.choice([-1, 1]) * rng.uniform(0.6, 1.6)
                events.append(
                    {
                        "t_s": t,
                        "s_m": ss,
                        "lap": lap,
                        "kind": "lateral_disturbance",
                        "severity": "warn",
                        "detail": f"lateral kick at v={v:.1f} m/s, kappa={kappa[i]:.3f}",
                    }
                )
            if not mapping and rng.random() < cm.dnf_hazard * nl:
                e += rng.choice([-1, 1]) * rng.uniform(1.9, 2.8)
                events.append(
                    {
                        "t_s": t,
                        "s_m": ss,
                        "lap": lap,
                        "kind": "lateral_disturbance",
                        "severity": "error",
                        "detail": f"large excursion at v={v:.1f} m/s (planner lost the boundary)",
                    }
                )
            x, y = tr.center[i] + e * nrm[i]
            heading_err = math.atan2(
                e - (tel[-1]["cte_m"] if tel else 0.0), ds[i]
            ) + rng.normal(0, 0.01 * nl)
            dt = ds[i] / max(v, 0.3)
            prev_v = tel[-1]["speed_mps"] if tel else 0.0
            acc = (v - prev_v) / dt
            steer = math.atan(1.53 * kappa[i]) + 0.25 * e + rng.normal(0, 0.004 * osc)
            # localisation
            if mapping:
                slam_err += rng.normal(
                    0, cm.slam_drift * math.sqrt(nl) * ds[i] ** 0.5, 2
                )
            else:
                slam_err = 0.85 * slam_err + rng.normal(0, 0.02 * math.sqrt(nl), 2)
            odom_err += (
                rng.normal(0, 0.008 * math.sqrt(nl) * ds[i] ** 0.5, 2) + 0.0005 * ds[i]
            )
            tel.append(
                {
                    "t_s": t,
                    "s_m": ss,
                    "s_lap_m": tr.s[i],
                    "lap": lap,
                    "x": x,
                    "y": y,
                    "speed_mps": v,
                    "cte_m": e,
                    "heading_err_rad": heading_err,
                    "kappa_1pm": kappa[i],
                    "lat_acc_mps2": v * v * kappa[i],
                    "steering_rad": steer,
                    "throttle": float(np.clip(acc / sc.a_acc, 0, 1)),
                    "brake": float(np.clip(-acc / sc.a_brk, 0, 1)),
                    "slam_err_m": float(np.hypot(*slam_err)),
                    "odom_err_m": float(np.hypot(*odom_err)),
                    "slam_x": x + slam_err[0],
                    "slam_y": y + slam_err[1],
                }
            )
            # referee: cones near this segment
            near = np.where(
                (np.abs(cs - tr.s[i]) < 0.6)
                & ~np.isin(np.arange(len(cones)), list(hit_cones))
            )[0]
            for c in near:
                gap = abs(ce[c] - e) - CAR_HALF_WIDTH
                if gap < 0.05 and rng.random() < 0.8:
                    hit_cones.add(int(c))
                    doo += 1
                    events.append(
                        {
                            "t_s": t,
                            "s_m": ss,
                            "lap": lap,
                            "kind": "doo",
                            "severity": "warn",
                            "detail": f"{cone_color[c]} cone hit (cte={e:+.2f} m)",
                            "x": float(cones[c, 0]),
                            "y": float(cones[c, 1]),
                        }
                    )
            if abs(e) > half_w + 0.3:
                oc += 1
                events.append(
                    {
                        "t_s": t,
                        "s_m": ss,
                        "lap": lap,
                        "kind": "off_course",
                        "severity": "error",
                        "detail": f"all wheels out (cte={e:+.2f} m)",
                        "x": x,
                        "y": y,
                    }
                )
                if abs(e) > half_w + 0.9 or rng.random() < 0.35:
                    dnf = True
                    events.append(
                        {
                            "t_s": t,
                            "s_m": ss,
                            "lap": lap,
                            "kind": "dnf",
                            "severity": "error",
                            "detail": "lost the track — SLAM relocalisation failed",
                            "x": x,
                            "y": y,
                        }
                    )
                    break
                e *= 0.3  # recovers
            t += dt
            if rng.random() < 0.002:
                events.append(
                    {
                        "t_s": t,
                        "s_m": ss,
                        "lap": lap,
                        "kind": "latency_spike",
                        "severity": "warn",
                        "detail": "perception cycle 140 ms (GC pause)",
                    }
                )
        if dnf:
            laps.append(
                {
                    "lap": lap,
                    "time_s": None,
                    "doo": doo,
                    "off_course": oc,
                    "completed": 0,
                }
            )
            break
        laps.append(
            {
                "lap": lap,
                "time_s": t - lap_t0,
                "doo": doo,
                "off_course": oc,
                "completed": 1,
            }
        )
        if mapping:
            events.append(
                {
                    "t_s": t,
                    "s_m": lap * tr.length_m,
                    "lap": lap,
                    "kind": "loop_closure",
                    "severity": "info",
                    "detail": f"map closed, SLAM error {np.hypot(*slam_err):.2f} m before correction",
                }
            )
            slam_err *= 0.15

    tel_arr = {k: np.array([r[k] for r in tel]) for k in tel[0]}
    perc, perc_range = _perception(tr, cm, sc, tel_arr, rng)
    lat = _latency(cm, sc, perc["t_s"], rng, la_gain)
    map_rows = _map(tr, cm, sc, rng)
    return {
        "track": tr,
        "telemetry": tel_arr,
        "laps": laps,
        "events": events,
        "perception": perc,
        "perception_range": perc_range,
        "latency": lat,
        "map": map_rows,
        "dnf": dnf,
        "params": {
            "control.lookahead_gain": la_gain,
            "control.max_lat_acc": alat / cm.lat_acc_gain,
            **params,
        },
    }


def _perception(
    tr: Track, cm: Commit, sc: Scenario, tel: dict[str, np.ndarray], rng
) -> tuple[dict, list]:
    cones = np.vstack([tr.blue, tr.yellow])
    t_end = tel["t_s"][-1]
    ts = np.arange(0.0, t_end, 1.0 / LIDAR_HZ)
    x = np.interp(ts, tel["t_s"], tel["x"])
    y = np.interp(ts, tel["t_s"], tel["y"])
    hx = np.gradient(x)
    hy = np.gradient(y)
    s = np.interp(ts, tel["t_s"], tel["s_m"])
    nl = sc.noise_level
    rows = {
        k: []
        for k in (
            "t_s",
            "s_m",
            "n_gt",
            "n_tp",
            "n_fp",
            "n_fn",
            "recall",
            "precision",
            "pos_err_mean_m",
            "n_color_err",
        )
    }
    bins = np.arange(0, 32.5, 2.5)
    b_gt = np.zeros(len(bins) - 1)
    b_tp = np.zeros(len(bins) - 1)
    b_err = np.zeros(len(bins) - 1)
    for i in range(len(ts)):
        d = cones - [x[i], y[i]]
        r = np.hypot(*d.T)
        fwd = (d[:, 0] * hx[i] + d[:, 1] * hy[i]) > 0
        vis = fwd & (r < 30)
        rv = r[vis]
        p = 1 / (1 + np.exp((rv - cm.perception_range_m) / (1.8 * math.sqrt(nl))))
        p *= 0.99 - 0.02 * (nl - 1)
        det = rng.random(len(rv)) < p
        err = np.abs(rng.normal(0, 0.03 + 0.004 * rv * math.sqrt(nl)))
        n_fp = rng.poisson(cm.perception_fp_rate * nl)
        n_tp = int(det.sum())
        rows["t_s"].append(ts[i])
        rows["s_m"].append(s[i])
        rows["n_gt"].append(len(rv))
        rows["n_tp"].append(n_tp)
        rows["n_fp"].append(n_fp)
        rows["n_fn"].append(len(rv) - n_tp)
        rows["recall"].append(n_tp / len(rv) if len(rv) else math.nan)
        rows["precision"].append(n_tp / (n_tp + n_fp) if n_tp + n_fp else math.nan)
        rows["pos_err_mean_m"].append(float(err[det].mean()) if n_tp else math.nan)
        rows["n_color_err"].append(int(rng.binomial(n_tp, 0.01 * nl)))
        idx = np.digitize(rv, bins) - 1
        np.add.at(b_gt, idx, 1)
        np.add.at(b_tp, idx, det)
        np.add.at(b_err, idx, np.where(det, err, 0))
    rng_rows = [
        {
            "range_lo_m": bins[j],
            "range_hi_m": bins[j + 1],
            "n_gt": int(b_gt[j]),
            "n_tp": int(b_tp[j]),
            "recall": b_tp[j] / b_gt[j] if b_gt[j] else math.nan,
            "pos_err_mean_m": b_err[j] / b_tp[j] if b_tp[j] else math.nan,
        }
        for j in range(len(bins) - 1)
    ]
    return {k: np.array(v, dtype=float) for k, v in rows.items()}, rng_rows


def _latency(
    cm: Commit, sc: Scenario, ts: np.ndarray, rng, la_gain: float
) -> dict[str, np.ndarray]:
    n = len(ts)
    nl = sc.noise_level
    perc = rng.lognormal(math.log(cm.perception_ms * (1 + 0.08 * (nl - 1))), 0.18, n)
    spikes = rng.random(n) < 0.004
    perc[spikes] += rng.uniform(60, 140, spikes.sum())
    slam = rng.lognormal(math.log(cm.slam_ms), 0.35, n)
    plan = rng.lognormal(math.log(12.0), 0.25, n)
    ctrl = rng.lognormal(math.log(1.4 + 0.5 * la_gain), 0.2, n)
    transport = rng.lognormal(math.log(14.0), 0.15, n)
    cpu = np.clip(rng.normal(0.46 + 0.02 * (cm.perception_ms - 28) / 3, 0.05, n), 0, 1)
    return {
        "t_s": ts,
        "perception_ms": perc,
        "slam_ms": slam,
        "planning_ms": plan,
        "control_ms": ctrl,
        "e2e_ms": perc + slam + plan + ctrl + transport,
        "cpu_frac": cpu,
    }


def _map(tr: Track, cm: Commit, sc: Scenario, rng) -> list[dict[str, Any]]:
    rows = []
    for color, pts in (("blue", tr.blue), ("yellow", tr.yellow), ("orange", tr.orange)):
        for x, y in pts:
            rows.append({"x": x, "y": y, "color": color, "source": "gt"})
            if rng.random() < 0.985:
                err = rng.normal(
                    0, 0.06 * math.sqrt(sc.noise_level) + cm.slam_drift * 5, 2
                )
                rows.append(
                    {"x": x + err[0], "y": y + err[1], "color": color, "source": "slam"}
                )
            if rng.random() < 0.02 * sc.noise_level:  # duplicate landmark
                err = rng.normal(0, 0.9, 2)
                rows.append(
                    {"x": x + err[0], "y": y + err[1], "color": color, "source": "slam"}
                )
    return rows


def summarize(sim: dict[str, Any], sc: Scenario) -> dict[str, float | int | None]:
    tel, laps, perc, lat = (
        sim["telemetry"],
        sim["laps"],
        sim["perception"],
        sim["latency"],
    )
    lap_times = [lp["time_s"] for lp in laps if lp["completed"]]
    racing = lap_times[1:] if len(lap_times) > 1 else lap_times
    doo = sum(lp["doo"] for lp in laps)
    oc = sum(lp["off_course"] for lp in laps)
    finished = int(not sim["dnf"] and len(lap_times) == sc.laps)
    fail = [e for e in sim["events"] if e["kind"] in ("dnf", "off_course")]
    tp, fp, fn = perc["n_tp"].sum(), perc["n_fp"].sum(), perc["n_fn"].sum()
    rr = sim["perception_range"]

    def rec_upto(r):
        g = sum(b["n_gt"] for b in rr if b["range_hi_m"] <= r)
        t_ = sum(b["n_tp"] for b in rr if b["range_hi_m"] <= r)
        return t_ / g if g else None

    slam_n = sum(1 for m in sim["map"] if m["source"] == "slam")
    gt_n = sum(1 for m in sim["map"] if m["source"] == "gt")
    racing_mask = tel["lap"] > 1 if sc.laps > 1 else np.ones_like(tel["lap"], bool)
    return {
        "race/finished": finished,
        "race/n_laps": len(lap_times),
        "race/lap_time_best_s": min(racing) if racing else None,
        "race/lap_time_mean_s": float(np.mean(racing)) if racing else None,
        "race/lap_time_std_s": float(np.std(racing)) if len(racing) > 1 else None,
        "race/total_time_s": (sum(lap_times) + doo * DOO_PENALTY_S + oc * OC_PENALTY_S)
        if finished
        else None,
        "race/n_doo": doo,
        "race/n_off_track": oc,
        "race/penalty_s": doo * DOO_PENALTY_S + oc * OC_PENALTY_S,
        "race/first_failure_s_m": fail[0]["s_m"] if fail else float(tel["s_m"][-1]),
        "control/cross_track_rms_m": float(
            np.sqrt(np.mean(tel["cte_m"][racing_mask] ** 2))
        ),
        "control/cross_track_max_m": float(np.abs(tel["cte_m"]).max()),
        "control/heading_err_rms_rad": float(
            np.sqrt(np.mean(tel["heading_err_rad"] ** 2))
        ),
        "control/speed_mean_mps": float(np.mean(tel["speed_mps"][racing_mask])),
        "control/speed_max_mps": float(tel["speed_mps"].max()),
        "control/lat_acc_max_mps2": float(np.abs(tel["lat_acc_mps2"]).max()),
        "control/steer_rate_rms_radps": float(
            np.sqrt(
                np.mean(
                    (
                        np.diff(tel["steering_rad"])
                        / np.maximum(np.diff(tel["t_s"]), 1e-3)
                    )
                    ** 2
                )
            )
        ),
        "perception/precision_frac": float(tp / (tp + fp)) if tp + fp else None,
        "perception/recall_frac": float(tp / (tp + fn)) if tp + fn else None,
        "perception/recall@10m_frac": rec_upto(10),
        "perception/recall@20m_frac": rec_upto(20),
        "perception/pos_err_mean_m": float(np.nanmean(perc["pos_err_mean_m"])),
        "perception/pos_err_p95_m": float(np.nanpercentile(perc["pos_err_mean_m"], 95)),
        "perception/color_acc_frac": float(1 - perc["n_color_err"].sum() / max(tp, 1)),
        "perception/n_cones_mean": float(np.mean(perc["n_tp"] + perc["n_fp"])),
        "perception/empty_frame_rate_frac": float(
            np.mean((perc["n_tp"] + perc["n_fp"]) == 0)
        ),
        "slam/pos_err_rms_m": float(
            np.sqrt(np.mean(tel["slam_err_m"][racing_mask] ** 2))
        ),
        "odom/pos_err_rms_m": float(np.sqrt(np.mean(tel["odom_err_m"] ** 2))),
        "slam/n_map_cones": slam_n,
        "slam/map_cones_vs_track_frac": slam_n / gt_n if gt_n else None,
        "latency/perception_p95_ms": float(np.percentile(lat["perception_ms"], 95)),
        "latency/slam_p95_ms": float(np.percentile(lat["slam_ms"], 95)),
        "latency/planning_p95_ms": float(np.percentile(lat["planning_ms"], 95)),
        "latency/control_p95_ms": float(np.percentile(lat["control_ms"], 95)),
        "latency/e2e_p95_ms": float(np.percentile(lat["e2e_ms"], 95)),
        "compute/cpu_mean_frac": float(lat["cpu_frac"].mean()),
    }


def _write_csv(path: Path, rows: list[dict[str, Any]] | dict[str, np.ndarray]) -> None:
    if isinstance(rows, dict):
        keys = list(rows)
        rows = [dict(zip(keys, vals)) for vals in zip(*(rows[k] for k in keys))]
    if not rows:
        path.write_text("")
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(
                {k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()}
            )


def write_run(
    out: Path,
    sc: Scenario,
    cm: Commit,
    seed: int,
    sim: dict[str, Any],
    *,
    started_at: datetime,
    tags: list[str],
    job_type: str = "sim_e2e",
    extra: dict[str, Any] | None = None,
) -> Path:
    out.mkdir(parents=True, exist_ok=True)
    summary = summarize(sim, sc)
    tr = sim["track"]
    lap_times = [lp["time_s"] for lp in sim["laps"] if lp["completed"]]
    manifest = {
        "mock": True,
        "job_type": job_type,
        "started_at": started_at.isoformat(),
        "tags": tags,
        "seed": seed,
        "scenario": {
            "name": sc.name,
            "event": sc.event,
            "laps": sc.laps,
            "noise": sc.noise,
            "track": {
                "name": tr.name,
                "csv": sc.track_csv,
                "length_m": tr.length_m,
                "n_cones": int(len(tr.blue) + len(tr.yellow) + len(tr.orange)),
            },
        },
        "code": {
            "pipeline": {
                "sha": cm.sha,
                "branch": cm.branch,
                "dirty": False,
                "message": cm.message,
            },
            "ifssim": {"sha": "c43ee7d", "branch": "dev", "dirty": False},
            "sim_build": {"id": "ifssim-linux-2026.09.20", "plugin_sha": "c43ee7d"},
            "image": {
                "id": "sha256:4f1c0e9b7d2a",
                "tag": "ifssim-dv_pipeline_stack:latest",
            },
        },
        "params": sim["params"],
        "referee": {
            "Laps": lap_times,
            "DooCounter": summary["race/n_doo"],
            "OffTrackCounter": summary["race/n_off_track"],
            "bFinished": bool(summary["race/finished"]),
            "RequiredLaps": sc.laps,
            "EventType": sc.event,
        },
        **(extra or {}),
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=float))
    (out / "results.json").write_text(json.dumps(summary, indent=2, default=float))
    _write_csv(out / "telemetry.csv", sim["telemetry"])
    _write_csv(out / "laps.csv", sim["laps"])
    _write_csv(out / "perception.csv", sim["perception"])
    _write_csv(out / "perception_range.csv", sim["perception_range"])
    _write_csv(out / "latency.csv", sim["latency"])
    _write_csv(
        out / "events.csv",
        sim["events"]
        or [
            {
                "t_s": None,
                "s_m": None,
                "lap": None,
                "kind": None,
                "severity": None,
                "detail": None,
            }
        ],
    )
    _write_csv(out / "map_cones.csv", sim["map"])
    return out


# ---------------------------------------------------------------- whole suite
def generate_suite(
    out_root: Path,
    tracks_dir: Path,
    *,
    seeds: int = 5,
    nightly_nights: int = 8,
    sweep: bool = True,
) -> list[Path]:
    """Scenario × commit × seed matrix, a nightly series and a parameter sweep."""
    runs: list[Path] = []
    t0 = datetime(2026, 9, 22, 20, 0, tzinfo=timezone.utc)
    k = 0
    for sc in SCENARIOS:
        for cm in (BASELINE, CANDIDATE):
            for seed in range(1, seeds + 1):
                sim = simulate(sc, cm, seed, tracks_dir)
                tags = ["mock", "matrix", f"branch:{cm.branch}"] + (
                    ["baseline"] if cm is BASELINE else []
                )
                runs.append(
                    write_run(
                        out_root / "matrix" / sc.name / cm.sha / f"seed{seed}",
                        sc,
                        cm,
                        seed,
                        sim,
                        started_at=t0 + timedelta(minutes=4 * k),
                        tags=tags,
                    )
                )
                k += 1

    # nightly: trackdrive nominal, 3 seeds a night; a regression lands on night 5, fixed on night 7
    sc = SCENARIOS[0]
    night0 = datetime(2026, 9, 14, 2, 0, tzinfo=timezone.utc)
    for n in range(nightly_nights):
        regress = n in (4, 5)
        cm = replace(
            BASELINE,
            sha=f"n{n:02d}{'bad' if regress else 'ok0'}"[:7],
            branch="dev",
            message=f"dev @ nightly {n}",
            corner_cut=BASELINE.corner_cut * (1.9 if regress else 1.0) * (0.97**n),
            slam_ms=BASELINE.slam_ms * (1.0 + 0.04 * n),
            perception_ms=BASELINE.perception_ms * (1.35 if regress else 1.0),
            speed_scale=0.975 if regress else 1.0,
        )
        for seed in range(1, 4):
            sim = simulate(sc, cm, 100 + seed, tracks_dir)
            runs.append(
                write_run(
                    out_root / "nightly" / f"night{n:02d}" / f"seed{seed}",
                    sc,
                    cm,
                    100 + seed,
                    sim,
                    started_at=night0 + timedelta(days=n, minutes=6 * seed),
                    tags=["mock", "nightly"],
                    extra={
                        "nightly": {
                            "night": n,
                            "date": (night0 + timedelta(days=n)).date().isoformat(),
                        }
                    },
                )
            )

    if sweep:
        sc3 = replace(SCENARIOS[0], name="trackdrive_T23_sweep", laps=4)
        i = 0
        for la in (0.2, 0.3, 0.4, 0.5):
            for alat in (8.0, 9.5, 11.0):
                p = {"control.lookahead_gain": la, "control.max_lat_acc": alat}
                for seed in range(1, 4):
                    sim = simulate(sc3, BASELINE, 200 + seed, tracks_dir, params=p)
                    runs.append(
                        write_run(
                            out_root / "sweep" / f"trial{i:02d}" / f"seed{seed}",
                            sc3,
                            BASELINE,
                            200 + seed,
                            sim,
                            started_at=t0 + timedelta(hours=6, minutes=10 * i + seed),
                            tags=["mock", "sweep"],
                            job_type="sim_e2e",
                            extra={
                                "sweep": {"id": "sweep-pp-lookahead-latacc", "trial": i}
                            },
                        )
                    )
                i += 1
    return runs
