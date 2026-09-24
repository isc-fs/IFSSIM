"""Turn the per-node ROS logs of a replay run into series, events and scalars.

Every replay dir has ``logs/<node>.log``, including ``--live`` runs that
write nothing else, and the nodes print periodic diagnostics (CONE_FILTER,
SLAM_LAT, SLAM_PROF, SLAM_OBS, PATH_RATE, the 2 Hz control status line).
This parser is how the tracker gets pipeline health for *every* run, not just
``--report`` ones.

Time base: seconds since ``ros2 bag play`` started (``play.log``). If the bag
never started, since the first node became Ready. These are wall-clock
seconds from the log stamps. They match the replay-bag time axis to within
the startup offset (bag play at rate 1.0), which is close enough for overlays
but is labelled ``t/log_s`` so it is never confused with ``t/replay_s``.
"""

from __future__ import annotations

import math
import re
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .bundle import Event, Series, Table

LINE = re.compile(
    r"^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[(\d+\.\d+)\] \[([\w./-]+)\]: (.*)$"
)
NUM = r"([-+]?\d+(?:\.\d+)?|nan)"

RE_CONE_FILTER = re.compile(
    rf"CONE_FILTER \(avg/scan over (\d+)\): pts={NUM} clusters=\s*{NUM} -> >3pts=\s*{NUM} -> "
    rf"shape=\s*{NUM} -> residual=\s*{NUM} accepted=\s*{NUM} far_dropped=\s*{NUM} "
    rf"by-side: L=\s*{NUM} R=\s*{NUM} C=\s*{NUM} BO=\s*{NUM} dbscan_guard=(\d+)/(\d+) hz=\s*{NUM}"
)
RE_DBSCAN = re.compile(r"DBSCAN guard: .*dropped (\d+) pts")
RE_PATH_RATE = re.compile(
    rf"PATH_RATE cb=\s*{NUM}/s pub=\s*{NUM}/s no_cones=(\d+) tf_miss=(\d+) plan_empty=(\d+)"
)
RE_TF_FAIL = re.compile(r"TF lookup failed")
RE_SLAM_LAT = re.compile(
    rf"SLAM_LAT proc=\s*{NUM}ms age=\s*{NUM}ms dt_pub=\s*{NUM}ms "
    rf"corr=\(\s*{NUM},\s*{NUM}\|\s*{NUM}m,\s*{NUM}deg\)"
)
RE_SLAM_PROF = re.compile(
    rf"SLAM_PROF\[(\w+)\] total=\s*{NUM}ms pre=\s*{NUM} assoc=\s*{NUM} spawn=\s*{NUM} "
    rf"commit=\s*{NUM}\(upd=\s*{NUM} est=\s*{NUM} x(\d+)\) db=\s*{NUM} pub=\s*{NUM} obs=(\d+) map=(\d+)"
)
RE_SLAM_OBS = re.compile(
    rf"SLAM_OBS \(avg/scan over (\d+)\): obs=\s*{NUM} assoc=\s*{NUM} new=\s*{NUM} "
    rf"vetoed=\s*{NUM} frozen=\s*{NUM} skip=\s*{NUM}"
)
RE_SLAM_STEP = re.compile(
    rf"step=(\d+) obs=(\d+) new=(\d+) assoc=(\d+) map=(\d+) pose=\(\s*{NUM},\s*{NUM},yaw=\s*{NUM}°\)"
)
RE_POSE_JUMP = re.compile(rf"pose-jump rejected: dev=\(\s*{NUM} m,\s*{NUM}°\)")
RE_DA_SKIP = re.compile(
    r"skip cone factors: DA-failure spike \[([^\]]+)\] \(obs=(\d+) new=(\d+) assoc=(\d+)\)"
)
RE_CASCADE = re.compile(r"CASCADE_SKIP_RECOVERY: force-accepting scan after (\d+)")
RE_PROX = re.compile(r"proximity-veto: dropped (\d+)")
RE_CTRL = re.compile(
    rf"v={NUM} travelled={NUM}m -> thr={NUM} regen={NUM} steer={NUM} \| "
    rf"path_n=(\d+) path_len={NUM}m stop_d=(inf|{NUM}) latched=(True|False)"
)
RE_STOP = re.compile(rf"stop latched at (\w+)=\(\s*{NUM},\s*{NUM}\) from (\d+)")
RE_FOX_CLIENT = re.compile(r"Registered client ([\d.:]+)")
RE_PLAY = re.compile(r"\[(\d+\.\d+)\] \[rosbag2_player\]")

SHUTDOWN_NOISE = ("rcl_shutdown already called", "ExternalShutdownException")
SLAM_MODE_CODE = {"ok": 0, "loc": 1, "skip": 2}


def _f(s: str) -> float:
    return float("nan") if s in ("nan", "inf") else float(s)


@dataclass
class LogDigest:
    t0_wall: float | None
    t0_source: str
    series: dict[str, Series] = field(default_factory=dict)
    events: list[Event] = field(default_factory=list)
    summary: dict[str, float | int | None] = field(default_factory=dict)
    lifecycle: Table | None = None
    warn_catalogue: Table | None = None
    played: bool = False
    viewer_clients: int = 0


def _pct(a: list[float] | np.ndarray, q: float) -> float | None:
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if a.size else None


def _mean(a) -> float | None:
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(a.mean()) if a.size else None


def _series(
    family: str, rows: list[tuple[float, dict[str, float]]], step_name: str = "t/log_s"
) -> Series | None:
    if not rows:
        return None
    keys = sorted({k for _, r in rows for k in r})
    step = np.array([t for t, _ in rows])
    vals = {
        k: np.array([r.get(k, math.nan) for _, r in rows], dtype=float) for k in keys
    }
    return Series(family, step_name, step, vals)


def parse_logs(log_dir: Path) -> LogDigest:
    lines_by_node: dict[str, list[tuple[str, float, str]]] = defaultdict(list)
    crashes: list[tuple[str, float | None, str]] = []
    viewer_clients = 0
    play_t: float | None = None

    for path in sorted(log_dir.glob("*.log")):
        node_file = path.stem
        text = path.read_text(errors="replace")
        if node_file == "play":
            m = RE_PLAY.search(text)
            if m:
                play_t = float(m.group(1))
            continue
        viewer_clients += (
            len(RE_FOX_CLIENT.findall(text)) if node_file == "foxglove_bridge" else 0
        )
        last_t: float | None = None
        tb: list[str] | None = None
        for raw in text.splitlines():
            m = LINE.match(raw)
            if m:
                if tb is not None:
                    crashes.append((node_file, last_t, "\n".join(tb)))
                    tb = None
                level, t, node, msg = (
                    m.group(1),
                    float(m.group(2)),
                    m.group(3),
                    m.group(4),
                )
                last_t = t
                lines_by_node[node].append((level, t, msg))
            elif raw.startswith("Traceback"):
                if tb is not None:
                    tb.append(raw)  # chained exception inside the same block
                else:
                    tb = [raw]
            elif tb is not None:
                tb.append(raw)
        if tb is not None:
            crashes.append((node_file, last_t, "\n".join(tb)))

    # ------------------------------------------------------------- time base
    ready_ts = [
        t
        for lines in lines_by_node.values()
        for lvl, t, msg in lines
        if msg.startswith("Ready:")
    ]
    if play_t is not None:
        t0, t0_src = play_t, "bag_play_start"
    elif ready_ts:
        t0, t0_src = min(ready_ts), "first_node_ready"
    else:
        all_t = [t for lines in lines_by_node.values() for _, t, _ in lines]
        t0, t0_src = (min(all_t) if all_t else None), "first_log_line"
    rel = (lambda t: t - t0) if t0 is not None else (lambda t: t)

    dig = LogDigest(
        t0_wall=t0,
        t0_source=t0_src,
        played=play_t is not None,
        viewer_clients=viewer_clients,
    )
    S = dig.summary

    # ------------------------------------------------------------- lifecycle
    life_rows = []
    life: dict[str, dict[str, float]] = {}
    for node, lines in lines_by_node.items():
        d: dict[str, float] = {}
        for _, t, msg in lines:
            if msg.startswith("Ready:") and "ready" not in d:
                d["ready"] = t
            elif msg.startswith("Configured") and "configured" not in d:
                d["configured"] = t
            elif msg.startswith("Activated in mode") and "activated" not in d:
                d["activated"] = t
            elif msg.startswith("Numba warmup complete") or msg.startswith(
                "FaSTTUBe warmup complete"
            ):
                d["warmup_done"] = t
            elif msg.startswith("warming up") and "warmup_start" not in d:
                d["warmup_start"] = t
            elif msg.startswith("/odom first publish"):
                d["first_output"] = t
        if d:
            life[node] = d
            life_rows.append(
                [
                    node,
                    *(
                        round(rel(d[k]), 3) if k in d else None
                        for k in (
                            "ready",
                            "configured",
                            "activated",
                            "warmup_start",
                            "warmup_done",
                            "first_output",
                        )
                    ),
                ]
            )
    if life_rows:
        dig.lifecycle = Table(
            "lifecycle",
            [
                "node",
                "ready_s",
                "configured_s",
                "activated_s",
                "warmup_start_s",
                "warmup_done_s",
                "first_output_s",
            ],
            sorted(life_rows, key=lambda r: (r[1] is None, r[1] or 0)),
            "Node lifecycle milestones, seconds relative to bag play start",
        )
        readies = [d["ready"] for d in life.values() if "ready" in d]
        acts = [d["activated"] for d in life.values() if "activated" in d]
        if readies and acts:
            S["run/startup_s"] = max(acts) - min(readies)
        cd = life.get("cone_detection_node", {})
        if "warmup_start" in cd and "warmup_done" in cd:
            S["run/numba_warmup_s"] = cd["warmup_done"] - cd["warmup_start"]
        od = life.get("odometry_filter_node", {})
        if "activated" in od and "first_output" in od:
            S["odom/t_calibrated_s"] = od["first_output"] - od["activated"]

    # ------------------------------------------------------------- per-pattern
    cf, guard, pr, lat, prof, obs, step, ctrl = ([] for _ in range(8))
    warn_cat: dict[tuple[str, str, str], int] = defaultdict(int)
    n_warn = n_err = 0
    counters = defaultdict(int)
    pose_jump_devs: list[float] = []

    for node, lines in lines_by_node.items():
        for level, t, msg in lines:
            tr = rel(t)
            if level == "WARN":
                n_warn += 1
            elif level in ("ERROR", "FATAL"):
                n_err += 1
            if level in ("WARN", "ERROR", "FATAL"):
                key = re.sub(r"[-+]?\d+(\.\d+)?", "N", msg)[:70]
                warn_cat[(level, node, key)] += 1

            if m := RE_CONE_FILTER.search(msg):
                g = [_f(x) for x in m.groups()]
                n_scans = g[0]
                cf.append(
                    (
                        tr,
                        {
                            "points": g[1],
                            "clusters": g[2],
                            "gt3pts": g[3],
                            "shape_pass": g[4],
                            "residual": g[5],
                            "accepted": g[6],
                            "far_dropped": g[7],
                            "left": g[8],
                            "right": g[9],
                            "center": g[10],
                            "big_orange": g[11],
                            "dbscan_guard_frac": g[12] / g[13] if g[13] else math.nan,
                            "hz": g[14],
                            "scans_in_window": n_scans,
                        },
                    )
                )
            elif m := RE_DBSCAN.search(msg):
                guard.append((tr, {"pts_dropped": float(m.group(1))}))
                dig.events.append(Event(tr, node, "dbscan_guard", "warn", msg[:160]))
            elif m := RE_PATH_RATE.search(msg):
                g = [_f(x) for x in m.groups()]
                pr.append(
                    (
                        tr,
                        {
                            "cb_hz": g[0],
                            "pub_hz": g[1],
                            "no_cones": g[2],
                            "tf_miss": g[3],
                            "plan_empty": g[4],
                        },
                    )
                )
            elif RE_TF_FAIL.search(msg):
                counters["tf_fail"] += 1
                dig.events.append(
                    Event(tr, node, "tf_lookup_failed", "warn", msg[:160])
                )
            elif m := RE_SLAM_LAT.search(msg):
                g = [_f(x) for x in m.groups()]
                lat.append(
                    (
                        tr,
                        {
                            "proc_ms": g[0],
                            "age_ms": g[1],
                            "dt_pub_ms": g[2],
                            "corr_dx_m": g[3],
                            "corr_dy_m": g[4],
                            "corr_m": g[5],
                            "corr_deg": g[6],
                        },
                    )
                )
            elif m := RE_SLAM_PROF.search(msg):
                mode = m.group(1)
                g = [_f(x) for x in m.groups()[1:]]
                prof.append(
                    (
                        tr,
                        {
                            "total_ms": g[0],
                            "pre_ms": g[1],
                            "assoc_ms": g[2],
                            "spawn_ms": g[3],
                            "commit_ms": g[4],
                            "upd_ms": g[5],
                            "est_ms": g[6],
                            "iters": g[7],
                            "db_ms": g[8],
                            "pub_ms": g[9],
                            "obs": g[10],
                            "map": g[11],
                            "mode": float(SLAM_MODE_CODE.get(mode, -1)),
                        },
                    )
                )
            elif m := RE_SLAM_OBS.search(msg):
                g = [_f(x) for x in m.groups()]
                obs.append(
                    (
                        tr,
                        {
                            "obs": g[1],
                            "assoc": g[2],
                            "new": g[3],
                            "vetoed": g[4],
                            "frozen": g[5],
                            "skip": g[6],
                        },
                    )
                )
            elif m := RE_SLAM_STEP.search(msg):
                g = [_f(x) for x in m.groups()]
                step.append(
                    (
                        tr,
                        {
                            "step": g[0],
                            "obs": g[1],
                            "new": g[2],
                            "assoc": g[3],
                            "map": g[4],
                            "x": g[5],
                            "y": g[6],
                            "yaw_deg": g[7],
                        },
                    )
                )
            elif m := RE_POSE_JUMP.search(msg):
                dev_m, dev_deg = _f(m.group(1)), _f(m.group(2))
                pose_jump_devs.append(dev_m)
                counters["pose_jump"] += 1
                dig.events.append(
                    Event(
                        tr,
                        node,
                        "pose_jump_rejected",
                        "warn",
                        f"dev={dev_m:.2f} m {dev_deg:.1f} deg",
                    )
                )
            elif m := RE_DA_SKIP.search(msg):
                counters["da_skip"] += 1
                dig.events.append(
                    Event(
                        tr,
                        node,
                        "da_failure_skip",
                        "warn",
                        f"[{m.group(1)}] obs={m.group(2)} new={m.group(3)} assoc={m.group(4)}",
                    )
                )
            elif m := RE_CASCADE.search(msg):
                counters["cascade"] += 1
                dig.events.append(
                    Event(tr, node, "cascade_recovery", "warn", msg[:160])
                )
            elif m := RE_PROX.search(msg):
                counters["prox"] += int(m.group(1))
            elif m := RE_CTRL.search(msg):
                g = m.groups()
                ctrl.append(
                    (
                        tr,
                        {
                            "v_mps": _f(g[0]),
                            "travelled_m": _f(g[1]),
                            "throttle": _f(g[2]),
                            "regen": _f(g[3]),
                            "steer": _f(g[4]),
                            "path_n": _f(g[5]),
                            "path_len_m": _f(g[6]),
                            "stop_d_m": _f(g[7]),
                            "latched": 1.0 if g[9] == "True" else 0.0,
                        },
                    )
                )
            elif m := RE_STOP.search(msg):
                counters["stop_latched"] = 1
                dig.events.append(
                    Event(
                        tr,
                        node,
                        "stop_latched",
                        "info",
                        msg[:160],
                        x=_f(m.group(2)),
                        y=_f(m.group(3)),
                    )
                )

    for fam, rows in (
        ("log_perception_filter", cf),
        ("log_perception_guard", guard),
        ("log_planning_rate", pr),
        ("log_slam_latency", lat),
        ("log_slam_profile", prof),
        ("log_slam_obs", obs),
        ("log_slam_step", step),
        ("log_control_status", ctrl),
    ):
        s = _series(fam, sorted(rows, key=lambda r: r[0]))
        if s is not None:
            dig.series[fam] = s

    # ------------------------------------------------------------- crashes
    n_crash = 0
    for node, t, tb in crashes:
        noise = any(k in tb for k in SHUTDOWN_NOISE)
        last = tb.strip().splitlines()[-1][:200] if tb.strip() else "?"
        if noise:
            counters["shutdown_noise"] += 1
            continue
        n_crash += 1
        dig.events.append(Event(rel(t) if t else None, node, "crash", "error", last))

    # ------------------------------------------------------------- scalars
    if cf:
        v = dig.series["log_perception_filter"].values
        S["perception/filter_rate_mean_hz"] = _mean(v["hz"])
        S["perception/funnel_clusters_mean"] = _mean(v["clusters"])
        S["perception/funnel_shape_pass_mean"] = _mean(v["shape_pass"])
        S["perception/funnel_accepted_mean"] = _mean(v["accepted"])
        S["perception/funnel_far_dropped_mean"] = _mean(v["far_dropped"])
        lsum, rsum = np.nansum(v["left"]), np.nansum(v["right"])
        S["perception/side_balance_frac"] = (
            float(lsum / (lsum + rsum)) if lsum + rsum else None
        )
    S["perception/n_dbscan_guard_trips"] = len(guard)
    S["perception/dbscan_guard_pts_dropped_mean"] = _mean(
        [r["pts_dropped"] for _, r in guard]
    )
    if pr:
        v = dig.series["log_planning_rate"].values
        S["planning/cb_rate_mean_hz"] = _mean(v["cb_hz"])
        S["planning/pub_rate_mean_hz"] = _mean(v["pub_hz"])
        S["planning/n_no_cones"] = int(np.nansum(v["no_cones"]))
        S["planning/n_tf_miss"] = int(np.nansum(v["tf_miss"])) + counters["tf_fail"]
        S["planning/n_plan_empty"] = int(np.nansum(v["plan_empty"]))
    if lat:
        v = dig.series["log_slam_latency"].values
        S["slam/proc_p50_ms"] = _pct(v["proc_ms"], 50)
        S["slam/proc_p95_ms"] = _pct(v["proc_ms"], 95)
        S["slam/age_p95_ms"] = _pct(v["age_ms"], 95)
        S["latency/slam_p95_ms"] = S["slam/proc_p95_ms"]
    if prof:
        v = dig.series["log_slam_profile"].values
        S["slam/prof_total_p95_ms"] = _pct(v["total_ms"], 95)
        mode = v["mode"]
        S["slam/loc_mode_frac"] = float(np.mean(mode == 1))
        loc_t = dig.series["log_slam_profile"].step[mode == 1]
        S["slam/t_first_loc_s"] = float(loc_t[0]) if loc_t.size else None
    if obs:
        v = dig.series["log_slam_obs"].values
        o, a = np.nansum(v["obs"]), np.nansum(v["assoc"])
        S["slam/assoc_frac"] = float(a / o) if o else None
    S["slam/n_pose_jump_rejected"] = counters["pose_jump"]
    S["slam/pose_jump_dev_max_m"] = max(pose_jump_devs) if pose_jump_devs else None
    S["slam/n_da_failure_skips"] = counters["da_skip"]
    S["slam/n_cascade_recoveries"] = counters["cascade"]
    S["slam/n_proximity_vetoes"] = counters["prox"]
    if ctrl:
        v = dig.series["log_control_status"].values
        S["planning/path_length_mean_m"] = _mean(v["path_len_m"])
    S["control/stop_latched"] = counters["stop_latched"]
    S["run/n_crashes"] = n_crash
    S["run/n_warnings"] = n_warn
    S["run/n_errors"] = n_err
    S["run/n_viewer_clients"] = viewer_clients
    if dig.played and dig.series:
        S["run/replay_duration_s"] = max(
            float(s.step.max()) for s in dig.series.values()
        )

    dig.warn_catalogue = Table(
        "warn_catalogue",
        ["level", "node", "message_pattern", "count"],
        sorted(
            ([lvl, node, key, n] for (lvl, node, key), n in warn_cat.items()),
            key=lambda r: -r[3],
        ),
        "WARN/ERROR lines grouped by message pattern (numbers replaced by N)",
    )
    return dig
