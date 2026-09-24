"""Baselines and deltas, computed once and shared by every backend.

A baseline is chosen per ``scenario.id`` (see ``choose_baselines``). Every
other bundle of that scenario gets:

* ``<metric>.delta`` summary entries for registry metrics with a direction,
* ``compare/n_regressions`` / ``compare/n_improvements``,
* a ``baseline_comparison`` table (metric, value, baseline, delta, verdict),
* delta *series* where the x axes are comparable: cone count vs replay time
  for same-bag replays, |cross-track| vs track distance for sim aggregates.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from . import registry
from .bundle import RunBundle, Series, Table
from .geometry import interp

REPLAY_JOBS = {"onboard_replay", "live_replay"}
SIM_AGG_JOBS = {"sim_aggregate", "sim_sweep_trial"}


def choose_baselines(bundles: Iterable[RunBundle]) -> dict[str, RunBundle]:
    """Default convention (open question Q3 — replace when the team decides):

    * bag replays: the earliest *finished* ``onboard_replay`` of the scenario;
    * sim: the aggregate tagged ``baseline`` (the dev commit of the matrix).
    """
    out: dict[str, RunBundle] = {}
    bl = list(bundles)
    for b in sorted(bl, key=lambda b: b.started_at):
        sid = b.scenario_id
        if b.job_type == "onboard_replay" and b.status == "finished" and sid not in out:
            out[sid] = b
    for b in bl:
        if (
            b.job_type == "sim_aggregate"
            and "baseline" in b.tags
            and "matrix" in b.tags
        ):
            out[b.scenario_id] = b
    return out


def _comparable(b: RunBundle, base: RunBundle) -> bool:
    if b is base:
        return False
    if b.job_type in REPLAY_JOBS:
        return base.job_type in REPLAY_JOBS
    if b.job_type in SIM_AGG_JOBS or b.job_type == "sim_e2e":
        return base.job_type in SIM_AGG_JOBS
    return False


def apply(bundles: list[RunBundle], baselines: dict[str, RunBundle]) -> None:
    for b in bundles:
        base = baselines.get(b.scenario_id)
        if base is b:
            b.tags = sorted(set(b.tags) | {"baseline"})
            b.config.setdefault("compare", {})["is_baseline"] = True
            continue
        if base is None or not _comparable(b, base):
            continue
        b.config.setdefault("compare", {}).update(
            {"baseline_run": base.name, "baseline_group": base.group}
        )
        rows = []
        n_reg = n_imp = 0
        for k, v in sorted(b.summary.items()):
            sp = registry.spec(k)
            if sp is None or sp.direction == "none" or not isinstance(v, (int, float)):
                continue
            bv = base.summary.get(k)
            if not isinstance(bv, (int, float)):
                continue
            d = registry.delta(k, float(v), float(bv))
            if d["delta"] is None:
                continue
            b.summary[f"{k}.delta"] = d["delta"]
            verdict = (
                "regression"
                if d["regression"]
                else "improvement"
                if d["improvement"]
                else "within tolerance"
            )
            n_reg += bool(d["regression"])
            n_imp += bool(d["improvement"])
            rel = d["delta"] / abs(bv) if bv else math.nan
            rows.append(
                [
                    k,
                    float(v),
                    float(bv),
                    d["delta"],
                    rel,
                    sp.direction,
                    sp.threshold,
                    verdict,
                ]
            )
        b.summary["compare/n_regressions"] = n_reg
        b.summary["compare/n_improvements"] = n_imp
        b.add_table(
            Table(
                "baseline_comparison",
                [
                    "metric",
                    "value",
                    "baseline",
                    "delta",
                    "rel_delta",
                    "better",
                    "tolerance",
                    "verdict",
                ],
                rows,
                f"vs baseline {base.name}",
            )
        )
        _delta_series(b, base)


def _delta_series(b: RunBundle, base: RunBundle) -> None:
    same_bag = b.config.get("scenario", {}).get("bag_id") and b.config["scenario"].get(
        "bag_id"
    ) == base.config.get("scenario", {}).get("bag_id")
    if (
        same_bag
        and "replay_perception" in b.series
        and "replay_perception" in base.series
    ):
        s, sb = b.series["replay_perception"], base.series["replay_perception"]
        # smooth both over 1 s before differencing: scan stamps never coincide
        k = 10
        ker = np.ones(k) / k
        v = np.convolve(s.values["n_cones_raw"], ker, "same")
        vb = np.convolve(sb.values["n_cones_raw"], ker, "same")
        bi = interp(sb.step, vb, s.step)
        b.add_series(
            Series(
                "replay_perception_delta",
                "t/replay_s",
                s.step,
                {
                    "n_cones_raw_1s": v,
                    "baseline_n_cones_raw_1s": bi,
                    "n_cones_raw_delta": v - bi,
                },
            )
        )
    if "agg_track_profile" in b.series and "agg_track_profile" in base.series:
        s, sb = b.series["agg_track_profile"], base.series["agg_track_profile"]
        if len(s) == len(sb):
            b.add_series(
                Series(
                    "agg_track_profile_delta",
                    "track/s_lap_m",
                    s.step,
                    {
                        "cte_m_mean_delta": s.values["cte_m_mean"]
                        - sb.values["cte_m_mean"],
                        "speed_mps_mean_delta": s.values["speed_mps_mean"]
                        - sb.values["speed_mps_mean"],
                        "baseline_cte_m_mean": sb.values["cte_m_mean"],
                        "baseline_speed_mps_mean": sb.values["speed_mps_mean"],
                    },
                )
            )


def bootstrap_diff(
    a: list[float], b: list[float], n: int = 4000, seed: int = 0
) -> dict[str, float | None]:
    """Mean(b) - mean(a) with a 95% bootstrap CI and P(diff > 0). No scipy needed."""
    a = np.array([x for x in a if x is not None and math.isfinite(x)], float)
    b = np.array([x for x in b if x is not None and math.isfinite(x)], float)
    if len(a) < 2 or len(b) < 2:
        return {"diff": None, "lo": None, "hi": None, "p_gt0": None}
    rng = np.random.default_rng(seed)
    d = rng.choice(b, (n, len(b))).mean(1) - rng.choice(a, (n, len(a))).mean(1)
    return {
        "diff": float(b.mean() - a.mean()),
        "lo": float(np.percentile(d, 2.5)),
        "hi": float(np.percentile(d, 97.5)),
        "p_gt0": float((d > 0).mean()),
    }
