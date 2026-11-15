"""clutter_report: outcomes per kind, the Fisher p and the P2a verdict, from fake runs."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import clutter_report as cr  # noqa: E402


def _runs(root: Path, kind: str, outcomes: list[str], commit="abc1234"):
    for i, outcome in enumerate(outcomes):
        env, seed = i // 5 + 1, i % 5 + 1
        d = root / "mission" / f"autocross_clutter_{kind}_e{env}" / commit / f"seed{seed}"
        d.mkdir(parents=True, exist_ok=True)
        (d / "manifest.json").write_text(json.dumps({"outcome": outcome}))
        (d / "results.json").write_text(json.dumps({"race/n_doo": 1, "race/run_sim_time_s": 50.0}))


def test_a_kind_that_stops_early_where_the_control_never_does_is_reproduced(tmp_path):
    _runs(tmp_path, "control", ["finished"] * 25)
    _runs(tmp_path, "narrow", ["stopped_early"] * 4 + ["finished"] * 21)
    _runs(tmp_path, "tripod", ["stopped_early"] * 2 + ["finished"] * 23)
    s = cr.summarise(cr.load(tmp_path))
    assert s["control_runs"] == 25 and s["control_stops"] == 0
    assert s["kinds"]["narrow"]["reproduced"] and s["kinds"]["narrow"]["stopped_early"] == 4
    assert not s["kinds"]["tripod"]["reproduced"]          # fewer than 3
    assert s["kinds"]["narrow"]["p_vs_control"] < 0.06
    assert s["kinds"]["narrow"]["stops_per_env_seed"] == {1: 4}
    md = cr.markdown(s)
    assert "**reproduced**" in md and "| control | 25 | 25 | 0 |" in md


def test_a_control_that_stops_too_means_nothing_is_reproduced(tmp_path):
    _runs(tmp_path, "control", ["stopped_early"] + ["finished"] * 24)
    _runs(tmp_path, "narrow", ["stopped_early"] * 10 + ["finished"] * 15)
    assert not cr.summarise(cr.load(tmp_path))["kinds"]["narrow"]["reproduced"]


def test_fisher_exact():
    assert cr.fisher_greater(0, 25, 0, 25) == 1.0
    # 3 of 25 against 0 of 25: p = C(25,3) / C(50,3)
    assert abs(cr.fisher_greater(3, 25, 0, 25) - 2300 / 19600) < 1e-12


def _telemetry(d: Path, stop_at_s: float | None):
    """5 m/s along x from t=0, standing still from stop_at_s, to t=60."""
    rows, x = ["t,x,y,speed,throttle,steering"], 0.0
    for i in range(601):
        t = i * 0.1
        v = 0.0 if stop_at_s is not None and t >= stop_at_s else 5.0
        x += v * 0.1
        rows.append(f"{t:.1f},{x:.3f},0.0,{v},0.0,0.0")
    (d / "telemetry.csv").write_text("\n".join(rows))


def test_a_car_parked_short_of_the_finish_stopped_early_whatever_the_benchmark_said(tmp_path):
    _runs(tmp_path, "narrow", ["timeout", "timeout", "emergency"])
    seeds = sorted((tmp_path / "mission").rglob("seed*"))
    _telemetry(seeds[0], 6.0)      # 30 m, then parked: stopped early
    _telemetry(seeds[1], None)     # drove to the timeout: a timeout
    _telemetry(seeds[2], 1.0)      # 5 m, then still: never really left, not an early stop
    rs = sorted(cr.load(tmp_path)["narrow"], key=lambda r: r["seed"])
    assert [r["outcome"] for r in rs] == ["stopped_early", "timeout", "emergency"]
    assert rs[0]["benchmark_outcome"] == "timeout" and abs(rs[0]["stall"]["travelled_m"] - 30.0) <= 0.5
    assert cr.standstill(tmp_path / "missing.csv") is None


def test_runs_under_headless_run_folders_are_found(tmp_path):
    _runs(tmp_path / "narrow" / "headless" / "20261115_212323", "narrow", ["finished"] * 2)
    assert len(cr.load(tmp_path)["narrow"]) == 2


def test_the_newest_commit_wins_unless_one_is_given(tmp_path):
    _runs(tmp_path, "narrow", ["finished"] * 5, commit="old0000")
    _runs(tmp_path, "narrow", ["stopped_early"] * 5, commit="new1111")
    import os, time
    for f in (tmp_path / "mission").rglob("old0000/*/manifest.json"):
        os.utime(f, (time.time() - 3600, time.time() - 3600))
    assert cr.summarise(cr.load(tmp_path))["kinds"]["narrow"]["stopped_early"] == 5
    assert cr.summarise(cr.load(tmp_path, "old"))["kinds"]["narrow"]["stopped_early"] == 0
