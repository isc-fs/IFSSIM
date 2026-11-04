"""One headless run, and a loop of them.

A run is: start the sim offscreen, make sure Mission Control's backend sees
it, run the mission benchmark (tools/sim_benchmark/run_mission_benchmark.py,
which loads tracks, starts missions, follows the referee and scores), watch
the sim the whole time, stop it. The benchmark owns everything about the
missions; this owns the processes around it, which is what an unattended
machine needs: nobody is there to restart a sim that crashed.

Every run writes ``<results>/headless/<stamp>/run.json`` and appends one line
to ``<results>/headless/index.jsonl``. The benchmark's own results for the run
go in that same folder (``<stamp>/mission/...``): the benchmark names its
folders by mission, track, commit and seed, so two runs sharing a results root
would overwrite each other, and a loop repeats exactly those.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Dict, List, Optional

from .simproc import RPC_PORT, SimCommand, SimError, SimProcess, rpc

TOOLS = Path(__file__).resolve().parent.parent
REPO = TOOLS.parent
BENCHMARK = TOOLS / "sim_benchmark" / "run_mission_benchmark.py"

# What the benchmark's exit code means (see its main()).
BENCH_OUTCOMES = {0: "finished", 1: "not_finished", 2: "benchmark_error"}
# Outcomes that say the machine, not the pipeline, failed. A loop gives up
# after a few of these in a row instead of failing all night.
INFRA_FAILURES = {"sim_failed_to_start", "sim_crashed", "backend_unreachable", "benchmark_error",
                  "run_timeout"}


@dataclass
class RunConfig:
    sim_command: Optional[SimCommand]          # None: attach to a running sim
    results_root: Path
    benchmark_args: List[str] = field(default_factory=list)
    mc_url: str = "http://127.0.0.1:8000"
    mc_key: str = ""
    mc_up: bool = False
    compose_dir: Path = REPO
    host: str = "127.0.0.1"
    port: int = RPC_PORT
    sim_ready_timeout_s: float = 900.0         # first boot compiles shaders
    mc_timeout_s: float = 180.0
    max_run_s: float = 0.0                     # 0: no cap beyond the benchmark's own timeouts
    # No mission starts until the sim keeps real time. For ~30 s after boot UE compiles its PSO
    # cache on every core and the sim runs at 0.7-0.9x; a car driven then starts to oscillate and
    # leaves the track (the same happens to a slowed-down sim at any age). Until 2026-11 the
    # stack's ~36 s colcon rebuild hid this. Waits until sim_time advanced at >= settle_rtf of the
    # wall clock over settle_window_s, at most settle_s (then goes ahead, noted in the record).
    settle_s: float = 120.0                    # 0: don't wait
    settle_rtf: float = 0.98
    settle_window_s: float = 5.0
    benchmark_cmd: List[str] = field(default_factory=lambda: [sys.executable, str(BENCHMARK)])
    poll_s: float = 2.0


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _get_json(url: str, key: str = "", timeout: float = 3.0) -> Optional[dict]:
    req = urllib.request.Request(url, headers={"X-API-Key": key} if key else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except (urllib.error.URLError, OSError, ValueError):
        return None


def bring_up_backend(cfg: RunConfig, log) -> None:
    """``docker compose up -d mission_control_backend``.

    On Linux the backend runs with host networking, so it reaches the sim on
    127.0.0.1; compose's default (host.docker.internal) is for Docker Desktop.
    """
    env = dict(os.environ)
    if sys.platform.startswith("linux"):
        env.setdefault("IFSSIM_HOST", "127.0.0.1")
    cmd = ["docker", "compose", "up", "-d", "mission_control_backend"]
    log(f"$ {' '.join(cmd)}  (in {cfg.compose_dir})")
    subprocess.run(cmd, cwd=cfg.compose_dir, env=env, check=True)


def wait_backend_sees_sim(cfg: RunConfig, should_stop: Callable[[], bool]) -> float:
    t0 = time.monotonic()
    while time.monotonic() - t0 < cfg.mc_timeout_s:
        if should_stop():
            raise KeyboardInterrupt
        st = _get_json(f"{cfg.mc_url}/api/sim/status", cfg.mc_key)
        if st and st.get("connected"):
            return time.monotonic() - t0
        time.sleep(2.0)
    raise SimError(f"Mission Control at {cfg.mc_url} didn't see the sim within {cfg.mc_timeout_s:.0f} s "
                   "(is the backend up? --mc-up starts it)")


def wait_real_time(cfg: RunConfig, should_stop: Callable[[], bool]) -> Dict:
    """Waits until the sim's clock keeps up with the wall clock (see RunConfig.settle_s).
    Returns settle_s (the wait), settle_rtf (the last rate measured) and settle_timeout."""
    t0 = time.monotonic()
    samples: List = []  # (wall, sim_time)
    rtf = 0.0
    while True:
        if should_stop():
            raise KeyboardInterrupt
        now = time.monotonic()
        try:
            st = json.loads(rpc(cfg.host, "getSimStatus", port=cfg.port) or "{}")
            samples.append((now, float(st["sim_time"])))
        except (OSError, ValueError, KeyError, TypeError):
            pass
        old = [s for s in samples if samples[-1][0] - s[0] >= cfg.settle_window_s]
        if old:
            (w0, s0), (w1, s1) = old[-1], samples[-1]
            rtf = (s1 - s0) / (w1 - w0)
            if s1 > 1.0 and rtf >= cfg.settle_rtf:
                return {"settle_s": round(now - t0, 1), "settle_rtf": round(rtf, 3)}
        if now - t0 >= cfg.settle_s:
            return {"settle_s": round(now - t0, 1), "settle_rtf": round(rtf, 3),
                    "settle_timeout": True}
        time.sleep(min(cfg.poll_s, 0.5))


def _manifests_since(root: Path, since: float) -> set:
    """Benchmark runs written since ``since`` (epoch s). By time, not by name:
    a repeat of the same mission, track, commit and seed rewrites the same folder."""
    mission = root / "mission"
    if not mission.is_dir():
        return set()
    return {p for p in mission.rglob("manifest.json") if p.stat().st_mtime >= since}


def _terminate(proc: subprocess.Popen, grace_s: float = 30.0) -> None:
    if proc.poll() is not None:
        return
    try:
        if sys.platform == "win32":
            proc.terminate()
        else:
            os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=grace_s)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            if sys.platform == "win32":
                proc.kill()
            else:
                os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=10)


def run_once(cfg: RunConfig, label: str = "", log=print,
             should_stop: Callable[[], bool] = lambda: False) -> Dict:
    """One run. Never raises for a failed run: the outcome is in the record."""
    root = Path(cfg.results_root)
    run_dir = root / "headless" / (_stamp() + (f"_{label}" if label else ""))
    run_dir.mkdir(parents=True, exist_ok=True)
    sim = SimProcess(cfg.sim_command, log_path=run_dir / "sim.log", host=cfg.host,
                     port=cfg.port)
    rec: Dict = {
        "started": _now(),
        "label": label,
        "run_dir": str(run_dir),
        "sim": "attach" if sim.attached else cfg.sim_command.argv,
        "benchmark_args": cfg.benchmark_args,
        "outcome": None,
    }
    since = time.time() - 1.0
    bench: Optional[subprocess.Popen] = None
    t_start = time.monotonic()
    try:
        # Cheap preflight before a slow sim boot: does the benchmark even start
        # with this Python (its imports, e.g. PyYAML)?
        pre = subprocess.run(cfg.benchmark_cmd + ["--help"], capture_output=True, text=True,
                             timeout=60, cwd=str(REPO))
        if pre.returncode != 0:
            tail = (pre.stderr or pre.stdout).strip().splitlines()[-1:] or [""]
            rec["outcome"] = "benchmark_error"
            rec["error"] = f"the benchmark doesn't start with {cfg.benchmark_cmd[0]}: {tail[0]}"
            return rec
        log(f"[{label or 'run'}] starting the sim" + (" (attach)" if sim.attached else ""))
        try:
            sim.start()
            rec["sim_ready_s"] = round(sim.wait_ready(cfg.sim_ready_timeout_s, should_stop=should_stop), 1)
        except SimError as e:
            rec["outcome"], rec["error"] = "sim_failed_to_start", str(e)
            return rec
        try:
            rec["sim_status"] = json.loads(rpc(cfg.host, "getSimStatus", port=cfg.port) or "{}")
        except (OSError, ValueError):
            pass
        log(f"[{label or 'run'}] sim ready in {rec.get('sim_ready_s', 0)} s")

        try:
            if cfg.mc_up:
                bring_up_backend(cfg, log)
            rec["backend_ready_s"] = round(wait_backend_sees_sim(cfg, should_stop), 1)
        except (SimError, subprocess.CalledProcessError, FileNotFoundError) as e:
            rec["outcome"], rec["error"] = "backend_unreachable", str(e)
            return rec

        if not sim.attached and cfg.settle_s > 0:
            log(f"[{label or 'run'}] waiting for the sim to keep real time")
            rec.update(wait_real_time(cfg, should_stop))
            log(f"[{label or 'run'}] sim at {rec['settle_rtf']:.2f}x real time after "
                f"{rec['settle_s']:.0f} s" + (" (gave up waiting)" if rec.get("settle_timeout") else ""))

        argv = cfg.benchmark_cmd + [
            "--sim", f"{cfg.host}:{cfg.port}", "--mc-url", cfg.mc_url,
            "--results-root", str(run_dir),
        ] + (["--mc-key", cfg.mc_key] if cfg.mc_key else []) + cfg.benchmark_args
        rec["benchmark_cmd"] = argv
        log(f"[{label or 'run'}] benchmark: {' '.join(argv[1:])}")
        with open(run_dir / "benchmark.log", "wb") as blog:
            bench = subprocess.Popen(argv, stdout=blog, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     cwd=str(REPO), start_new_session=(sys.platform != "win32"))
            while bench.poll() is None:
                time.sleep(cfg.poll_s)
                if should_stop():
                    _terminate(bench)
                    rec["outcome"] = "interrupted"
                    return rec
                if not sim.alive():
                    _terminate(bench)
                    rec["outcome"] = "sim_crashed"
                    rec["error"] = "the sim exited mid-run; see sim.log"
                    return rec
                if cfg.max_run_s and time.monotonic() - t_start > cfg.max_run_s:
                    _terminate(bench)
                    rec["outcome"] = "run_timeout"
                    return rec
        rec["benchmark_exit"] = bench.returncode
        rec["outcome"] = BENCH_OUTCOMES.get(bench.returncode, "benchmark_error")
        # 1 means "a mission didn't finish", but it is also what a Python
        # traceback exits with. No mission run written = the benchmark itself
        # failed (a missing module, a bad flag), not the pipeline.
        if rec["outcome"] == "not_finished" and not _manifests_since(run_dir, since):
            rec["outcome"] = "benchmark_error"
            rec["error"] = "the benchmark exited 1 without running a mission; see benchmark.log"
        return rec
    except KeyboardInterrupt:
        if bench is not None:
            _terminate(bench)
        rec["outcome"] = "interrupted"
        return rec
    finally:
        sim.stop()
        rec["mission_runs"] = sorted(str(p.parent) for p in _manifests_since(run_dir, since))
        rec["ended"] = _now()
        rec["duration_s"] = round(time.monotonic() - t_start, 1)
        (run_dir / "run.json").write_text(json.dumps(rec, indent=2) + "\n")
        with open(root / "headless" / "index.jsonl", "a") as idx:
            idx.write(json.dumps({k: rec.get(k) for k in
                                  ("started", "ended", "label", "outcome", "duration_s", "run_dir",
                                   "mission_runs")}) + "\n")
        log(f"[{label or 'run'}] {rec['outcome']} in {rec['duration_s']} s -> {run_dir}")


def exit_code(outcome: str) -> int:
    return {"finished": 0, "not_finished": 1, "interrupted": 130}.get(outcome, 3)


def run_loop(cfg: RunConfig, cycles: int = 0, pause_s: float = 10.0, max_infra_failures: int = 3,
             log=print, should_stop: Callable[[], bool] = lambda: False) -> List[Dict]:
    """Run again and again: ``cycles`` times (0: until stopped), a fresh sim each time.

    Stops early after ``max_infra_failures`` machine failures in a row (sim
    won't start, crashes, backend down): those don't fix themselves overnight.
    """
    records: List[Dict] = []
    streak = 0
    i = 0
    while not should_stop() and (cycles == 0 or i < cycles):
        i += 1
        rec = run_once(cfg, label=f"cycle{i:04d}", log=log, should_stop=should_stop)
        records.append(rec)
        if rec["outcome"] == "interrupted":
            break
        streak = streak + 1 if rec["outcome"] in INFRA_FAILURES else 0
        if max_infra_failures and streak >= max_infra_failures:
            log(f"stopping: {streak} machine failures in a row (last: {rec['outcome']}: "
                f"{rec.get('error', '')})")
            break
        if cycles == 0 or i < cycles:
            t = time.monotonic()
            while time.monotonic() - t < pause_s and not should_stop():
                time.sleep(0.5)
    return records
