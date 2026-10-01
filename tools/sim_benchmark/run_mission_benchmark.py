#!/usr/bin/env python3
"""Mission benchmark: the pipeline drives the missions in the simulator, scored by the referee.

For each mission, track and repeat (in that order), one run:

  1. tear down the previous run's autonomy (Mission Control ``/api/pipeline/stop``);
  2. load the track, and reset the scenario with the repeat's seed (``resetScenario <seed>``:
     the sim's sensor noise is seeded too, so a repeat is a genuine sample);
  3. optionally move the car off the start gate by a seeded random offset (``start_pose.noise``);
  4. start the mission the way the operator panel does (``/api/event/start``: arm, wait for the
     pipeline to be ready, the EKF's calibration window with the brake held, go);
  5. follow the referee (``/api/event/state``) until it says finished, or the timeout;
  6. write the run: ``<results>/mission/<mission>_<track>/<code>/seed<k>/`` with ``manifest.json``
     (code, scenario, referee state), ``results.json`` (race metrics), ``laps.csv``,
     ``events.csv`` and a coarse ``telemetry.csv``, the layout bench-view's Sim pages read.

WHERE THE PIPELINE RUNS (``--pipeline-on``):

  bench_pc     On this computer. With ``--stack own`` (the default) the benchmark starts its own
               pipeline container (``bench-dv-stack``) from THIS checkout's ``pipeline/``: a copy of
               it is mounted into the image (``$IFSSIM_DV_IMAGE``, else the one ``docker compose``
               builds) and rebuilt at start-up, so the runs use exactly this checkout's pipeline
               commit, which is what gets recorded. This is how to try the benchmark on any of our
               computers, without the latte panda.
  latte_panda  On the latte panda. The benchmark's container runs only the sim bridge here; the
               pipeline (``ros2 launch bringup sim_pipeline.launch.py``: the autonomy against a
               bridge elsewhere) runs there, started and stopped by ``--panda-start`` /
               ``--panda-stop`` (e.g. over ``ssh``), which are required because how the latte panda
               is reached isn't settled yet (docs/PLAN.md in IFS-DV-BENCHWEB). ``--panda-sha``
               prints the commit it runs, recorded with every run.

``--stack running`` uses the ``dv_pipeline_stack`` that is already running instead (quick tries):
the code is then whatever that container has, and each run says so. Only one pipeline may drive
the sim, so ``--stack own`` refuses while another stack runs, unless ``--replace-stack`` (it stops
that one for the benchmark and starts it again at the end).

PARAMETER OVERRIDES (``--pipeline-overrides FILE``, JSON ``{node: {param: value}}``; the launcher
writes it from a spec's ``pipeline:`` section): checked against this checkout's
``pipeline/bringup/config/params.yaml`` (unknown node, unknown parameter or wrong type: refused
before anything runs), merged into it, and loaded by every launch of the benchmark's own stack.
``cone_detection`` means ``cone_detection_node`` (the same settings the simulator bag benchmark
calls ``cone_detection``). Needs ``--stack own`` on the bench PC, and a pipeline commit that has
``params.yaml``; each run records what was overridden.

The sim and Mission Control must already be up: the sim on ``--sim`` (default 127.0.0.1:41451),
Mission Control's backend on ``--mc-url`` (default http://127.0.0.1:8000, ``--mc-key`` if it has an
API key). This script itself needs no ROS: it's HTTP, the sim's line protocol and ``docker``.

Progress (mission, track, lap) goes to ``$BENCH_PROGRESS_FILE`` when the launcher's worker sets it.

    python3 tools/sim_benchmark/run_mission_benchmark.py --missions acceleration,skidpad
    python3 tools/sim_benchmark/run_mission_benchmark.py --missions trackdrive --repeats 3 \\
        --start-noise --pipeline-on latte_panda --panda-start "ssh dv@panda ./start.sh" ...
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
import random
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
DEFAULTS_FILE = HERE / "missions.yaml"
MISSIONS = ("acceleration", "skidpad", "autocross", "trackdrive")
POSE_KEYS = ("noise", "lateral_m", "longitudinal_m", "yaw_deg")
MISSION_KEYS = ("mc_name", "tracks", "repeats", "start_pose", "laps", "timeout_s")
DOO_PENALTY_S = 2.0  # FS rules: +2 s per cone down or out
OC_PENALTY_S = 10.0  # +10 s per off course
START_TIMEOUT_S = 330.0  # /api/event/start: prepare (Numba JIT) + EKF calibration + go
POLL_S = 1.0


class BenchmarkError(Exception):
    """Something the run can't recover from (sim or Mission Control not there, bad settings)."""


# ------------------------------------------------------------------ settings
def load_missions(path: Path = DEFAULTS_FILE) -> dict[str, dict]:
    return yaml.safe_load(path.read_text())


def mission_settings(
    names: list[str],
    *,
    defaults: dict[str, dict] | None = None,
    repeats: int | None = None,
    laps: int | None = None,
    timeout_s: float | None = None,
    start_noise: bool | None = None,
    tracks: list[str] | None = None,
    overrides: dict[str, dict] | None = None,
) -> dict[str, dict]:
    """Each mission's settings: the defaults, then the flags (for every mission), then per-mission
    ``overrides`` (``{"trackdrive": {"laps": 3}}``). Unknown missions or keys are refused."""
    base = defaults or load_missions()
    out = {}
    for name in names:
        if name not in base:
            raise BenchmarkError(f"unknown mission {name!r} (known: {', '.join(base)})")
        m = copy.deepcopy(base[name])
        if repeats is not None:
            m["repeats"] = repeats
        if laps is not None:
            m["laps"] = laps
        if timeout_s is not None:
            m["timeout_s"] = timeout_s
        if start_noise is not None:
            m["start_pose"]["noise"] = start_noise
        if tracks:
            m["tracks"] = list(tracks)
        for k, v in ((overrides or {}).get(name) or {}).items():
            if k not in MISSION_KEYS:
                raise BenchmarkError(
                    f"{name}: unknown setting {k!r} (known: {', '.join(MISSION_KEYS)})"
                )
            if k == "start_pose":
                bad = set(v) - set(POSE_KEYS)
                if bad:
                    raise BenchmarkError(
                        f"{name}.start_pose: unknown {', '.join(sorted(bad))}"
                    )
                m["start_pose"].update(v)
            else:
                m[k] = v
        if (
            not m["tracks"]
            or int(m["repeats"]) < 1
            or int(m["laps"]) < 1
            or float(m["timeout_s"]) <= 0
        ):
            raise BenchmarkError(
                f"{name}: needs tracks, repeats >= 1, laps >= 1 and a timeout"
            )
        out[name] = m
    return out


@dataclass
class PlannedRun:
    index: int
    mission: str
    track: str
    seed: int  # the repeat, 1-based
    settings: dict[str, Any]


def plan(missions: dict[str, dict]) -> list[PlannedRun]:
    """Every run in order: missions as given, then tracks, then repeats."""
    runs: list[PlannedRun] = []
    for name, m in missions.items():
        for track in m["tracks"]:
            for seed in range(1, int(m["repeats"]) + 1):
                runs.append(PlannedRun(len(runs), name, track, seed, m))
    return runs


# ------------------------------------------------------------------ the sim and Mission Control
class Sim:
    """The sim's RPC: one command per line, one JSON (or text) answer per line."""

    def __init__(
        self, host: str = "127.0.0.1", port: int = 41451, timeout: float = 30.0
    ) -> None:
        self.host, self.port, self.timeout = host, port, timeout

    def cmd(self, line: str) -> str:
        try:
            with socket.create_connection(
                (self.host, self.port), timeout=self.timeout
            ) as s:
                s.sendall((line + "\n").encode())
                buf = b""
                while b"\n" not in buf:
                    chunk = s.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
        except OSError as e:
            raise BenchmarkError(
                f"the sim at {self.host}:{self.port} isn't reachable ({e})"
            ) from None
        return buf.split(b"\n", 1)[0].decode(errors="replace")

    def json(self, line: str) -> dict:
        try:
            out = json.loads(self.cmd(line) or "{}")
        except ValueError:
            return {}
        return out if isinstance(out, dict) else {}


class MissionControl:
    def __init__(
        self,
        url: str = "http://127.0.0.1:8000",
        key: str | None = None,
        timeout: float = 30.0,
    ) -> None:
        self.url, self.key, self.timeout = url.rstrip("/"), key, timeout

    def call(
        self,
        method: str,
        path: str,
        body: dict | None = None,
        timeout: float | None = None,
    ) -> tuple[int, Any]:
        headers = {"Content-Type": "application/json"}
        if self.key:
            headers["X-API-Key"] = self.key
        req = urllib.request.Request(
            self.url + path,
            method=method,
            headers=headers,
            data=json.dumps(body).encode()
            if body is not None
            else (b"" if method == "POST" else None),
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                raw = r.read()
                return r.status, json.loads(raw) if raw else None
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read() or b"null")
            except ValueError:
                return e.code, None
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise BenchmarkError(
                f"Mission Control at {self.url} isn't reachable ({e})"
            ) from None

    def get(self, path: str) -> Any:
        status, body = self.call("GET", path)
        return body if status == 200 else None

    def post(
        self, path: str, body: dict | None = None, timeout: float | None = None
    ) -> tuple[int, Any]:
        return self.call("POST", path, body, timeout)


# ------------------------------------------------------------------ where the pipeline runs
@dataclass
class PipelineHost:
    where: str  # bench_pc | latte_panda
    start: str | None = None
    stop: str | None = None
    sha_cmd: str | None = None
    sha: str | None = None

    def check(self) -> None:
        if self.where not in ("bench_pc", "latte_panda"):
            raise BenchmarkError(
                f"--pipeline-on: bench_pc or latte_panda, not {self.where!r}"
            )
        if self.where == "latte_panda" and not (self.start and self.stop):
            raise BenchmarkError(
                "--pipeline-on latte_panda needs --panda-start and --panda-stop: the commands that start "
                "and stop the pipeline on the latte panda (how it's reached isn't settled yet). "
                "To try the benchmark without it, use --pipeline-on bench_pc."
            )

    def _run(self, what: str, command: str) -> str:
        r = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            text=True,
            check=False,
            timeout=600,
        )
        if r.returncode != 0:
            raise BenchmarkError(
                f"{what} failed ({r.returncode}): {(r.stderr or r.stdout).strip()[:300]}"
            )
        return r.stdout.strip()

    def up(self, log) -> None:
        if self.where == "latte_panda":
            log(f"starting the pipeline on the latte panda: {self.start}")
            self._run("--panda-start", self.start)
        if self.sha_cmd:
            try:
                self.sha = self._run("--panda-sha", self.sha_cmd).split()[0]
            except (BenchmarkError, IndexError):
                self.sha = None

    def down(self, log) -> None:
        if self.where == "latte_panda" and self.stop:
            log("stopping the pipeline on the latte panda")
            try:
                self._run("--panda-stop", self.stop)
            except (BenchmarkError, subprocess.TimeoutExpired) as e:
                log(f"warning: {e}")


# ------------------------------------------------------------------ the pipeline stack
CONTAINER = "bench-dv-stack"
WS = "/dv_pipeline_stack_ws"
# the sim, seen from the stack's container, as docker-compose.yml's dv_pipeline_stack sees it:
# under --network host, 127.0.0.1 is this computer only on Linux; on Docker Desktop for Windows
# it is the Linux VM's own, and this computer is host.docker.internal
SIM_HOST = "host.docker.internal"
NETWORK = ["--network", "host", "--add-host", f"{SIM_HOST}:host-gateway"]
DEFAULT_IMAGE = "ifssim-dv_pipeline_stack:latest"  # what `docker compose build` makes
STACK_READY = (
    "IFSSIM ROS stack starting"  # the entrypoint says it after the rebuild and warmup
)
STACK_UP_TIMEOUT_S = 1200.0  # rebuild (C++ too) + a cold Numba cache
PARAMS = Path("bringup") / "config" / "params.yaml"
COMPONENT_ALIASES = {"cone_detection": "cone_detection_node"}


def docker(*args: str, timeout: float = 120.0) -> subprocess.CompletedProcess:
    """Run ``docker`` (tests replace this)."""
    try:
        return subprocess.run(
            ["docker", *args],
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        raise BenchmarkError(f"docker {args[0]}: {e}") from None


def running_stacks() -> list[str]:
    """Other pipeline stack containers running now (docker compose's dv_pipeline_stack)."""
    r = docker("ps", "--format", "{{.Names}}")
    return [n for n in r.stdout.split() if "dv_pipeline_stack" in n and n != CONTAINER]


def _kind(v: Any) -> str:
    if isinstance(v, bool):
        return "true/false"
    if isinstance(v, int):
        return "an integer"
    if isinstance(v, float):
        return "a number"
    return (
        "a list"
        if isinstance(v, list)
        else "a string"
        if isinstance(v, str)
        else type(v).__name__
    )


def merged_params(
    params_file: Path, overrides: dict[str, dict]
) -> tuple[str, dict[str, Any]]:
    """``params.yaml`` with ``overrides`` applied. Returns (the file's new text, {"node.param": value}
    of what changed). Every problem is reported at once."""
    if not params_file.is_file():
        raise BenchmarkError(
            f"parameter overrides need the pipeline's {PARAMS} (added in IFS09-DV-PIPELINE #8); "
            f"this checkout's pipeline commit predates it"
        )
    doc = yaml.safe_load(params_file.read_text()) or {}
    errors, changed = [], {}
    for comp, values in overrides.items():
        node = COMPONENT_ALIASES.get(comp, comp)
        params = (doc.get(node) or {}).get("ros__parameters")
        if not isinstance(params, dict):
            errors.append(f"{comp}: not a node in {PARAMS} ({', '.join(doc)})")
            continue
        if not isinstance(values, dict):
            errors.append(f"{comp}: must be a mapping of parameter to value")
            continue
        for key, value in values.items():
            # a dotted name is a nested parameter: "ekf.q_pos" is ekf: {q_pos: ...}
            *parents, name = str(key).split(".")
            target = params
            for part in parents:
                target = target.get(part) if isinstance(target, dict) else None
            if not isinstance(target, dict) or name not in target:
                errors.append(f"{comp}.{key}: not a parameter of {node}")
                continue
            old = target[name]
            if _kind(old) == "a number" and _kind(value) == "an integer":
                value = float(
                    value
                )  # 1 for a double parameter is 1.0 (ROS refuses an int)
            if _kind(value) != _kind(old):
                errors.append(
                    f"{comp}.{key}: must be {_kind(old)} like its default {old!r}, not {value!r}"
                )
                continue
            target[name] = value
            changed[f"{node}.{key}"] = value
    if errors:
        raise BenchmarkError("pipeline overrides:\n  " + "\n  ".join(errors))
    note = "# params.yaml with the benchmark's overrides applied (run_mission_benchmark.py)\n"
    return note + yaml.safe_dump(doc, sort_keys=False), changed


@dataclass
class Stack:
    """The pipeline stack the runs drive: the benchmark's own container, or the running one."""

    mode: str  # own | running
    where: str  # bench_pc | latte_panda
    image: str
    checkout: Path
    sim_port: int = 41451
    overrides: dict[str, dict] = field(default_factory=dict)
    replace: bool = False
    changed: dict[str, Any] = field(default_factory=dict)
    stopped: list[str] = field(default_factory=list)
    started: bool = False
    tmp: Path | None = None
    log_file: Path | None = None

    def check(self) -> None:
        if self.mode not in ("own", "running"):
            raise BenchmarkError(f"--stack: own or running, not {self.mode!r}")
        if self.mode == "running" and self.where == "latte_panda":
            raise BenchmarkError(
                "--pipeline-on latte_panda needs --stack own: the running dv_pipeline_stack has its "
                "own pipeline, and two pipelines can't drive one sim"
            )
        if self.overrides and (self.mode != "own" or self.where != "bench_pc"):
            raise BenchmarkError(
                "pipeline overrides need --stack own and --pipeline-on bench_pc (they go into the "
                "benchmark's own stack; not yet to the running one or the latte panda)"
            )
        if self.overrides:  # before anything starts: a typo fails the job here
            _, self.changed = merged_params(
                self.checkout / "pipeline" / PARAMS, self.overrides
            )

    def up(self, log, wait_s: float = STACK_UP_TIMEOUT_S, poll_s: float = 5.0) -> None:
        if self.mode == "running":
            if not running_stacks():
                raise BenchmarkError(
                    "--stack running: no dv_pipeline_stack is running (docker compose up -d), "
                    "or leave --stack own to start one from this checkout"
                )
            return
        others = running_stacks()
        if others and not self.replace:
            raise BenchmarkError(
                f"another pipeline stack is running ({', '.join(others)}), and only one may drive the "
                "sim: stop it (docker compose stop dv_pipeline_stack), or pass --replace-stack to "
                "stop it for the benchmark and start it again after"
            )
        self.reach_sim()  # before anything is stopped or started
        for name in others:
            log(f"stopping {name} for the benchmark (started again at the end)")
            docker("stop", name, timeout=120)
            self.stopped.append(name)
        docker("rm", "-f", CONTAINER)
        run = [
            "run",
            "-d",
            "--name",
            CONTAINER,
            *NETWORK,
            # as docker-compose.yml: Fast DDS's shared-memory transport needs more than the
            # default 64 MB /dev/shm (else it falls back to UDP, much slower for the LiDAR)
            "--shm-size",
            "1g",
            "--memory",
            "8g",
            "--memory-swap",
            "8g",
            "-e",
            f"IFSSIM_HOST={SIM_HOST}",
            "-e",
            f"IFSSIM_PORT={self.sim_port}",
            "-e",
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}",
            "-e",
            "NUMBA_CACHE_DIR=/numba_cache",
            "-v",
            "bench_numba_cache:/numba_cache",
            "-e",
            f"FASTRTPS_DEFAULT_PROFILES_FILE={WS}/fastdds_profile.xml",
        ]
        for var in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "OMP_NUM_THREADS"):
            run += [
                "-e",
                f"{var}={os.environ.get(var, '2')}",
            ]  # as docker-compose.yml (#247)
        if self.where == "bench_pc":
            # a copy of this checkout's pipeline: the container builds into it, never into the checkout
            self.tmp = Path(tempfile.mkdtemp(prefix="bench-dv-stack-"))
            src = self.tmp / "pipeline"
            shutil.copytree(
                self.checkout / "pipeline", src, ignore=shutil.ignore_patterns(".git")
            )
            if self.overrides:
                text, self.changed = merged_params(src / PARAMS, self.overrides)
                (src / PARAMS).write_text(text)
                log(
                    "pipeline overrides: "
                    + ", ".join(f"{k}={v!r}" for k, v in self.changed.items())
                )
            for pkg in sorted(
                p for p in src.iterdir() if (p / "package.xml").is_file()
            ):
                run += ["-v", f"{pkg}:{WS}/src/{pkg.name}"]
            run += ["-e", "DV_REBUILD_ON_STARTUP=true", self.image]
            ready = STACK_READY
        else:  # the latte panda runs the pipeline: here only the sim bridge
            launch = (
                f"source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && "
                f"exec ros2 launch {WS}/bridge.launch.py host:={SIM_HOST} port:={self.sim_port}"
            )
            run += ["--entrypoint", "bash", self.image, "-c", launch]
            ready = None
        log(
            f"starting the pipeline stack ({'pipeline from this checkout' if ready else 'bridge only'}, {self.image})"
        )
        r = docker(*run, timeout=600)
        self.started = (
            True  # a failed run may still leave a container: down() removes it
        )
        if r.returncode != 0:
            raise BenchmarkError(
                f"docker run {self.image}: {(r.stderr or r.stdout).strip()[:300]}"
            )
        t0 = time.monotonic()
        while True:
            state = docker(
                "inspect", "-f", "{{.State.Running}}", CONTAINER
            ).stdout.strip()
            logs = docker("logs", CONTAINER, timeout=60)
            text = (logs.stdout or "") + (logs.stderr or "")
            if state != "true":
                tail = "\n".join(text.strip().splitlines()[-15:])
                raise BenchmarkError(
                    f"the pipeline stack stopped while starting:\n{tail}"
                )
            if ready is None or ready in text:
                break
            if time.monotonic() - t0 > wait_s:
                raise BenchmarkError(
                    f"the pipeline stack didn't come up within {wait_s:.0f} s"
                )
            time.sleep(poll_s)
        log(f"pipeline stack up ({time.monotonic() - t0:.0f} s)")

    def reach_sim(self) -> None:
        """Whether the stack's container can reach the sim. Its entrypoint says it is up before
        the bridge connects, so an unreachable sim would otherwise only show as missions that
        never start."""
        probe = f"timeout 5 bash -c '</dev/tcp/{SIM_HOST}/{self.sim_port}'"
        r = docker(
            "run", "--rm", *NETWORK, "--entrypoint", "bash", self.image, "-c", probe
        )
        if r.returncode != 0:
            detail = (r.stderr or r.stdout).strip()[:300]
            raise BenchmarkError(
                f"the pipeline stack's container can't reach the sim at "
                f"{SIM_HOST}:{self.sim_port}: is the sim running? {detail}".strip()
            )

    def down(self, log, results: Path | None = None) -> None:
        if not self.started:
            for name in self.stopped:  # stopped, then failed before our stack ran
                docker("start", name)
            return
        if results is not None:
            logs = docker("logs", CONTAINER, timeout=60)
            try:
                results.mkdir(parents=True, exist_ok=True)
                self.log_file = (
                    results / f"stack_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}.log"
                )
                self.log_file.write_text((logs.stdout or "") + (logs.stderr or ""))
            except OSError:
                pass
        docker("rm", "-f", CONTAINER)
        for name in self.stopped:
            log(f"starting {name} again")
            docker("start", name)
        if self.tmp is not None:
            # the stack ran as root and left root-owned files (__pycache__) in the copy
            docker(
                "run",
                "--rm",
                "--entrypoint",
                "rm",
                "-v",
                f"{self.tmp}:/copy",
                self.image,
                "-rf",
                "/copy/pipeline",
            )
            shutil.rmtree(self.tmp, ignore_errors=True)


# ------------------------------------------------------------------ one run
def yaw_of(q: dict) -> float:
    qw, qx, qy, qz = (float(q.get(k, 0.0)) for k in ("qw", "qx", "qy", "qz"))
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def offset_pose(gate: dict, pose: dict, seed: int) -> tuple[dict, dict]:
    """The start gate moved by a seeded random offset in the car's frame. Returns (new pose, offsets)."""
    rng = random.Random(seed)
    lat = rng.uniform(-pose["lateral_m"], pose["lateral_m"])
    lon = rng.uniform(-pose["longitudinal_m"], pose["longitudinal_m"])
    dyaw = math.radians(rng.uniform(-pose["yaw_deg"], pose["yaw_deg"]))
    yaw = yaw_of(gate)
    x = float(gate["x"]) + lon * math.cos(yaw) - lat * math.sin(yaw)
    y = float(gate["y"]) + lon * math.sin(yaw) + lat * math.cos(yaw)
    ny = yaw + dyaw
    new = {
        "x": x,
        "y": y,
        "z": float(gate.get("z", 0.0)),
        "qw": math.cos(ny / 2),
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(ny / 2),
    }
    return new, {"lateral_m": lat, "longitudinal_m": lon, "yaw_deg": math.degrees(dyaw)}


@dataclass
class RunResult:
    run: PlannedRun
    outcome: str  # finished | timeout | error
    started: datetime
    seconds: float
    referee: dict = field(default_factory=dict)
    start_offset: dict | None = None
    error: str | None = None
    telemetry: list[dict] = field(default_factory=list)


class Progress:
    """What the queue page shows (the launcher's rig/progress.py format), when the worker asks."""

    def __init__(self, runs: list[PlannedRun]) -> None:
        self.path = os.environ.get("BENCH_PROGRESS_FILE")
        self.runs = runs

    def report(self, index: int, lap_fraction: float, detail: str) -> None:
        if not self.path or not self.runs:
            return
        state = {
            "fraction": min(
                1.0, (index + max(0.0, min(1.0, lap_fraction))) / len(self.runs)
            ),
            "detail": detail,
            "updated": datetime.now(timezone.utc).isoformat(),
        }
        try:
            p = Path(self.path)
            p.parent.mkdir(parents=True, exist_ok=True)
            tmp = p.with_name(p.name + ".tmp")
            tmp.write_text(json.dumps(state))
            os.replace(tmp, p)
        except OSError:
            pass


def run_one(
    r: PlannedRun,
    sim: Sim,
    mc: MissionControl,
    progress: Progress,
    log,
    *,
    poll_s: float = POLL_S,
) -> RunResult:
    m = r.settings
    where = f"{r.mission} · {Path(r.track).stem} · seed {r.seed}"
    mc.post("/api/pipeline/stop")  # the previous run's autonomy, if any
    status, body = mc.post(f"/api/track/{r.track}/load", timeout=60)
    if status != 200:
        raise BenchmarkError(f"loading {r.track}: {status} {body}")
    reset = sim.cmd(f"resetScenario {r.seed}")
    if "error" in reset.lower():
        raise BenchmarkError(f"resetScenario {r.seed}: {reset}")
    offset = None
    if m["start_pose"].get("noise"):
        gate = sim.json("getStartGatePose")
        if "x" not in gate:
            raise BenchmarkError(f"no start gate pose after loading {r.track}: {gate}")
        pose, offset = offset_pose(gate, m["start_pose"], r.seed)
        sim.cmd("simSetVehiclePose {x} {y} {z} {qw} {qx} {qy} {qz}".format(**pose))
    started = datetime.now(timezone.utc)
    t0 = time.monotonic()
    progress.report(r.index, 0.0, f"{where} · starting the pipeline")
    status, body = mc.post(
        "/api/event/start",
        {"mission": m["mc_name"], "num_laps": int(m["laps"])},
        timeout=START_TIMEOUT_S,
    )
    if status != 200 or (isinstance(body, dict) and body.get("ok") is False):
        mc.post("/api/pipeline/stop")
        err = (body or {}).get("error") if isinstance(body, dict) else body
        return RunResult(
            r,
            "error",
            started,
            time.monotonic() - t0,
            error=f"mission didn't start: {status} {err}",
        )
    log(f"{where}: running")
    ref: dict = {}
    telemetry: list[dict] = []
    outcome = "timeout"
    while time.monotonic() - t0 < float(m["timeout_s"]):
        ref = mc.get("/api/event/state") or ref
        st = mc.get("/api/vehicle/state") or {}
        t = time.monotonic() - t0
        if st:
            telemetry.append(
                {
                    "t": round(t, 2),
                    "x": st.get("x"),
                    "y": st.get("y"),
                    "speed": st.get("speed"),
                    "throttle": (st.get("controls") or {}).get("throttle"),
                    "steering": (st.get("controls") or {}).get("steering"),
                }
            )
        laps, need = (
            int(ref.get("laps") or 0),
            max(1, int(ref.get("required_laps") or m["laps"])),
        )
        progress.report(
            r.index, laps / need, f"{where} · lap {min(laps + 1, need)}/{need}"
        )
        if ref.get("finished"):
            outcome = "finished"
            break
        time.sleep(poll_s)
    mc.post("/api/pipeline/stop")
    return RunResult(
        r, outcome, started, time.monotonic() - t0, ref, offset, telemetry=telemetry
    )


# ------------------------------------------------------------------ writing a run
def race_metrics(res: RunResult) -> dict[str, float]:
    ref = res.referee
    laps = [float(x) for x in ref.get("lap_times") or []]
    doo, oc = int(ref.get("doo_counter") or 0), int(ref.get("oc_counter") or 0)
    out = {
        "race/finished": 1 if res.outcome == "finished" else 0,
        "race/n_laps": len(laps),
        "race/n_doo": doo,
        "race/n_off_track": oc,
        "race/penalty_s": doo * DOO_PENALTY_S + oc * OC_PENALTY_S,
        "race/run_time_s": res.seconds,
    }
    if laps:
        out |= {
            "race/lap_time_best_s": min(laps),
            "race/lap_time_mean_s": sum(laps) / len(laps),
            "race/total_time_s": sum(laps),
        }
        if len(laps) > 1:
            mean = sum(laps) / len(laps)
            out["race/lap_time_std_s"] = math.sqrt(
                sum((x - mean) ** 2 for x in laps) / (len(laps) - 1)
            )
    speeds = [
        s["speed"] for s in res.telemetry if isinstance(s.get("speed"), (int, float))
    ]
    if speeds:
        out |= {
            "control/speed_mean_mps": sum(speeds) / len(speeds),
            "control/speed_max_mps": max(speeds),
        }
    return out


def _write_csv(path: Path, rows: list[dict], cols: list[str]) -> None:
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def code_state(
    host: PipelineHost, stack: Stack | None = None, repo: Path = REPO
) -> dict[str, Any]:
    def git(*args: str) -> str | None:
        r = subprocess.run(
            ["git", "-C", str(repo), *args], capture_output=True, text=True, check=False
        )
        return r.stdout.strip() if r.returncode == 0 else None

    sub = (
        git("-C", "pipeline", "rev-parse", "--short", "HEAD")
        if (repo / "pipeline").exists()
        else None
    )
    own = stack is not None and stack.mode == "own" and host.where == "bench_pc"
    if host.where == "latte_panda":
        pipe_sha = host.sha[:7] if host.sha else "unknown"
        source = (
            "the latte panda (--panda-sha)" if host.sha else "unknown (no --panda-sha)"
        )
    elif own:
        pipe_sha, source = (
            sub or "unknown",
            "this checkout, run in the benchmark's own stack",
        )
    else:
        pipe_sha = sub or "unknown"
        source = "this checkout's pipeline submodule; the running stack may differ (--stack running)"
    return {
        "ifssim": {
            "sha": (git("rev-parse", "--short", "HEAD") or "unknown"),
            "branch": git("rev-parse", "--abbrev-ref", "HEAD") or "",
            "dirty": bool(git("status", "--porcelain")),
        },
        "pipeline": {
            "sha": pipe_sha,
            "branch": "",
            "dirty": bool(git("-C", "pipeline", "status", "--porcelain"))
            if own
            else False,
            # where it ran, and how sure the sha is (see the module docstring)
            "ran_on": host.where,
            "sha_source": source,
        },
        "sim_build": {"id": "running sim", "plugin_sha": ""},
    }


def write_run(
    results: Path,
    res: RunResult,
    code: dict[str, Any],
    params: dict[str, Any] | None = None,
) -> Path:
    r, m = res.run, res.run.settings
    track = Path(r.track).stem
    scenario = f"{r.mission}_{track}" + (
        "_noisy" if m["start_pose"].get("noise") else ""
    )
    d = results / "mission" / scenario / code["pipeline"]["sha"] / f"seed{r.seed}"
    d.mkdir(parents=True, exist_ok=True)
    ref = res.referee
    manifest = {
        "mock": False,
        "job_type": "sim_e2e",
        "started_at": res.started.isoformat(),
        "tags": ["mission", f"pipeline_on:{code['pipeline']['ran_on']}", res.outcome],
        "seed": r.seed,
        "scenario": {
            "name": scenario,
            "event": r.mission,
            "laps": int(m["laps"]),
            "noise": {"start_pose": m["start_pose"], "start_offset": res.start_offset},
            "track": {
                "name": track,
                "csv": r.track,
                "length_m": None,
                "n_cones": int(ref.get("cones") or 0),
            },
        },
        "code": code,
        "params": dict(
            params or {}
        ),  # the pipeline overrides in effect, "node.param": value
        "referee": {
            "Laps": ref.get("lap_times") or [],
            "DooCounter": int(ref.get("doo_counter") or 0),
            "OffTrackCounter": int(ref.get("oc_counter") or 0),
            "bFinished": bool(ref.get("finished")),
            "RequiredLaps": int(ref.get("required_laps") or m["laps"]),
            "EventType": ref.get("event") or r.mission,
        },
        "outcome": res.outcome,
        "error": res.error,
        "settings": m,
    }
    (d / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    (d / "results.json").write_text(json.dumps(race_metrics(res), indent=2))
    laps = ref.get("lap_times") or []
    _write_csv(
        d / "laps.csv",
        [{"lap": i + 1, "time_s": t} for i, t in enumerate(laps)],
        ["lap", "time_s"],
    )
    events = (
        [
            {
                "kind": "dnf",
                "t": round(res.seconds, 1),
                "detail": res.error or res.outcome,
            }
        ]
        if res.outcome != "finished"
        else []
    )
    _write_csv(d / "events.csv", events, ["kind", "t", "detail"])
    _write_csv(
        d / "telemetry.csv",
        res.telemetry,
        ["t", "x", "y", "speed", "throttle", "steering"],
    )
    spec = os.environ.get("BENCH_SPEC")
    if spec and Path(spec).is_file():
        (d / "spec.json").write_text(Path(spec).read_text())
    return d


# ------------------------------------------------------------------ main
def run(
    args: argparse.Namespace,
    sim: Sim | None = None,
    mc: MissionControl | None = None,
    log=print,
) -> int:
    names = [x.strip() for x in (args.missions or "all").split(",") if x.strip()]
    if names == ["all"]:
        names = list(MISSIONS)
    overrides = (
        json.loads(Path(args.mission_overrides).read_text())
        if args.mission_overrides
        else None
    )
    missions = mission_settings(
        names,
        repeats=args.repeats,
        laps=args.laps,
        timeout_s=args.timeout_s,
        start_noise=True if args.start_noise else None,
        tracks=[x.strip() for x in args.tracks.split(",")] if args.tracks else None,
        overrides=overrides,
    )
    host = PipelineHost(
        args.pipeline_on, args.panda_start, args.panda_stop, args.panda_sha
    )
    host.check()
    pipeline_overrides = (
        json.loads(Path(args.pipeline_overrides).read_text() or "{}")
        if args.pipeline_overrides
        else {}
    )
    stack = Stack(
        args.stack,
        args.pipeline_on,
        args.image or os.environ.get("IFSSIM_DV_IMAGE") or DEFAULT_IMAGE,
        Path(getattr(args, "checkout", None) or REPO),
        int(args.sim.rsplit(":", 1)[1]),
        pipeline_overrides,
        args.replace_stack,
    )
    stack.check()
    sim = sim or Sim(*(args.sim.rsplit(":", 1)[0], int(args.sim.rsplit(":", 1)[1])))
    mc = mc or MissionControl(args.mc_url, args.mc_key or os.environ.get("MC_API_KEY"))
    status = mc.get("/api/sim/status") or {}
    if not status.get("connected"):
        raise BenchmarkError(
            f"Mission Control at {mc.url} reports the sim isn't connected: start the sim first"
        )
    runs = plan(missions)
    log(f"{len(runs)} run(s); pipeline on {host.where}")
    progress = Progress(runs)
    results = Path(args.results_root)
    code = code_state(host, stack, stack.checkout)
    worst = 0
    try:
        stack.up(log)
        host.up(log)
        if host.where == "latte_panda":
            code = code_state(
                host, stack, stack.checkout
            )  # now with the latte panda's commit
        for r in runs:
            try:
                res = run_one(r, sim, mc, progress, log, poll_s=args.poll_s)
            except BenchmarkError as e:
                res = RunResult(
                    r, "error", datetime.now(timezone.utc), 0.0, error=str(e)
                )
            d = write_run(results, res, code, stack.changed)
            laps = res.referee.get("lap_times") or []
            log(
                f"{r.mission} · {Path(r.track).stem} · seed {r.seed}: {res.outcome}"
                + (f", laps {', '.join(f'{x:.1f}' for x in laps)}" if laps else "")
                + (f" ({res.error})" if res.error else "")
                + f" -> {d}"
            )
            worst = max(worst, 0 if res.outcome == "finished" else 1)
    finally:
        host.down(log)
        stack.down(log, results / "mission")
    return worst


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--missions",
        default="all",
        help="comma separated: acceleration, skidpad, autocross, trackdrive; or all",
    )
    ap.add_argument(
        "--tracks", help="these tracks instead of each mission's (comma separated CSVs)"
    )
    ap.add_argument("--repeats", type=int, help="runs per track (repeat i uses seed i)")
    ap.add_argument("--laps", type=int, help="laps, for every mission")
    ap.add_argument("--timeout-s", type=float, help="per run, for every mission")
    ap.add_argument(
        "--start-noise",
        action="store_true",
        help="start each repeat from a seeded random offset",
    )
    ap.add_argument(
        "--mission-overrides", help='JSON file: {"trackdrive": {"laps": 3}, ...}'
    )
    ap.add_argument(
        "--pipeline-on",
        default="bench_pc",
        choices=("bench_pc", "latte_panda"),
        help="where the pipeline runs",
    )
    ap.add_argument(
        "--panda-start", help="command that starts the pipeline on the latte panda"
    )
    ap.add_argument("--panda-stop", help="command that stops it")
    ap.add_argument(
        "--panda-sha", help="command that prints the pipeline commit on the latte panda"
    )
    ap.add_argument("--sim", default="127.0.0.1:41451", help="the sim's RPC host:port")
    ap.add_argument(
        "--mc-url", default="http://127.0.0.1:8000", help="Mission Control's backend"
    )
    ap.add_argument("--mc-key", help="Mission Control's API key (default $MC_API_KEY)")
    ap.add_argument(
        "--results-root",
        default=str(HERE / "results"),
        help="where runs go (<root>/mission/...)",
    )
    ap.add_argument("--poll-s", type=float, default=POLL_S, help=argparse.SUPPRESS)
    ap.add_argument(
        "--pipeline-overrides",
        help="JSON file {node: {param: value}}, merged into params.yaml (see PARAMETER OVERRIDES)",
    )
    ap.add_argument(
        "--stack",
        default="own",
        choices=("own", "running"),
        help="own: start a pipeline stack from this checkout (default); running: use the running one",
    )
    ap.add_argument(
        "--replace-stack",
        action="store_true",
        help="stop a running dv_pipeline_stack for the benchmark, and start it again after",
    )
    ap.add_argument(
        "--image",
        help=f"the stack's image (default $IFSSIM_DV_IMAGE or {DEFAULT_IMAGE})",
    )
    args = ap.parse_args(argv)
    try:
        return run(args)
    except BenchmarkError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
