"""Upload runs to the team's MLflow as soon as a benchmark finishes.

Called on the host (never inside the benchmark container) once a benchmark
exits: ``common.maybe_reexec_in_docker`` after the container returns, or at
exit for a run started with ``--no-docker``. It runs ``bench-track sync`` on
the results dir, which uploads every finished run the server does not have
yet. So a run made while the server was unreachable goes up with the next one,
or with a manual ``bench-track sync``.

Uploading needs a server: ``MLFLOW_TRACKING_URI`` in the environment or in
``~/.config/ifssim-bench/tracking.env`` (see tracking/DEPLOY.md). Without one
nothing is uploaded. ``IFSSIM_AUTO_UPLOAD=0`` (environment or that file) turns
it off; ``run_sim_bag_benchmark.py`` sets it for the benchmarks it starts and
uploads the whole session at the end instead.

Standard library only, and it never fails the benchmark.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

ENV_OFF = "IFSSIM_AUTO_UPLOAD"
CONFIG = "~/.config/ifssim-bench/tracking.env"
TRACKING = Path(__file__).resolve().parent / "tracking"


def _config() -> dict[str, str]:
    p = Path(os.environ.get("IFSSIM_BENCH_CONFIG", CONFIG)).expanduser()
    out: dict[str, str] = {}
    try:
        for line in p.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip().strip("'\"")
    except OSError:
        pass
    return out


def _setting(name: str) -> str | None:
    return os.environ.get(name) or _config().get(name)


def enabled() -> bool:
    return (
        os.environ.get("IFSSIM_BENCHMARK_IN_DOCKER") != "1"
        and _setting(ENV_OFF) != "0"
        and bool(_setting("MLFLOW_TRACKING_URI"))
    )


def command(results: Path) -> list[str]:
    return [
        "uv",
        "run",
        "--quiet",
        "--project",
        str(TRACKING),
        "bench-track",
        "sync",
        "--results-root",
        str(Path(results).resolve()),
    ]


def after_run(results: Path) -> None:
    """Upload what is new under ``results``. Prints, never raises."""
    if not enabled():
        return
    cmd = command(results)
    if shutil.which("uv") is None:
        print(
            "Not uploaded: uv is not installed. Upload later with: "
            + " ".join(cmd[4:]),
            file=sys.stderr,
        )
        return
    print("\nUploading to the benchmark tracker ...", flush=True)
    try:
        rc = subprocess.call(cmd)
    except OSError as e:
        rc = f"{e}"
    if rc != 0:
        print(
            "Upload did not finish; the results are safe on disk and the next "
            f"benchmark (or `{' '.join(cmd[4:])}`) uploads them.",
            file=sys.stderr,
        )


_registered: set[Path] = set()


def at_exit(results: Path) -> None:
    """Upload when this process exits (runs on the host without Docker)."""
    import atexit

    results = Path(results).resolve()
    if results in _registered or not enabled():
        return
    _registered.add(results)
    atexit.register(after_run, results)
