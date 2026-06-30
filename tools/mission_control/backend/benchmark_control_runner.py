"""Start/stop sim-only benchmark control (track driver + metrics harness)."""
from __future__ import annotations

import logging
import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path

_LOG = logging.getLogger(__name__)

_tools_dir_cache: Path | None = None


def _tools_dir() -> Path:
    """Resolve repo ``tools/`` directory (lazy — safe at import in /app)."""
    global _tools_dir_cache
    if _tools_dir_cache is not None:
        return _tools_dir_cache

    env = os.environ.get("IFSSIM_TOOLS_DIR", "").strip()
    if env:
        _tools_dir_cache = Path(env)
        return _tools_dir_cache

    here = Path(__file__).resolve().parent
    # Local layout: tools/mission_control/backend/this_file.py → tools/
    candidate = here.parent.parent
    if (candidate / "track_driver.py").is_file():
        _tools_dir_cache = candidate
        return _tools_dir_cache

    # Docker layout: backend copied to /app; tools bind-mounted at /ifssim_tools.
    fallback = Path("/ifssim_tools")
    _tools_dir_cache = fallback
    return _tools_dir_cache


def _results_root() -> str:
    return os.environ.get(
        "IFSSIM_BENCHMARK_RESULTS_ROOT",
        str(_tools_dir() / "sim_benchmark" / "results"),
    )

_active: list[subprocess.Popen] = []


def _ros_shell_prefix() -> str:
    parts = ["set -e", "source /opt/ros/humble/setup.bash"]
    for setup in (
        "/msgs_ws/install/setup.bash",  # mission_control_backend image
        "/dv_pipeline_stack_ws/install/setup.bash",  # dv_pipeline_stack
    ):
        if Path(setup).is_file():
            parts.append(f"source {setup}")
            break
    control_pkg = os.environ.get("IFSSIM_CONTROL_PKG", "/control_pkg").strip()
    if control_pkg and Path(control_pkg).is_dir():
        parts.append(f'export PYTHONPATH="{control_pkg}:$PYTHONPATH"')
    sim_bench = _tools_dir() / "sim_benchmark"
    if sim_bench.is_dir():
        parts.append(f'export PYTHONPATH="{sim_bench}:$PYTHONPATH"')
    return " && ".join(parts)


def _python_cmd(script: Path, args: list[str]) -> list[str]:
    """Build a bash -lc command that runs a repo tool with ROS env."""
    py = sys.executable.replace("\\", "/")
    script_posix = script.as_posix()
    arg_str = " ".join(_shell_quote(a) for a in args)
    inner = f"{py} {script_posix} {arg_str}".strip()
    return ["bash", "-lc", f"{_ros_shell_prefix()} && {inner}"]


def _shell_quote(s: str) -> str:
    if not s:
        return "''"
    if all(c.isalnum() or c in "/._-:" for c in s):
        return s
    return "'" + s.replace("'", "'\"'\"'") + "'"


def is_running() -> bool:
    _reap()
    return bool(_active)


def _reap() -> None:
    global _active
    alive: list[subprocess.Popen] = []
    for proc in _active:
        if proc.poll() is None:
            alive.append(proc)
    _active = alive


def start(centerline_csv: str) -> dict:
    """Spawn track_driver + control_benchmark_node. Returns status dict."""
    _reap()
    if _active:
        stop()

    tools = _tools_dir()
    track_driver = tools / "track_driver.py"
    benchmark_node = tools / "sim_benchmark" / "control_benchmark_node.py"

    csv_path = Path(centerline_csv)
    if not csv_path.is_file():
        return {
            "ok": False,
            "error": f"centerline CSV not found: {csv_path}",
        }
    if not track_driver.is_file():
        return {
            "ok": False,
            "error": f"track_driver missing: {track_driver} (IFSSIM_TOOLS_DIR={tools})",
        }
    if not benchmark_node.is_file():
        return {
            "ok": False,
            "error": f"control benchmark script missing: {benchmark_node}",
        }

    env = os.environ.copy()
    py_paths: list[str] = [str(tools / "sim_benchmark")]
    control_pkg = os.environ.get("IFSSIM_CONTROL_PKG", "").strip()
    if not control_pkg:
        local_control = tools.parent / "pipeline" / "control"
        if local_control.is_dir():
            control_pkg = str(local_control)
    if control_pkg:
        py_paths.insert(0, control_pkg)
    env["PYTHONPATH"] = os.pathsep.join(
        filter(None, [*py_paths, env.get("PYTHONPATH", "")]),
    )

    bench_args = [
        "--centerline-source",
        "csv",
        "--centerline-csv",
        str(csv_path.resolve()),
        "--results-root",
        _results_root(),
        "--duration-s",
        "86400",
    ]

    def _spawn(script: Path, args: list[str]) -> subprocess.Popen:
        if shutil.which("bash"):
            cmd = _python_cmd(script, args)
        else:
            cmd = [sys.executable, str(script), *args]
        return subprocess.Popen(
            cmd,
            env=env,
            start_new_session=(os.name != "nt"),
        )

    try:
        driver = _spawn(
            track_driver,
            [
                str(csv_path.resolve()),
                "--target-speed",
                "5.0",
                "--steer",
                "1.0",
            ],
        )
        bench = _spawn(benchmark_node, bench_args)
    except Exception as ex:
        stop()
        return {"ok": False, "error": f"failed to spawn benchmark processes: {ex}"}

    _active.extend([driver, bench])
    return {
        "ok": True,
        "pids": [driver.pid, bench.pid],
        "centerline_csv": str(csv_path.resolve()),
    }


def stop() -> None:
    """SIGINT benchmark child process groups, then reap."""
    global _active
    for proc in list(_active):
        if proc.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError) as ex:
            _LOG.warning("benchmark_control stop: killpg failed: %s", ex)
            try:
                proc.terminate()
            except Exception:
                pass
    for proc in _active:
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                proc.kill()
            except Exception:
                pass
    _active = []
