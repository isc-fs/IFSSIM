from __future__ import annotations

import csv
import json
import os
import shlex
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent.parent


def sim_benchmark_dir() -> Path:
    return repo_root() / "tools" / "sim_benchmark"


def results_dir() -> Path:
    return sim_benchmark_dir() / "results"


def default_results_root() -> str:
    """Host path by default; /results when re-exec'd in the benchmark container."""
    if os.environ.get("IFSSIM_BENCHMARK_IN_DOCKER") == "1":
        return "/results"
    return "tools/sim_benchmark/results"


def bag_storage_id(bag_uri: str | Path) -> str:
    meta = Path(bag_uri) / "metadata.yaml"
    if meta.is_file() and "storage_identifier: mcap" in meta.read_text():
        return "mcap"
    return "sqlite3"


def bag_topic_names(bag_uri: str | Path) -> set[str]:
    """Topic names listed in rosbag2 metadata.yaml (empty if missing)."""
    meta = Path(bag_uri) / "metadata.yaml"
    if not meta.is_file():
        return set()
    names: set[str] = set()
    for line in meta.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("name: /"):
            names.add(stripped.split("name:", 1)[1].strip())
    return names


def dv_pipeline_ros_setup_shell() -> str:
    """Shell snippet matching dv_pipeline_stack entrypoint ROS env (includes fs_msgs)."""
    return (
        "source /opt/ros/humble/setup.bash; "
        "source /dv_pipeline_stack_ws/install/setup.bash; "
        'export AMENT_PREFIX_PATH="/dv_pipeline_stack_ws/install/fs_msgs:'
        '/dv_pipeline_stack_ws/install/ifssim_bridge:'
        '/dv_pipeline_stack_ws/install/dv_msgs:${AMENT_PREFIX_PATH:-}"; '
    )


def _in_ros_env() -> bool:
    try:
        import rclpy  # noqa: F401

        return True
    except ImportError:
        return False


def resolve_benchmark_path(host: str | Path) -> Path:
    """Resolve bag/results paths from repo root or ``tools/sim_benchmark`` cwd."""
    p = Path(host)
    if p.is_absolute():
        return p.resolve()
    under_bench = (sim_benchmark_dir() / p).resolve()
    under_cwd = (Path.cwd() / p).resolve()
    if under_bench.exists():
        return under_bench
    if under_cwd.exists():
        return under_cwd
    # ``results/capture/...`` is defined relative to tools/sim_benchmark/.
    return under_bench


def _translate_host_path(host: str) -> str:
    p = resolve_benchmark_path(host)

    results_mount = results_dir().resolve()
    bench_mount = sim_benchmark_dir().resolve()
    try:
        return "/results/" + p.relative_to(results_mount).as_posix()
    except ValueError:
        pass
    try:
        return "/bench/" + p.relative_to(bench_mount).as_posix()
    except ValueError:
        pass
    raise RuntimeError(
        f"Path must be under tools/sim_benchmark (got {p}). "
        "Move the bag/results under that tree or pass --no-docker with a sourced ROS shell."
    )


def _translate_argv(argv: list[str]) -> list[str]:
    path_flags = {"--results-root", "--output-dir"}
    out: list[str] = []
    i = 0
    while i < len(argv):
        token = argv[i]
        if token in ("--local-ros", "--no-docker"):
            i += 1
            continue
        if token.startswith("--") and "=" in token:
            key, value = token.split("=", 1)
            if key in path_flags:
                out.append(f"{key}={_translate_host_path(value)}")
            else:
                out.append(token)
            i += 1
            continue
        if token in path_flags:
            out.append(token)
            i += 1
            if i >= len(argv):
                raise RuntimeError(f"missing value for {token}")
            out.append(_translate_host_path(argv[i]))
            i += 1
            continue
        if not out and not token.startswith("-"):
            out.append(_translate_host_path(token))
            i += 1
            continue
        out.append(token)
        i += 1
    return out


def maybe_reexec_in_docker(script_name: str) -> None:
    """Re-run this benchmark inside dv_pipeline_stack when host lacks ROS."""
    if os.environ.get("IFSSIM_BENCHMARK_IN_DOCKER") == "1":
        return
    argv = sys.argv[1:]
    if "--local-ros" in argv or "--no-docker" in argv:
        return
    if _in_ros_env():
        return
    if shutil.which("docker") is None:
        raise RuntimeError(
            "ROS Python packages (rclpy) are not available on the host. "
            "Install/use a ROS 2 environment, or run with Docker available "
            "(default: re-execs in ifssim-dv_pipeline_stack:latest)."
        )

    bench = sim_benchmark_dir()
    results = results_dir()
    results.mkdir(parents=True, exist_ok=True)
    cone_detection_src = repo_root() / "pipeline" / "cone_detection"
    cone_slam_src = repo_root() / "pipeline" / "cone_slam"
    image = os.environ.get("IFSSIM_DV_IMAGE", "ifssim-dv_pipeline_stack:latest")
    inner_argv = _translate_argv(argv)
    if not any(a == "--results-root" or a.startswith("--results-root=") for a in inner_argv):
        inner_argv = ["--results-root", "/results", *inner_argv]
    arg_str = " ".join(shlex.quote(a) for a in inner_argv)

    # Compiled odometry_filter_py bindings (built by build_native_filter.sh).
    # Mounting them lets the benchmark run the REAL C++ EKF instead of the
    # Python fallback. Optional: absent → OdometryFilterCpp falls back.
    native_dir = bench / "_native"
    have_native = native_dir.is_dir() and any(native_dir.glob("odometry_filter_py*.so"))
    pythonpath_dirs = "/dev_cone_detection:/dev_cone_slam"
    if have_native:
        pythonpath_dirs = "/native:" + pythonpath_dirs

    inner = (
        "set -eo pipefail; "
        "export IFSSIM_BENCHMARK_IN_DOCKER=1; "
        f"{dv_pipeline_ros_setup_shell()}"
        f"export PYTHONPATH={pythonpath_dirs}:${{PYTHONPATH}}; "
        f"cd /bench && python3 {shlex.quote(script_name)} {arg_str}"
    )
    # Override image ENTRYPOINT (/entrypoint.sh launches the full sim stack).
    cmd = [
        "docker",
        "run",
        "--rm",
        "--entrypoint",
        "bash",
        "-v",
        f"{bench.resolve()}:/bench:ro",
        "-v",
        f"{cone_detection_src.resolve()}:/dev_cone_detection:ro",
        "-v",
        f"{cone_slam_src.resolve()}:/dev_cone_slam:ro",
        "-v",
        f"{results.resolve()}:/results",
    ]
    if have_native:
        cmd += ["-v", f"{native_dir.resolve()}:/native:ro"]
    cmd += [
        "-e",
        "IFSSIM_BENCHMARK_IN_DOCKER=1",
        image,
        "-lc",
        inner,
    ]
    print("Host lacks ROS; running benchmark in Docker:")
    print(" ", " ".join(cmd[:10]), "...")
    raise SystemExit(subprocess.call(cmd))


def make_run_dir(root: str | Path, module: str, strategy: str) -> Path:
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out = Path(root) / module / f"{strategy}_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    Path(path).write_text(json.dumps(payload, indent=2, sort_keys=True))


def write_csv(path: str | Path, rows: list[dict[str, Any]]) -> None:
    p = Path(path)
    if not rows:
        p.write_text("")
        return
    with p.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def load_csv_centerline(track_csv: str, rotate_ccw_90: bool = True) -> list[tuple[float, float]]:
    path = Path(track_csv)
    if not path.is_file():
        raise FileNotFoundError(f"track csv not found: {track_csv}")

    blues: list[tuple[float, float]] = []
    yellows: list[tuple[float, float]] = []
    with path.open(newline="") as fh:
        for row in csv.reader(fh):
            if not row or len(row) < 3:
                continue
            color = row[0].strip().lower()
            x = float(row[1])
            y = float(row[2])
            if rotate_ccw_90:
                x, y = -y, x
            if color == "blue":
                blues.append((x, y))
            elif color == "yellow":
                yellows.append((x, y))

    if not blues or not yellows:
        raise ValueError("track csv requires both blue and yellow cones")

    mids: list[tuple[float, float]] = []
    for bx, by in blues:
        yx, yy = min(
            yellows,
            key=lambda y: (y[0] - bx) * (y[0] - bx) + (y[1] - by) * (y[1] - by),
        )
        mids.append(((bx + yx) * 0.5, (by + yy) * 0.5))
    return mids
