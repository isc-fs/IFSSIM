#!/usr/bin/env python3
"""Replay an onboard (no-GT) rosbag through the live DV pipeline.

Plays sensor topics into the autonomy nodes (car topic names). Pass
``--report`` to record pipeline outputs and write the HTML summary
(detection counts, odom/SLAM trajectories, map, autonomy vs pilot
steering). ``--live`` without ``--report`` only plays into the nodes
(no second bag). There is no ground truth.

Usage (repo root; auto re-execs in ifssim-dv_pipeline_stack when host has no ROS):

    python tools/sim_benchmark/run_onboard_replay.py \\
        results/capture/manual_20260920_154527_indexed --live

    python tools/sim_benchmark/run_onboard_replay.py <bag> --report
"""

from __future__ import annotations

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from common import (
    bag_storage_id,
    default_results_root,
    make_run_dir,
    maybe_reexec_in_docker,
    resolve_benchmark_path,
)
from onboard_report import summarize, write_onboard_report

DEFAULT_FOXGLOVE_PORT = 8766
FOXGLOVE_WHITELIST = (
    "/Conos",
    "/Conos_full",
    "/Conos_Orange",
    "/Conos_raw",
    "/Path",
    "/path_planning/debug",
    "/path_planning/hz",
    "/path_planning/latency_ms",
    "/path_planning/n_waypoints",
    "/path_planning/length_m",
    "/path_planning/empty",
    "/path_planning/tf_miss",
    "/slam/pose",
    "/slam/finished",
    "/slam/final_lap",
    "/slam/stop_request",
    "/odom",
    "/odom_diag/yaw_residual_rad_s",
    "/odom_diag/slip_flag",
    "/odom_diag/effective_alpha_vx",
    "/ctrl/cmd_internal",
    "/lidar_points",
    "/lidar_points/ground",
    "/lidar_points/above_ground",
    "/imu",
    "/motor_rpm",
    "/steering_angle",
    "/tf",
    "/tf_static",
    "/control/v_set_mps",
    "/control/kappa_max_per_m",
    "/control/latency_ms",
    "/cone_detection/hz",
    "/cone_detection/latency_ms",
    "/cone_detection/ransac_ms",
    "/cone_detection/dbscan_ms",
    "/cone_detection/fit_ms",
    "/cone_detection/n_accepted",
    "/cone_detection/n_clusters",
    "/cone_detection/n_input_points",
    "/cone_detection/n_after_shape",
    "/cone_detection/n_far_dropped",
    "/cone_detection/n_left",
    "/cone_detection/n_right",
    "/cone_slam/hz",
    "/cone_slam/latency_ms",
    "/cone_slam/age_ms",
    "/cone_slam/commit_ms",
    "/cone_slam/map_size",
    "/cone_slam/n_obs",
)

PLAY_TOPICS = (
    "/imu",
    "/lidar_points",
    "/motor_rpm",
    "/steering_angle",
    "/tf_static",
)
RECORD_TOPICS = (
    "/Conos_raw",
    "/Conos",
    "/Conos_Orange",
    "/slam/pose",
    "/odom",
    "/Path",
    "/ctrl/cmd_internal",
    "/steering_angle",
    "/tf",
    "/tf_static",
    "/path_planning/debug",
)
# Matches pipeline/mode_manager/mode_registry.py (kept local so this toolkit
# does not import the car submodule at report-generation time).
MISSION_BEHAVIORS = {
    "trackdrive": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "trackdrive",
        "path_planning_node": "trackdrive",
        "control_node": "pure_pursuit",
    },
    "autocross": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "autocross",
        "path_planning_node": "autocross",
        "control_node": "stanley",
    },
    "accel": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "accel",
        "path_planning_node": "accel",
        "control_node": "pure_pursuit",
    },
    "skidpad": {
        "odometry_filter_node": "base",
        "cone_detection_node": "base",
        "slam_node": "skidpad",
        "path_planning_node": "skidpad",
        "control_node": "stanley",
    },
}
NODES = (
    ("odometry_filter_node", "odometry_filter_node", "odometry_filter_node", ()),
    (
        "cone_detection",
        "cone_detection_node",
        "cone_detection_node",
        (("/fsds/lidar/Lidar1", "/lidar_points"),),
    ),
    ("cone_slam", "slam_node", "slam_node", ()),
    ("path_planning", "path_planning_node", "path_planning_node", ()),
    ("control", "control_node", "control_node", ()),
)
MARKER_DELETEALL = 3


def _open_bag(path: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=path, storage_id=bag_storage_id(path)),
        ConverterOptions("", ""),
    )
    return reader


def _topic_classes(reader) -> dict[str, object]:
    from rosidl_runtime_py.utilities import get_message

    out: dict[str, object] = {}
    for tt in reader.get_all_topics_and_types():
        out[tt.name] = get_message(tt.type)
    return out


def bag_topic_counts(bag_uri: Path) -> dict[str, int]:
    meta = bag_uri / "metadata.yaml"
    if not meta.is_file():
        return {}
    counts: dict[str, int] = {}
    name: str | None = None
    for line in meta.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("name: /"):
            name = stripped.split("name:", 1)[1].strip()
        elif stripped.startswith("message_count:") and name:
            counts[name] = int(stripped.split(":", 1)[1].strip())
            name = None
    return counts


def bag_duration_s(bag_uri: Path) -> float:
    meta = bag_uri / "metadata.yaml"
    if not meta.is_file():
        return 0.0
    pending = False
    for line in meta.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped == "duration:":
            pending = True
            continue
        if pending and stripped.startswith("nanoseconds:"):
            return int(stripped.split(":", 1)[1].strip()) / 1e9
        if pending and stripped and not stripped.startswith("nanoseconds"):
            pending = False
    return 0.0


def _yaw_from_quat(q) -> float:
    w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _marker_xy(msg) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for m in getattr(msg, "markers", []) or []:
        if int(getattr(m, "action", 0)) == MARKER_DELETEALL:
            continue
        out.append((float(m.pose.position.x), float(m.pose.position.y)))
    return out


def _t0_from_samples(samples: dict[str, Any]) -> float | None:
    first: list[float] = []
    for key in ("steering", "conos_raw", "odom", "slam", "cmd"):
        rows = samples.get(key) or []
        if rows:
            first.append(float(rows[0]["t_s"]))
    return min(first) if first else None


def _shift_times(samples: dict[str, Any]) -> None:
    t0 = _t0_from_samples(samples)
    if t0 is None:
        return
    for key in ("steering", "conos_raw", "conos", "odom", "slam", "path", "cmd"):
        for row in samples.get(key) or []:
            row["t_s"] = float(row["t_s"]) - t0


def load_steering(bag: Path) -> list[dict[str, float]]:
    from rclpy.serialization import deserialize_message

    reader = _open_bag(str(bag))
    classes = _topic_classes(reader)
    cls = classes.get("/steering_angle")
    rows: list[dict[str, float]] = []
    if cls is None:
        return rows
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic != "/steering_angle":
            continue
        msg = deserialize_message(raw, cls)
        rows.append({"t_s": t_ns / 1e9, "rad": float(msg.data)})
    return rows


def load_pipeline_outputs(bag: Path) -> dict[str, Any]:
    from rclpy.serialization import deserialize_message

    reader = _open_bag(str(bag))
    classes = _topic_classes(reader)
    samples: dict[str, Any] = {
        "conos_raw": [],
        "conos": [],
        "odom": [],
        "slam": [],
        "path": [],
        "cmd": [],
        "steering": [],
        "map_x": [],
        "map_y": [],
    }
    last_map: list[tuple[float, float]] = []
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        cls = classes.get(topic)
        if cls is None:
            continue
        msg = deserialize_message(raw, cls)
        t_s = t_ns / 1e9
        if topic == "/Conos_raw":
            xy = _marker_xy(msg)
            samples["conos_raw"].append({"t_s": t_s, "n": len(xy)})
        elif topic == "/Conos":
            xy = _marker_xy(msg)
            samples["conos"].append({"t_s": t_s, "n": len(xy)})
            if xy:
                last_map = xy
        elif topic == "/odom":
            samples["odom"].append(
                {
                    "t_s": t_s,
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": _yaw_from_quat(msg.pose.pose.orientation),
                }
            )
        elif topic == "/slam/pose":
            samples["slam"].append(
                {
                    "t_s": t_s,
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": _yaw_from_quat(msg.pose.pose.orientation),
                }
            )
        elif topic == "/Path":
            xs = [float(p.pose.position.x) for p in msg.poses]
            ys = [float(p.pose.position.y) for p in msg.poses]
            samples["path"].append({"t_s": t_s, "n": len(xs), "xs": xs, "ys": ys})
        elif topic == "/ctrl/cmd_internal":
            samples["cmd"].append(
                {
                    "t_s": t_s,
                    "throttle": float(msg.throttle),
                    "steering": float(msg.steering),
                    "brake": float(getattr(msg, "brake", 0.0)),
                }
            )
        elif topic == "/steering_angle":
            samples["steering"].append({"t_s": t_s, "rad": float(msg.data)})
    if last_map:
        samples["map_x"] = [p[0] for p in last_map]
        samples["map_y"] = [p[1] for p in last_map]
    return samples


def foxglove_params_yaml(port: int) -> str:
    topics = "\n".join(f'      - "{t}"' for t in FOXGLOVE_WHITELIST)
    return (
        "/**:\n"
        "  ros__parameters:\n"
        f"    port: {int(port)}\n"
        '    address: "0.0.0.0"\n'
        "    use_sim_time: true\n"
        "    send_buffer_limit: 67108864\n"
        "    use_compression: true\n"
        "    topic_whitelist:\n"
        f"{topics}\n"
    )


def live_docker_publish_args(argv: list[str] | None = None) -> list[str]:
    """Host port map for --live so Lichtblick can reach foxglove_bridge."""
    args = ["--shm-size=1g"]
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--live" not in argv:
        return args
    port = str(DEFAULT_FOXGLOVE_PORT)
    if "--foxglove-port" in argv:
        idx = argv.index("--foxglove-port")
        if idx + 1 < len(argv) and not argv[idx + 1].startswith("-"):
            port = argv[idx + 1]
    for token in argv:
        if token.startswith("--foxglove-port="):
            port = token.split("=", 1)[1]
    args += ["-p", f"{port}:{port}"]
    return args


def _popen(cmd: list[str], log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fh = log_path.open("w")
    return subprocess.Popen(
        cmd,
        stdout=fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _run(cmd: list[str], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        timeout=timeout,
        check=False,
        capture_output=True,
        text=True,
    )


def _stop(proc: subprocess.Popen | None, sig: int = signal.SIGINT, wait_s: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        proc.send_signal(sig)
    deadline = time.time() + wait_s
    while time.time() < deadline and proc.poll() is None:
        time.sleep(0.2)
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            proc.kill()


def _start_nodes(log_dir: Path) -> list[tuple[str, subprocess.Popen]]:
    procs: list[tuple[str, subprocess.Popen]] = []
    for package, executable, name, remaps in NODES:
        cmd = [
            "ros2",
            "run",
            package,
            executable,
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ]
        for src, dst in remaps:
            cmd += ["-r", f"{src}:={dst}"]
        procs.append((name, _popen(cmd, log_dir / f"{name}.log")))
        print(f"  started {name} pid={procs[-1][1].pid}")
    return procs


def _setup_and_activate(mission: str, log_dir: Path) -> None:
    behaviors = MISSION_BEHAVIORS[mission]
    for _, _, name, _ in NODES:
        behavior = behaviors[name]
        cmd = [
            "ros2",
            "service",
            "call",
            f"/{name}/setup",
            "dv_msgs/srv/Setup",
            f"{{mode_name: '{mission}', behavior: '{behavior}'}}",
        ]
        print(f"  setup {name} ({behavior})")
        result = _run(cmd, timeout=60)
        if result.returncode != 0:
            (log_dir / f"{name}_setup.log").write_text(
                (result.stdout or "") + (result.stderr or "")
            )
            raise RuntimeError(f"{name} ~/setup failed:\n{result.stdout}\n{result.stderr}")
    for _, _, name, _ in NODES:
        timeout = 180 if name == "cone_detection_node" else 45
        for transition in ("configure", "activate"):
            print(f"  lifecycle {name} {transition}")
            result = _run(
                ["ros2", "lifecycle", "set", f"/{name}", transition],
                timeout=timeout,
            )
            if result.returncode != 0:
                log = (log_dir / f"{name}.log").read_text()[-4000:]
                raise RuntimeError(
                    f"{name} {transition} failed:\n{result.stdout}\n{result.stderr}\n"
                    f"--- node log ---\n{log}"
                )


def _start_foxglove(log_dir: Path, params_path: Path, port: int) -> subprocess.Popen:
    params_path.write_text(foxglove_params_yaml(port), encoding="utf-8")
    cmd = [
        "ros2",
        "run",
        "foxglove_bridge",
        "foxglove_bridge",
        "--ros-args",
        "--params-file",
        str(params_path),
    ]
    # Line-buffer C++ logs so "client connected" is visible before play.
    if Path("/usr/bin/stdbuf").is_file():
        cmd = ["stdbuf", "-oL", "-eL", *cmd]
    proc = _popen(cmd, log_dir / "foxglove_bridge.log")
    print(f"  started foxglove_bridge pid={proc.pid} ws://localhost:{port}")
    return proc


_FOXGLOVE_CLIENT_RE = re.compile(
    r"(client(?:\s+\d+)?\s+connected|connection opened|"
    r"new client connection|accepted connection)",
    re.IGNORECASE,
)
_TCP_ESTABLISHED = "01"


def foxglove_client_connected(log_text: str) -> bool:
    """True when foxglove_bridge has logged a Lichtblick / Studio client."""
    return _FOXGLOVE_CLIENT_RE.search(log_text) is not None


def count_tcp_established(port: int, *proc_net_texts: str) -> int:
    """Count ESTABLISHED sockets whose local port is ``port``.

    Each argument is a ``/proc/net/tcp`` or ``/proc/net/tcp6`` dump.
    The listen socket itself is state ``0A`` and is not counted — that is
    why a foxglove_bridge that is merely *listening* on 8766 does not
    count as a client.
    """
    want = f"{int(port):04X}"
    n = 0
    for text in proc_net_texts:
        for line in text.splitlines()[1:]:
            parts = line.split()
            if len(parts) < 4:
                continue
            local, _rem, st = parts[1], parts[2], parts[3]
            if st != _TCP_ESTABLISHED:
                continue
            if local.rsplit(":", 1)[-1].upper() == want:
                n += 1
    return n


def foxglove_tcp_clients(port: int) -> int:
    """ESTABLISHED peers on ``port`` (IPv4 + IPv6). 0 if ``/proc/net`` is missing."""
    chunks: list[str] = []
    for name in ("tcp", "tcp6"):
        path = Path("/proc/net") / name
        if path.is_file():
            chunks.append(path.read_text(encoding="utf-8", errors="replace"))
    return count_tcp_established(port, *chunks)


def wait_for_foxglove_client(
    log_path: Path,
    timeout_s: float,
    *,
    port: int | None = None,
    poll_s: float = 0.2,
) -> bool:
    """Block until a client connects, or ``timeout_s`` elapses.

    Prefers an ESTABLISHED TCP peer on ``port`` (Lichtblick connecting
    through docker ``-p`` does not always flush the bridge log in time).
    Falls back to scraping ``foxglove_bridge.log``. ``timeout_s <= 0``
    skips the wait.
    """
    if timeout_s <= 0:
        return False
    deadline = time.time() + timeout_s
    while True:
        if port is not None and foxglove_tcp_clients(port) > 0:
            return True
        if log_path.is_file() and foxglove_client_connected(
            log_path.read_text(encoding="utf-8", errors="replace")
        ):
            return True
        if time.time() >= deadline:
            return False
        time.sleep(poll_s)


def _print_live_connect(port: int) -> None:
    print()
    print("==> live Foxglove / Lichtblick — connect now (bag starts on connect)")
    print("    1. Open http://localhost:8080 (compose lichtblick) or Foxglove Studio")
    print(f"    2. Open connection → Foxglove WebSocket → ws://localhost:{port}")
    print("       (8766 by default so it does not collide with the sim stack on 8765)")
    print("    3. Layout: lichtblick/onboard_live.json")
    print("       BEV: /lidar_points/above_ground  persp: /lidar_points/ground")
    print()


def _record_and_play(
    source_bag: Path,
    replay_bag: Path,
    log_dir: Path,
    *,
    rate: float,
    duration_s: float,
    loop: bool = False,
    play_delay_s: float = 0.0,
    record: bool = True,
) -> None:
    recorder = None
    if record:
        record_cmd = [
            "ros2",
            "bag",
            "record",
            "--use-sim-time",
            "-s",
            "mcap",
            "-o",
            str(replay_bag),
            *RECORD_TOPICS,
        ]
        recorder = _popen(record_cmd, log_dir / "record.log")
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise RuntimeError(
                "ros2 bag record exited immediately:\n"
                + (log_dir / "record.log").read_text()[-2000:]
            )

    play_cmd = [
        "ros2",
        "bag",
        "play",
        str(source_bag),
        "-s",
        bag_storage_id(source_bag),
        "--clock",
        "--rate",
        str(rate),
        "--disable-keyboard-controls",
        "--topics",
        *PLAY_TOPICS,
    ]
    if loop:
        play_cmd.append("--loop")
    if play_delay_s > 0:
        play_cmd += ["--delay", str(play_delay_s)]
    print("  playing", source_bag)
    player = _popen(play_cmd, log_dir / "play.log")
    wall_limit = None
    if duration_s > 0:
        wall_limit = duration_s / max(rate, 1e-6) + 15.0
    try:
        try:
            if wall_limit is None:
                player.wait()
            else:
                try:
                    player.wait(timeout=wall_limit)
                except subprocess.TimeoutExpired:
                    print(f"  stopping playback after {duration_s:.0f}s bag time")
                    _stop(player)
        except KeyboardInterrupt:
            print("  interrupted — stopping playback")
            _stop(player)
        if player.returncode not in (0, None, -signal.SIGINT, -signal.SIGTERM):
            print("  play log tail:\n", (log_dir / "play.log").read_text()[-1500:])
    finally:
        if recorder is not None:
            time.sleep(2.0)
            _stop(recorder)
        _stop(player)


def replay_pipeline(
    source_bag: Path,
    run_dir: Path,
    *,
    mission: str,
    rate: float,
    duration_s: float,
    live: bool = False,
    foxglove_port: int = DEFAULT_FOXGLOVE_PORT,
    live_wait_s: float = 20.0,
    loop: bool = False,
    record: bool = True,
) -> Path:
    log_dir = run_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    replay_bag = run_dir / "replay_bag"
    procs: list[tuple[str, subprocess.Popen]] = []
    try:
        print("==> starting autonomy nodes (use_sim_time, car LiDAR remap)")
        procs = _start_nodes(log_dir)
        if live:
            procs.append(
                (
                    "foxglove_bridge",
                    _start_foxglove(
                        log_dir,
                        run_dir / "foxglove_params.yaml",
                        foxglove_port,
                    ),
                )
            )
            _print_live_connect(foxglove_port)
        print("==> waiting for ~/setup")
        time.sleep(4.0)
        dead = [name for name, proc in procs if proc.poll() is not None]
        if dead:
            raise RuntimeError(
                "nodes died at startup: "
                + ", ".join(dead)
                + "\n"
                + (log_dir / f"{dead[0]}.log").read_text()[-2000:]
            )
        _setup_and_activate(mission, log_dir)
        time.sleep(2.0)
        if live:
            fox = next((p for n, p in procs if n == "foxglove_bridge"), None)
            if fox is not None and fox.poll() is not None:
                raise RuntimeError(
                    "foxglove_bridge died — is the port already bound?\n"
                    + (log_dir / "foxglove_bridge.log").read_text()[-2000:]
                )
            fox_log = log_dir / "foxglove_bridge.log"
            print(
                f"==> waiting for Foxglove client (starts immediately on connect, "
                f"else after {live_wait_s:.0f}s)"
            )
            if wait_for_foxglove_client(
                fox_log, live_wait_s, port=foxglove_port
            ):
                print("    client connected — starting bag")
            else:
                print(
                    "    no WebSocket client on "
                    f"ws://localhost:{foxglove_port} — starting bag anyway\n"
                    "    (Lichtblick must Open connection to this port; "
                    "the compose stack on 8765 is a different bridge. "
                    "Reconnect if a previous replay died.)"
                )
        if record:
            print("==> recording pipeline outputs + playing bag")
        else:
            print("==> playing bag (no output recording; pass --report to record)")
        _record_and_play(
            source_bag,
            replay_bag,
            log_dir,
            rate=rate,
            duration_s=duration_s,
            loop=loop,
            record=record,
        )
    finally:
        for _, proc in procs:
            _stop(proc, sig=signal.SIGTERM, wait_s=5.0)
    if record and not (replay_bag / "metadata.yaml").is_file():
        raise RuntimeError(
            f"replay bag missing metadata.yaml under {replay_bag}\n"
            + (log_dir / "record.log").read_text()[-2000:]
        )
    return replay_bag


def build_samples(source_bag: Path, replay_bag: Path) -> dict[str, Any]:
    print("==> reading replay bag for report")
    samples = load_pipeline_outputs(replay_bag)
    if not samples.get("steering"):
        print("==> replay bag has no /steering_angle; reading source bag")
        samples["steering"] = load_steering(source_bag)
    samples["source_counts"] = bag_topic_counts(source_bag)
    samples["source_duration_s"] = bag_duration_s(source_bag)
    _shift_times(samples)
    return samples


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Replay an onboard bag through the DV pipeline.",
    )
    ap.add_argument("bag", help="rosbag2 directory (under tools/sim_benchmark/)")
    ap.add_argument("--mission", default="trackdrive", choices=sorted(MISSION_BEHAVIORS))
    ap.add_argument("--rate", type=float, default=1.0, help="ros2 bag play rate")
    ap.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Play only this many bag-seconds (0 = full bag).",
    )
    ap.add_argument("--results-root", default=default_results_root())
    ap.add_argument(
        "--report",
        action="store_true",
        help=(
            "Record pipeline outputs to replay_bag and write the no-GT HTML "
            "report (or from --replay-bag with --skip-replay). Without this "
            "flag, playback does not record a second bag."
        ),
    )
    ap.add_argument(
        "--skip-replay",
        action="store_true",
        help="Skip playback; only generate a report from --replay-bag (requires --report).",
    )
    ap.add_argument("--replay-bag", default="", help="Existing replay bag for --skip-replay")
    ap.add_argument(
        "--live",
        action="store_true",
        help="Start foxglove_bridge and wait so Lichtblick can watch playback.",
    )
    ap.add_argument(
        "--foxglove-port",
        type=int,
        default=DEFAULT_FOXGLOVE_PORT,
        help="WebSocket port for --live (default 8766, avoids the sim stack on 8765).",
    )
    ap.add_argument(
        "--live-wait-s",
        type=float,
        default=20.0,
        help=(
            "Max seconds to wait for a Lichtblick/Foxglove client before "
            "playing the bag anyway (--live). 0 = do not wait. Playback "
            "starts as soon as a client connects."
        ),
    )
    ap.add_argument(
        "--loop",
        action="store_true",
        help="Loop bag play (useful with --live). Ctrl-C stops playback.",
    )
    args = ap.parse_args()
    if args.skip_replay and not args.report:
        raise SystemExit("--skip-replay requires --report")

    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    source = resolve_benchmark_path(args.bag)
    if not source.is_dir():
        raise SystemExit(f"bag directory not found: {source}")

    run_dir = make_run_dir(args.results_root, "onboard", args.mission)
    replay_bag = resolve_benchmark_path(args.replay_bag) if args.replay_bag else run_dir / "replay_bag"
    if not args.skip_replay:
        replay_bag = replay_pipeline(
            source,
            run_dir,
            mission=args.mission,
            rate=args.rate,
            duration_s=args.duration_s,
            live=args.live,
            foxglove_port=args.foxglove_port,
            live_wait_s=args.live_wait_s,
            loop=args.loop,
            record=args.report,
        )
    elif not (replay_bag / "metadata.yaml").is_file():
        raise SystemExit(f"--skip-replay needs a recorded bag at {replay_bag}")

    if not args.report:
        print(f"Replay finished (no output bag or report; pass --report for both). Logs in {run_dir}")
        return

    samples = build_samples(source, replay_bag)
    summary = summarize(samples)
    summary["bag"] = str(source)
    summary["mission"] = args.mission
    summary["strategy"] = args.mission
    summary["rate"] = args.rate
    summary["duration_s"] = args.duration_s
    report = write_onboard_report(summary, samples, run_dir)
    print(f"Wrote {run_dir / 'results.json'}")
    print(f"Wrote {report}")
    if summary["n_conos_raw"] == 0:
        print("Note: no /Conos_raw in the replay bag — check logs/cone_detection_node.log")
    if summary["n_slam"] == 0:
        print("Note: no /slam/pose in the replay bag — check logs/slam_node.log")


if __name__ == "__main__":
    maybe_reexec_in_docker(
        "run_onboard_replay.py",
        extra_docker_args=live_docker_publish_args(),
        extra_env={"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "42")},
    )
    main()
