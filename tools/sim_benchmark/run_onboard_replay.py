#!/usr/bin/env python3
"""Replay an onboard (no-GT) rosbag through the live DV pipeline and report.

Plays sensor topics into the autonomy nodes (car topic names), records the
pipeline outputs, and writes an HTML report — detection counts, odom/SLAM
trajectories, map, autonomy vs pilot steering. There is no ground truth.

Usage (repo root; auto re-execs in ifssim-dv_pipeline_stack when host has no ROS):

    python tools/sim_benchmark/run_onboard_replay.py \\
        results/capture/manual_20260920_154527_indexed

    python tools/sim_benchmark/run_onboard_replay.py <bag> --duration-s 30 --rate 1.0
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
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


def _record_and_play(
    source_bag: Path,
    replay_bag: Path,
    log_dir: Path,
    *,
    rate: float,
    duration_s: float,
) -> None:
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
    print("  playing", source_bag)
    player = _popen(play_cmd, log_dir / "play.log")
    wall_limit = None
    if duration_s > 0:
        wall_limit = duration_s / max(rate, 1e-6) + 15.0
    try:
        if wall_limit is None:
            player.wait()
        else:
            try:
                player.wait(timeout=wall_limit)
            except subprocess.TimeoutExpired:
                print(f"  stopping playback after {duration_s:.0f}s bag time")
                _stop(player)
        if player.returncode not in (0, None, -signal.SIGINT, -signal.SIGTERM):
            print("  play log tail:\n", (log_dir / "play.log").read_text()[-1500:])
    finally:
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
) -> Path:
    log_dir = run_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    replay_bag = run_dir / "replay_bag"
    procs: list[tuple[str, subprocess.Popen]] = []
    try:
        print("==> starting autonomy nodes (use_sim_time, car LiDAR remap)")
        procs = _start_nodes(log_dir)
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
        print("==> recording pipeline outputs + playing bag")
        _record_and_play(
            source_bag,
            replay_bag,
            log_dir,
            rate=rate,
            duration_s=duration_s,
        )
    finally:
        for _, proc in procs:
            _stop(proc, sig=signal.SIGTERM, wait_s=5.0)
    if not (replay_bag / "metadata.yaml").is_file():
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
        description="Replay an onboard bag through the DV pipeline and write a no-GT HTML report.",
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
        "--skip-replay",
        action="store_true",
        help="Only regenerate the report from an existing --replay-bag.",
    )
    ap.add_argument("--replay-bag", default="", help="Existing replay bag for --skip-replay")
    args = ap.parse_args()

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
        )
    elif not (replay_bag / "metadata.yaml").is_file():
        raise SystemExit(f"--skip-replay needs a recorded bag at {replay_bag}")

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
        extra_docker_args=["--shm-size=1g"],
        extra_env={"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "42")},
    )
    main()
