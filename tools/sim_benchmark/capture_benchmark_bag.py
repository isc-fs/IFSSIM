from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from common import bag_topic_names, dv_pipeline_ros_setup_shell


DEFAULT_TOPICS = [
    "/lidar/Lidar1",
    "/imu",
    "/motor_rpm",
    "/odom",  # optional: SLAM benchmark can synthesize from IMU/RPM if omitted
    "/steering_angle",
    "/brake_pressure",
    "/testing_only/odom",
    "/testing_only/track",  # sim GT cone positions (perception + SLAM GT-cone replay)
    # "/Conos_raw",  # optional: enable when pipeline is running (SLAM perception-cone replay)
]


def main() -> None:
    ap = argparse.ArgumentParser(description="Capture simulator-only benchmark rosbag.")
    ap.add_argument("--output-dir", default="tools/sim_benchmark/results/capture")
    ap.add_argument("--bag-name", default="")
    ap.add_argument("--duration-s", type=float, default=60.0)
    ap.add_argument("--topics", nargs="*", default=DEFAULT_TOPICS)
    ap.add_argument("--manual-only", action="store_true")
    ap.add_argument(
        "--container-name",
        default=os.environ.get("IFSSIM_DV_CONTAINER", "ifssim-dv_pipeline_stack-1"),
        help="Docker container running ROS 2 (default: IFSSIM_DV_CONTAINER or ifssim-dv_pipeline_stack-1).",
    )
    ap.add_argument(
        "--local-ros2",
        action="store_true",
        help="Record with host ros2 CLI instead of docker exec.",
    )
    ap.add_argument(
        "--with-conos-raw",
        action="store_true",
        help="Also record /Conos_raw (pipeline must be running; for perception benchmark, not SLAM).",
    )
    args = ap.parse_args()

    topics = list(args.topics)
    if args.with_conos_raw and "/Conos_raw" not in topics:
        topics.append("/Conos_raw")

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    bag_name = args.bag_name or f"sim_benchmark_{ts}"
    manifest = {
        "created_at_utc": ts,
        "bag_name": bag_name,
        "topics": topics,
        "duration_s": args.duration_s,
        "manual_only": bool(args.manual_only),
    }
    (out_dir / f"{bag_name}.manifest.json").write_text(json.dumps(manifest, indent=2))

    if args.manual_only:
        print("Wrote manifest only.")
        return

    if args.local_ros2:
        if shutil.which("ros2") is None:
            raise RuntimeError(
                "ros2 executable not found on host PATH. "
                "Remove --local-ros2 to record from the docker container."
            )
        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", str(out_dir / bag_name), *topics]
        print("Running:", " ".join(cmd))
        proc = subprocess.Popen(cmd)
        try:
            proc.wait(timeout=args.duration_s)
        except subprocess.TimeoutExpired:
            proc.terminate()
            proc.wait(timeout=10)
        print(f"Capture finished: {out_dir / bag_name}")
        return

    duration_s = max(1, int(args.duration_s))
    topics_quoted = " ".join(shlex.quote(topic) for topic in topics)
    bag_name_quoted = shlex.quote(bag_name)
    container_cmd = (
        "set -eo pipefail; "
        f"{dv_pipeline_ros_setup_shell()}"
        "mkdir -p /tmp/recording; "
        "cd /tmp/recording; "
        f"rm -rf {bag_name_quoted}; "
        f"ros2 bag record -s mcap -o {bag_name_quoted} {topics_quoted} & "
        "REC_PID=$!; "
        f"sleep {duration_s}; "
        "kill -INT $REC_PID 2>/dev/null || true; "
        "wait $REC_PID 2>/dev/null || true"
    )

    print(f"Running in container {args.container_name}: ros2 bag record -s mcap -o {bag_name} ...")
    subprocess.run(
        ["docker", "exec", "-i", args.container_name, "bash", "-lc", container_cmd],
        check=True,
    )

    host_bag_dir = out_dir / bag_name
    subprocess.run(
        [
            "docker",
            "cp",
            f"{args.container_name}:/tmp/recording/{bag_name}",
            str(host_bag_dir),
        ],
        check=True,
    )
    subprocess.run(
        ["docker", "exec", "-i", args.container_name, "rm", "-rf", f"/tmp/recording/{bag_name}"],
        check=True,
    )
    print(f"Capture finished: {host_bag_dir}")
    recorded = bag_topic_names(host_bag_dir)
    for topic in topics:
        if topic not in recorded:
            print(
                f"WARNING: {topic} was requested but is not in the bag. "
                "If this is /testing_only/track, ensure the sim is running and "
                "re-capture after updating capture_benchmark_bag.py (fs_msgs env)."
            )


if __name__ == "__main__":
    main()
