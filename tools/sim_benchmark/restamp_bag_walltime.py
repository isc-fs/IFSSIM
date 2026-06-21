"""Throwaway: rewrite a bag so every consumed header.stamp carries the bag
LOG (wall-clock) time instead of the sim-time stamp. Tests whether the
sim-time stamp clumping (IMU dt-median=0, 52% of preintegration steps at
dt<1e-5) is what's corrupting SLAM's IMU prior.

Only the topics the SLAM benchmark reads as *headered* are restamped
(/imu, /odom, /testing_only/odom, /lidar/Lidar1); everything else is passed
through byte-for-byte. The write timestamp is kept = original bag_t.

    python restamp_bag_walltime.py results/capture/<bag>  [--out <name>]
"""
from __future__ import annotations

import argparse

from common import maybe_reexec_in_docker, resolve_benchmark_path

# Headered topics the slam benchmark integrates on.
RESTAMP = {"/imu", "/odom", "/testing_only/odom", "/lidar/Lidar1"}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default=None, help="output bag dir name")
    ap.add_argument("--results-root", default=None)
    args = ap.parse_args()
    maybe_reexec_in_docker("restamp_bag_walltime.py")

    from rosbag2_py import (
        ConverterOptions, SequentialReader, SequentialWriter,
        StorageOptions, TopicMetadata,
    )
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message, serialize_message

    src = resolve_benchmark_path(args.bag)
    out_name = args.out or (src.name + "_walltime")
    out_dir = src.parent / out_name
    print(f"in : {src}\nout: {out_dir}")

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(src), storage_id="mcap"),
                ConverterOptions("", ""))
    topics = reader.get_all_topics_and_types()
    types = {t.name: t.type for t in topics}
    classes = {n: get_message(t) for n, t in types.items() if n in RESTAMP}

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(out_dir), storage_id="mcap"),
                ConverterOptions("", ""))
    for t in topics:
        writer.create_topic(TopicMetadata(
            name=t.name, type=t.type, serialization_format="cdr"))

    n = 0
    restamped = 0
    while reader.has_next():
        topic, data, bag_t = reader.read_next()
        if topic in classes:
            msg = deserialize_message(data, classes[topic])
            sec = bag_t // 1_000_000_000
            nsec = bag_t % 1_000_000_000
            msg.header.stamp.sec = int(sec)
            msg.header.stamp.nanosec = int(nsec)
            data = serialize_message(msg)
            restamped += 1
        writer.write(topic, data, bag_t)
        n += 1
    print(f"wrote {n} msgs ({restamped} restamped to wall time across {sorted(RESTAMP)})")


if __name__ == "__main__":
    main()
