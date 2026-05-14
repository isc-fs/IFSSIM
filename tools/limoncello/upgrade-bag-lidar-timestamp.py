#!/usr/bin/env python3
"""Add a per-point FLOAT64 `timestamp` field to /lidar/Lidar1 in an old bag.

Pre-#497 bags carry `/lidar/Lidar1` as PointCloud2 with the layout
(x, y, z, intensity) × FLOAT32 — 16 B per point, four fields.
LIMOncello's HESAI handler expects an additional FLOAT64 `timestamp`
field per point and refuses to deserialize the cloud without it.

This script reads an existing bag and writes a new one where every
`/lidar/Lidar1` message has been rewritten to the post-#497 layout
(24 B per point, five fields). All other topics are copied verbatim.

## Per-point timestamp choice

The IFSSIM LiDAR sim snapshots the car transform once per scan and
ray-traces every point from that single pose, so there's no
intra-scan motion distortion to compensate. Every point gets the
same value: `msg.header.stamp` converted to absolute seconds. That
matches what the post-#497 bridge emits live, so LIMOncello sees
identical input whether replayed from a rewritten bag or consumed
live.

## Usage

Run inside the dv_pipeline_stack container:

    docker exec -it ifssim-dv_pipeline_stack-1 bash -lc '
      source /opt/ros/humble/setup.bash &&
      source /dv_pipeline_stack_ws/install/setup.bash &&
      python3 /tools/limoncello/upgrade-bag-lidar-timestamp.py \
        /bags/lap_postveto_20260511_183638 \
        /bags/lap_postveto_20260511_183638_for_limoncello'

The output bag is overwriting-protected: existing target dirs are
left alone, the script bails out instead of clobbering them.

## Cost

A 60-second bag at 10 Hz LiDAR is 600 scans. Each scan's per-point
loop is vectorised through numpy (single broadcast for the
timestamp slot, single memcpy for the xyz+intensity quartets), so
the rewrite finishes in a few seconds for a multi-GB bag — bound
by mcap read/write, not the Python overhead.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
from pathlib import Path

import numpy as np

import rclpy.serialization  # noqa: F401  — registers (de)serializers
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField


LIDAR_TOPIC = "/lidar/Lidar1"


def upgrade_pointcloud(msg: PointCloud2) -> PointCloud2:
    """Return a new PointCloud2 with the FLOAT64 `timestamp` field.

    Idempotent: if `msg` already has a 24-byte stride with a
    `timestamp` field, return it unchanged.
    """
    has_timestamp = any(f.name == "timestamp" for f in msg.fields)
    if msg.point_step == 24 and has_timestamp:
        return msg

    n = msg.width * msg.height
    if n == 0:
        return msg

    # Source: raw byte view of the existing buffer, shape (n, point_step).
    src = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    # Destination: 24-byte stride, layout (x, y, z, intensity, timestamp).
    dst = np.zeros((n, 24), dtype=np.uint8)

    # Bulk-copy the leading xyz+intensity quartet — 16 B per point.
    # On every pre-#497 bag this is the entire source payload.
    dst[:, 0:16] = src[:, 0:16]

    # Broadcast the message timestamp into every point's `timestamp`
    # slot. The sim has no intra-scan motion, so per-point timestamps
    # are physically identical anyway — this is the same value the
    # post-#497 bridge writes live.
    stamp_seconds = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    stamp_bytes = np.frombuffer(struct.pack("<d", stamp_seconds), dtype=np.uint8)
    dst[:, 16:24] = stamp_bytes  # broadcasts (8,) onto (n, 8)

    new_msg = PointCloud2()
    new_msg.header = msg.header
    new_msg.height = msg.height
    new_msg.width = msg.width
    new_msg.is_bigendian = msg.is_bigendian
    new_msg.is_dense = msg.is_dense
    new_msg.point_step = 24
    new_msg.row_step = 24 * msg.width
    new_msg.fields = [
        PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
    ]
    new_msg.data = dst.tobytes()
    return new_msg


def rewrite_bag(src: Path, dst: Path) -> None:
    if not src.is_dir():
        raise SystemExit(f"source bag dir not found: {src}")
    if dst.exists():
        raise SystemExit(
            f"target {dst} already exists — refusing to overwrite. "
            "Pick a fresh name or `rm -rf` it first."
        )

    # Reader: open the existing bag, autodetect storage backend (mcap
    # in practice for #465 bags; sqlite3 for older ones).
    storage_in = rosbag2_py.StorageOptions(uri=str(src))
    converter_in = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_in, converter_in)

    # Mirror metadata onto the writer side.
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    storage_out = rosbag2_py.StorageOptions(uri=str(dst), storage_id="mcap")
    converter_out = rosbag2_py.ConverterOptions("", "")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_out, converter_out)
    for t in topic_types:
        writer.create_topic(t)

    # Walk every message; rewrite /lidar/Lidar1, pass everything else
    # through verbatim. rosbag2_py returns (topic_name, raw_bytes,
    # timestamp_ns) tuples — for non-LiDAR topics we don't even
    # deserialize, just re-serialize the same bytes.
    n_lidar = 0
    n_other = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == LIDAR_TOPIC:
            msg_type = type_map.get(topic, "sensor_msgs/msg/PointCloud2")
            assert msg_type == "sensor_msgs/msg/PointCloud2", (
                f"{LIDAR_TOPIC} is {msg_type}, expected sensor_msgs/msg/PointCloud2"
            )
            msg = deserialize_message(data, PointCloud2)
            msg = upgrade_pointcloud(msg)
            writer.write(topic, serialize_message(msg), t_ns)
            n_lidar += 1
        else:
            writer.write(topic, data, t_ns)
            n_other += 1

    print(f"==> {n_lidar} /lidar/Lidar1 msgs rewritten")
    print(f"==> {n_other} other msgs passed through")
    print(f"==> output: {dst}")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Add a per-point FLOAT64 timestamp field to /lidar/Lidar1 "
                    "in an old bag (pre-#497 bridge format) so LIMOncello can "
                    "consume it.",
    )
    ap.add_argument("src", type=Path, help="Source bag directory (mcap or sqlite3).")
    ap.add_argument("dst", type=Path, help="Destination bag directory (will be mcap).")
    args = ap.parse_args()

    rewrite_bag(args.src.expanduser().resolve(), args.dst.expanduser().resolve())


if __name__ == "__main__":
    main()
