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
LIDAR_REP103_TOPIC = "/lidar/Lidar1_rep103"


# Common PointField layout for the post-#497 cloud — five fields,
# 24-byte stride, FLOAT64 timestamp at offset 16.
def _post497_fields():
    return [
        PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
    ]


def upgrade_pointcloud(msg: PointCloud2) -> PointCloud2:
    """Return a new PointCloud2 with the FLOAT64 `timestamp` field.

    Idempotent: if `msg` already has a 24-byte stride with a
    `timestamp` field, return it unchanged. Axis convention is
    preserved — see `flip_y` for the REP-103 variant.
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
    new_msg.fields = _post497_fields()
    new_msg.data = dst.tobytes()
    return new_msg


def flip_y(msg: PointCloud2) -> PointCloud2:
    """Return a deep copy of `msg` with every point's Y negated.

    Use to produce the /lidar/Lidar1_rep103 companion from a
    post-#497-layout /lidar/Lidar1 message (matches what
    ifssim_bridge does live for the REP-103 publisher). Per-point
    layout is preserved — only the Y field (FLOAT32 at offset 4) is
    sign-flipped. Header, fields, point_step, row_step copied
    verbatim.
    """
    n = msg.width * msg.height
    if n == 0 or msg.point_step != 24:
        return msg

    new_data = bytearray(msg.data)
    # View as (n, 6) float32 — point_step 24 = 6 × float32 worth of
    # space, with the trailing 8 bytes being a single float64. Slicing
    # column 1 gives every point's Y; we negate in place. The float64
    # at columns 4-5 stays untouched because we never write to those
    # columns.
    view = np.frombuffer(new_data, dtype=np.float32).reshape(n, 6)
    view[:, 1] = -view[:, 1]

    new_msg = PointCloud2()
    new_msg.header = msg.header
    new_msg.height = msg.height
    new_msg.width = msg.width
    new_msg.is_bigendian = msg.is_bigendian
    new_msg.is_dense = msg.is_dense
    new_msg.point_step = msg.point_step
    new_msg.row_step = msg.row_step
    new_msg.fields = list(msg.fields)
    new_msg.data = bytes(new_data)
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

    # Also create the REP-103 companion topic. Need a TopicMetadata
    # object with the same QoS profile as /lidar/Lidar1 so consumers
    # don't see QoS mismatch on replay.
    src_lidar_meta = next((t for t in topic_types if t.name == LIDAR_TOPIC), None)
    if src_lidar_meta is not None:
        rep103_meta = rosbag2_py.TopicMetadata(
            name=LIDAR_REP103_TOPIC,
            type=src_lidar_meta.type,
            serialization_format=src_lidar_meta.serialization_format,
            offered_qos_profiles=src_lidar_meta.offered_qos_profiles,
        )
        writer.create_topic(rep103_meta)

    # Walk every message; rewrite /lidar/Lidar1 in place AND emit a
    # Y-flipped /lidar/Lidar1_rep103 companion. Everything else
    # passes through verbatim. rosbag2_py returns (topic_name,
    # raw_bytes, timestamp_ns) tuples — for non-LiDAR topics we
    # don't even deserialize, just re-serialize the same bytes.
    n_lidar = 0
    n_rep103 = 0
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
            # REP-103 companion: same scan with Y negated. Bridge
            # publishes both topics live; we synthesize the same
            # pairing post-hoc here so LIMOncello sees identical
            # inputs in replay vs live.
            rep103 = flip_y(msg)
            writer.write(LIDAR_REP103_TOPIC, serialize_message(rep103), t_ns)
            n_rep103 += 1
        else:
            writer.write(topic, data, t_ns)
            n_other += 1

    print(f"==> {n_lidar} {LIDAR_TOPIC} msgs rewritten")
    print(f"==> {n_rep103} {LIDAR_REP103_TOPIC} msgs synthesised (Y-flipped)")
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
