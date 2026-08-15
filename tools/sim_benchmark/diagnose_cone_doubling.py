"""Throwaway: inspect a bag's recorded /Conos (live SLAM map) and /Conos_raw
(perception) marker arrays — colours used, and whether a single array contains
DOUBLED landmarks (two markers for one physical cone) vs a clean single row.

    python diagnose_cone_doubling.py results/capture/<bag_name>
"""
from __future__ import annotations

import argparse
import math

from common import maybe_reexec_in_docker, resolve_benchmark_path


def _open(path: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id="mcap"), ConverterOptions("", ""))
    return r


def _last_markerarray(bag: str, topic: str):
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
    reader = _open(bag)
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        return None
    cls = get_message(topics[topic])
    last = None
    count = 0
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            last = deserialize_message(data, cls)
            count += 1
    return last, count


def _points(ma):
    """Return [(x,y,(r,g,b))] for marker points, handling per-marker or per-point colour."""
    out = []
    if ma is None:
        return out
    for m in ma.markers:
        # SPHERE_LIST / POINTS style: points + colors arrays
        if getattr(m, "points", None):
            cols = getattr(m, "colors", []) or []
            for i, p in enumerate(m.points):
                c = cols[i] if i < len(cols) else m.color
                out.append((p.x, p.y, (round(c.r, 2), round(c.g, 2), round(c.b, 2))))
        else:
            p = m.pose.position
            c = m.color
            out.append((p.x, p.y, (round(c.r, 2), round(c.g, 2), round(c.b, 2))))
    return out


def _analyze(name, pts):
    print(f"\n== {name}: {len(pts)} markers ==")
    if not pts:
        return
    # colour histogram
    hist = {}
    for _, _, c in pts:
        hist[c] = hist.get(c, 0) + 1
    print("  colours (r,g,b)->count:", dict(sorted(hist.items(), key=lambda kv: -kv[1])))
    # intra-array nearest-neighbour (duplication detector)
    xy = [(x, y) for x, y, _ in pts]
    nn = []
    for i, p in enumerate(xy):
        best = 1e9
        for j, q in enumerate(xy):
            if i == j:
                continue
            d = math.hypot(p[0] - q[0], p[1] - q[1])
            if d < best:
                best = d
        nn.append(best)
    nn.sort()
    if nn:
        print("  intra nearest-neighbour (m): min=%.2f p10=%.2f median=%.2f"
              % (nn[0], nn[len(nn) // 10], nn[len(nn) // 2]))
        print("  pairs <0.8m (stacked dup):", sum(1 for d in nn if d < 0.8),
              " <1.5m:", sum(1 for d in nn if d < 1.5))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--results-root", default=None)
    args = ap.parse_args()
    maybe_reexec_in_docker("diagnose_cone_doubling.py")

    bag = str(resolve_benchmark_path(args.bag))
    print(f"bag={bag}")
    for topic in ("/Conos", "/Conos_raw", "/Conos_Orange"):
        res = _last_markerarray(bag, topic)
        if res is None:
            print(f"\n== {topic}: NOT IN BAG ==")
            continue
        last, count = res
        print(f"\n--- {topic}: {count} msgs in bag; analysing LAST ---")
        _analyze(topic, _points(last))


if __name__ == "__main__":
    main()
