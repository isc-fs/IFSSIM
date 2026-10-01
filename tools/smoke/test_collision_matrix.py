#!/usr/bin/env python3
"""
docs/collision_matrix.md against a running sim (RPC only).

validateCollisionMatrix spawns a solid FSDSProp box and an FSDSPropLidarOnly
strip in front of the car, waits a few frames for Chaos to add them to its
scene queries, then checks every row of the matrix except the GPU LiDAR:
the wheel trace and the ground query ignore both props, the CPU LiDAR sees
both, the solid one stops the car and the cones, the strip lets both through,
and the channel slots match FSDSCollision.h. The props are removed afterwards
unless --keep is given (to drive over the strip, or to run
lidar_sees_test_props.py for the GPU LiDAR row).

    python tools/smoke/test_collision_matrix.py            # props 10 m ahead of the car
    python tools/smoke/test_collision_matrix.py --ahead 30 --keep
"""
import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))
from ifssim.client import IFSSIMClient  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default="127.0.0.1:41451")
    ap.add_argument("--ahead", type=float, default=10.0, help="metres from the car to the strip")
    ap.add_argument("--keep", action="store_true", help="leave the props in place")
    args = ap.parse_args()

    host, port = args.sim.rsplit(":", 1)
    client = IFSSIMClient(ip=host, port=int(port))
    if not client.ping():
        print(f"FAIL: no sim at {args.sim}")
        sys.exit(1)

    reply = json.loads(client._text_cmd(f"validateCollisionMatrix {args.ahead}"))
    if "checks" not in reply:
        print(f"FAIL: {reply}")
        sys.exit(1)
    for p in reply["props"]:
        print(f"prop {p['name']:10s} {p['ahead_m']:5.1f} m ahead, {p['left_m']:4.1f} m left, size {p['size_m']} m")
    for c in reply["checks"]:
        print(f"  {'ok  ' if c['ok'] else 'FAIL'} {c['name']:40s} {c['detail']}")

    if not args.keep:
        client._text_cmd("validateCollisionMatrix clear")

    if not reply["ok"]:
        print("\nFAIL")
        sys.exit(1)
    print("\nPASS")


if __name__ == "__main__":
    main()
