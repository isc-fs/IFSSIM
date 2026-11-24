"""Compare the body's motion (vertical acceleration, pitch and roll rate) between
bags, band by band: the car against the sim, or one sim build against another.

    cd tools
    python3 -m body_motion car=/path/to/manual.mcap sim=/path/to/sim_bag/ \\
        [--matched] [--cache ~/.cache/ifssim-body-motion]

Each bag needs /imu and /motor_rpm. A label starting with ``car`` gets the
car's accelerometer freezes bridged (analysis.unfreeze); ``--freeze-fix`` /
``--no-freeze-fix`` name the labels explicitly instead.

Without ``--matched``: 4 s windows at steady speed, 0.5-15 Hz bands, by speed.
With it: 2 s windows split by mean |a_y| as well, 3-15 Hz. Compare car and
sim in the same |a_y| row; the 0-2.5 m/s^2 row is the straight driving.

Needs numpy, scipy, mcap and mcap-ros2-support (pip install). Used for #959
(damping and road roughness) and #963 (the pitch run).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    __package__ = "body_motion"

from . import analysis as A  # noqa: E402
from . import bag  # noqa: E402

QUANTITIES = ["a_z m/s2", "pitch deg/s", "roll deg/s"]


def _fmt_band(lo, hi):
    return f"{lo:g}-{hi:g}"


def _print_by_speed(res):
    for s in A.SPEEDS:
        print(f"\n== {_fmt_band(*s)} m/s  (windows: " + ", ".join(f"{k} {res[k][s][1]}" for k in res) + ")")
        print(f"{'band Hz':>10} | " + " | ".join(f"{n:^{9 * len(res)}}" for n in QUANTITIES))
        print(f"{'':>10} | " + " | ".join(" ".join(f"{k[:8]:>8}" for k in res) for _ in QUANTITIES))
        for b, band in enumerate(A.BANDS):
            cells = [" ".join(f"{res[k][s][0][q][b]:8.2f}" if res[k][s][0] is not None else f"{'-':>8}"
                              for k in res) for q in range(3)]
            print(f"{_fmt_band(*band):>10} | " + " | ".join(cells))


def _print_matched(res):
    bands = " ".join(f"{_fmt_band(*b):>6}" for b in A.MATCHED_BANDS)
    print(f"{'':8} {'m/s':>8} {'|a_y|':>8} {'n':>4} | " + " | ".join(f"{q:^20}" for q in QUANTITIES))
    print(f"{'':8} {'':>8} {'':>8} {'':>4} | " + " | ".join(bands for _ in QUANTITIES))
    for s in A.MATCHED_SPEEDS:
        for a in A.LATERAL:
            for label, r in res.items():
                rms, n = r[(s, a)]
                if rms is None or n < 3:
                    continue
                hi = "+" if math.isinf(a[1]) else f"{a[1]:g}"
                cells = [" ".join(f"{x:6.2f}" for x in rms[q]) for q in range(3)]
                print(f"{label[:8]:8} {_fmt_band(*s):>8} {f'{a[0]:g}-{hi}':>8} {n:4d} | " + " | ".join(cells))
        print()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python3 -m body_motion", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+", metavar="LABEL=BAG", help="a bag file or folder, with a short label")
    ap.add_argument("--matched", action="store_true", help="split by |a_y| as well (2 s windows, 3-15 Hz)")
    ap.add_argument("--cache", help="folder for .npz copies of the bags' /imu and /motor_rpm")
    ap.add_argument("--settings", default=str(Path(__file__).resolve().parents[2] / "settings.json"),
                    help="for WheelRadius and GearRatio (rpm -> m/s)")
    ap.add_argument("--freeze-fix", nargs="*", metavar="LABEL", help="labels to bridge accel freezes on "
                                                                     "(default: those starting with 'car')")
    ap.add_argument("--no-freeze-fix", action="store_true", help="bridge no freezes at all")
    a = ap.parse_args(argv)

    vp = json.loads(Path(a.settings).read_text())["Vehicles"]["FSCar"]["VehiclePhysics"]
    k_rpm = A.rpm_to_ms(vp["WheelRadius"], vp["GearRatio"])

    res = {}
    for item in a.bags:
        if "=" not in item:
            ap.error(f"{item!r}: expected LABEL=BAG")
        label, path = item.split("=", 1)
        if a.no_freeze_fix:
            fix = False
        elif a.freeze_fix is not None:
            fix = label in a.freeze_fix
        else:
            fix = label.startswith("car")
        d = A.attach_speed(bag.load(path, a.cache), k_rpm)
        res[label] = A.matched(d, fix) if a.matched else A.by_speed(d, fix)

    print(f"rpm -> m/s {k_rpm:.5f} (WheelRadius {vp['WheelRadius']}, GearRatio {vp['GearRatio']})")
    (_print_matched if a.matched else _print_by_speed)(res)
    return 0


if __name__ == "__main__":
    sys.exit(main())
