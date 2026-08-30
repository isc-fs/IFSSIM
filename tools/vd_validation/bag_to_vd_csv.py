#!/usr/bin/env python3
"""Export the four signals a vehicle-dynamics validation needs, from a rosbag.

    steering angle, motor rpm, yaw rate, lateral acceleration

That is the whole vocabulary of the standard validation in the literature:
drive the model with the STEERING the vehicle was given, hold it at the SPEED
the vehicle was doing, and compare the YAW RATE it produces against the one
that was measured. See docs/vehicle_dynamics_alignment.md.

Reads both storage formats without needing ROS on the host: .mcap through the
mcap-ros2 reader, .db3 by parsing CDR directly, because the sqlite bags in
tools/bags predate the switch to mcap and are the only ones carrying steering.

    python3 bag_to_vd_csv.py <bag-dir-or-file> -o out.csv
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sqlite3
import struct
import sys

# The topics we need, and the ones we accept as aliases for them. /steering_angle
# is the road-wheel command in radians; /motor_rpm is at the MOTOR, not the wheel.
WANT = {
    "steer":  ("/steering_angle",),
    "rpm":    ("/motor_rpm",),
    "imu":    ("/imu", "/imu/data"),
}


class Cdr:
    """Just enough CDR to read the two message types we care about.

    Every primitive is aligned to its own size, counted from the start of the
    BODY -- i.e. after the 4-byte encapsulation header, which is why `base` is
    subtracted before taking the modulus. Getting that wrong reads plausible
    garbage rather than failing, so it is worth stating.
    """

    def __init__(self, buf: bytes):
        self.b = buf
        self.base = 4                      # encapsulation header
        self.p = 4
        self.le = buf[1] in (1, 3)         # 0x01 little-endian CDR

    def _align(self, n: int) -> None:
        off = (self.p - self.base) % n
        if off:
            self.p += n - off

    def u32(self) -> int:
        self._align(4)
        v = struct.unpack_from("<I" if self.le else ">I", self.b, self.p)[0]
        self.p += 4
        return v

    def i32(self) -> int:
        self._align(4)
        v = struct.unpack_from("<i" if self.le else ">i", self.b, self.p)[0]
        self.p += 4
        return v

    def f32(self) -> float:
        self._align(4)
        v = struct.unpack_from("<f" if self.le else ">f", self.b, self.p)[0]
        self.p += 4
        return v

    def f64(self) -> float:
        self._align(8)
        v = struct.unpack_from("<d" if self.le else ">d", self.b, self.p)[0]
        self.p += 8
        return v

    def string(self) -> str:
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode("utf-8", "replace") if n else ""
        self.p += n
        return s


def decode_float32(buf: bytes) -> float:
    return Cdr(buf).f32()


def decode_imu(buf: bytes) -> tuple[float, float, float]:
    """-> (yaw_rate, ax, ay). Orientation and all three covariances skipped."""
    c = Cdr(buf)
    c.i32(); c.u32(); c.string()            # header
    for _ in range(4):                      # orientation quaternion
        c.f64()
    for _ in range(9):                      # orientation covariance
        c.f64()
    c.f64(); c.f64()                        # angular velocity x, y
    wz = c.f64()
    for _ in range(9):
        c.f64()
    ax = c.f64()
    ay = c.f64()
    return wz, ax, ay


def read_db3(path: str):
    con = sqlite3.connect(path)
    topics = {tid: name for tid, name in
              con.execute("SELECT id, name FROM topics")}
    out = []
    for tid, ts, data in con.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
        out.append((topics.get(tid, ""), ts, bytes(data)))
    con.close()
    return out


def read_mcap(path: str):
    from mcap.reader import make_reader
    out = []
    with open(path, "rb") as fh:
        for sch, chan, msg in make_reader(fh).iter_messages():
            out.append((chan.topic, msg.log_time, msg.data))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()

    files = []
    if os.path.isdir(a.bag):
        files = sorted(glob.glob(os.path.join(a.bag, "*.db3"))) or \
                sorted(glob.glob(os.path.join(a.bag, "*.mcap")))
    else:
        files = [a.bag]
    if not files:
        print(f"no .db3 or .mcap under {a.bag}", file=sys.stderr)
        return 1

    msgs = []
    for f in files:
        msgs += read_db3(f) if f.endswith(".db3") else read_mcap(f)
    msgs.sort(key=lambda m: m[1])

    alias = {t: k for k, ts in WANT.items() for t in ts}
    series: dict[str, list] = {"steer": [], "rpm": [], "imu": []}
    for topic, ts, data in msgs:
        k = alias.get(topic)
        if k is None:
            continue
        try:
            if k == "imu":
                series[k].append((ts,) + decode_imu(data))
            else:
                series[k].append((ts, decode_float32(data)))
        except (struct.error, IndexError):
            pass

    missing = [k for k, v in series.items() if not v]
    if missing:
        print(f"bag has no usable {', '.join(missing)}", file=sys.stderr)
        return 1

    # The IMU is the fastest channel and the reference signal, so it sets the
    # time base; steering and rpm are held (zero-order) onto it, which is what
    # they physically are between samples anyway.
    t0 = series["imu"][0][0]

    def hold(rows, t, i_state=[0]):
        pass

    def resample(rows, t):
        out, j, last = [], 0, rows[0][1]
        for ti in t:
            while j < len(rows) and rows[j][0] <= ti:
                last = rows[j][1]
                j += 1
            out.append(last)
        return out

    tt = [r[0] for r in series["imu"]]
    steer = resample(series["steer"], tt)
    rpm = resample(series["rpm"], tt)

    with open(a.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "steer_rad", "motor_rpm", "yaw_rate_rps", "ax_mps2", "ay_mps2"])
        for (ts, wz, ax, ay), s, r in zip(series["imu"], steer, rpm):
            w.writerow([f"{(ts - t0) / 1e9:.6f}", f"{s:.6f}", f"{r:.3f}",
                        f"{wz:.6f}", f"{ax:.5f}", f"{ay:.5f}"])

    n = len(series["imu"])
    dur = (tt[-1] - t0) / 1e9
    print(f"wrote {a.out}: {n} rows, {dur:.1f} s, {n/max(dur,1e-9):.0f} Hz")
    print(f"  steering  {min(steer):+.4f} .. {max(steer):+.4f} rad")
    print(f"  motor rpm {min(rpm):+.0f} .. {max(rpm):+.0f}")
    print(f"  yaw rate  {min(r[1] for r in series['imu']):+.3f} .. "
          f"{max(r[1] for r in series['imu']):+.3f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
