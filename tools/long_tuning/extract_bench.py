#!/usr/bin/env python3
"""Extract steering + longitudinal signals from a bench-replay mcap into CSV.

Standalone — uses the `mcap` + `mcap-ros2-support` readers (no ROS install).
Decodes CDR via the schemas embedded in the bag.

Usage: python3 extract_bench.py <bag_dir_or_mcap> [out.csv]
"""
import sys, os, glob, csv
from mcap_ros2.reader import read_ros2_messages

def find_mcap(path):
    if os.path.isdir(path):
        m = glob.glob(os.path.join(path, "*.mcap"))
        if not m: sys.exit(f"no .mcap in {path}")
        return m[0]
    return path

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else \
        "bags/_bench_runs/bench_autocross_hairpin_9m_left_20260712_202331_carparity_20260712_231526"
    out = sys.argv[2] if len(sys.argv) > 2 else "tools/long_tuning/hairpin_steering.csv"
    mcap_path = find_mcap(src)

    WANT = {"/ctrl/cmd", "/steering/feedback", "/steering_angle", "/assi/state",
            "/odom", "/motor_rpm"}
    rows = {}   # t_ns -> dict; we merge by nearest later. Keep per-topic streams instead:
    streams = {t: [] for t in WANT}
    t0 = None
    for m in read_ros2_messages(mcap_path, topics=list(WANT)):
        topic = m.channel.topic
        raw = m.log_time                     # datetime or int-ns depending on version
        ts = raw.timestamp() if hasattr(raw, "timestamp") else raw / 1e9
        if t0 is None: t0 = ts
        t = ts - t0                          # seconds
        r = m.ros_msg
        if topic == "/ctrl/cmd":
            streams[topic].append((t, r.angular.z, r.linear.x))
        elif topic == "/steering/feedback":
            d = list(r.data)
            streams[topic].append((t, d[0] if len(d)>0 else 0.0,
                                      d[1] if len(d)>1 else 0.0,
                                      d[2] if len(d)>2 else 0.0))
        elif topic in ("/steering_angle", "/motor_rpm"):
            streams[topic].append((t, r.data))
        elif topic == "/assi/state":
            streams[topic].append((t, r.data))
        elif topic == "/odom":
            streams[topic].append((t, r.twist.twist.linear.x))

    # Build a merged, time-sorted CSV sampled on /steering/feedback ticks (100 Hz).
    def near(seq, t):
        return min(seq, key=lambda x: abs(x[0]-t)) if seq else None
    fb = streams["/steering/feedback"]
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t","assi","cmd_az","cmd_linx","target_deg","actual_deg","motor_deg","lws_rad","odom_vx","motor_rpm"])
        for t, actual, target, motor in fb:
            a = near(streams["/assi/state"], t)
            c = near(streams["/ctrl/cmd"], t)
            l = near(streams["/steering_angle"], t)
            o = near(streams["/odom"], t)
            mr = near(streams["/motor_rpm"], t)
            w.writerow([f"{t:.3f}", a[1] if a else "",
                        f"{c[1]:.4f}" if c else "", f"{c[2]:.4f}" if c else "",
                        f"{target:.2f}", f"{actual:.2f}", f"{motor:.2f}",
                        f"{l[1]:.4f}" if l else "", f"{o[1]:.3f}" if o else "",
                        f"{mr[1]:.1f}" if mr else ""])
    print(f"wrote {out}")
    print(f"  streams: " + ", ".join(f"{k}={len(v)}" for k,v in streams.items()))
    # quick sanity: driving window + steering divergence
    drive = [t for t,s in streams["/assi/state"] if s==3]
    if drive:
        A,B = min(drive), max(drive)
        print(f"  DRIVING {A:.2f}..{B:.2f}s ({B-A:.1f}s)")
        w2 = [(t,a,tg,mo) for (t,a,tg,mo) in fb if A<=t<=B]
        if w2:
            errs=[abs(a-tg) for _,a,tg,_ in w2]
            print(f"  |actual-target|: mean={sum(errs)/len(errs):.1f}°  max={max(errs):.1f}°")
            merr=[abs(mo-tg) for _,_,tg,mo in w2]
            print(f"  |motor-target|:  mean={sum(merr)/len(merr):.1f}°  max={max(merr):.1f}°  <- stepper vs command")

if __name__ == "__main__":
    main()
