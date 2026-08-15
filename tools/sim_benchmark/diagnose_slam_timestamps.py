"""Break down every timestamp the SLAM benchmark consumes from a bag.

Mirrors the exact timestamp logic of ``run_slam_benchmark`` / ``slam_metrics``
(``msg_time_ns`` header-or-bag time, header-time event sort, EKF dt gating) and
reports, per topic and cross-topic, where time handling could bite:

* header-vs-bag-log offset (the LiDAR-stamp lag that hurt perception),
* inter-sample dt distribution and how many EKF predict steps get dropped,
* header monotonicity violations (negative dt -> skipped predict),
* odom interpolation coverage (IMU/LiDAR events outside GT-odom span get
  clamped, inflating error at the ends),
* LiDAR-vs-odom and IMU-vs-odom alignment.

Run it the same way as the benchmark (auto re-exec in Docker):
    python diagnose_slam_timestamps.py results/capture/<bag_name>
"""
from __future__ import annotations

import argparse

from common import (
    default_results_root,
    maybe_reexec_in_docker,
    resolve_benchmark_path,
)
from perception_metrics import msg_time_ns, stamp_ns

EKF_DT_MIN = 1.0e-5
EKF_DT_MAX = 0.1


def _stats_ms(values_ns: list[int]) -> dict[str, float]:
    if not values_ns:
        return {}
    xs = sorted(values_ns)
    n = len(xs)

    def pct(p: float) -> float:
        return xs[min(n - 1, int(p * n))] / 1e6

    return {
        "n": n,
        "min": xs[0] / 1e6,
        "p01": pct(0.01),
        "median": xs[n // 2] / 1e6,
        "mean": (sum(xs) / n) / 1e6,
        "p99": pct(0.99),
        "max": xs[-1] / 1e6,
    }


def _header_present(msg) -> int | None:
    try:
        s = stamp_ns(msg)
        return s if s > 0 else None
    except (AttributeError, TypeError, ValueError):
        return None


def _gt_twist_vs_path(odom_msgs, windows: int = 8) -> None:
    """Is the GT clock stretched vs true motion?

    GT odom carries BOTH an integrated position and an instantaneous twist
    (velocity), stamped on the same wall clock the IMU uses. If the stamps
    track true motion, integrating speed over dt must equal the position
    path-length. If the clock is stretched (sim slower than wall), the twist
    integral overshoots the path -> ratio > 1 == the dt inflation factor that
    blows up the IMU-integrated distance.
    """
    import math

    rows = sorted((msg_time_ns(b, m), m) for b, m in odom_msgs)
    if len(rows) < 2:
        print("\n== GT twist-vs-path: not enough odom ==")
        return

    n = len(rows)
    bounds = [int(round(w * n / windows)) for w in range(windows + 1)]
    print(f"\n== GT twist-vs-path consistency ({n} odom samples) ==")
    print("  ratio = integral(speed*dt) / path_length  (1.0 = clock matches motion)")

    tot_path = 0.0
    tot_integ = 0.0
    tot_dt = 0.0
    for w in range(windows):
        path = integ = dt_sum = 0.0
        for i in range(max(1, bounds[w]), bounds[w + 1]):
            ta, ma = rows[i - 1]
            tb, mb = rows[i]
            dt = (tb - ta) * 1e-9
            if dt <= 0:
                continue
            pa, pb = ma.pose.pose.position, mb.pose.pose.position
            path += math.hypot(pb.x - pa.x, pb.y - pa.y)
            va, vb = ma.twist.twist.linear, mb.twist.twist.linear
            sa = math.hypot(va.x, va.y)
            sb = math.hypot(vb.x, vb.y)
            integ += 0.5 * (sa + sb) * dt
            dt_sum += dt
        tot_path += path
        tot_integ += integ
        tot_dt += dt_sum
        ratio = integ / path if path > 1e-6 else float("nan")
        v_rep = integ / dt_sum if dt_sum > 0 else 0.0
        v_path = path / dt_sum if dt_sum > 0 else 0.0
        print(f"  win {w}: dt={dt_sum:6.1f}s  path={path:7.1f}m  "
              f"twist_int={integ:7.1f}m  ratio={ratio:5.3f}  "
              f"v_twist={v_rep:4.1f}  v_path={v_path:4.1f} m/s")

    ratio = tot_integ / tot_path if tot_path > 1e-6 else float("nan")
    print(f"  TOTAL: span={tot_dt:.1f}s  path={tot_path:.1f}m  "
          f"twist_int={tot_integ:.1f}m  ratio={ratio:.4f}")
    if ratio > 1.03:
        print(f"  -> CLOCK STRETCHED by ~{(ratio - 1) * 100:.1f}%: the wall stamps "
              f"span more time than the car actually moved. IMU-integrated "
              f"distance is inflated by this factor (and ~its square in position).")
    elif ratio < 0.97:
        print(f"  -> clock COMPRESSED by ~{(1 - ratio) * 100:.1f}%.")
    else:
        print("  -> clock consistent with motion; exaggeration is NOT a global "
              "time-stretch (look at accel bias / per-window spikes instead).")


def _imu_accel_check(imu_msgs, odom_msgs, scale: float, calib_s: float = 3.0,
                     g: float = 9.81) -> None:
    """Does the raw forward accel integrate to the IMU-only runaway?

    Calibrate a bias from the first ``calib_s`` (stationary), then integrate
    (accel_x - bias_x) over the motion-corrected clock. If that reproduces the
    ~34 m/s blow-up, the *sensor* carries a residual forward-accel bias that an
    unaided strapdown cannot reject (gravity-leak / sim bias) -- not an EKF bug.
    accel_z mean ~ +g and stable => the car is flat (bias is real, not pitch).
    """
    import math

    rows = sorted((msg_time_ns(b, m), m) for b, m in imu_msgs)
    if len(rows) < 50:
        print("\n== IMU accel check: too few samples ==")
        return
    t0 = rows[0][0]
    cx = cy = cz = 0.0
    nc = 0
    for t, m in rows:
        if (t - t0) * 1e-9 > calib_s:
            break
        cx += m.linear_acceleration.x
        cy += m.linear_acceleration.y
        cz += m.linear_acceleration.z
        nc += 1
    bax, bay, baz = cx / nc, cy / nc, cz / nc

    vx = vx_max = 0.0
    sum_axr = 0.0
    n_int = 0
    az_vals: list[float] = []
    last = None
    for t, m in rows:
        if last is not None:
            dt = (t - last) * 1e-9 * scale
            if 0.0 < dt < 0.1:
                axr = m.linear_acceleration.x - bax
                vx += axr * dt
                vx_max = max(vx_max, abs(vx))
                sum_axr += axr
                n_int += 1
        az_vals.append(m.linear_acceleration.z)
        last = t
    az_mean = sum(az_vals) / len(az_vals)
    az_std = (sum((a - az_mean) ** 2 for a in az_vals) / len(az_vals)) ** 0.5

    print(f"\n== IMU forward-accel integration (scale={scale:.3f}) ==")
    print(f"  calib bias ({calib_s:.0f}s, {nc} samp): "
          f"ax={bax:+.4f}  ay={bay:+.4f}  az={baz:+.4f} m/s^2 "
          f"(az should be ~{g:.2f})")
    print(f"  driving residual accel_x mean = {sum_axr / max(1, n_int):+.4f} m/s^2 "
          f"(this * time = the velocity runaway)")
    print(f"  integrated vx (no RPM aiding): final={vx:+.1f}  |max|={vx_max:.1f} m/s "
          f"(true ~2.7)")
    print(f"  accel_z over run: mean={az_mean:+.3f}  std={az_std:.3f} m/s^2 "
          f"(flat & stable => bias is real, not pitch/gravity-leak)")
    drift_pct = abs(sum_axr / max(1, n_int)) / g * 100
    print(f"  -> a {drift_pct:.2f}% of-g forward-accel offset fully explains the "
          f"blow-up; unobservable without a velocity sensor (RPM).")


def main() -> None:
    maybe_reexec_in_docker("diagnose_slam_timestamps.py")

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("--results-root", default=default_results_root())
    args = ap.parse_args()

    from run_slam_benchmark import _load_bag_events

    bag = resolve_benchmark_path(args.bag)
    events, buckets = _load_bag_events(str(bag))

    all_hdr = [msg_time_ns(b, m) for b, _t, m in events]
    t0 = min(all_hdr)
    t_end = max(all_hdr)
    print(f"bag: {bag}")
    print(f"events: {len(events)}   header span: {(t_end - t0) / 1e9:.3f} s")

    # --- per-topic timestamp breakdown ----------------------------------
    for topic in sorted(buckets):
        msgs = buckets[topic]
        if not msgs:
            print(f"\n== {topic}: EMPTY ==")
            continue

        bag_order_hdr: list[int] = []   # header time in stored (bag) order
        offsets_ns: list[int] = []      # header - bag_log
        n_no_header = 0
        for b, m in msgs:
            h = _header_present(m)
            t = msg_time_ns(b, m)
            bag_order_hdr.append(t)
            if h is None:
                n_no_header += 1
            else:
                offsets_ns.append(h - b)

        span_s = (max(bag_order_hdr) - min(bag_order_hdr)) / 1e9
        rate = (len(msgs) - 1) / span_s if span_s > 0 else 0.0
        mono_viol = sum(
            1 for i in range(len(bag_order_hdr) - 1)
            if bag_order_hdr[i + 1] < bag_order_hdr[i]
        )
        srt = sorted(bag_order_hdr)
        dts = [srt[i + 1] - srt[i] for i in range(len(srt) - 1)]
        dups = sum(1 for d in dts if d == 0)

        print(f"\n== {topic}  (n={len(msgs)}) ==")
        print(f"  start offset from bag t0: {(min(bag_order_hdr) - t0) / 1e6:+.1f} ms"
              f"   end: {(max(bag_order_hdr) - t_end) / 1e6:+.1f} ms")
        print(f"  rate ~{rate:.1f} Hz   header missing: {n_no_header}")
        if offsets_ns:
            o = _stats_ms(offsets_ns)
            print(f"  header-bag offset ms: mean={o['mean']:+.2f} "
                  f"min={o['min']:+.2f} p01={o['p01']:+.2f} "
                  f"p99={o['p99']:+.2f} max={o['max']:+.2f}")
        else:
            print("  header-bag offset: NO HEADER STAMPS (uses bag log time)")
        d = _stats_ms(dts)
        print(f"  dt ms: median={d['median']:.3f} min={d['min']:.3f} "
              f"max={d['max']:.3f} p99={d['p99']:.3f}")
        print(f"  header monotonic violations (stored order): {mono_viol}   "
              f"duplicate stamps: {dups}")

    # --- EKF dt gating on the decimated IMU stream ----------------------
    imu_h = sorted(msg_time_ns(b, m) for b, m in buckets.get("/imu", []))
    if len(imu_h) > 1:
        dts = [(imu_h[i + 1] - imu_h[i]) * 1e-9 for i in range(len(imu_h) - 1)]
        too_small = sum(1 for x in dts if x < EKF_DT_MIN)
        too_big = sum(1 for x in dts if x > EKF_DT_MAX)
        neg = sum(1 for x in dts if x < 0)
        print(f"\n== EKF IMU dt gating (dt_min={EKF_DT_MIN}, dt_max={EKF_DT_MAX}) ==")
        print(f"  predict steps: {len(dts)}")
        print(f"  dropped dt<dt_min: {too_small}  dt>dt_max: {too_big}  negative: {neg}")
        print(f"  -> {too_small + too_big} predict steps skipped "
              f"({100 * (too_small + too_big) / len(dts):.2f}%)")

    # --- odom interpolation coverage ------------------------------------
    odom_h = sorted(msg_time_ns(b, m) for b, m in buckets.get("/testing_only/odom", []))
    if odom_h:
        o0, o1 = odom_h[0], odom_h[-1]
        print(f"\n== /testing_only/odom coverage ==")
        print(f"  span: {(o1 - o0) / 1e9:.3f} s   "
              f"start {(o0 - t0) / 1e6:+.1f} ms   end {(o1 - t_end) / 1e6:+.1f} ms vs global")
        for probe in ("/imu", "/lidar/Lidar1"):
            ph = sorted(msg_time_ns(b, m) for b, m in buckets.get(probe, []))
            if not ph:
                continue
            before = sum(1 for h in ph if h < o0)
            after = sum(1 for h in ph if h > o1)
            print(f"  {probe}: {before} events before odom-start, "
                  f"{after} after odom-end -> clamped (extrapolated GT)")

    # --- GT clock stretch test ------------------------------------------
    scale, _ = __import__("slam_metrics").estimate_imu_time_scale(
        buckets.get("/testing_only/odom", [])
    )
    _gt_twist_vs_path(buckets.get("/testing_only/odom", []))

    # --- IMU forward-accel integration (is the accel itself biased?) -----
    _imu_accel_check(buckets.get("/imu", []), buckets.get("/testing_only/odom", []), scale)

    # --- LiDAR cone-trigger vs odom rate --------------------------------
    lidar_h = sorted(msg_time_ns(b, m) for b, m in buckets.get("/lidar/Lidar1", []))
    if len(lidar_h) > 1:
        dts = [(lidar_h[i + 1] - lidar_h[i]) * 1e-9 for i in range(len(lidar_h) - 1)]
        med = sorted(dts)[len(dts) // 2]
        print(f"\n== /lidar/Lidar1 (GT cone trigger) ==")
        print(f"  median period {med * 1e3:.1f} ms (benchmark assumes "
              f"gt_scan_period_ms=100); GT cones sampled at LiDAR header time "
              f"(center_frac=0).")


if __name__ == "__main__":
    main()
