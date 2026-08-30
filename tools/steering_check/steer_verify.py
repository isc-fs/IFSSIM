#!/usr/bin/env python3
"""Verify the lateral (steering) chain ON STANDS: does a commanded road-wheel
angle produce the REAL physical wheel angle?

The chain (after uDV #172 + pipeline max_steer_deg=18.2):

  cmd δ_road --[uDV ×E]--> grados(target) --[stepper]--> motor --[mech]--> PHYSICAL wheel
                                                              LWS reads the column ↑

  E = STEERING_RATIO(5.0) × MOTOR_TO_COLUMN(1.1) = 5.5     (grados = δ_road × E)
  column angle = grados / MOTOR_TO_COLUMN = δ_road × STEERING_RATIO
  LWS(actual) SHOULD read the column ≈ δ_road × STEERING_RATIO

Every on-board sensor reads THROUGH these ratios, so the only ground truth for
"is the wheel angle real" is a physical gauge on the wheel. This tool checks:

  Stage 1  uDV ratio     : feedback[target] / δ_road   → expect E (5.5)   [logs]
  Stage 2  stepper exec  : feedback[motor]  vs [target]                   [logs]
  Stage 3  LWS vs stepper: feedback[actual] vs [motor] (the divergence)   [logs]
  Stage 4  column→wheel  : PHYSICAL δ vs commanded δ_road → slope 1.0      [gauge]
           + which ratio the LWS implies: actual / physical

/steering/feedback = [actual(LWS,deg), target(commanded grados,deg), motor(stepper,deg)]
(order confirmed in uDV ros_task.c).

Usage:
  python3 steer_verify.py <bag_dir_or_mcap> <holds.csv> [gauge.csv] [--E 5.5 --ratio 5.0 --max-steer 18.2]
    holds.csv : hold_index,t_start,t_end,commanded_deg,norm   (written by steer_staircase.py)
    gauge.csv : hold_index,physical_deg                       (operator- or camera-recorded; optional)
Without gauge.csv it runs Stages 1-3 (log-only) and skips the physical check.
"""
import sys, os, glob, csv, argparse, statistics as st
from mcap_ros2.reader import read_ros2_messages


def find_mcap(p):
    return glob.glob(os.path.join(p, "*.mcap"))[0] if os.path.isdir(p) else p


def load_streams(mcap):
    S = {"/ctrl/cmd": [], "/steering/feedback": [], "/steering_angle": []}
    t0 = None
    for m in read_ros2_messages(mcap, topics=list(S)):
        raw = m.log_time
        ts = raw.timestamp() if hasattr(raw, "timestamp") else raw / 1e9
        if t0 is None:
            t0 = ts
        t = ts - t0
        tp, r = m.channel.topic, m.ros_msg
        if tp == "/ctrl/cmd":
            S[tp].append((t, r.angular.z))
        elif tp == "/steering/feedback":
            d = list(r.data)
            S[tp].append((t, d[0] if len(d) > 0 else 0.0, d[1] if len(d) > 1 else 0.0,
                          d[2] if len(d) > 2 else 0.0))
        elif tp == "/steering_angle":
            S[tp].append((t, r.data))
    return S


def avg_in(seq, a, b, idx):
    vals = [row[idx] for row in seq if a <= row[0] <= b]
    return st.mean(vals) if vals else float("nan")


def detect_holds(cmd, max_steer, min_hold=1.5, tol_norm=0.02):
    """Auto-detect steady steering-command segments from /ctrl/cmd.angular.z.
    Returns holds keyed by ORDER (hold_index = Nth steady segment), so the
    operator records gauge.csv against the same order. No clock alignment needed."""
    holds = []
    seg_start = None; seg_val = None
    for k, (t, az) in enumerate(cmd):
        if seg_val is None or abs(az - seg_val) > tol_norm:
            # close previous
            if seg_val is not None and seg_start is not None:
                dur = cmd[k-1][0] - seg_start
                if dur >= min_hold:
                    # trim first/last 0.3s (settling)
                    a = seg_start + 0.3; b = cmd[k-1][0] - 0.2
                    holds.append((a, b, seg_val * max_steer, seg_val))
            seg_start = t; seg_val = az
    if seg_val is not None and seg_start is not None and cmd[-1][0] - seg_start >= min_hold:
        holds.append((seg_start + 0.3, cmd[-1][0] - 0.2, seg_val * max_steer, seg_val))
    return [{"hold_index": i, "t_start": a, "t_end": b, "commanded_deg": c, "norm": n}
            for i, (a, b, c, n) in enumerate(holds)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("holds", nargs="?"); ap.add_argument("gauge", nargs="?")
    ap.add_argument("--E", type=float, default=5.5, help="uDV grados/road-wheel factor (5.0*1.1)")
    ap.add_argument("--ratio", type=float, default=5.0, help="column:road-wheel steering ratio")
    ap.add_argument("--max-steer", type=float, default=18.2)
    args = ap.parse_args()

    S = load_streams(find_mcap(args.bag))
    # holds: explicit holds.csv, or auto-detect from the /ctrl/cmd staircase.
    if args.holds and os.path.exists(args.holds):
        rows0 = list(csv.DictReader(open(args.holds)))
        if rows0 and "physical_deg" in rows0[0] and "t_start" not in rows0[0]:
            args.gauge = args.holds                       # 2nd positional was actually the gauge
            holds = detect_holds(S["/ctrl/cmd"], args.max_steer)
        else:
            holds = rows0
    else:
        holds = detect_holds(S["/ctrl/cmd"], args.max_steer)
    if not holds:
        sys.exit("no steady steering-command holds found (need a staircase in /ctrl/cmd)")
    gauge = {}
    if args.gauge and os.path.exists(args.gauge):
        for row in csv.DictReader(open(args.gauge)):
            gauge[int(row["hold_index"])] = float(row["physical_deg"])

    print(f"# E={args.E}  ratio(col:wheel)={args.ratio}  max_steer_deg={args.max_steer}")
    print(f"# feedback = [actual(LWS/col), target(grados), motor(stepper)] deg\n")
    hdr = ("idx  cmdδ  norm | target  motor  actual |  E_obs  mot/tgt  act/mot"
           + ("  physδ  phys/cmd  act/phys" if gauge else ""))
    print(hdr); print("-" * len(hdr))
    rows = []
    for h in holds:
        i = int(h["hold_index"]); a = float(h["t_start"]); b = float(h["t_end"])
        cmd = float(h["commanded_deg"]); norm = float(h.get("norm", cmd / args.max_steer))
        tgt = avg_in(S["/steering/feedback"], a, b, 2)
        mot = avg_in(S["/steering/feedback"], a, b, 3)
        act = avg_in(S["/steering/feedback"], a, b, 1)
        E_obs = tgt / cmd if abs(cmd) > 0.5 else float("nan")
        mt = mot - tgt
        am = act - mot
        line = f"{i:3d} {cmd:5.1f} {norm:5.2f} | {tgt:6.1f} {mot:6.1f} {act:6.1f} | {E_obs:6.2f} {mt:+7.1f} {am:+7.1f}"
        r = dict(i=i, cmd=cmd, tgt=tgt, mot=mot, act=act, E_obs=E_obs)
        if gauge:
            ph = gauge.get(i, float("nan"))
            pc = ph / cmd if abs(cmd) > 0.5 else float("nan")
            ap_ = act / ph if abs(ph) > 0.5 else float("nan")
            line += f" | {ph:5.1f} {pc:8.2f} {ap_:8.2f}"
            r.update(phys=ph, phys_cmd=pc, act_phys=ap_)
        print(line); rows.append(r)

    # ---- verdicts ----
    def fin(xs): return [x for x in xs if x == x]  # drop nan
    print("\n=== STAGE CHECKS ===")
    Eo = fin([r["E_obs"] for r in rows])
    if Eo:
        print(f"Stage 1 (uDV ratio): E_obs mean={st.mean(Eo):.2f} (expect {args.E}) "
              f"-> {'OK' if abs(st.mean(Eo)-args.E) < 0.3 else 'MISMATCH: uDV grados scaling off'}")
    mt = fin([r["mot"] - r["tgt"] for r in rows])
    if mt:
        print(f"Stage 2 (stepper exec): |motor-target| mean={st.mean([abs(x) for x in mt]):.1f}° "
              f"max={max(abs(x) for x in mt):.1f}° -> {'OK' if max(abs(x) for x in mt) < 5 else 'stepper lags/loses steps'}")
    am = fin([r["act"] - r["mot"] for r in rows])
    if am:
        mx = max(abs(x) for x in am)
        print(f"Stage 3 (LWS vs stepper): |actual-motor| mean={st.mean([abs(x) for x in am]):.1f}° max={mx:.1f}° "
              f"-> {'OK (tracks)' if mx < 5 else 'DIVERGENCE: LWS != stepper (slip / LWS fault / un-modeled 1.1)'}")
        # if it's the 1.1: actual ≈ motor/1.1
        r11 = fin([r["act"] / r["mot"] for r in rows if abs(r["mot"]) > 5])
        if r11:
            print(f"         actual/motor mean={st.mean(r11):.3f} (1.00=coupled, 0.91=un-modeled 1.1 motor→col)")
    if gauge:
        pc = fin([r.get("phys_cmd", float('nan')) for r in rows])
        if pc:
            # linear regression physical vs commanded
            xs = [r["cmd"] for r in rows if r.get("phys") == r.get("phys")]
            ys = [r["phys"] for r in rows if r.get("phys") == r.get("phys")]
            n = len(xs); mx_ = sum(xs)/n; my = sum(ys)/n
            sxx = sum((x-mx_)**2 for x in xs); sxy = sum((x-mx_)*(y-my) for x, y in zip(xs, ys))
            slope = sxy/sxx if sxx else float("nan"); off = my - slope*mx_
            print(f"\nStage 4 (PHYSICAL, ground truth): physical δ = {slope:.3f}·commanded + {off:+.2f}°")
            print(f"   -> slope {'OK (~1.0)' if abs(slope-1) < 0.08 else f'NET RATIO OFF by {slope:.2f}x — rescale'}; "
                  f"offset {'OK' if abs(off) < 1.5 else 'CENTERING off'}")
            apv = fin([r.get("act_phys", float('nan')) for r in rows])
            if apv:
                print(f"   LWS(actual)/physical mean={st.mean(apv):.2f} -> the TRUE column:wheel ratio "
                      f"(compare to configured {args.ratio}); if physical=command but this != {args.ratio}, the LWS/ratio const is the error")
    else:
        print("\n(no gauge.csv -> Stage 4 skipped. The physical wheel angle is the ONLY independent")
        print(" ground truth; add gauge.csv to prove the commanded angle is REAL.)")


if __name__ == "__main__":
    main()
