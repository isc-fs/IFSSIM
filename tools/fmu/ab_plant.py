#!/usr/bin/env python3
"""Quantify how far the shadow FMU drifts from the Chaos reference.

Reads the `FSDS Plant shadow:` lines the pawn emits once a second and
reports the divergence summary docs/fmu_plant_migration.md asks for at
the Phase 6 boundary.

Divergence is NOT a defect count. The FMU reproduces the settings.json
car; Chaos reproduces three arcade assists, a snap-to-ground wheel model
and a hidden aero model. The Chaos trace is a reference trajectory, not
a target. What this answers is narrower and more useful: are the two in
the same regime, and where do they part company?

    python3 tools/fmu/ab_plant.py [path-to-IFSSIM.log]
"""
import os
import re
import sys

LINE = re.compile(
    r"FSDS Plant shadow: t=(?P<t>[-\d.]+) "
    r"pos_err=(?P<pos>[-\d.]+) m \(worst (?P<pworst>[-\d.]+), mean (?P<pmean>[-\d.]+)\) "
    r"yaw_err=(?P<yaw>[-\d.]+) deg \(worst (?P<yworst>[-\d.]+)\) \| "
    r"chaos v=(?P<vc>[-\d.]+) fmu v=(?P<vf>[-\d.]+)(?: m/s)?"
    r"(?: \| road (?P<rv>\d)/4 valid, z=(?P<rz>[-\d.]+) m)?"
    r"(?: \| synced=(?P<sync>\d) acc_err=(?P<acc>[-\d.]+) m/s2 "
    r"\(ax (?P<ax>[-+\d.]+) ay (?P<ay>[-+\d.]+)\) "
    r"yawrate_err=(?P<yr>[-+\d.]+))?"
)

DEFAULT_LOG = os.path.expanduser(
    "~/Library/Logs/Unreal Engine/IFSSIMEditor/IFSSIM.log")


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
    except OSError as ex:
        print(f"cannot read {path}: {ex}")
        return 1

    rows = [m.groupdict() for m in LINE.finditer(text)]
    if not rows:
        print(f"no shadow-plant lines in {path}")
        print("is Plant.Type set to 'shadow' in settings.json, and has PIE run?")
        return 1

    # A run restarts when sim time goes backwards; only the last one is current.
    starts = [i for i in range(1, len(rows))
              if float(rows[i]["t"]) < float(rows[i - 1]["t"])]
    if starts:
        rows = rows[starts[-1]:]

    # A mission reset teleports the reference car; the FMU cannot follow, so a
    # step jump appears that has nothing to do with the plant. The pawn
    # re-latches on it, but older logs predate that, so drop everything before
    # the last jump here too rather than reporting the teleport as divergence.
    all_rows = None
    # 5.0 was far too tight and misread ordinary divergence as teleports: two
    # cars on diverging trajectories at ~3 m/s each separate several metres per
    # second quite legitimately, and a 1 s sample period turns that into a
    # multi-metre step. It reported 20 "teleports" in a run containing one real
    # reset. The bound that means something is kinematic — neither car exceeds
    # v_max, so they cannot separate faster than 2*v_max, and 25 m in one
    # sample is beyond anything the pair can do by driving.
    #
    # This is now only a backstop for logs predating the explicit reset call.
    # The authority is the pawn, which re-latches on a teleport at TICK
    # resolution (1 m in 1/60 s) and logs it — a threshold ordinary divergence
    # cannot reach, because it would mean 60 m/s of separation.
    jumps = [i for i in range(1, len(rows))
             if float(rows[i]["pos"]) - float(rows[i - 1]["pos"]) > 25.0]
    if jumps:
        # Keep the LONGEST clean stretch, not the last one. Taking the last
        # leaves whatever tail follows the final reset, which is usually the
        # few seconds after the car has already stopped — the lap itself gets
        # thrown away and the summary describes nothing.
        bounds = [0] + jumps + [len(rows)]
        segs = [(bounds[i + 1] - bounds[i], bounds[i], bounds[i + 1])
                for i in range(len(bounds) - 1)]
        n, lo, hi = max(segs)
        print(f"note: {len(jumps)} reference teleport(s) detected — resets the "
              f"FMU cannot follow, not plant divergence.")
        print(f"      POSITION is reported over the longest clean stretch "
              f"({n} of {len(rows)} samples).")
        print(f"      SPEED is reported over the WHOLE run: a teleport moves "
              f"the car, not its speed, so speed needs no such surgery — and "
              f"it is the reading this experiment can actually support.")
        all_rows = rows
        rows = rows[lo:hi]

    t = [float(r["t"]) for r in rows]
    pos = [float(r["pos"]) for r in rows]
    yaw = [float(r["yaw"]) for r in rows]
    speed_rows = all_rows if all_rows is not None else rows
    vc = [float(r["vc"]) for r in speed_rows]
    vf = [float(r["vf"]) for r in speed_rows]
    dv = [f - c for f, c in zip(vf, vc)]

    def rms(xs):
        return (sum(x * x for x in xs) / len(xs)) ** 0.5

    moved = [i for i, v in enumerate(vc) if abs(v) > 0.5]
    print(f"samples        : {len(rows)}   span {t[0]:.0f}–{t[-1]:.0f} s")
    if not moved:
        # "Both stationary" is NOT "nothing to report". Two parked cars should
        # agree exactly, so any growth here is the plant moving when it should
        # be still — and a constant rate is a velocity offset, which a lap will
        # integrate into real position error.
        print()
        print("reference car never exceeded 0.5 m/s — comparing AT REST")
        span = t[-1] - t[0]
        drift = pos[-1] - pos[0]
        print(f"  divergence   : {pos[0]:.4f} -> {pos[-1]:.4f} m over {span:.0f} s")
        if span > 5.0:
            rate = drift / span
            print(f"  rate         : {rate*1000:+.2f} mm/s")
            if abs(rate) < 1e-4:
                print("  Two parked cars agreeing, as they should.")
            else:
                print("  CREEP. A parked car should not move. A constant rate is")
                print("  a velocity offset, not a settling transient — over a")
                print(f"  174 s lap it integrates to ~{abs(rate)*174:.2f} m before")
                print("  the car has turned a wheel. Likely a missing static")
                print("  friction / rolling-resistance term: with no force at")
                print("  zero slip, any residual imbalance integrates freely.")
        return 0
    print(f"moving         : {len(moved)} of {len(vc)} samples above 0.5 m/s")
    print()
    print("displacement divergence (each plant measured from its OWN origin)")
    print(f"  final        : {pos[-1]:.3f} m")
    print(f"  worst        : {max(pos):.3f} m")
    print(f"  RMS          : {rms(pos):.3f} m")
    print()
    print("heading divergence (change-from-start, so spawn pose cancels)")
    print(f"  final        : {yaw[-1]:.2f} deg")
    print(f"  worst        : {max(yaw):.2f} deg")
    print()
    print("forward speed, where BOTH are moving")
    mv = [i for i in moved]
    print(f"  chaos        : mean {sum(vc[i] for i in mv)/len(mv):.2f} m/s   "
          f"max {max(vc):.2f}")
    print(f"  fmu          : mean {sum(vf[i] for i in mv)/len(mv):.2f} m/s   "
          f"max {max(vf):.2f}")
    print(f"  fmu - chaos  : mean {sum(dv[i] for i in mv)/len(mv):+.2f} m/s   "
          f"worst {max((abs(dv[i]), dv[i]) for i in mv)[1]:+.2f}")
    print()

    # Growth rate separates "offset" from "unstable", which matters far more
    # than the absolute number: a constant lag is a modelling difference, a
    # compounding one is a plant that will not survive being made
    # authoritative.
    span = t[-1] - t[0]
    if span > 5.0:
        rate = (pos[-1] - pos[0]) / span
        print(f"drift rate     : {rate:+.3f} m/s over the clean stretch")
    # The synced comparison, when present, is the one that answers the question.
    synced = [r for r in rows if r.get("sync") == "1" and r.get("acc") is not None]
    if synced:
        acc = [float(r["acc"]) for r in synced]
        ax = [abs(float(r["ax"])) for r in synced]
        ay = [abs(float(r["ay"])) for r in synced]
        yr = [abs(float(r["yr"])) for r in synced]
        mv = [i for i, r in enumerate(synced) if abs(float(r["vc"])) > 0.5]
        print()
        print("SAME-STATE RESPONSE  (state forced equal each step — this is the")
        print("one that measures the PLANT rather than the experiment)")
        sub = mv if mv else range(len(acc))
        print(f"  samples      : {len(list(sub))} while moving, of {len(synced)} synced")
        print(f"  |accel diff| : mean {sum(acc[i] for i in sub)/len(list(sub)):.3f}  "
              f"max {max(acc[i] for i in sub):.3f} m/s2")
        print(f"    longitudinal: mean {sum(ax[i] for i in sub)/len(list(sub)):.3f} m/s2")
        print(f"    lateral     : mean {sum(ay[i] for i in sub)/len(list(sub)):.3f} m/s2")
        print(f"  |yaw rate diff|: mean {sum(yr[i] for i in sub)/len(list(sub)):.2f}  "
              f"max {max(yr[i] for i in sub):.2f} deg/s")
        print()
        print("  Position and heading divergence above are ~meaningless while")
        print("  synced: the states are forced equal, so they measure the one")
        print("  step between syncs, not accumulated drift.")
        return 0

    print()
    print("READ THE POSITION NUMBERS WITH CARE. The shadow is OPEN LOOP:")
    print("the controller measures the REFERENCE car and computes throttle")
    print("and steering for it; the shadow is handed those same commands and")
    print("nothing feeds its own state back. Any difference in resistance or")
    print("grip therefore integrates without correction, so compounding")
    print("trajectory divergence is a property of the EXPERIMENT, not")
    print("evidence that the plant is unstable. This setup cannot measure")
    print("trajectory parity at all — only per-signal response.")
    print()
    print("What IS interpretable here is the speed comparison above: two")
    print("plants given identical throttle. A shadow that settles at a")
    print("different speed has different resistance, and that reading does")
    print("not depend on the trajectories agreeing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
