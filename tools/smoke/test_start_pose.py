#!/usr/bin/env python3
"""
Start pose on flat and sloped ground, against a running sim (RPC only; no
Mission Control, no Docker):

  1. Every Content/tracks/*.csv: loadTrack, then getStartGatePose. With
     --baseline, each pose must match the baseline within 1 mm and 0.01 deg:
     on the flat floor the ground-following start pose must change nothing.
     --write-baseline records the poses instead.
  2. Ramp: validateRoadProbe builds the 8 deg test terrain, then
     tools/smoke/fixtures/ramp_gate.csv puts the start gate where the car's
     3 m back-off lands on the ramp's centre (x = 39 m, surface 0.702 m).
     The car must sit 0.55 m above the surface (HeightOffset 5 cm + 50 cm)
     within 1 cm, nose up 8 deg within 0.2 deg, heading along +x.

The flat tracks run first: the test terrain persists once built and would
otherwise sit under some of them.

    python tools/smoke/test_start_pose.py --write-baseline before.json   # old build
    python tools/smoke/test_start_pose.py --baseline before.json         # new build
"""
import argparse
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))
from ifssim.client import IFSSIMClient  # noqa: E402

RAMP_FIXTURE = REPO / "tools" / "smoke" / "fixtures" / "ramp_gate.csv"
RAMP_SURFACE_Z_M = 0.702   # FSDSTestTerrain "ramp_up" height at its centre (x = 39 m)
RAMP_PITCH_DEG = 8.0
# The fixture drives along its CSV +x, which is UE +X and therefore ENU north:
# getStartGatePose reports ENU, so that heading is 90 deg, not 0.
RAMP_HEADING_DEG = 90.0
START_CLEARANCE_M = 0.55   # AFSDSConeSpawner HeightOffset (5 cm) + 50 cm


def rpc(client, cmd):
    resp = client._text_cmd(cmd)
    try:
        return json.loads(resp)
    except (TypeError, ValueError):
        return {"raw": resp}


def forward_angles(p):
    """Heading and elevation (deg) of the body x-axis, from an ENU pose."""
    w, x, y, z = p["qw"], p["qx"], p["qy"], p["qz"]
    fx = 1 - 2 * (y * y + z * z)
    fy = 2 * (x * y + w * z)
    fz = 2 * (x * z - w * y)
    return math.degrees(math.atan2(fy, fx)), math.degrees(math.asin(max(-1.0, min(1.0, fz))))


def start_pose(client, track):
    loaded = rpc(client, f"loadTrack {track.as_posix()}")
    if "error" in loaded or "raw" in loaded:
        raise RuntimeError(f"loadTrack {track.name}: {loaded}")
    pose = rpc(client, "getStartGatePose")
    if "error" in pose or "raw" in pose:
        raise RuntimeError(f"getStartGatePose after {track.name}: {pose}")
    heading, elevation = forward_angles(pose)
    return {**pose, "heading_deg": heading, "elevation_deg": elevation, "cones": loaded.get("cones")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default="127.0.0.1:41451")
    ap.add_argument("--baseline", help="compare flat-track poses against this JSON")
    ap.add_argument("--write-baseline", help="write flat-track poses to this JSON")
    ap.add_argument("--skip-ramp", action="store_true")
    args = ap.parse_args()

    host, port = args.sim.rsplit(":", 1)
    client = IFSSIMClient(ip=host, port=int(port))
    if not client.ping():
        print(f"FAIL: no sim at {args.sim}")
        sys.exit(1)

    failures = []
    poses = {}
    for track in sorted((REPO / "Content" / "tracks").glob("*.csv")):
        poses[track.name] = start_pose(client, track)
        p = poses[track.name]
        print(f"{track.name:32s} xyz=({p['x']:8.3f},{p['y']:8.3f},{p['z']:6.3f}) "
              f"heading={p['heading_deg']:7.2f} elev={p['elevation_deg']:5.2f} cones={p['cones']}")

    if args.write_baseline:
        Path(args.write_baseline).write_text(json.dumps(poses, indent=2))
        print(f"baseline written: {args.write_baseline}")
    if args.baseline:
        base = json.loads(Path(args.baseline).read_text())
        for name, p in poses.items():
            b = base.get(name)
            if b is None:
                failures.append(f"{name}: not in baseline")
                continue
            d_mm = 1000 * math.dist((p["x"], p["y"], p["z"]), (b["x"], b["y"], b["z"]))
            d_head = abs((p["heading_deg"] - b["heading_deg"] + 180) % 360 - 180)
            d_elev = abs(p["elevation_deg"] - b["elevation_deg"])
            ok = d_mm <= 1.0 and d_head <= 0.01 and d_elev <= 0.01
            print(f"  {'ok  ' if ok else 'FAIL'} {name}: {d_mm:.3f} mm, heading {d_head:.4f} deg, "
                  f"elevation {d_elev:.4f} deg")
            if not ok:
                failures.append(f"{name}: moved {d_mm:.3f} mm / {d_head:.4f} deg / {d_elev:.4f} deg")

    if not args.skip_ramp:
        # Only used here to build the test terrain. Its own verdict is not this
        # test's: on the call that first builds the terrain it probes before the
        # physics scene has registered the new slabs, and reports misses.
        probe = rpc(client, "validateRoadProbe")
        print(f"validateRoadProbe (builds the terrain): ok={probe.get('ok')}")
        p = start_pose(client, RAMP_FIXTURE)
        # Pitch is read from the plant (the pawn's own UE rotator, + = nose up),
        # not from getStartGatePose: FSDSCoord::UEQuatToENU is only correct for
        # yaw and reports a nose-up pose as nose-down (#638). Heading and
        # position go through it correctly.
        plant = rpc(client, "getPlantState")
        pitch_deg = math.degrees(plant["attitude"][1])
        want_z = RAMP_SURFACE_Z_M + START_CLEARANCE_M
        print(f"ramp_gate.csv: xyz=({p['x']:.3f},{p['y']:.3f},{p['z']:.3f}) (want z {want_z:.3f}), "
              f"pitch={pitch_deg:+.2f} (want +{RAMP_PITCH_DEG}), "
              f"heading={p['heading_deg']:.2f} (want {RAMP_HEADING_DEG})")
        if abs(p["z"] - want_z) > 0.01:
            failures.append(f"ramp: z {p['z']:.3f} m, want {want_z:.3f} ± 0.01")
        if abs(pitch_deg - RAMP_PITCH_DEG) > 0.2:
            failures.append(f"ramp: pitch {pitch_deg:+.2f} deg, want +{RAMP_PITCH_DEG} ± 0.2")
        if abs((p["heading_deg"] - RAMP_HEADING_DEG + 180) % 360 - 180) > 0.2:
            failures.append(f"ramp: heading {p['heading_deg']:.2f} deg, want {RAMP_HEADING_DEG} ± 0.2")

    if failures:
        print("\nFAIL:\n  " + "\n  ".join(failures))
        sys.exit(1)
    print("\nPASS")


if __name__ == "__main__":
    main()
