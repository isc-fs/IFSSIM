"""Command line for the track creator.

    cd tools
    python3 -m envgen create my.track.json -o ../Content/tracks/my_track.csv
    python3 -m envgen create my.track.json --check          # report only
    python3 -m envgen startpose ../Content/tracks/*.csv     # where the sim spawns

`create` writes the CSV and, next to it, the spec it came from
(`<name>.track.json`), so the track can be reopened in the editor. It refuses
to write a track with errors unless given --force.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

from . import SpecError, create_track, parse_spec
from .startpose import compute_start_pose


def _create(args) -> int:
    try:
        spec = parse_spec(json.loads(Path(args.spec).read_text()))
    except (OSError, json.JSONDecodeError, SpecError) as e:
        print(f"envgen: {args.spec}: {e}", file=sys.stderr)
        return 2
    res = create_track(spec, strict=args.strict)
    summ = res.summary()
    print(f"{args.spec}: {summ['length_m']} m {'loop' if spec.closed else 'open track'}, "
          f"{summ['cones']} cones {summ['by_type']}")
    for i in res.issues:
        where = f" at s={i['s']} m" if "s" in i else ""
        print(f"  [{i['severity']:5}] {i['code']}: {i['message']}{where}")
    if args.check or not args.out:
        return 0 if res.ok else 1
    if not res.ok and not args.force:
        print("envgen: not written: the track has errors (--force to write anyway)", file=sys.stderr)
        return 1
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(res.csv)
    sidecar = out.with_suffix(".track.json")
    sidecar.write_text(json.dumps(spec.to_dict(), indent=2) + "\n")
    print(f"wrote {out} and {sidecar}")
    return 0


def _startpose(args) -> int:
    bad = 0
    for p in args.csv:
        rows = []
        with open(p) as f:
            for r in csv.reader(f):
                if len(r) >= 3 and r[0].strip() in ("blue", "yellow", "big_orange", "small_orange", "orange"):
                    rows.append((r[0].strip(), float(r[1]), float(r[2])))
        pose = compute_start_pose(rows)
        if pose is None:
            print(f"{p}: no start pose (needs 4 big_orange and a track cone)")
            bad += 1
            continue
        print(f"{p}: {pose.method}, {pose.gates} gate(s), forward "
              f"({pose.forward[0]:+.2f}, {pose.forward[1]:+.2f}), spawn "
              f"({pose.spawn[0]:.1f}, {pose.spawn[1]:.1f})")
    return 1 if bad else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="envgen", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("create", help="build a track spec into a cone CSV")
    c.add_argument("spec")
    c.add_argument("-o", "--out", help="CSV to write (omit to only report)")
    c.add_argument("--check", action="store_true", help="report only, write nothing")
    c.add_argument("--strict", action="store_true", help="FS rule breaches are errors")
    c.add_argument("--force", action="store_true", help="write even with errors")
    s = sub.add_parser("startpose", help="where the sim will spawn the car on a CSV")
    s.add_argument("csv", nargs="+")
    args = ap.parse_args(argv)
    return _create(args) if args.cmd == "create" else _startpose(args)


if __name__ == "__main__":
    sys.exit(main())
