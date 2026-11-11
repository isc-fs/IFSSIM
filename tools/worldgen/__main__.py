"""python3 -m worldgen {build,check} ... (see worldgen/__init__.py)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import world as W

TRACKS = Path(__file__).resolve().parents[2] / "Content" / "tracks"
BUILDERS = ("montmelo_straight",)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="worldgen")
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="measure a world from its sources and write it")
    b.add_argument("name", choices=BUILDERS)
    b.add_argument("--cache", type=Path, required=True, help="folder holding the downloaded LiDAR tiles")
    b.add_argument("--tracks", type=Path, default=TRACKS, help="Content/tracks to write into")
    c = sub.add_parser("check", help="check a world directory the way the plugin reads it")
    c.add_argument("world_dir", type=Path)
    a = ap.parse_args(argv)

    if a.cmd == "check":
        try:
            world, heights = W.load(a.world_dir)
        except (W.WorldError, OSError, ValueError) as e:
            print(f"{a.world_dir}: {e}", file=sys.stderr)
            return 1
        print(f"{world['name']}: ok, {world['terrain']['nx']}x{world['terrain']['ny']} terrain, "
              f"z {min(heights) * world['terrain']['z_unit_m']:.2f}..{max(heights) * world['terrain']['z_unit_m']:.2f} m")
        return 0

    from . import montmelo
    out = a.tracks / "worlds" / a.name
    world = montmelo.build(a.cache, out)
    csv, sidecar = montmelo.acceleration_track()
    # The acceleration event always runs on this world: acceleration.csv and
    # its sidecar, which loads the world.
    (a.tracks / "acceleration.csv").write_text(csv)
    (a.tracks / "acceleration.env.json").write_text(json.dumps(sidecar, indent=2) + "\n")
    print(f"wrote {out} (origin z {world['frame']['origin_z']} m, roof "
          f"{world['grandstands'][0]['roof_front_z']}/{world['grandstands'][0]['roof_back_z']} m, "
          f"pit building {world['buildings'][0]['top_z']} m) and acceleration.csv with its sidecar")
    return 0


if __name__ == "__main__":
    sys.exit(main())
