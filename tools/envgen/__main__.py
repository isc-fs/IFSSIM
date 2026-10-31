"""python3 -m envgen TRACK.csv --seed N --profile stress [--out FILE | --stdout]"""

from __future__ import annotations

import argparse
import sys
from collections import Counter

from .catalogue import PROFILES
from .generate import dumps, generate, sha256, sidecar_path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m envgen", description=__doc__)
    ap.add_argument("track", help="track CSV")
    ap.add_argument("--seed", type=int, required=True, help="environment seed (not the scenario seed)")
    ap.add_argument("--profile", default="stress", choices=sorted(PROFILES))
    ap.add_argument("--out", help="where to write (default: <track>.env.json next to the CSV)")
    ap.add_argument("--stdout", action="store_true", help="print instead of writing")
    a = ap.parse_args(argv)
    try:
        env = generate(a.track, a.seed, a.profile)
    except (OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    text = dumps(env)
    if a.stdout:
        sys.stdout.write(text)
        return 0
    out = a.out or str(sidecar_path(a.track))
    with open(out, "w") as f:
        f.write(text)
    counts = Counter(p["class"] for p in env["props"])
    print(f"{out}: {len(env['props'])} props "
          f"({', '.join(f'{k} {v}' for k, v in sorted(counts.items())) or 'none'}), "
          f"sha256 {sha256(text)[:12]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
