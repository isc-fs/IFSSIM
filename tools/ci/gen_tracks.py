#!/usr/bin/env python3
"""Random autocross tracks for the sim check, one per seed.

    python3 tools/ci/gen_tracks.py --gen <random-track-generator dir> --out <tracks dir> \\
        --seeds 101,102,103 [--prefix ci_rand]

Runs the random-track-generator submodule (tools/random-track-generator), the
generator behind Mission Control's "Generate" button, with the same settings,
and writes <out>/<prefix>_<seed>.csv. The generator draws everything from
numpy's global RNG, so a seed gives the same track every time: a failing track
can be rebuilt from its name alone.

A seed whose track is unusable (the generator failed, too few cones, too
short or long to drive in the check's time, or with a centreline corner
tighter than --min-radius) is replaced by the next seed up, up to 20 tries.
The corner floor (default 5.0 m, env GEN_MIN_RADIUS_M, 0 disables) sits just
above the car's 4.95 m turning circle: a tighter corner cannot be driven by
any controller. Rejection keeps seeds deterministic (the same seed and floor
always give the same file, and the file name carries the seed actually used),
but the seed list now depends on the floor: rebuild a track with the floor it
was made with, or ask for its exact seed (a seed that passed always passes). Prints the written file names, comma-separated, on the last
line of stdout.

Needs numpy, scipy, shapely, gpxpy, PyYAML and matplotlib: the sim check runs
it in the Mission Control backend image, which has them.
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import track_geometry  # noqa: E402

# Mission Control's defaults (tools/mission_control/backend/main.py, TrackGenerate).
N_POINTS, N_REGIONS, MIN_BOUND, MAX_BOUND = 50, 30, 10.0, 150.0
# A track the car can lap at ~3 m/s inside the check's autocross timeout.
MIN_CONES, MIN_LENGTH_M, MAX_LENGTH_M = 60, 150.0, 650.0
TRIES = 20


def read_cones(path: Path) -> list[tuple[str, float, float]]:
    out = []
    for line in path.read_text().splitlines():
        f = line.split(",")
        if len(f) >= 3:
            try:
                out.append((f[0].strip(), float(f[1]), float(f[2])))
            except ValueError:
                continue
    return out


def length_m(cones: list[tuple[str, float, float]]) -> float:
    """The blue edge's length, closed: about the track's length."""
    blue = [(x, y) for kind, x, y in cones if kind == "blue"]
    if len(blue) < 2:
        return 0.0
    return sum(math.dist(a, b) for a, b in zip(blue, blue[1:] + blue[:1]))


_COLOUR = {"blue": track_geometry.BLUE, "yellow": track_geometry.YELLOW}


def problem(cones: list[tuple[str, float, float]], min_radius: float = 0.0) -> str | None:
    if len(cones) < MIN_CONES:
        return f"{len(cones)} cones"
    kinds = {k for k, _, _ in cones}
    if not {"blue", "yellow"} <= kinds:
        return f"cone types {sorted(kinds)}"
    n = length_m(cones)
    if not MIN_LENGTH_M <= n <= MAX_LENGTH_M:
        return f"{n:.0f} m long"
    if min_radius > 0:
        r = track_geometry.min_radius([(x, y, _COLOUR[k]) for k, x, y in cones if k in _COLOUR])
        if r is not None and r < min_radius:
            return f"corner R {r:.1f} m < {min_radius:.1f} m"
    return None


def generate(gen_dir: Path, seed: int) -> Path:
    """One track from the generator, as a CSV in a temp dir."""
    import numpy as np

    sys.path.insert(0, str(gen_dir))
    from track_generator import TrackGenerator
    from utils import Mode, SimType

    tmp = Path(tempfile.mkdtemp())
    # The generator writes to realpath(dirname(its file)) + output_location, so
    # reach the temp dir from there (as Mission Control's endpoint does).
    rel = "/" + os.path.relpath(tmp, os.path.realpath(gen_dir))
    np.random.seed(seed)
    cwd = os.getcwd()
    os.chdir(gen_dir)
    try:
        TrackGenerator(
            n_points=N_POINTS, n_regions=N_REGIONS, min_bound=MIN_BOUND, max_bound=MAX_BOUND,
            mode=Mode.RANDOM, plot_track=False, visualise_voronoi=False,
            create_output_file=True, output_location=rel, sim_type=SimType.FSDS,
        ).create_track()
    finally:
        os.chdir(cwd)
    out = tmp / "random_track.csv"
    if not out.exists():
        raise RuntimeError("the generator wrote no track")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gen", type=Path, required=True, help="the random-track-generator checkout")
    ap.add_argument("--out", type=Path, required=True, help="where the tracks go (Content/tracks)")
    ap.add_argument("--seeds", required=True, help="comma-separated seeds")
    ap.add_argument("--prefix", default="ci_rand")
    ap.add_argument("--min-radius", type=float,
                    default=float(os.environ.get("GEN_MIN_RADIUS_M", track_geometry.DEFAULT_FLOOR_M)),
                    help="reject tracks with a centreline corner tighter than this, m (0 = off; "
                         "env GEN_MIN_RADIUS_M; the car's turning circle is 4.95 m)")
    args = ap.parse_args(argv)

    import matplotlib

    matplotlib.use("Agg")  # the generator imports pyplot; no display here
    args.out.mkdir(parents=True, exist_ok=True)
    names: list[str] = []
    used: set[int] = set()
    for seed in (int(s) for s in args.seeds.split(",") if s.strip()):
        for attempt in range(TRIES):
            s = seed + attempt
            if s in used:
                continue
            used.add(s)
            try:
                tmp = generate(args.gen, s)
            except ImportError:
                raise  # a missing dependency: no seed will work
            except Exception as e:  # the generator raises on some layouts
                print(f"seed {s}: no track ({type(e).__name__}: {e})", file=sys.stderr)
                continue
            why = problem(read_cones(tmp), args.min_radius)
            if why:
                print(f"seed {s}: skipped ({why})", file=sys.stderr)
                shutil.rmtree(tmp.parent, ignore_errors=True)
                continue
            name = f"{args.prefix}_{s}.csv"
            shutil.move(str(tmp), args.out / name)
            shutil.rmtree(tmp.parent, ignore_errors=True)
            cones = read_cones(args.out / name)
            print(f"seed {s}: {name}, {len(cones)} cones, {length_m(cones):.0f} m", file=sys.stderr)
            names.append(name)
            break
        else:
            print(f"seed {seed}: no usable track in {TRIES} tries", file=sys.stderr)
    print(",".join(names))
    return 0 if names else 1


if __name__ == "__main__":
    sys.exit(main())
