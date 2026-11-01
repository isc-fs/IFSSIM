# envgen: the track creator

Draw a path; get a track the sim can load.

```bash
cd tools
python3 -m envgen create envgen/examples/stress_loop.track.json --check     # report only
python3 -m envgen create envgen/examples/stress_loop.track.json -o ../Content/tracks/stress_loop.csv
python3 -m envgen startpose ../Content/tracks/*.csv                          # where the sim spawns the car
python3 -m pytest envgen/tests -q
```

Mission Control does the same over HTTP: `POST /api/track/preview_spec`
(build, don't save), `POST /api/track/create` (save), `GET
/api/track/{name}/spec` (reopen).

## The spec

A spec is the editable source of a track; the CSV is built from it and never
edited by hand. The format is documented at the top of [`spec.py`](spec.py);
[`examples/stress_loop.track.json`](examples/stress_loop.track.json) uses
every feature.

- `points`: the drawn control points, in metres, in the track CSV's frame
  (+y is left of +x). The car drives from the first point toward the second.
  A smooth curve runs through them. A curve through far-apart points bows,
  so draw a straight as points in a line.
- `closed`: a loop (autocross, trackdrive) or an open track with a finish
  gate at the end.
- `width`, `spacing.straight`, `spacing.corner_min`: the corridor. Corner
  spacing tightens with the radius, to about 20 degrees of arc per cone.
- `sections`: test one stretch of track, from control point `from` to `to`:
  `width`, `spacing`, `drop` (missing cones), `recolor` (wrong colours),
  `jitter` (misplaced cones), on one `side` or both. Seeded by `seed`.

## Stress tracks

Some test shapes are too exact to draw: a circle of exactly 9 m, a corner
that tightens from 20 m to 6 m. [`primitives.py`](primitives.py) builds them
from straights, arcs and spirals into ordinary specs, so they get the same
checks and start gate as a drawn track and reopen in the editor.

```bash
python3 -m envgen stress --list                     # the library and what each tests
python3 -m envgen stress -o ../Content/tracks        # regenerate all of it
python3 -m envgen stress circle_r9                   # check one, write nothing
```

The library is committed as `Content/tracks/stress_*.csv` plus the spec of
each. Never edit those by hand: change the generator and regenerate.
`tests/test_primitives.py` fails if a committed CSV no longer matches.

## What it checks

| Severity | Check |
|---|---|
| error | the inside cone line folds (a corner tighter than half the width) |
| error | the track crosses or overlaps itself |
| error | the sim would spawn the car facing away from the drawn direction |
| warn | FS rules: width < 3 m, cones > 5 m apart, straight > 80 m, hairpin < 9 m outside diameter, lap outside 200–500 m |
| warn | two parts of the track closer than 1.5 m; cones off customMap's floor |

The FS rules are warnings because a test track may break them on purpose. A
missing-cone section breaks the 5 m rule by design. Use `--strict` (or
`"strict": true` over HTTP) to make them errors.

## The start gate

The CSV has no start-pose field. The sim spawns the car 3 m behind the four
big orange cones and, on a loop, picks which way is forward from the four
track cones nearest the gate (`FSDSConeSpawner::ComputeStartGatePose`).
`startpose.py` is a port of that function. The creator lays out the start
zone so that the four nearest cones are always ahead of the gate: two pairs
at 2.0 m and 3.5 m, and nothing within 5 m behind it. Test sections never
touch the start zone. `test_start_pose_points_the_drawn_way_on_random_loops`
holds this over 150 random loops in both directions.
