# Track environment sidecar (`<track>.env.json`)

A track's cones live in `<track>.csv`. Everything around them lives in a sidecar file next to it, `<track>.env.json`. That means the ground the track sits on and the trackside props. This is the plumbing for the perception stress environments in [ENVIRONMENT_ROADMAP.md](ENVIRONMENT_ROADMAP.md).

The roadmap's design rules shape the file:

- **Generated, not drawn (rule 3).** A generator (`tools/envgen`) turns a seed and a profile into the sidecar. The plugin only instantiates what the file says and draws no randomness of its own. A world is reproduced exactly by its file.
- **Opt-in (rule 5).** Nothing is loaded unless `settings.json` has `"Environment": { "Enabled": true }`. The default is `false`, so the flat world stays the regression baseline, and while it is off a sidecar next to a track is ignored.
- **Loaded with the track, before its cones (rule 4).** This happens both for the track the sim boots with and for `loadTrack`.

## Format v1

```json
{
  "format": "ifssim-env/1",
  "seed": 7,
  "profile": "stress",
  "ground": { "extent": { "x_min": -60.0, "y_min": -60.0, "x_max": 160.0, "y_max": 140.0 } },
  "props": [
    { "class": "bollard", "x": 12.0, "y": -3.5, "yaw_deg": 30.0 },
    { "class": "tripod",  "x": 40.0, "y": 2.25 }
  ]
}
```

| Key | Required | Meaning |
|---|---|---|
| `format` | yes | Exactly `"ifssim-env/1"`. |
| `seed` | yes | The generator's seed: a non-negative integer, recorded for provenance. |
| `profile` | yes | The generator's profile name, for example `flat_baseline` or `stress`. |
| `ground.extent` | no | The ground the generator laid out: `x_min`, `y_min`, `x_max`, `y_max` in metres, with min below max. |
| `props` | no | One entry per prop. `class` is a non-empty string; `x` and `y` are in metres; `yaw_deg` is optional (default 0). |

### Prop classes

The plugin builds each class from primitives (engine basic shapes and the small cone mesh), with the `FSDSProp` collision profile: the LiDAR sees them and the car hits them, but they are never ground ([collision_matrix.md](collision_matrix.md)). Each prop stands on the ground under its `x`, `y`, found by the same WorldStatic trace the cones use. A prop of an unknown class, or with no ground under it, is skipped and counted.

| `class` | What | Size |
|---|---|---|
| `bollard` | a post at cone scale: the narrow prop a detector reads as a cone | Ø 0.20 m, 0.75 m tall |
| `bin` | a bin | Ø 0.45 m, 0.65 m tall |
| `tripod` | a timekeeping light barrier on three thin legs | 0.5 m leg radius, 1.15 m tall |
| `tyre_wall` | four stacks of tyres in a row, along `yaw_deg` | 2.4 × 0.6 m, 0.6 m tall |
| `cone_stack` | four spare small cones nested, about as tall as a big orange cone | Ø 0.23 m, 0.50 m tall |

## Generating one

`tools/envgen` writes a sidecar from a seed and a profile:

```bash
cd tools
python3 -m envgen ../Content/tracks/<track>.csv --seed 7 --profile stress
```

- **`flat_baseline`** has no props.
- **`stress`** has bollards, bins, tyre walls and cone stacks spread along the track (per 100 m of cone line: 3, 1.5, 1.5 and 1), plus a light-barrier tripod on each side of up to two timing lines (clusters of big orange cones).

Every prop keeps out of the track area (the cone corridor, or the convex hull of the cones for acceleration and skidpad), within its class's distance band from it, at least 0.5 m from every cone and 0.3 m from every other prop, and on the sim's floor. A prop that finds no valid spot is dropped, so a cramped track gets fewer props, never a misplaced one. The same seed, profile and CSV give the same bytes on every platform; `tools/envgen/tests` pins the hashes.

The environment seed is not the scenario seed: a noise sweep over scenario seeds runs in one identical world.

Mission Control recentres a track on the floor when it loads it (`track_centering.py`), and moves the track's sidecar by the same amount, so props stay where they were generated relative to the cones.

**Strict on purpose.** A sidecar is written by a program, so the loader treats anything unexpected as a bug:
- unknown keys at any level are errors, so a misspelt `yaw` doesn't silently become 0;
- values must have the right JSON type, so `"1"` is not a number;
- coordinates beyond ±1000 m are rejected as a probable cm-for-m mistake.

An invalid sidecar is not loaded at all. The track's cones still load.

## Coordinate frame

Positions use the **track CSV's frame**: the same `x` and `y` as the CSV's columns, in metres. The plugin maps them into UE exactly as it maps cones (`FSDSConeSpawner::SpawnFromCSV`):

| Track CSV | UE | ENU |
|---|---|---|
| `+x` | `+X` | north |
| `+y` | `−Y` | west |

`yaw_deg` turns counter-clockwise from `+x`, seen from above, so a prop with `yaw_deg: 90` faces `+y`. In UE that is a yaw of −90°. `FSDSEnvironment::CsvToUeCm` and `CsvYawToUeDeg` do the conversion.

## How to check one

- **`python tools/validate_tracks.py`** applies the plugin's rules to every `*.env.json` under `Content/tracks/`, and checks that each one sits next to its track CSV. CI runs it.
- **The `loadTrack` reply** carries `"environment"`:
  - `"off"`: Environment is disabled;
  - `"none"`: the track has no sidecar;
  - `"loaded"`;
  - `"invalid"`.
- **`getEnvironment`** returns the current track's environment as authored: `enabled`, `status`, `error` when invalid, and, when loaded, `sidecar`, `seed`, `profile`, `ground_extent`, `props` (a count) and `classes` (a count per class).
  Once built, it also returns `built` and `skipped` (counts), `unknown_classes`, and `transform_sha1`: a hash of the built props' transforms, the same on every boot of the same sidecar whatever the scenario seed.
- **`python tools/smoke/test_environment.py`** checks all of this against a running sim, with the environment off or on.

Packaging copies every `Content/tracks/*.env.json` next to the CSVs.
