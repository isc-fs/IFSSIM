# Track environment sidecar (`<track>.env.json`)

A track's cones live in `<track>.csv`. Everything around them lives in a sidecar file next to it, `<track>.env.json`. That means the ground the track sits on and the trackside props. This is the plumbing for the perception stress environments in [ENVIRONMENT_ROADMAP.md](ENVIRONMENT_ROADMAP.md).

The roadmap's design rules shape the file:

- **Generated, not drawn (rule 3).** A generator (`tools/envgen`, coming in P2a) turns a seed and a profile into the sidecar. The plugin only instantiates what the file says and draws no randomness of its own. A world is reproduced exactly by its file.
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

Prop classes are defined by P2a, which adds the builder that spawns them. Until then the plugin loads and reports props but does not spawn anything.

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
- **`python tools/smoke/test_environment.py`** checks all of this against a running sim, with the environment off or on.

Packaging copies every `Content/tracks/*.env.json` next to the CSVs.
