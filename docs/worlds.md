# Worlds: real places around a track

A **world** is a real place the sim builds around a track: its terrain, the surfaces and lines painted on it, walls and fences, buildings and grandstands. It comes from survey data, so tests can run where an event is actually held. The first world is `montmelo_straight`, the main straight of the Circuit de Barcelona-Catalunya, where Formula Student Spain runs its acceleration event.

Everything is generated from data files at runtime. There is no editor map to open, and a world moves with its track when Mission Control recentres it.

## Using one

A track names its world in its environment sidecar ([environment_sidecar.md](environment_sidecar.md)):

```json
{
  "format": "ifssim-env/1",
  "seed": 0,
  "profile": "world_montmelo_straight",
  "world": { "name": "montmelo_straight", "x": 0.0, "y": 0.0 }
}
```

`x`, `y` is where the world's origin sits in the track CSV's frame. Mission Control shifts it along with the cones when it recentres a track.

The world loads whatever `Environment.Enabled` says in `settings.json`: it is the place itself, not optional dressing. Only the sidecar's props follow that switch.

Load `montmelo_straight_acceleration` to drive the acceleration event on the real straight. Its cones match `acceleration.csv`, with the start line on the painted start line.

## What the sim builds

`AFSDSWorldActor` (`Plugins/FSDSPlugin/.../Environment/FSDSWorldActor.cpp`) builds it before the props and cones are placed, so they stand on the new ground.

| Part | Collision ([collision_matrix.md](collision_matrix.md)) | Notes |
|---|---|---|
| Terrain | `FSDSTerrain` (ground) | 1 m cells inside the detail zone within 60 m of the axis; 4 m spacing along x outside the detail zone and across y beyond 60 m. |
| Asphalt, concrete, grass surfaces | `FSDSTerrain` (ground) | Draped 3 cm above the terrain, the largest step the matrix allows. |
| Light painted bands, lines | none | Paint only. |
| Walls, fence posts | `FSDSProp` | Follow the ground in 2 m steps. The fence mesh has no collision, so the LiDAR sees through it. |
| Buildings, grandstands | `FSDSProp` | Grandstand: stepped rows, back wall, roof slab on rear columns. |

While a world is built, the level's own flat floor is hidden and its collision turned off: any WorldStatic part at least 90 m across and at most 5 m thick. The log line `FSDS World: built ...` names what it hid. Loading a track without a world puts the floor back.

`getEnvironment` reports the world under `world`: status, name, attribution, triangles, build time and the hidden floor. `loadTrack` replies with `"world": "none" | "loaded" | "invalid"`.

## Format (`ifssim-world/1`)

A world is a folder, `Content/tracks/worlds/<name>/`:

| File | What |
|---|---|
| `world.json` | What to build. |
| `terrain.i16` | Ground heights: `nx × ny` little-endian int16, row-major, index `j * nx + i` (`i` along x). |
| `ATTRIBUTION.md` | Data sources and licences. |

The world frame is in metres: `+x` along the place's main axis, `+y` to its left, `+z` up, with the origin on the ground at a named reference point. For Montmeló, `+x` is the race direction and the origin is the painted start line. The frame maps into the track frame by translation only.

| Key | Meaning |
|---|---|
| `format`, `name`, `title` | `"ifssim-world/1"`, the folder name, a display name. |
| `attribution` | Every data source and its licence (non-empty list). |
| `frame` | Provenance: `crs`, `origin_e`, `origin_n`, `origin_z` (the source CRS coordinates of the origin) and `x_bearing_deg`. |
| `terrain` | `file` (`terrain.i16`), `x0`, `y0` (first sample), `cell_m`, `nx`, `ny`, `z_unit_m` (0.01: heights in cm). At most 2,000,000 cells. |
| `detail` | Optional `x_min`, `x_max`: where the terrain is drawn at full resolution. |
| `surfaces` | Boxes of `asphalt`, `asphalt_light`, `concrete` or `grass`: `material`, `x_min`, `x_max`, `y_min`, `y_max`. |
| `lines` | `x0`, `y0`, `x1`, `y1`, `width_m`, `colour` (`white` or `yellow`). |
| `walls` | `material` (`concrete`), `x0`, `y0`, `x1`, `y1`, `height_m`, `thickness_m`, optional `fence_height_m` (above `height_m`). |
| `buildings` | `material` (`concrete` or `glass`), a box, and `top_z`: the roof height in the world frame. |
| `grandstands` | `x_min`, `x_max`; seating rising from (`front_y`, `front_z`) to (`back_y`, `back_z`); roof from (`roof_front_y`, `roof_front_z`) to (`roof_back_y`, `roof_back_z`). |

Coordinates are within ±2000 m, and unknown keys are errors. `tools/worldgen/world.py` and `FSDSWorld::Parse` apply the same rules; `tools/validate_tracks.py` checks every world a sidecar names, in CI.

## Montmeló: data and survey

| Source | Licence | Used for |
|---|---|---|
| ICGC Territorial LiDAR v3r1 (2021-2023), tiles 438601 and 438602 | CC BY 4.0 | Terrain (ground classes 2 and 8, 1 m grid), roof and wall heights, painted lines (intensity) |
| OpenStreetMap | ODbL 1.0 | Straight axis, start line, extents of the pit lane, pit building and main grandstand |

Measured along the straight (EPSG:25831, 1049 m, bearing 212.7°):

- The painted start line is 365.3 m from the north-east end; OSM's start node agrees to 0.1 m.
- The ground falls 2.8 m over the first 225 m, is flat to within 5 cm from 225 m to 525 m (the acceleration zone), then falls 7 m to turn 1.
- The track is about 20 m of asphalt, with about 2% cross-fall towards the pits.
- Pit wall at y = −11.2 m (1.1 m high, fence to 2.1 m). Barrier on the grandstand side at y = +10.5 m (1.0 m, fence to 1.8 m).
- The pit building roof is at 8.25 m. The main grandstand roof edge is at 28.5 m, falling to 25.6 m at the back.

The grandstand seating is under the roof and can't be seen from the air, so its rake is an estimate. Signage is generic on purpose: no circuit or sponsor logos.

## Regenerating

The LiDAR tiles are about 550 MB, so they are not in the repo. Download the two tiles from the ICGC download centre into a cache folder. Then, with `numpy`, `laspy[lazrs]`, `pyproj` and `shapely` installed:

```bash
cd tools
python3 -m worldgen build montmelo_straight --cache ~/Library/Caches/ifssim-geo/icgc --tracks ../Content/tracks
python3 -m worldgen check ../Content/tracks/worlds/montmelo_straight
```

`--tracks` also writes `montmelo_straight_acceleration.csv` and its sidecar. The tests are `tools/tests/test_worldgen.py` and the `FSDS.World` automation tests.
