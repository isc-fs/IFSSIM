# Worlds: real places around a track

A **world** is a real place the sim builds around a track: its terrain, the surfaces and lines painted on it, walls and fences, buildings, grandstands and trees. It comes from survey data, so tests can run where an event is actually held. The first world is `montmelo_straight`, the main straight of the Circuit de Barcelona-Catalunya, where Formula Student Spain runs its acceleration event.

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

The acceleration track always runs here: `acceleration.csv` has a sidecar (`acceleration.env.json`) that loads this world, with the start line on the painted start line. The car characterisation (`tools/characterise`) drives the same cones on flat ground instead (`characterisation.csv`), because its turns need open space beside the start.

## What the sim builds

`AFSDSWorldActor` (`Plugins/FSDSPlugin/.../Environment/FSDSWorldActor.cpp`) builds it before the props and cones are placed, so they stand on the new ground.

| Part | Collision ([collision_matrix.md](collision_matrix.md)) | Notes |
|---|---|---|
| Terrain | `FSDSTerrain` (ground) | 1 m cells inside the detail zone within 60 m of the axis; 4 m spacing along x outside the detail zone and across y beyond 60 m. |
| Asphalt, concrete, grass surfaces | `FSDSTerrain` (ground) | Draped 3 cm above the terrain, the largest step the matrix allows. Asphalt inside the wear map gets a vertex row every `dy_m` across the track, and the map's value in its vertex colour (see below). |
| Light painted bands, lines | none | Paint only: worn road paint, white, yellow, red or green ([world_assets.md](world_assets.md)). A line can taper (`end_width_m`), so a painted wedge is one line. |
| Walls, fence posts | `FSDSProp` | Follow the ground in 2 m steps. The fence mesh has no collision, so the LiDAR sees through it. |
| Buildings, grandstands | `FSDSProp` | Buildings: one box each, turned by `yaw_deg`; plain concrete blocks get a band of windows per 3.5 m storey. A deck (`base_z`) is a slab in the air, for footbridges. The pit building: garage fronts (every third one open), and a canopy (`canopy_m`) at roof level over them. Grandstand: stepped rows, back wall, roof slab if it has one; under the roof, galvanised sheet on a tapered rafter at each column and purlins every 3 m. |
| Roof panels | none | Thin slabs on a building's flat roof or a grandstand's sloping one. Out of every sensor's reach. |
| Trees | none | Instanced: two pine and two broadleaf models, scaled to each tree's height and crown, variant and turn picked from its position. Shadows on and culled past 1.5 km in `showcase`; culled past 300 m without shadows in `sim`. |
| Ground beyond the survey | none | A flat skirt from the terrain's edge out to 40 km, so the ground reaches the horizon instead of ending in sky. |
| Props: seats, rails, roof columns, fence posts, boards, garage fronts, listed props | none | Instanced, with LODs ([world_assets.md](world_assets.md)). |

While a world is built, the level's own flat floor is hidden and its collision turned off: any WorldStatic part at least 90 m across and at most 5 m thick. So is the ring of tyre stacks around it (`spline_tires`), which would otherwise cross the world's track: the car runs on until it brakes, and the world's own barriers bound it. The log line `FSDS World: built ...` names what it hid. Loading a track without a world puts the floor back.

`getEnvironment` reports the world under `world`: status, name, attribution, generated triangles, prop instances, build time and the hidden floor. `loadTrack` replies with `"world": "none" | "loaded" | "invalid"`.

## Format (`ifssim-world/1`)

A world is a folder, `Content/tracks/worlds/<name>/`:

| File | What |
|---|---|
| `world.json` | What to build. |
| `terrain.i16` | Ground heights: `nx × ny` little-endian int16, row-major, index `j * nx + i` (`i` along x). |
| `wear.u8` | Optional asphalt wear: `nx × ny` uint8, same layout. 128 is the asphalt as scanned; the colour is multiplied by value / 128, so rubber is below 128. |
| `ATTRIBUTION.md` | Data sources and licences. |

The world frame is in metres: `+x` along the place's main axis, `+y` to its left, `+z` up, with the origin on the ground at a named reference point. For Montmeló, `+x` is the race direction and the origin is the painted start line. The frame maps into the track frame by translation only.

| Key | Meaning |
|---|---|
| `format`, `name`, `title` | `"ifssim-world/1"`, the folder name, a display name. |
| `attribution` | Every data source and its licence (non-empty list). |
| `frame` | Provenance: `crs`, `origin_e`, `origin_n`, `origin_z` (the source CRS coordinates of the origin) and `x_bearing_deg`. |
| `terrain` | `file` (`terrain.i16`), `x0`, `y0` (first sample), `cell_m`, `nx`, `ny`, `z_unit_m` (0.01: heights in cm). At most 2,000,000 cells. |
| `detail` | Optional `x_min`, `x_max`: where the terrain is drawn at full resolution. |
| `wear` | Optional: `file` (`wear.u8`), `x0`, `y0` (first node), `dx_m`, `dy_m` (node spacing, 0.05 to 10 m), `nx`, `ny`. At most 2,000,000 nodes. Applies to `asphalt` surfaces that lie wholly inside it; outside it the asphalt is as scanned. |
| `surfaces` | Boxes of `asphalt`, `asphalt_light`, `concrete` or `grass`: `material`, `x_min`, `x_max`, `y_min`, `y_max`. |
| `lines` | `x0`, `y0`, `x1`, `y1`, `width_m` (0.02 to 10 m), `colour` (`white`, `yellow`, `red` or `green`), optional `end_width_m`: the width at (`x1`, `y1`), tapering from `width_m`. |
| `walls` | `material` (`concrete`), `x0`, `y0`, `x1`, `y1`, `height_m`, `thickness_m`, optional `fence_height_m` (above `height_m`). |
| `buildings` | `material` (`concrete` or `glass`), a box, and `top_z`: the roof height in the world frame. Optional: `yaw_deg` (the box turned anticlockwise about its own centre), `base_z` (below `top_z`: the box floats, as a deck or footbridge), `garages` (`true`: pit-garage fronts on its track-side face; only on unturned buildings), `canopy_m` (0.5 to 10 m; needs `garages`: a canopy that deep over the garage fronts, at roof level) and `roof_panels` (on an unturned building without `base_z`, all on one side of the axis: see below). |
| `grandstands` | `x_min`, `x_max`; seating rising from (`front_y`, `front_z`) to (`back_y`, `back_z`); optional roof from (`roof_front_y`, `roof_front_z`) to (`roof_back_y`, `roof_back_z`), all four or none; optional `roof_panels` on the roof. |
| `roof_panels` | 1 to 500 boxes (`x_min`, `x_max`, `y_min`, `y_max`) inside the roof: within its x range and, in distance from the axis, between its front and back edges. A panel lies on the roof, following its slope. |
| `trees` | Optional: `x`, `y`, `h` (height, 1 to 60 m), `r` (crown radius, 0.3 to 20 m), `kind` (`pine` or `broadleaf`). At most 20,000. |
| `props` | `mesh` (`StartGantry`, `MarshalPost` or `SignBoard`), `x`, `y`, optional `yaw_deg` (anticlockwise from +x). Placed on the ground; see [world_assets.md](world_assets.md). |

Coordinates are within ±2000 m, and unknown keys are errors. `tools/worldgen/world.py` and `FSDSWorld::Parse` apply the same rules; `tools/validate_tracks.py` checks every world a sidecar names, in CI.

## Montmeló: data and survey

| Source | Licence | Used for |
|---|---|---|
| ICGC Territorial LiDAR v3r1 (2021-2023), tiles 438601 and 438602 | CC BY 4.0 | Terrain (ground classes 2 and 8, 1 m grid), roof and wall heights, painted lines (intensity), the other buildings, open grandstands and trees (classes 6 and 3 to 5) |
| ICGC territorial orthophoto (current colour, 25 cm/pixel) | CC BY 4.0 | Track width, every painted line and area, the roof panels, and the asphalt's brightness across the track (the wear map). Measured by hand once; the generator holds the numbers, not the photo. |
| OpenStreetMap | ODbL 1.0 | Straight axis, start line, extents of the pit lane, pit building and main grandstand |

Measured along the straight (EPSG:25831, 1049 m, bearing 212.7°):

- The painted start line is 365.3 m from the north-east end; OSM's start node agrees to 0.1 m.
- The ground falls 2.8 m over the first 225 m, is flat to within 5 cm from 225 m to 525 m (the acceleration zone), then falls 7 m to turn 1.
- The track is 13.35 m of asphalt (y = −7.3 to 6.05 m), white edge lines at −7.0 and 5.69 m, with about 2% cross-fall towards the pits. The 3 to 4 m beyond each edge line is dry grass, not asphalt.
- Painted across the track: the start line (x = 0), the finish line (x = −127.0), and two more full-width lines at −197.7 and 32.4 m. The grid's 28 slots (white brackets, each with a yellow mark to the middle) run back to x = −208 in two lanes, 7.38 m staggered.
- Pit lane: a white line along the wall at y = −13.06, a red band with white borders at y = −17.5, and green on the verge at two places. The pit wall ends at x = 224 m; the exit lane runs between a green-painted wedge and a white line, and joins the track at x = 316 m.
- Roofs: six rows of dark panels on each block of the pit roof; two bands of solar panels on the main grandstand roof, in 20 m sections from x = −129 to 94 m.
- The asphalt is darker left of centre, where the rubber is: 0.86 of its mean brightness at y = 2.75, 1.16 at the unused right edge (linear light, from the photo's brightness to the power 2.2). Along the track the photo varies by under ±1 % per 10 m, and it shows no repair patches or seams, so the wear map adds only ±2 % of gentle noise.
- Pit wall at y = −11.2 m (1.1 m high, fence to 2.8 m). Barrier on the grandstand side at y = +10.5 m (1.0 m, fence to 2.1 m). The walls' heights are from the LiDAR; the fences' are from a ground-level photo at the start line, scaled by the walls, because the LiDAR only caught the lower part of the mesh. Their leaning tops aren't modelled.
- The pit building roof is at 8.25 m. The main grandstand roof edge is at 28.5 m, falling to 25.6 m at the back.

Everything else around the straight comes from the LiDAR classes (`tools/worldgen/lidar_features.py`, numpy only):

- **Buildings** (class 6, more than 2 m above the ground): the points are rasterised on a 1 m grid and split into connected blobs. A blob that fills at least 60% of its smallest turned rectangle becomes that box; any other is covered by boxes of at least 15 cells, none thinner than 2 m. The roof is the 90th percentile of the heights under each box. A box at most 6 m wide, at least 4 m above the ground and with ground seen under more than half of it is a deck: a 1.2 m slab, so footbridges don't turn into walls.
- **Open grandstands** (no roof) along the far side of the straight: runs of class-6 points along x at least 40 m long; the seating's front and back edge are the 2nd and 98th percentile of |y|, its rise a line fitted to the median height per metre of depth. There are four, from x = 276 m to 707 m.
- **Trees** (classes 3 to 5): one per peak of the canopy height, a peak being the highest point within 2 to 4 m depending on the tree's height. The crown radius is the median reach, over eight directions, until the canopy falls below half the height, capped at half the height plus 1.5 m so a hedge doesn't become one huge flat tree. A tall, narrow crown with its base high on the trunk is a pine; the rest are broadleaf.

The pit building, the main grandstand and the open grandstands are modelled; LiDAR buildings and trees inside them are dropped. The current build has 44 buildings (7 of them decks), 5 grandstands and 775 trees.

The aerial photo shows roofs a few metres off their footprint (they stand 8 to 28 m up), so roof positions and heights come from the LiDAR, and only the patterns on them from the photo. The grandstand seating is under the roof and can't be seen from the air, so its rake is an estimate; so is the structure under the roof (rafters and purlins), and which garages stand open. Signage is generic on purpose: no circuit or sponsor logos. The start gantry's shape and sizes are measured on a ground-level photo (about ±10 %, scaled by the walls); its position on the start line is assumed. The two marshal posts are generic models at assumed positions.

## Regenerating

The LiDAR tiles are about 550 MB, so they are not in the repo. Download the two tiles from the ICGC download centre into a cache folder. Then, with `numpy`, `laspy[lazrs]`, `pyproj` and `shapely` installed:

```bash
cd tools
python3 -m worldgen build montmelo_straight --cache ~/Library/Caches/ifssim-geo/icgc --tracks ../Content/tracks
python3 -m worldgen check ../Content/tracks/worlds/montmelo_straight
```

It writes `world.json`, `terrain.i16` and `wear.u8`. `--tracks` also writes `acceleration.csv` and its sidecar. The tests are `tools/tests/test_worldgen.py` and the `FSDS.World` automation tests.
