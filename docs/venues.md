# Venues: places to drive, with their surroundings

A **venue** is a fixed place, such as a parking lot, that several tracks share. The cones change from track to track; the trees, lamp posts, parked cars, sidewalks and banks around them stay put. The goal is to drive and test the pipeline in places like the ones the team really tests in, with the clutter a LiDAR sees there, and to draw new tracks on them with the track editor.

The word is "venue" because "environment" already means a track's props sidecar ([environment_sidecar.md](environment_sidecar.md)). The design follows [ENVIRONMENT_ROADMAP.md](ENVIRONMENT_ROADMAP.md)'s rules; this document only adds what a venue needs on top.

## How it works

A venue is data, built by the plugin at `loadTrack` like any environment. It is not a separate level: the roadmap rules out level switching (rule 4, and section 6). Nothing in the plugin is specific to venues.

1. **The venue file**, `Content/tracks/venues/<venue>.venue.json`, describes the place once, in its own frame (metres, the track CSV's axes):
   - `lot`: the open asphalt where tracks go, as a polygon;
   - `margin_m`: how far inside the lot the cones keep (default 2 m);
   - `props`: single objects (`class`, `x`, `y`, `yaw_deg`), the sidecar's classes;
   - `rows`: lines of one class from `from` to `to`, `spacing` apart, facing along the line unless `yaw_deg` says otherwise. These are sidewalks, tree lines, lamp posts, and a column of parked cars.
2. **Its tracks** are named `<venue>__<track>.csv`, in `Content/tracks` with every other track. Each one is drawn: its spec, `<venue>__<track>.track.json`, is in the venue's frame.
3. **`python -m envgen venue Content/tracks/venues/<venue>.venue.json`** checks the venue and each of its tracks, and writes each track's `env.json` holding the venue's props:
   - **The venue:** no prop stands on the lot, and no two solid props overlap. Sidewalk slabs may sit under other things.
   - **Each track:** every cone stays `margin_m` inside the lot, and the venue still fits on the sim's floor once moved with the track.

   Every track is recentred on its own cones, by envgen and again by Mission Control on load. So a sidecar holds the venue's props shifted by that track's offset, which `envgen` gets by rebuilding the track's spec. This is why a venue track has to be drawn.
4. **The plugin** loads the sidecar with the track, as it does any environment. Today that only happens with `"Environment": {"Enabled": true}` in settings.json (roadmap rule 5).

## Drawing a track on a venue

In the sim's menu, **Track editor**, pick the venue under **Location**:
- **The backdrop:** the lot's outline and its props are drawn behind the canvas, in the venue's frame, which is the frame you draw in.
- **The check:** every preview is checked against the lot too (`preview_spec` with `venue`), so cones off it are an error in the issues list, like a fold, and Save stays off.
- **The name:** the track is saved as `<venue>__<name>`; type only the name.
- **The sidecar:** Mission Control's backend writes the track's `env.json` as it saves it (`envgen.venue.write_sidecar`), and deletes it with the track. You don't run step 3 by hand.

Opening a venue track picks its venue. On **Drive**, the **Location** dropdown lists the venues: pick one to see only its tracks (on Anywhere the list is grouped by venue), drawn with the lot and props around the cones. The menu reads the venue files itself (`FSDSVenue`), so Drive needs no backend; packaging stages `Content/tracks/venues/` with the tracks.

Rerun step 3 after changing a venue file: every one of its tracks needs its sidecar written again. `tools/envgen/tests/test_venue.py` fails if a committed sidecar is out of date.

## The classes a venue adds

Built from primitives in `FSDSEnvironmentBuilder.cpp`. Each one can be swapped for a real mesh under `/Game/ThirdParty/Props/` ([environment_props.md](environment_props.md)). Names and footprints match `tools/envgen/catalogue.py`.

| `class` | What | Size | Collision |
|---|---|---|---|
| `tree` | a trunk under a round canopy | Ø 3 m canopy, 5.3 m tall | `FSDSProp` |
| `lamp_post` | a street lamp, its arm along `yaw_deg` | 6 m tall, 1.3 m arm | `FSDSProp` |
| `parked_car` | body, cabin and four wheels, facing `yaw_deg` | 4.4 × 1.8 m, 1.5 m tall | `FSDSProp` |
| `bench` | a seat with a back rest, along `yaw_deg` | 1.8 × 0.5 m, 0.9 m tall | `FSDSProp` |
| `sidewalk` | a slab of pavement | 2 × 2 m, 0.15 m tall | `FSDSPropLidarOnly` |
| `berm` | an earth bank along `yaw_deg`: the hill, for now | 10 × 5.9 m, 2.9 m tall | `FSDSProp` |

Why the collisions are what they are:
- **`sidewalk` is LiDAR-only.** It is under the 0.30 m line of rule 2, so a wheel that reached it would climb it. The LiDAR sees a kerb; the car drives through it. It sits outside the lot, so reaching it means the car has already left the cone corridor.
- **`berm` is a box rolled 45° and sunk into the ground,** so the ridge you see is exactly its collision (rule 1). The car hits it and never climbs it. A real hill, a slope the car drives over, is ground, so it waits for the roadmap's terrain (P3).
- **Every solid prop stays off the lot,** so nothing overhangs a corridor (rule 2). The venue check enforces it.

## `parking_a`, the first venue

`Content/tracks/venues/parking_a.venue.json`:
- **The lot:** 80 × 54 m of open asphalt.
- **North and south edges:** sidewalks, lamp posts every 18 m, and a line of trees behind them, with benches and bins.
- **West edge:** a column of 17 parked cars.
- **East edge:** five earth banks, and bollards at the entrance.

That makes 144 props. One track so far: `parking_a__loop`, a 206 m loop with 100 cones.

## What is next

1. ~~Drive it in the menu~~ and ~~draw on it in the track editor~~: done, see above.
2. **Load a venue's environment even with `Environment.Enabled` off**, since the props are the point (decision 1 below).
3. **More venues.** Two or three parking lots of different sizes and layouts, each with room for several tracks.
4. **A real hill** once terrain (roadmap P3) lands: a slope inside or beside the lot that the car drives over.
5. **Real meshes** for the new classes, if the primitives prove too simple for the LiDAR. From CARLA (CC-BY, the roadmap's preferred source) or from our own models.

## Decisions to take with Raúl

1. **Should a venue's environment load whatever `Environment.Enabled` says?**
   - Rule 5 keeps environments opt-in, so the flat world stays the regression baseline.
   - But a venue track is a venue track; without its props it is only cones.
   - Proposal: venue tracks always load their sidecar, and other tracks stay opt-in.
2. **Where do venue tracks live?**
   - Flat in `Content/tracks` with a `<venue>__` prefix (as now) works with packaging, Mission Control, the menu and `loadTrack` unchanged.
   - `Content/tracks/venues/<venue>/` (the roadmap's layout for geodata venues) needs subfolder support in all four.
3. **Recentring.**
   - Mission Control recentres a track on load, and its sidecar with it (#720). For a venue that is harmless: the props move with the cones.
   - The roadmap's "fixed-venue mode" (open question 10) would keep a venue where it was built instead. Worth deciding before venues get large.
