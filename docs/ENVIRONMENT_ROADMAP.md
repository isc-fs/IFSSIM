# IFSSIM perception stress environments roadmap

*Status: draft for team review, 2026-10-01. Applies to `dev`; the FMU/plant work on `dev-manual` is out of scope.*

**Objective:** make the sim environment realistic enough to stress the perception stack before the car goes to a real track. That covers cone detection, colour classification, the big-orange stop logic, and the SLAM data association that consumes them. A perception failure found in the sim costs a re-run; the same failure found on track costs a test day or a DNF.

**Spine:** environment features are perception test instruments, not scenery. Every feature is seeded, can be switched on or off, and is parametric. Each one targets a named perception failure mode and ships together with the metric that shows it. Features are prioritised by how badly they hurt perception, not by how realistic they look. The flat `customMap` stays the default regression baseline throughout. Gates that involve physics are statistical. Comparisons against the real car stay provisional until real raw-LiDAR data exists.

---

## 1. Why environment realism matters for this stack

- **The real car sees much more than the sim does.** IFS-08 has no cameras and no ground-speed sensor, so the LiDAR is the whole perception and odometry input.
  - In the sim, 97.7–99.1% of LiDAR returns are one perfect plane and about 1% are cones.
  - 45 of 116 channels (39%) can never return a point.
  - MaxRange is 30 m, while the real ATX reaches about 200 m.
  - *These figures come from synthetic scans and historical sim recordings. P0 re-measures them on current dev.*
- **Clutter is the motivation, not yet the evidence.**
  - At FSG in July 2026 the real stack produced about 1,490 clusters per scan and accepted 0 cones. The sim produces about 10–60.
  - That run is cited as motivation only, for two reasons:
    - The 1,490 figure came from the full ~174k-point cloud before the range pre-crop (pipeline 74e6b19), compared against a sim capped at 30 m.
    - The 0 accepted cones were caused by `residual_gate_mse=0.002`, plus a PointCloud2 `point_step` decoding bug fixed that morning (e219322).
  - Until a matched-crop comparison says otherwise, the gap is mostly range and tuning. P0 makes that comparison as soon as a real bag with the raw cloud exists.
- **The rules require distractors.** DS 2026 §1.4–1.5 explicitly lists:
  - spare-cone stacks;
  - timekeeping tripods that "could be recognised as cone" (DS 2026);
  - other disciplines' markings;
  - patchy surfaces.

  Synthetic runs of the real `cone_detection` code ranked the worst cases:
  - **Narrow props and low tyre walls.** 0.45–0.75 m narrow props and low tyre walls near the cone line produce 4–5 false big-orange cones per scan. This can trip the stop latch in `control_node._on_orange`, which never releases, and cause a DNF.
  - **Grade changes and undulation.** These produce 3–7× more clusters, and 20–25% of cones are missed.
  - **Kerbs or walls near the cone line.** Within about 0.5 m of the cone line, 16–36% of cones are missed.
- **Orography breaks flat-world code on both sides of the bridge.**
  - Plugin:
    - The cone snap window is ±10 m around Z≈0.
    - The start pose is `Z=HeightOffset+50` and yaw-only (`FSDSConeSpawner.cpp:560/:658`).
    - The referee trigger only fires between -1 and +3 m.
  - Pipeline:
    - one global RANSAC plane;
    - EKF calibration that assumes a level car;
    - flat priors in cone_slam;
    - no grade feed-forward, even though regen braking is only 1.5–2 m/s² (a 5% downhill grade removes about 0.49 m/s² of that).
  - FSS (Montmeló) has about 30 m of elevation change, and its DV events run on the main straight and the T1–T3 sector.

### What hurts perception most, and what does not

Ranked from synthetic runs of the real `cone_detection` code. P0 re-measures each one on current dev.

| Rank | Feature | Effect on perception | Phase |
|---|---|---|---|
| 1 | Narrow 0.45–0.75 m props (timekeeping tripods, bins, bollards) and low tyre walls near the cone line | 4–5 false big-orange cones per scan; can trip the permanent stop latch | P2a |
| 2 | Grade changes and undulation | 3–7× more clusters; 20–25% of cones missed | P3 |
| 3 | Kerbs or walls within about 0.5 m of the cone line | 16–36% of cones missed | P2b |
| 4 | Spare-cone stacks and other disciplines' cones | real cones in the wrong place; colour and association confusion | P2a |
| 5 | Patchy surfaces, crowns, drainage | ground-removal residual | P3 |
| — | Tall structure away from the cone line (buildings, grandstands, trees) | almost no effect on cone detection | not planned |

**Deliberately not chased:**
- **LiDAR range.** `cone_detection` drops every point beyond 25 m before RANSAC and clustering (`input_range_crop_m = 25.0`, `cone_detection.py:78`). The sim's 30 m range already covers everything perception sees, so the range stays at 30 m.
- **Far-field structure.** It matters for LiDAR odometry, not for cone detection. LiDAR odometry is a separate workstream and out of scope here (§6).

### Codebase facts checked for this plan

- **`FSDSConeSpawner.cpp`:**
  - :167-173 traces the `ECC_WorldStatic` *channel* (complex) over ±1000 cm and ignores only the spawner.
  - :445-458 caches the Z value from before the ground snap.
- **`FSDSLidarSensor.cpp`:**
  - `Noise()` is one shared `FRandomStream`, called from multiple threads by the scan's `ParallelFor`.
  - Scans run as an `AsyncTask` on `AnyBackgroundThreadNormalTask` against the live Chaos scene (:236-246).
- **Determinism is unproven.** `verify_determinism.py` verifies only `/imu` noise. Its docstring says the closed-loop determinism claim "was never demonstrated".
- **Unseeded track generation.** `track_generator.py:111` calls `np.random` unseeded, so tracks generated in Mission Control (MC) are not reproducible.
- **The floor is finite.** It is about 200 m × 250 m (`track_centering.py:4`), while MC generates tracks with `max_bound` up to 150 m (`main.py:356`).
- **Cone mass is uniform.** `FSDSReferee.cpp:216` sets 1.0 kg for every cone. DS 2026 Table 1 gives 0.45 kg (small) and 1.05 kg (big orange).
- **Suspension travel is ±3.5 cm** (`FSDSWheelFront.cpp:58-59`, `FSDSWheelRear.cpp:35-36`).
- **Packaging copies only `*.csv`:** `package_windows.sh:114`, `package_windows.ps1:75`, `.github/workflows/package-windows.yml:65` and `package-linux.yml:73`.
- **No CI job builds the UE plugin.** `ci.yml:5-6` refers to a `plugin-cook.yml` that does not exist. Self-hosted runners for Windows, Linux and macOS do exist (`package-*.yml`).
- **`tools/sim_benchmark/tests` is not in CI.**
- **`FSDSPlugin.Build.cs`** has no Landscape, ProceduralMeshComponent or GeometryFramework dependency.
- **Local bags are sim-only.** `bags/` holds no real-car recordings.
- **Real bags before 2026-07-14 have no raw cloud.** Pipeline commit 33cc5e4 (2026-07-14) is when the recorder stopped excluding `/lidar_points`. Real bags recorded before that date, including FSG `race_20260713_093453`, contain no raw cloud.

---

## 2. Design rules (apply to every phase)

1. **Collision is the LiDAR geometry.**
   - The CPU LiDAR traces only simple collision (`ECC_Visibility`, `bTraceComplex=false`), so every LiDAR-relevant asset needs authored simple collision.
   - Ground must be WorldStatic, QueryAndPhysics and BlockAll, with simple collision identical to complex. That means either a Landscape with `CollisionMipLevel == SimpleCollisionMipLevel == 0`, or a mesh with complex-as-simple.
2. **Props never count as ground, and wheels never drive on them.** Every collision class is defined against the five queries in this matrix. `docs/collision_matrix.md` (written in P1) documents it per prop class.

   | Query | Mechanism | Ground (floor, `FSDSGroundPad`, `FSDSTerrain`) | `FSDSProp` (solid) | `FSDSPropLidarOnly` |
   |---|---|---|---|---|
   | CPU LiDAR | `ECC_Visibility` trace, simple collision | Block | Block | Block |
   | Cone ground snap | object-type WorldStatic (after P1) | Hit | not WorldStatic, so no hit | no hit |
   | Road probe | object-type WorldStatic (`FSDSVehiclePawn.cpp:1815-1841`) | Hit | no hit | no hit |
   | Chaos wheel trace | wheel-trace channel / `WheelTraceCollisionResponses` (channel confirmed in P1) | Block | **Ignore** | **Ignore** |
   | Chassis and cone rigid bodies | Vehicle / PhysicsBody | Block | Block | Ignore |

   - Any prop a wheel could reach from the corridor must be either `FSDSPropLidarOnly`, or drivable ground on the terrain contract (WorldStatic, steps ≤ 3 cm). This covers every kerb or strip under 0.30 m.
   - A LiDAR-only kerb is a phantom to the vehicle. That is acceptable, because reaching it means the car has already left the cone corridor.
   - Solid props stop the chassis but are never climbed by a wheel.
   - Nothing may overhang the track corridor.
3. **Randomness lives in Python, not UE.**
   - `tools/envgen` turns a seed and a profile into a `<track>.env.json` sidecar, and the plugin only instantiates it.
   - The cone CSV stays 7-column.
   - The environment seed is independent of `ScenarioSeed`, so N-seed noise sweeps run on an identical world.
4. **Load synchronously at boot or at loadTrack, before cones spawn.** LiDAR is gated until the environment is ready. No OpenLevel, no World Partition streaming, no loading mid-run.
5. **Opt-in only.** With the environment disabled:
   - outputs that do not depend on physics (start poses, snap logs, spawned transforms) are bit-identical to the flat baseline;
   - closed-loop metrics are within the P0 noise band (rule 9).
6. **Every phase has a perf, size and LFS gate,** measured on the Windows dev box with the UE window focused:
   - LiDAR at 10 Hz with 0 skipped scans;
   - scan p95 within the phase budget;
   - at least 58 FPS at the fixed 60 Hz step;
   - cooked zip delta (the v1.0.0 zip is 327.9 MB);
   - LFS delta.

   Overheads are measured A/B over at least 10 alternating runs, never from a single pair.
7. **Pipeline fixes go upstream** to IFS08-DV-PIPELINE as issues with bags, in a separate **non-gating lane**. Sim acceptance means *reproducing* the failure, not fixing it.
8. **Licensing.**
   - Allowed: own, CC0 or CC-BY assets, plus CC BY / ODbL geodata.
   - **CARLA** (`carla-simulator/carla`) is the preferred source for trackside props. Its assets are CC-BY (per the CARLA README) and already Unreal Engine 5 content, so walls, barriers, fences, bins and bollards import without a format conversion. Rule 1 still applies: every imported asset has its simple collision audited with `auditPropCollision` and re-authored where it does not match the visual silhouette. Copy only the meshes in use, never the CARLA content package, and give each one an attribution entry in `THIRD_PARTY_ASSETS.md`. CARLA's traffic cones are not FS cones; keep the existing DS 2026 cone meshes.
   - Not allowed: Fab Standard/Megascans assets, Google 3D Tiles, any runtime network dependency, circuit or FS branding.
   - Kit pieces from `Content/RaceCourse` are used only once their provenance is recorded in `THIRD_PARTY_ASSETS.md`. Until then, props come from CARLA or own primitives.
   - Geodata-derived files live under `Content/tracks/venues/<venue>/` with that directory's own `LICENSE`:
     - ODbL 1.0 for OSM-derived databases;
     - CC BY 4.0 attribution for ICGC and Catastro.
   - The provenance CI check covers every `.uasset` under `Content/Environment` and every file under `Content/tracks/venues`.
9. **Closed-loop gates are statistical.** Scans race the physics scene and the pipeline runs over DDS in Docker, so closed-loop runs are not assumed to repeat exactly from a seed.
   - Byte-identity is claimed only for outputs that do not depend on physics (envgen files, spawned transforms, terrain, start poses) and for sensor-side scan hashes of a stationary car with sleeping cones.
   - Every closed-loop gate runs N ≥ 5 times per seed and is judged against the P0 noise band (σ_P0 = the run-to-run standard deviation at a fixed seed):
     - **Within band** means |Δmean| ≤ 2σ_P0.
     - **Reproduces** means the effect's rate or mean over N runs exceeds that of the same seeds with the feature disabled by more than 2σ_P0.
10. **Real-band gates are provisional and marked [RB].** A gate that compares against real-car scan statistics is evaluated only once P0 has a real band from bags that contain raw `/lidar_points`. Until then it is recorded as "skipped, no real band" and does not block the phase.

---

## 3. Phases

Effort is given as base effort and as effort with 1.5–2× contingency. Calendar time is in §4.

### P0: Measure first: perception baseline and instruments (base 3.5 pw; 5–7 pw with contingency)

**Goal:** build the perception instruments every later gate relies on, and baseline perception on the flat world so each stressor's effect can be measured against it.

**Deliverables**
- **Real-data inventory (week 1).**
  - List every real IFS-08 bag recorded after pipeline 33cc5e4 (2026-07-14). For each: storage location, venue, duration, and whether `/lidar_points` carries `ring` and per-point `timestamp` fields.
  - Bags from before that date have no raw cloud, and `race_20260713_093453` is dropped from P0.
  - Output: `docs/history/real_lidar_bag_inventory.md`.
- **D1 data-collection request.** This is car-team work outside this repo, filed as an issue. If the inventory finds fewer than 3 usable segments, record at the team's test site:
  - `/lidar_points` with ring and per-point time fields (confirm the Hesai driver config emits them), plus `/imu`, wheel RPM and steering;
  - at least 3 × 60 s segments: static, straight at about 10 m/s, and a cone slalom;
  - laser-rangefinder or tape measurements of cone-lane-to-structure distances, with photos.

  Repeat at every 2027 DV event.
- **PR #1:** extend the existing `tools/sim_benchmark/perception_metrics.py`. It already matches detections to ground-truth cones by position (TP/FP/FN, match error, error by range). It does not yet score colour, the big-orange stop logic or clutter. See §5.
- **Fresh flat-world reference bag `flat_ref`** on current dev.
  - Recording setup:
    - CPU path at 300k pts/s;
    - the full `/lidar/Lidar1` topic, not only `/viz`;
    - today's IMU;
    - header stamps rather than `/clock` (#611);
    - recorded with `tools/sim_benchmark/capture_benchmark_bag.py` / `tools/record_bag.sh`.
  - Every P2/P3 perception gate is measured against this bag.
- **LiDAR instrumentation in `FSDSLidarSensor.cpp`:**
  - scan wall time, hits per scan, a skipped-scan counter (on the `bScanInProgress.exchange` reject at :209) and UDP bytes;
  - `TRACE_CPUPROFILER_EVENT_SCOPE` on `PerformScan`;
  - a cvar to switch the instrumentation off for the A/B overhead test;
  - a read-only `getLidarStats` RPC in `FSDSRpcServer.cpp`. It also returns a SHA-256 for each of the last 100 scans, computed inside the plugin before UDP serialisation.
- **Deterministic per-ray noise and dropout.**
  - Replace the shared `Noise()` stream with a counter-based hash of (seed, scan index, channel, h-step).
  - Extend `tools/scenario_runner/verify_determinism.py` with the stationary-car scan-hash check.
- **Run-to-run noise band.**
  - Matrix: {acceleration, skidpad, one trackdrive} × 3 seeds × N = 5 runs at a fixed `ScenarioSeed`. That is 45 runs, about 4–6 h with `scenario_runner`.
  - Metrics: lap time, DOOs, cone TP/FP/FN, stop-latch events, scan p50/p95, FPS.
  - Output, committed as `tools/scenario_runner/baselines/noise_band_v1.json`:
    - per-metric mean and σ;
    - zip MB;
    - LFS MB for a fresh clone.

  This replaces the earlier single-run "flat baseline".
- **Flat-world perception baseline**, written up in `docs/history/perception_baseline_flat.md`:
  - the PR #1 metrics on `flat_ref` for acceleration, skidpad and one trackdrive: recall, precision, colour accuracy, false big-orange per scan, and stop-latch events (expected 0);
  - the same metrics on a real raw-cloud bag, only if the inventory finds one. Otherwise this is deferred to D1 and recorded as "real reference pending".
- **[RB] real-car realism band**, from at least 3 real raw-cloud segments **at a matched 25 m crop**:
  - clusters per scan;
  - ground fraction;
  - non-ground, non-cone points per scan;
  - distance to the nearest vertical structure.
- **`docs/venues/fs_structure_distances.md`:** a histogram of cone-lane-to-structure distances from OSM/DEM for the FSS T1–T3 sector and the FSG Fahrerlager lots, within the 25 m perception crop. It sets the provisional prop densities until the real band exists.
- **Data requests filed as issues:** the FSS 2026 DV map, any Montmeló photos or bags, and the GitHub plan / LFS quota.

**Acceptance**
- **Tool:**
  - unit tests reproduce known TP/FP/FN, colour-confusion and stop-latch outcomes on synthetic frames;
  - on `flat_ref` it reports a ground fraction ≥ 0.97 and 0 stop-latch events.
- **Empty-world LiDAR:** 0 skipped scans in 5 min. Instrumentation overhead is measured A/B over at least 10 alternating 2-min runs, and the upper bound of the 95% CI must be below 2%.
- **Stationary scans:** stationary car, cones asleep after the 2 s settle, same seed. The sensor-side hashes of the first 100 scans are identical across 3 boots. The noise distribution is unchanged (KS-test p > 0.05; dropout 1.0 ± 0.1%).
- **Noise band:** published and cited by every later closed-loop tolerance. If lap-time σ exceeds 2% of the mean, file a determinism issue before P1's flat no-op gate relies on the band.
- **Baseline:** the flat-world perception baseline is published. The real-bag reference is recorded, or marked pending.
- **Priorities:** the record confirms or re-orders the §1 stressor ranking on current dev, and states the real-band status.

**Depends on:** none. D1 depends on car availability.

**Risks**
- No raw-cloud real bag may exist until the 2027 test season, so [RB] gates may stay skipped for months. This is accepted and made visible, not hidden.
- The per-ray RNG change invalidates noise-sample comparisons with old bags. Re-record baselines once and note it in CHANGELOG.
- Real bags may have frame or handedness surprises. Decode with `point_step` and the field offsets; never assume packing (cf. e219322).
- Perf numbers are noisy on a shared box. Pin the measurement procedure and use A/B runs.

### P1: CI build gate, ground contract and environment plumbing (code only; flat default unchanged) (base 4 pw; 6–8 pw)

**Goal:** restore a plugin build gate, remove the flat-world hardcodes, and add the seams the later phases need, with no change in default behaviour.

**Deliverables**
- **Prerequisite, PR #0:** a UE plugin build gate on the self-hosted Windows runner (§5). C++ PRs in this roadmap merge only once it is green.
- **PR #2 (parallel C++ lane):** a terrain-aware start pose. Z comes from a ground trace at the spawn XY, and pitch/roll from a 4-ray plane fit. Cone positions are cached after the snap. See §5.
- **Cone snap (`FSDSConeSpawner.cpp:149-194`):**
  - object-type WorldStatic query, ignoring pawns and cones;
  - window `GroundSearchHalfHeightCm` (default ±200 m);
  - optional `bAlignConesToGround` (off by default).
  - #483 (spline-spawned cones) is dropped from this phase. It concerns Blueprints in never-cooked `Content/RaceCourse` content and stays a roughly 1-hour Blueprint-only follow-up. CSV cones on `customMap` already go through `SpawnStaticMeshCone`'s snap.
- **`Config/DefaultEngine.ini`:**
  - new `FSDSProp` and `FSDSPropLidarOnly` object channels and profiles, following the §2 matrix;
  - a documented `FSDSTerrain` profile;
  - confirm which channel the Chaos wheel trace uses (`WheelTraceCollisionResponses`), and set both prop channels to Ignore for it;
  - `docs/collision_matrix.md`.
- **Referee (`FSDSReferee.cpp`):** the finish trigger Z is relative to the gate's ground (:298-299), and oranges are no longer forced to Z=0 (:236).
- **`ResetPlants`:** the probe window becomes relative to the corrected pose (`FSDSVehiclePawn.cpp:1194-1242`).
- **Physmat guard:** an explicit `PM_Asphalt` with friction 0.7 on the floor material, and a startup error if ground friction differs from the value `GroundMuCompensation` assumes (`FSDSPacejkaTireModel.cpp:54-56`).
- **Environment config:**
  - `settings.json` gets `Environment {Enabled:false, Seed:1, Profile:"flat"}`, parsed in `FSDSSettings.h`;
  - a new `Env.*` FSDSRandom namespace keyed on `Environment.Seed`, reserved for any UE-side draws.
- **Sidecar plumbing:**
  - an optional `<track>.env.json` resolved through the loadTrack search dirs (`FSDSRpcServer.cpp:1765-1774`), including `Content/tracks/venues/*`;
  - `validate_tracks.py` validates sidecars against the schema.
- **Packaging:** `*.env.json` and the venue `LICENSE` files are copied next to `*.csv` everywhere tracks are staged:
  - `package_{windows,linux,mac}.sh`;
  - `package_windows.ps1:75`;
  - `.github/workflows/package-{windows,linux,mac}.yml` (Windows :60-65, Linux :73).

  A packaging check fails if any `Content/tracks/**/*.env.json` or venue `LICENSE` is missing from the staged output.
- **Hygiene:**
  - delete the dead `FSDSCustomMapLoader.cpp/.h` (its `srand(time(NULL))` is a determinism hazard);
  - give `FSDSTestTerrain` an origin offset and a probe-relative seam probe (:1600);
  - add `/Engine/BasicShapes` to `DirectoriesToAlwaysCook` so `validateRoadProbe` works in packaged builds;
  - Mission Control: `home_pose` uses `getStartGatePose` instead of z=0.3 (`main.py:150`), and `parse_track_csv` reports bad rows instead of silently truncating.
- **Test fixture:** an RPC that spawns 5%-grade and 3%-camber slabs at a known origin, plus an `FSDSProp` slab and a 15 cm `FSDSPropLidarOnly` strip. It is exercised from `tests/test_rpc.py`.

**Acceptance**
- **PR #0** is green on `dev` and blocks a deliberately broken C++ change on a test branch.
- **Flat no-op:**
  - for every `Content/tracks` CSV, start pose within 1 mm and 0.01°;
  - the "ground-snap N hit / 0 missed" log line is identical;
  - `test_reset.py`, `test_slam_regression.py` and `verify_determinism.py` pass;
  - lap time, DOOs and TP/FP/FN are within the P0 noise band over 5 seeds × N=5 runs.
- **Slope:** car spawned on an 8° ramp gate:
  - Z = surface + 0.55 m ± 1 cm;
  - pitch 8° ± 0.2°;
  - no "no road under the reset pose" warning.
- **Cones on the 5% fixture:** 0 snap misses, settle drift under 2 cm, 0 phantom DOOs across 10 runs.
- **Props:**
  - An `FSDSProp` box 3 m above a cone does not capture it (error < 1 cm), and the road probe ignores it.
  - Driving over the `FSDSProp` slab and the `FSDSPropLidarOnly` strip at 10 m/s produces no wheel-trace hit: suspension state stays within the noise band of the run without props. The CPU LiDAR returns points from both.
- **Finish trigger:** fires on 10/10 crossings with the gate ground at -3 m and at +3 m.

**Depends on:** P0 for the RNG and the noise band; otherwise P1 can run in parallel.

**Risks**
- The self-hosted runner serves a public repo. Run the gate only for same-repo PRs, never for forks.
- The global channel change can affect the legacy maps. Check them, or declare them unsupported.
- Storing post-snap positions may move the start pose by a few cm. Re-baseline once.

### P2a: Seeded generator, ground pad and rule-mandated distractors (base 4 pw; 6–8 pw)

**Goal:** reproduce deterministically the DNF-class failure (a false big orange that trips the stop latch) and the DS 1.5 distractors, on a flat floor large enough to hold them.

**Deliverables**
- **`tools/envgen/`** (Python, no UE):
  - `seed.py`: FNV-1a stream naming identical to `FSDSRandom.cpp:19-29`, plus a pure-Python SplitMix64 RNG;
  - `corridor.py`:
    - centreline, track polygon and keep-out buffers with shapely;
    - a ground-extent polygon; any prop whose footprint is not entirely over ground is rejected;
  - `rules.py`: FS Rules 2026 / DS 2026 checks:
    - width ≥ 3 m and cone spacing ≤ 5 m;
    - straights ≤ 80 m and turning diameter ≥ 9 m;
    - lap 200–500 m;
    - skidpad and acceleration geometry;
  - profiles `flat_baseline`, `stress`, `paddock_nearfield`;
  - `env_v1.schema.json`, including `ground.extent`;
  - golden SHA-256 hashes for 20 seeds, with coordinates rounded to 1 mm;
  - a CI job with `lfs: false`.
- **Ground extent.** The sidecar's `ground.extent` is the track bounding box plus 60 m, at least 300 × 300 m. That covers the 25 m perception crop from anywhere on the track, with margin.
- **Mission Control generate:**
  - Seed the MIT generator's `np.random` from the derived `track` stream inside `_gen_lock`, restoring the state afterwards. The submodule is not forked.
  - The endpoint writes `<name>.csv` and `<name>.env.json`.
- **`Private/Environment/FSDSEnvironmentBuilder.cpp`:**
  - spawned in `FSDSGameMode::StartPlay` before the cone spawner, and rebuilt in loadTrack before `ReloadTrack`;
  - spawns an `FSDSGroundPad` sized from `ground.extent`:
    - WorldStatic, BlockAll, simple == complex;
    - `PM_Asphalt` on top, at the floor's Z;
    - hides the `floor` actor and disables its collision while the pad is active, so there is no seam and no z-fighting;
  - one HISM per prop class, with no randomness in UE;
  - LiDAR dispatch gated until ready;
  - logs a SHA-256 of the spawned transforms;
  - `resetScenario` leaves the environment untouched.
- **Prop classes**, imported from CARLA where a suitable mesh exists and built from own primitives otherwise (no RaceCourse kit pieces), under cooked `Content/Environment/Props`. Timekeeping tripods are own primitives; CARLA has no equivalent:
  - timekeeping light-barrier tripods with keep-out cone rings (`FSDSProp`);
  - spare-cone stacks, and another discipline's cone layout at 15–40 m (existing cone meshes, scored as clutter);
  - 0.45–0.75 m bins and bollards (`FSDSProp`);
  - low tyre walls, stacked from own cylinders (`FSDSProp`). Moved here from P2b because they rank with narrow props as the worst stressor.
- **`auditPropCollision` RPC:** a ray-grid silhouette test per class. It also runs on the cone meshes against the DS 2026 dimensions: 228×228×325 mm for small cones, 285×285×505 mm for big orange.
- **Cone-fidelity PR (a deliberate default change, kept separate):**
  - per-type mass from DS 2026 Table 1 (0.45 kg small, 1.05 kg big orange), replacing the uniform 1.0 kg at `FSDSReferee.cpp:216`;
  - re-baseline DOO counts and the 2 s settle invariant once, with a CHANGELOG entry.
- **Scenario matrices** `tools/scenario_runner/scenarios/clutter_{narrow,tripod,tyrewall}.json`: 5 seeds × N=5 runs each, plus the same seeds with props disabled as the control, with stop-latch event detection.
- **Provenance:** `THIRD_PARTY_ASSETS.md`, plus a CI check that every `.uasset` under `Content/Environment` has an entry.
- **Non-gating upstream issues, each with a bag:**
  - footprint/width and max-neighbourhood-height gates in `cone_detection.py:391-460`;
  - a big-orange pair-geometry check before the stop latch.

**Acceptance**
- **envgen:**
  - byte-identical output on Linux CI and on Windows;
  - 1000 seeds × profiles all pass `rules.py`;
  - 0 props inside the keep-out buffer, overlapping the track polygon in plan view, or off the ground polygon;
  - sidecars ≤ 100 KB;
  - all existing CSVs still load with no sidecar.
- **Ground pad:** enabled with zero props on existing tracks, closed-loop metrics stay within the P0 noise band of the disabled case.
- **Audit:**
  - solid props reach a silhouette hit ratio ≥ 90%;
  - cone silhouettes match the DS dimensions within ±10% at 3 heights, or an issue is filed.
- **Reproduction on pipeline @3eae497** (5 seeds × N=5 runs against the props-disabled control):
  - narrow props: mean ≥ 2 big-orange false positives per scan, outside the control's band;
  - structures ≥ 1 m tall away from the cone line: ≤ 0.1 false positives per scan;
  - premature stop latch in ≥ 3 of 25 clutter runs, and in 0 of 25 control runs, with the rate reported per seed;
  - a 0.6 m tyre wall 0.5 m behind the cone line: ≥ 3 false positives and ≥ 0.5 missed cones per scan.
- **[RB]:** sim clusters per scan at the 25 m crop come within 0.5–2× of the matched-crop real band, or the gap is itemised.
- **Environment hash:** the transform hash is identical across 3 boots and unchanged by `ScenarioSeed`.
- **Perf:** scan p95 ≤ 1.2× `flat_ref`, 0 skipped scans, ≥ 58 FPS.
- **Size:** zip ≤ +5 MB.
- **Disabled:** P1-identical outputs that do not depend on physics, and closed-loop metrics within band.

**Depends on:** P0, P1.

**Risks**
- The ideal ray model (no mixed pixels) may still make the sim easier than FSG. The [RB] band is the check once it exists.
- Overfitting the pipeline to fixtures: use held-out environment seeds.
- More hits per scan push the UDP/DDS fragmentation issue (#322). Record the largest message size.
- The cone-mass change interacts with the DOO thresholds. That is why it ships as its own PR with its own re-baseline.

### P2b: Remaining catalogue, ground-truth labels and repo hygiene (base 3.5 pw; 5–7 pw)

**Deliverables**
- **Props:**
  - concrete walls, barriers, fences and catch fences, from CARLA where a mesh fits and own primitives otherwise. The RaceCourse kit's `tire_stacked`, `concrete_wall` and `concreteFence_wall` are used only after open question 7 resolves their provenance;
  - static people as capsules;
  - marshal posts;
  - LiDAR-only kerb strips 0.3–1 m outside the cone line (`FSDSPropLidarOnly`, never inside the corridor). Drivable kerbs of ≤ 3 cm belong to P3 terrain.
- **Ground-truth labels:** `/testing_only/environment` from the bridge, and `perception_metrics.py` scoring false positives per clutter class.
- **Scenario matrices** `clutter_{wall,kerb,people}.json`: 5 seeds × N=5 runs plus the control.
- **`.lfsconfig`:** `fetchexclude` for `Content/RaceCourse/Maps` and `Textures/Environment/Terrain`, only after a cook log proves nothing cooked references them.
- **Non-gating upstream issue:** clutter persistence in cone_slam.

**Acceptance**
- **Reproduction** (against the control):
  - a concrete wall 0.5 m behind the cone line: ≥ 15% of cones missed, outside the control's band;
  - a 15 cm LiDAR-only kerb at 0.5 m gives ≥ 1.5 missed cones per scan, with 0 wheel-trace hits on the kerb.
- **Audit:** solid props reach ≥ 90% silhouette hit ratio, chainlink 10–50%.
- **Labels:** every spawned prop appears on `/testing_only/environment` with its class and a pose matching the sidecar within 1 mm.
- **Perf with the full catalogue:** scan p95 ≤ 1.2× `flat_ref`, 0 skipped scans, ≥ 58 FPS.
- **Size:** zip ≤ +10 MB in total for P2a and P2b.
- **LFS:** the default clone shrinks by ≥ 350 MB.
- **Disabled:** P1-identical results.

**Depends on:** P2a.

### P3: Seeded terrain at FS magnitudes (base 6 pw; 9–12 pw)

**Goal:** add grades, crossfall, crowns and micro-relief as presets whose collision matches the render surface. Terrain is the #2 perception stressor (§1): it targets ground removal and cone detection first, and also exercises the planar EKF and regen braking.

**Deliverables**
- **Spike, week 1.** In a **packaged Shipping build**, compare:
  - runtime `ProceduralMeshComponent`/DynamicMesh complex-as-simple terrain (about 125k triangles over the corridor, cooked synchronously);
  - a baked Landscape preset.

  Measure collision-cook time, LiDAR per-ray cost on grazing rays, and Chaos wheel behaviour. Record a decision.
- **Terrain block in `envgen`:**
  - analytic `h(x,y; preset, seed)` on an integer-hash lattice;
  - output is either parameters (runtime mesh) or a 16-bit heightmap PNG (Landscape), from one source of truth;
  - **extent:** terrain replaces the P2a ground pad and covers the same `ground.extent`, with ≤ 0.5 m cells;
  - **presets:**
    - `flat`;
    - `open_pad`: ZalaZONE-like 1% tilt, the degenerate control;
    - `fsg_paddock`: 1–2% falls, height patches of 5–10 mm, a drainage channel;
    - `fss_like`: -3% approach, +4% exit, 3% camber;
    - `undulating`: 0.12 m over 25 m;
  - drivable kerbs ≤ 3 cm live here, on the terrain contract;
  - **constraint checks:** vertical-curve radius ≥ 10 m, steps ≤ 3 cm, grade ≤ 6%.
  - Friction patches stay out of scope until the `GroundMuCompensation` guard supports a mu per surface.
- **UE terrain actor for the chosen path:**
  - WorldStatic/BlockAll, simple == complex;
  - `PM_Asphalt` 0.7;
  - the `floor` actor and the pad are hidden and their collision disabled while terrain is active;
  - `Build.cs` gains ProceduralMeshComponent/GeometryFramework or Landscape as needed.
- **`sampleTerrain` and `validateTerrain` RPCs:** road probe and simple Visibility trace against the analytic truth at N corridor points.
- **Scenarios** `terrain_*.json`. New metrics: EKF accel-bias error, braking distance against flat, ground-removal residual.
- **Non-gating upstream issues:**
  - local/piecewise ground segmentation;
  - gravity-aligned EKF calibration (`odometry_filter.cpp:441-445`);
  - a switch to relax cone_slam's flat priors;
  - grade feed-forward in `pi_velocity.py`.

**Acceptance**
- **Probe vs analytic surface:** height p99 < 5 mm and normal < 0.3°. LiDAR ground `|dz|` p99 < 1 cm, which shows simple and complex collision match.
- **Cones and car on every preset:**
  - 0 snap misses and settle drift < 2 cm;
  - 0 phantom DOOs across 10 runs;
  - 0 suspension saturations (±3.5 cm) at 12 m/s over crests and sags.
- **Reproduction on pipeline @3eae497** (N=5 × 5 seeds against the flat control, outside the P0 band):
  - undulating: recall down ≥ 10 pp;
  - 4% grade change: clusters ≥ 2×;
  - 2% sloped start: EKF bias error ≥ 0.15 m/s²;
  - 5% downhill: stopping distance up ≥ 20%.
- **Determinism:** the terrain hash is identical across 3 runs. If the runtime-mesh path is chosen, Python and C++ heights agree within 0.5 mm.
- **Perf and size:**
  - generation plus collision ≤ 2 s (runtime mesh), or venue load ≤ 5 s (Landscape);
  - scan p95 ≤ 1.5× `flat_ref`;
  - ≥ 58 FPS;
  - zip ≤ +5 MB (runtime mesh), or ≤ +20 MB per baked preset.
- **Flat or disabled terrain:** P1-identical results.

**Depends on:** P0, P1, P2a (builder, envgen, ground extent).

**Risks**
- Trimesh ray cost or Shipping collision cooking fails. Fall back to baked Landscape presets (editor work plus LFS).
- Raycast wheels "pop" at steps. Keep kerbs ≤ 3 cm in the drivable area; spherecast wheels are deferred.
- The pipeline regresses on grade. This is expected, which is why terrain stays opt-in.

### Deferred: LiDAR odometry, long range and far-field structure

Not part of this roadmap. Cone detection drops everything beyond 25 m, so range and far-field structure do not stress perception. They matter for LiDAR odometry (GLIM), which is a separate workstream. If that workstream starts, it can reuse the P2a builder and envgen with an extra far-field ring, an enlarged ground extent, a `long_range` LiDAR profile and per-point timing (#485).

### P5: FS Spain (Montmeló) DV-area twin (separately funded decision D-P5, conditional on data) (base 6.5 pw; 10–13 pw)

**Goal:** a pre-track rehearsal for the home event: run perception against the venue's real trackside and elevation before the car goes there. It is built from open data: the T1–T3 sector first, then the main straight (acceleration) and the paddock (skidpad).

**Deliverables**
- **`tools/venue_builder/`:**
  - fetches ICGC MET 0.5 m/2 m, Catastro INSPIRE buildings and OSM barriers into a gitignored, checksummed cache;
  - emits a heightmap PNG (≤ 8 MB) plus an **envgen-compatible sidecar** (structures as props), so it reuses the P3 load path;
  - drivable-surface conditioning (radius ≥ 10 m, steps ≤ 3 cm) with a diff report.
- **Venue outputs** live in `Content/tracks/venues/fss/` with that directory's own `LICENSE`:
  - OSM-derived sidecars are Derivative Databases under ODbL 1.0, with ICGC and Catastro attribution;
  - the ICGC-only heightmap is CC BY 4.0.

  Add `NOTICE` entries and an in-sim credits line. The provenance CI check covers this directory.
- **ENU origin:** the descriptor sets the GPS and baro home, replacing the Redmond default in `FSDSGpsSensor.h:35-41`.
- **Fixed-venue mode:** a sidecar flag bypasses MC `ensure_centered` (`main.py:1345-1349`). Three layouts that comply with the rules ship as CSV plus sidecar.

**Acceptance**
- **Reproducibility:** the build reproduces bit-identically from the cache.
- **Heightmap:** RMSE ≤ 0.20 m over the drivable corridor, measured against held-out ICGC ground points (spatial blocks not used in the build) or against a team GNSS/RTK walk if one exists.
- **Structure inventory:** recall ≥ 90% within 25 m of the corridor (the perception crop). The reference is hand-digitised from the ICGC 25 cm orthophoto by someone other than the builder's author.
- **Layouts:** pass `validate_tracks.py` with 0 snap misses.
- **Closed loop:** a 10-lap trackdrive completes on ≥ 4/5 seeds (majority of N=3 runs per seed), or each failure is triaged.
- **[RB]:** if team bags from Montmeló exist, scan statistics are within ±30% at a matched crop.
- **Perf and size:** P3 perf gates hold. Zip ≤ +60 MB.

**Depends on:** P2a, P2b, P3.

**Risks**
- The 2026 FSS DV area is unverified, and there are no wall-distance measurements. Without team photos or bags the twin may be a guess.
- OSM and Catastro coverage inside the circuit is likely incomplete, which may mean manual digitising (+1 pw).
- Landscape authoring is editor-only.
- Overfitting to one layout: keep procedural profiles in the regression suite.

### P6: Seeded regression farm and calibration (base 3 pw; 4.5–6 pw, ongoing after P2a)

**Deliverables**
- A golden suite of about 30 scenarios (text, < 3 MB) across all profiles, including the `open_pad` control. Each scenario runs N=3 times.
- Nightly closed-loop runs on the self-hosted Windows runner, reusing the packaged build rather than re-cloning LFS.
- An HTML trend report via `report_html.py`: DNF rate, false big-orange per scan, missed cones per scan, DOOs, lap time, each with its P0 band.
- Terrain profiles run in a separate, non-gating lane.

**Acceptance**
- The suite completes in < 4 h.
- On two consecutive nightlies with unchanged code, ≥ 95% of scenarios give the same majority verdict. Flaky scenarios are labelled automatically and quarantined from gating.
- CI LFS bandwidth stays < 2 GiB/month.

---

## 4. Effort, capacity and calendar

| Phase | Base | With 1.5–2× contingency | Lane | Status |
|---|---|---|---|---|
| P0 | 3.5 pw | 5–7 pw | Python + sim operator | Core |
| P1 (incl. PR #0 CI gate) | 4 pw | 6–8 pw | C++ | Core |
| P2a | 4 pw | 6–8 pw | Python + C++ | Core |
| P2b | 3.5 pw | 5–7 pw | Editor + Python + bridge | Core |
| P3 | 6 pw | 9–12 pw | Python + C++ | Core |
| P6 | 3 pw | 4.5–6 pw | CI | Core, ongoing |
| **Core subtotal** | **24 pw** | **36–48 pw** | | |
| P5 | 6.5 pw | 10–13 pw | Python + editor + geodata | Decision D-P5 |
| D1 real data | car team | — | outside this repo | Request |

**Capacity assumption, to be confirmed by the team (open question 11):** git history shows one or two people active on the sim and pipeline. The plan assumes two contributors at about 0.3 FTE each, about 0.6 pw per calendar week combined. Sim work pauses for exams (January, June) and for the competition season (July–August). That gives roughly 22–24 pw per year, so the core takes about 1.5–2 years at this capacity.

**Calendar at that capacity**
- **Oct–Dec 2026:**
  - Python lane: P0, including PR #1, `flat_ref`, the noise band and the flat-world perception baseline.
  - C++ lane: PR #0, then PR #2 and the start of P1.
  - File the D1 request with the car team.
- **Jan 2027:** exams, used as buffer.
- **Feb–Apr 2027:** finish P1, then P2a.
  - **Pre-track slice done before the first real-track test of 2027 (end of April at the latest):** P0, P1, and P2a with the narrow-prop, tripod and tyre-wall classes. This reproduces the stop-latch DNF in the sim before the car can hit it on track.
- **May–Jun 2027:** P2b and the P3 spike, at reduced pace during exams.
- **Jul–Aug 2027:** events. Sim development pauses. Record raw `/lidar_points` at every DV run (D1), which unlocks the [RB] gates.
- **Sep 2027 – early 2028:** P3 and P6. The raw LiDAR recorded at the 2027 events (D1) calibrates the [RB] gates.
- **P5:** only if funded and the venue data exists, no earlier than 2028.

If capacity allows, pull the P3 spike and the `fss_like` and `undulating` presets ahead of P2b. Terrain is the #2 stressor, and having it before the first track day is worth more than the rest of the prop catalogue. If confirmed capacity is lower, cut to the pre-track slice and re-plan after the 2027 season.

---

## 5. First PRs against `dev`

Branch off `dev`.

**PR #0 — `feat/<N>-plugin-build-gate`: "ci: UE plugin build gate on the self-hosted Windows runner".** C++ lane; lands first.
- `.github/workflows/plugin-cook.yml`, which fills the reference at `ci.yml:5-6`:
  - `runs-on: [self-hosted, Windows]`;
  - trigger: `pull_request` to `dev`/`main` with paths `Plugins/**`, `Config/**`, `*.uproject`;
  - `if: github.event.pull_request.head.repo.full_name == github.repository`, so fork code never runs on the self-hosted runner;
  - checkout with `lfs: false`, since compiling needs no assets;
  - runs the compile step of `tools/build_sim.ps1` (editor + Development game target);
  - a full BuildCookRun runs on push to `dev`, reusing the `package-windows.yml` job.
- Update the `ci.yml` header comment to match.
- **Verify:**
  - a throwaway branch with a deliberate compile error fails the check;
  - a clean branch passes;
  - record the incremental build time (target < 30 min).

**PR #1 — `feat/<N>-perception-stress-metrics`: "feat(sim_benchmark): score colour, false big-orange and stop-latch in perception metrics".** Python lane. No UE, LFS or settings change. It extends the existing tool rather than adding a new one.
- `tools/sim_benchmark/perception_metrics.py` today matches detections to ground truth by position only (`FrameMetrics`: TP/FP/FN and match error). `Cone2D.color` is carried through but never scored. Add:
  - **colour scoring:** a per-scan colour-confusion matrix over matched pairs;
  - **false big-orange per scan:** big-orange detections on `/Conos_Orange` with no big-orange ground-truth cone inside the match gate;
  - **offline stop-latch replay:** apply the `control_node._on_orange` rule to the detections (≥ 2 big-orange cones after `stop_latch_min_travel` metres, never released). Report the time and position of any latch whose anchor is not at the ground-truth finish;
  - **scan statistics:** ground fraction, above-ground points and clusters per scan at the 25 m crop. PointCloud2 is decoded through `point_step` and the field offsets (cf. pipeline e219322), so real `/lidar_points` bags decode correctly;
  - an optional per-class FP attribution hook. It reads `/testing_only/environment` once that topic exists (P2b) and reports "unattributed" until then.
- `run_perception_benchmark.py`: `--reference <summary.json>` reports deltas against a reference run (the P0 `flat_ref`).
- `perception_report.py`: the new metrics in the report.
- `tools/sim_benchmark/tests/test_perception_stress_metrics.py`, on synthetic frames:
  - colour confusion on a known mix;
  - a false big-orange pair after the minimum travel triggers the replayed latch; the same pair before it does not; the real finish oranges are not counted as premature;
  - a decode round-trip for a cloud with non-default `point_step` and extra ring/time fields.
- **Verify:**
  - run `pytest tools/sim_benchmark/tests/` locally (wiring the directory into `ci.yml` is a follow-up);
  - run the benchmark on an existing sim bag and paste the new summary into the PR;
  - the flat reference comes from the P0 `flat_ref` bag, in a follow-up note once recorded.

**PR #2 (parallel; merges after PR #0 is green) — `feat/<N>-terrain-aware-start-pose`: "fix(spawner): ground-snap start-gate pose and cache post-snap cone positions".** About 120 LOC.
- `Plugins/FSDSPlugin/Source/FSDSPlugin/Private/FSDSConeSpawner.cpp`:
  - new helper `TraceGroundAt(FVector2D, FHitResult&)`: object-type WorldStatic, complex, ±`GroundSearchHalfHeightCm`, ignoring the spawner, `SpawnedCones` and pawns;
  - `ComputeStartGatePose`, both branches (:553-563 and :626-661): `Z = hit + HeightOffset + 50`, and rotation = gate yaw composed with the normal from a 4-ray fit at ±0.8 m;
  - on a miss: warn and fall back to the old behaviour;
  - :445-458 caches the snapped Z;
  - the :167-173 snap window uses `GroundSearchHalfHeightCm`.
- `Public/FSDSConeSpawner.h`: `UPROPERTY GroundSearchHalfHeightCm = 20000`, and rewrite the stale doc block (:100-122).
- `CHANGELOG.md`: an `[Unreleased]` entry.
- **Verify** (PR #0 gate; build locally with `tools/build_sim.ps1` until it lands):
  - every `Content/tracks` CSV gives a `getStartGatePose` delta < 1 mm / 0.01° (no physics involved, so this is exact), and the snap log line is identical;
  - `test_reset.py` and `verify_determinism.py` pass;
  - a ramp-gate fixture (`tools/smoke/fixtures/ramp_gate.csv`) on the `validateRoadProbe` 8° ramp gives surface + 0.55 m ± 1 cm and pitch 8° ± 0.2°.

---

## 6. Out of scope

- `dev-manual` and all FMU/Simulink plant work, including FMU contact gating and road-normal use. Spherecast wheels are deferred until that parity work settles.
- Photorealism, cameras, visual-only foliage, weather and night rigs. Fab Standard/Megascans assets in the repo. Google Photorealistic 3D Tiles. Cesium ion or any runtime network dependency.
- OpenLevel switching, World Partition, level streaming mid-run, and runtime PCG as the source of truth.
- Forking the MITMotorsports generator, or changing the 7-column cone CSV.
- Fixing the autonomy pipeline itself. This roadmap supplies scenarios, bags and upstream issues only.
- LiDAR odometry of any kind (GLIM included), long LiDAR range, and far-field structure. See "Deferred" after P3.
- KISS-ICP, in any form: as a test arm, a baseline, or a motivation.
- New `/fsds/gss` dependencies.
- Moving clutter (walking people, vehicles).
- Per-material LiDAR reflectance and stripe signatures, unless LiDAR-intensity cone colouring is adopted. Friction patches, until the `GroundMuCompensation` guard supports a mu per surface.
- Twins of FSG, FSUK FS-AI, FSA, FSN or FS East. FSG exists only as a procedural `fsg_paddock` profile.
- Issues #592, #606 and #611, except where P0 measurements need them controlled.
- #483 (spline-cone Blueprints in never-cooked legacy content). It remains a separate Blueprint-only follow-up.

## 7. Open questions the team must decide

1. **When and where is the first real-track test of the 2027 season?** The pre-track slice is scheduled backwards from that date, and the venue decides which terrain preset and prop densities come first.
2. Terrain path after the P3 spike: a runtime mesh generated per seed, or a small library of baked Landscape presets?
3. Is LiDAR-intensity cone colouring planned? If so, physmat reflectance moves into scope.
4. Will the IFS08-DV-PIPELINE owners take the hardening issues (big-orange gate, stop latch, piecewise ground, gravity-aligned EKF)? Or should terrain stay within a grade budget the current pipeline tolerates?
5. **D1 and P5 data:**
   - Can the car team record raw `/lidar_points` with ring and time fields at the test site this autumn, and at every 2027 DV run?
   - Is there an FSS 2026 DV map, any Montmeló photos, walk videos or IFS-08 bags, and measured cone-lane-to-wall distances?
6. GitHub plan and LFS quota (Free is 10 GiB storage and bandwidth). Do you agree to `.lfsconfig` `fetchexclude` and pruning the legacy RaceCourse maps?
7. Provenance and licence of the RaceCourse kit inherited from FSDS: keep it or replace it? CARLA now covers most of what the kit offered (walls, fences, barriers), so replacing it is cheap; the kit blocks nothing.
8. Can the self-hosted Windows runner carry both the PR plugin builds (PR #0) and the P6 nightlies? Is "same-repo PRs only" acceptable for the plugin gate?
9. Must the GPU LiDAR path (ARM Macs) reach parity on dressed worlds, or is a smoke test enough?
10. Should fixed-venue mode (no MC recentring) coexist with random tracks clipped to a venue pad polygon?
11. Is the capacity assumption (about 0.6 pw/week combined, with pauses for exams and the season) right? If it is lower, the plan reduces to the pre-track slice.
