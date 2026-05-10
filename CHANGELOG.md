# Changelog

All notable changes to IFSSIM that affect operators (sim users) or
contributors (the people working on the codebase) are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com); the
project follows [Semantic Versioning](https://semver.org).

The autonomy pipeline (cone detection, SLAM, planning, control) lives
in the **autonomy submodule** and has its own changelog. Items below
are sim-side: Unreal Engine plugin, ROS 2 bridge, Mission Control,
shipping pipeline, documentation.

## [Unreleased]

### Changed

- **Mission Control session-start uses the action chain end-to-end**
  (#379, #381). `/api/event/start` now calls `StartMission` instead of
  writing `/pipeline_ctrl/enable` and sleeping 4.5 s; autonomy reaches
  `active` via the supervisor → mission_control → mode_manager chain
  before EBS releases. Single click in Mission Control →
  configure+activate+release in 3–4 s with no manual
  `/api/pipeline/start` needed. The legacy flag-file path
  (`/pipeline_ctrl/enable` + `entrypoint.sh` polling loop) is retired;
  the `pipeline_ctrl` volume mount is removed from `docker-compose.yml`.
- **`mode_manager.activate_mode` is now idempotent** (#379). The
  `_drive_transition` helper queries each node's current lifecycle
  state via `/<node>/get_state` and skips transitions whose target
  state is already reached. Calling `activate_mode("trackdrive")` on
  an already-active stack succeeds as a no-op in ~300 ms instead of
  failing with "transition X returned success=False" after the
  invalid CONFIGURE-from-active call. Same idempotency on the
  tear-down path: `activate_mode("")` against an unconfigured stack
  is now also a no-op.
- **`RPM_TO_MS` reverted to the doc-derived value `0.00821`** (#380).
  Lap-drift bag analysis showed the 2026-04-28 empirical value
  `0.00898` produced a +9.4 % vx overestimate on the current sim
  build (per-sample ratio mean = 0.914 across 3324 samples,
  spread p10..p90 = 0.05). The new value matches the pure-geometry
  formula `(2π · WheelRadius / GearRatio) / 60` from
  `docs/dv_pipeline_rebuild.md §3.5`. Both `OdometryFilter` and
  `cone_graph_slam_node`'s velocity prior share the constant.

### Added

- **DV pipeline diagram alignment** (#359) — five new ROS 2 packages
  under `pipeline/` (`dv_msgs`, `sim_supervisor`, `mission_control`,
  `mode_manager`, `coche_urdf`); four existing autonomy nodes
  (`cone_detection_node`, `slam_node`, `path_planning_node`,
  `control_node`) converted from plain Node to LifecycleNode; full
  StartMission action chain wired from web backend → supervisor →
  mission_control → mode_manager → autonomy lifecycle. Mission
  Control's `/api/pipeline/start` now drives the autonomy through the
  ROS Action protocol (legacy flag-file path retained as fallback).
- **DV pipeline `/odom` split** (#360, Phase 1 of `docs/autonomy_pipeline.md`
  Q1 resolution) — `sim_supervisor_node` now owns `/odom` (100 Hz,
  IMU+RPM complementary filter; no GSS, deliberately matching the
  real IFS-08 input set). `slam_node` keeps `/cone_slam/state` for
  absolute pose. `control_node` now reads pose from `/cone_slam/state`
  and twist from `/odom` — severs the coupling between SLAM
  data-association quality and the velocity estimate driving control.
- **Dependabot** weekly updates for npm (frontend), pip (backend), and
  github-actions, with minor/patch updates grouped per ecosystem.
- **PR template + issue templates** (bug report, feature request) under
  `.github/`. Blank issues are disabled — reporters land on the
  templated forms or the QUICKSTART / FUNCTIONALITIES quick links.
- **CI 5th job: markdown link checker** (lychee, `--offline` mode) —
  catches dead internal links + broken section anchors in `*.md`.
  External URLs not checked (GitHub-the-internet's flakiness is not
  our gate).
- **Pre-commit hooks** (`.pre-commit-config.yaml`) — opt-in local
  checks that mirror CI: ruff format + lint, ESLint, track CSV
  validator, repo hygiene (trailing whitespace, EOL, large-file
  guard). Setup instructions in README.

---

## [0.1.0] — first release

The first cut intended for use outside the ISC Racing Team's internal
dev loop. The sim is feature-complete for FS-DV mission practice; the
items in **Known limitations** below are real but not blockers for
that use case. See `docs/QUICKSTART.md` for first-run instructions.

### Added

#### Sensors and vehicle model

- **LiDAR with Hesai ATX-S01 working principle.** Per-point intensity
  computed as `ρ × cos(θ_inc) × (R_ref / range)²`. Per-cone-material
  reflectance values from the datasheet (blue ≈ 0.15, yellow ≈ 0.50,
  orange ≈ 0.65) keyed by `CustomDepthStencilValue` written by the
  cone spawner; non-cone surfaces (asphalt, walls, sky) fall back to
  Rec.709 luminance of the rendered colour. GPU-decode path renders
  depth + colour, runs a compute shader, and async-readbacks 95 k pts
  per scan at 10 Hz.
- **IMU (BMI088 model)** at 400 Hz with Ornstein–Uhlenbeck bias drift,
  Gaussian white noise on accel + gyro.
- **GPS (NavSatFix)** at 10 Hz with Gaussian position noise + flat-Earth
  ENU↔WGS84 conversion.
- **GSS (Kistler Correvit SFII model)** at 100 Hz, body-frame velocity.
- **Cameras** — 2× compressed image streams over the RPC server.
- **Barometer** — standard altitude formula.
- **Vehicle model** — IFS-08 chassis, Chaos physics, EMRAX 228 MV
  motor with NX-tech LUT-based torque envelope, regen gated by
  battery cell-input current limit (not motor mechanical envelope).

#### Wire layer

- **TCP RPC server** on port 41451 — commands, queries, camera
  request/response.
- **UDP broadcaster** on 41452 (sensor frames) + 41453 (LiDAR chunks).
  LiDAR-over-UDP is the only LiDAR transport (TCP and UDS variants
  were retired in #322 — UDP turned out to be reliable on every
  supported host).
- **Optional `/lidar/Lidar1/viz` subsampled cloud** for browser-based
  viewers. Off by default; opt in with
  `LIDAR_VIZ_DECIMATION=4 ./tools/refresh-bridge.sh` to drop the
  Foxglove tab CPU from ~35 % to ~10 % at 1/4 spatial density.

#### ROS 2 bridge (Humble)

- Single integration point for all topics. Subscribers see:
  `gps`, `imu`, `gss`, `motor_rpm`, `tire_loads`,
  `testing_only/odom`, `lidar/Lidar1`, `lidar/Lidar1/viz`
  (when enabled), `camera/cam{1,2}/compressed`, `signal/go`,
  `signal/finished`, `testing_only/extra_info`, `testing_only/track`,
  plus standard `tf` / `tf_static`.
- Foxglove WebSocket bridge on port 8765 for visualisation.

#### Mission Control

- **FastAPI backend** (port 8000) — orchestrates the FS-DV lifecycle:
  load track → set event → start session → SLAM warm-up → release
  EBS → driving. Persistent connection cache, async telemetry
  WebSocket, idempotency on Start, atomic state file, bounded
  session log, path-traversal-safe track endpoints.
- **React frontend** (port 3000) — telemetry dashboard, track
  manager, scoring panel, session log. WS reconnect with
  exponential backoff, shape-validated frames, non-blocking confirm
  modal, accessible keyboard navigation.

#### Track system

- **CSV-based track format** (`cone_type,x,y` per row) consumed by
  `FSDSConeSpawner` and the `loadTrack` RPC.
- **10 built-in tracks** including FS-Rules-canonical
  `acceleration.csv`, `skidpad.csv`, `trackdrive*.csv`.
- **Random track generator** (`tools/random-track-generator/`) for
  procedural autocross-style tracks.
- **Track CSV validator** (`tools/validate_tracks.py`) gates every
  PR via CI — schema check, cone-type whitelist, ±500 m sanity
  bound on coordinates.

#### Referee + scoring

- **FS-Rules 2026 D 9.1.1** — single-run scoring formula across all
  four DV disciplines (Acceleration, Skidpad, Autocross, Trackdrive).
- **D 10 penalties** — DOO (per-event seconds), OC (DQ for Acc/Skid,
  +10 s for Autocross/Trackdrive), USS (DQ for DV disciplines, −50 pts
  for Trackdrive).
- **D 9.2.1 runtime cap** — DV Acc/Skid runs > 25 s are DQ'd.
- **Skidpad averaging** — score time = avg(right laps) + avg(left laps).
- 30 unit tests pin every branch of the scoring formula against
  hand-calculated examples.

#### Shipping pipeline

- **`package_mac.sh`** — produces `IFSSIM-Mac-Shipping.app`, signed
  with sandbox entitlements (network.server for RPC port 41451),
  with `tracks/` sibling, `settings.json` staged to UserSettingsDir,
  and `UECommandLine.txt` patched for windowed mode + 60 FPS cap.
- **`package_linux.sh`** — Linux Shipping, native or cross-compile
  from a Mac dev box (with Epic's Linux Clang Toolchain).
- **`package_windows.ps1`** — Win64 Shipping, requires a Windows host
  with VS 2022 (UE5 cannot cross-compile to Windows from any other
  platform).

#### Continuous integration

- **PR gate** (`ci.yml`) — every PR to `dev`/`main` runs four jobs on
  free GitHub-hosted runners: bridge `colcon build`, Mission Control
  frontend (lint + typecheck + build), Mission Control backend
  (`pytest`, 69 unit tests covering path-traversal validators and
  scoring), and the track CSV validator. Total wall time ≤ 4 min.
  Cannot merge with a red status.
- **Release-on-tag workflows** (`package-{mac,linux,windows}.yml`) —
  trigger on version tag push, run on self-hosted platform-specific
  runners with UE5 5.7 installed.

#### Documentation

- **`docs/FUNCTIONALITIES.md`** — comprehensive technical reference
  for every system, sensor, RPC method, ROS 2 topic, and Mission
  Control endpoint.
- **`docs/autonomy_pipeline.md`** — end-to-end architecture (sim →
  bridge → SLAM/planning/control → Mission Control), with the
  integration contract every autonomy node should respect.
- **`docs/GETTING_STARTED_DOCKER.md`** — bringing up the full
  bridge + autonomy stack alongside the sim.
- **`docs/QUICKSTART.md`** — first-run instructions for someone who
  just wants to drive the sim.

### Known limitations

These are documented because they're real and an operator should be
aware of them; none block first-release usage.

- **IFS-08 physics asset CoG bias.** The `FormulaMesh_PhysicsAsset`
  ships with a forward-biased authored centre-of-mass that runtime
  `BodyInstance.COMNudge` overrides cannot fully correct — saturates
  non-monotonically. The load-transfer RPC reports the actual Chaos
  values so autonomy can consume *relative* wheel loads correctly,
  but the absolute distribution is biased. Tracked as **#142**;
  re-authoring the asset from a real SolidWorks model is the fix.
- **Lichtblick LiDAR flicker on Mac.** Browser-based pointcloud
  viewers (Foxglove web, Lichtblick web) deserialise the 1.5 MB/scan
  stream on the JS thread → 30–40 % CPU on a tab → occasional
  render hiccups that look like LiDAR flicker. Two mitigations:
  (a) use `LIDAR_VIZ_DECIMATION=4` to subscribe to the 1/4-density
  `/lidar/Lidar1/viz` topic; (b) use the native Foxglove Studio /
  Lichtblick desktop app instead of the browser.
- **Default-map BP** (`spline_cones_mini_orange`) has broken
  references that block cooking. `DefaultGame.ini` is scoped around
  it (`+DirectoriesToAlwaysCook` only includes the working
  `trafficones_scaled` subdirectory) so the sim still cooks; the BP
  itself needs a manual editor fix.
- **No autonomy submodule shipped with this release.** The sim runs
  standalone; the autonomy stack lives in a separate repository
  with its own release cadence and its own known issues
  (notably planner behaviour on tight turns, autonomy-side #180).

### Platform support

- **macOS 14+** with UE5 5.7 — first-class. Tested on Apple Silicon.
- **Linux** — release pipeline scripted but not yet exercised on a
  self-hosted runner; cross-compile from macOS works.
- **Windows** — release pipeline scripted but requires a Windows
  host (UE5 cannot cross-compile to Windows from other platforms).

[Unreleased]: https://github.com/isc-fs/IFSSIM/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/isc-fs/IFSSIM/releases/tag/v0.1.0
