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

### Added

- **One-command build from source** — `tools/build_sim.sh` (plus
  `tools/build_sim.ps1`, a PowerShell bootstrap for Windows machines
  with no Git yet). Detects the platform, checks disk space, verifies
  git / git-lfs / Xcode / Visual Studio / Unreal, installs what it
  safely can via brew / winget / apt, repairs unfetched LFS assets and
  uninitialised submodules, then runs the right `package_*.sh`. Reads
  the required engine version from `IFSSIM.uproject` rather than
  hardcoding it. `--check` runs the preflight and changes nothing.
  Written for contributors who have never used Unreal; the guide is
  `docs/BUILD_FROM_SOURCE.md`.

### Fixed

- **`package_mac.sh` now honours `UE_ROOT`.** It hardcoded
  `/Users/Shared/Epic Games/UE_5.7`, so the override that
  `docs/SETUP.md` documented — and that `package_windows.sh` (`UE_ROOT`)
  and `package_linux.sh` (`UE5_ROOT`) both already supported — silently
  did nothing on macOS. A missing engine now fails with a clear message
  instead of a bare "No such file or directory".

## [0.2.0] — 2026-08-26

**The vehicle dynamics left Chaos.** This release is dominated by one
thread: the car's physics moved out of Unreal's arcade vehicle
simulator and behind an interface, and a team-authored Simulink model
now drives the car. Alongside that, the autonomy pipeline became a
submodule, every stochastic source became seedable, and a series of
long-standing physics defects were found — several of which had been
silently wrong since the project started.

Two changes are breaking for anyone tracking `settings.json` or the
repository layout: the autonomy pipeline is no longer in this repo, and
`MaxSteerAngle` has changed.

### Added

- **`IFSDSPlant` — the platform/plant seam.** The simulator now has an
  explicit boundary between *the world* (terrain, sensors, cones,
  referee — Unreal's job) and *the vehicle* (tyres, suspension,
  powertrain, aero — the dynamics engineers' job). SI units, ISO 8855
  body frame, ENU world. Two implementations: `FFSDSChaosPlant`
  (default, and still the only validated reference) and
  `FFSDSFmuPlant`.
- **A Simulink vehicle model, in `matlab/plant/`.** Six subsystems with
  named owners — chassis, tyre/suspension, steering, powertrain, aero,
  brakes — each generated from a `build_*.m` script so a regenerated
  model is reviewable in a diff rather than an opaque binary. Every
  parameter comes from `settings.json`, with per-field provenance
  (`measured` / `default` / `ASSUMPTION`). Runs in MATLAB alone; no
  Unreal, Docker or ROS needed to work on it.
- **An FMI 3.0 co-simulation importer.** Reads, extracts and gates an
  `.fmu` from inside the engine, including a ZIP reader written against
  zlib because the engine's own only links under `bBuildEditor`. State
  save/restore round-trips bitwise exact.
- **`Plant.Type` in `settings.json`** — `chaos`, `shadow` or `fmu`. In
  `shadow`, the FMU steps alongside Chaos on identical inputs and the
  divergence is logged; it drives nothing, so it cannot change
  behaviour. In **`fmu`** the FMU integrates the vehicle and the mesh
  becomes a kinematic target written from the plant's pose each tick —
  sensors already read `PlantState`, so they follow for free.
- **The FMU drives the car (Phase 6).** Measured: 0 → 21.8 m/s in 10 s
  with all four wheels in contact, and a 0.5 steering command giving an
  8.3 m radius against 8.22 m from `L/tan(δ)` — within 1% of an
  independent kinematic prediction rather than a number tuned to match
  anything.
- **A road probe.** The platform now answers *what is under each wheel*
  — five rays per wheel, least-squares plane fit, reporting height,
  normal and an RMS residual so the plant can detect a bad fit rather
  than trust it. Validated against a ramp/crown/step test level with
  analytic ground truth, because on flat terrain a working probe and a
  stub returning zero produce identical logs.
- **Seeded determinism.** `resetScenario` RPC, a seeded scenario
  runner, `-fsds.seed=` override, and a verifier that proves the RNG
  reproduces — which on first run *failed*, showing `resetScenario`
  alone was insufficient.
- **Benchmarking toolkit** (`tools/sim_benchmark/`) — offline
  perception/SLAM/control benchmarks, the real C++ EKF driven through
  pybind11, bag-based drift checks with frame alignment.
- **Real-car parity for bag lift** — a sim uDV emulator on the stock
  Mission Control surface, and auto-derived `<name>_carparity` bags
  (LiDAR + IMU only) for replaying onto the car on stands.

### Changed

- **Breaking — the autonomy pipeline is now a submodule.** Cone
  detection, SLAM, planning and control moved to
  [`isc-fs/IFS08-DV-PIPELINE`](https://github.com/isc-fs/IFS08-DV-PIPELINE)
  and are consumed at `pipeline/`. Pipeline changes go to that repo.
  After pulling, run `git submodule update --init --recursive`.
- **Breaking — `MaxSteerAngle` 28° → 22.4°.** Constant-steer sweeps
  show lateral acceleration peaks at 22.4° (1.336 g) and *falls* to
  1.268 g by 28°, while yaw/kinematic collapses 0.873 → 0.651. Past the
  peak, more lock buys less turn, which inverts the sign of a path
  controller's feedback — it runs wide, adds lock, turns less, adds
  more. Nothing is lost: every angle removed produced less curvature
  than 22.4° already does. The 28° it replaced was never measured.
- **The sim LiDAR publishes on `/lidar_points`**, matching the car.
  Bags recorded before this need
  `--remap /lidar/Lidar1:=/lidar_points` on replay.
- **UE sim time is the authoritative capture clock** end-to-end.

### Fixed

Most of these had been wrong since the project started, and were found
by building the plant seam rather than by anything failing loudly.

- **Chaos was squaring the steering command.** `SquaredFunction` was
  the engine default and never overridden, so a 0.5 command produced
  0.25 of full lock — the autonomy had been getting roughly half the
  steering it asked for in the mid-range, on top of a rate limit
  needing 0.4 s to reach full lock.
- **The Pacejka tyre model never reached the solver.** Chaos builds its
  physics wheels from the wheel class's *class default object* before
  `BeginPlay`, so everything written to the per-instance wheels — the
  tyre curve, friction, brake torque, radius, steer limit — landed on
  an object the solver never reads. Much of `settings.json`'s
  `VehiclePhysics` block had been decoration.
- **The tyre curve was far too peaky** once it did reach the solver:
  `LatC 1.9 → 1.4`, `LatE −1.5 → −0.3`. The old shape peaked at 4.9° of
  slip and returned 34% of grip at full lock.
- **Regen never reached the sim.** The brake/regen channel was dropped
  in the `/ctrl/cmd` relay, so the car could only coast, never brake —
  the root cause of corner overshoot at speed.
- **IMU accelerometer noise was 100× too small** — applied in cm/s²
  while configured in m/s². The EKF had been tuned against a far too
  clean IMU.
- **Spring rate was 2.3× too soft**, aero was applied twice, the aero
  moment arm was 10× too long, steering ran reverse Ackermann, and the
  speed-dependent steering curve was authored in km/h while Chaos
  evaluates it in MPH.
- **Multi-gate tracks spawned the car 90° off** (acceleration, skidpad).
- **A finished bag could be abandoned** when the recorder's stop
  service timed out while finalising a multi-GB mcap — the caller read
  the timeout as failure and skipped the copy to the host.
- **`resetScenario` left scoring permanently blind** — every repeat run
  scored 0/0/0.
- **Plants were initialised twice on the same object.** Invisible for
  years because the Chaos plant is idempotent; it only became a crash
  once something non-reentrant sat behind the same call — the FMU
  declares one instance per process, and a second `Instantiate`
  segfaults rather than failing. Teardown is now deterministic in
  `EndPlay`, since PIE restarts `BeginPlay` on a new pawn while the old
  one is still alive.

### Known limitations

- **Chaos remains the default and the reference.** The FMU drives the
  car under `Plant.Type="fmu"`, but `chaos` is still what a fresh
  checkout runs, and it is the implementation every prior lap was
  validated against. Same-state parity between the two is ~0.8 m/s²
  mean.
- **`Crr = 0.020` and the 22.4° steering clamp both rest on a
  shape-fitted Pacejka, not measured tyre data.** Every dynamics number
  in this release is internally consistent and none of it is anchored
  to the real Hoosier. This is the measurement that would turn a
  self-consistent simulator into a validated one.
- **Four sources still disagree on maximum steering lock** — 28°
  (invented), 22.4° (tyre peak), 18.2° (uDV firmware), 19.25° (the
  IFS-08 workbook). The workbook's figure may derive from the *IFS-07*
  wheelbase; see `docs/IFS_08_measured_parameters.md`. Tracked at #462.
- The controller normalises steering by 18.2° while the sim maps full
  lock to 22.4° — a 1.23× scale mismatch. The clamp makes it safe, not
  consistent.


## [0.1.2] — 2026-05-26

The biggest autonomy-pipeline release since v0.1.0. Three intersecting
threads: a clean **9-state EKF** with Coriolis-correct mechanics
(rewrite + post-rewrite hardening), a **two-phase mission lifecycle**
that separates `SetMission` (configure) from `RuntimeControl` (run),
and a stack of **SLAM hardening** changes that took mid-lap `/odom`
position drift from 56 m down to under 2 m on the autocross course
and eliminated the late-corner SLAM yaw-cascade pattern.

Sim-side: documentation overhaul, GPL-3.0-or-later relicense, auto-
pull bags onto the host on session-stop, and a `compose-watch`-based
inner-loop for Python edits without rebuilding the image.

### Changed

- **Breaking — odometry filter rewritten as a 9-state EKF.**
  The complementary filter that previously lived in
  `pipeline/odometry_filter/` is replaced by a hand-rolled EKF over
  state `[x, y, θ, vx, vy, ω, ba_x, ba_y, bg_z]`. The predict step
  now carries the Coriolis cross-terms (`v̇x = ax + ω·vy`,
  `v̇y = ay − ω·vx`), which the complementary filter could not.
  During steady-state cornering the IMU's body-y axis reads
  centripetal acceleration `ω·vx` — the old filter integrated this
  directly into `vy`, so `/odom.vy` drifted unbounded (4+ m/s after
  68 s of cornering) and the inflated `state.speed` inside
  PurePursuit was the trackdrive ceiling. The new EKF cancels the
  Coriolis term in `v̇y` exactly when the model is consistent; the
  gtest regression `CoriolisCornering.SteadyVyIsNearZero`
  synthesises a 60 s constant-radius turn and asserts
  `|vy| < 0.10 m/s` (pre-rewrite this fails by ~14×).

  Measurements: motor-RPM is now a Kalman update on `vx` with
  `sigma_rpm = 2 cm/s`, so RPM dominates over IMU accel integration
  for `vx` tracking. Steering kinematic-bicycle
  (`ω_pred = (vx/L)·tan δ`) is a **gated** update on `ω` — applied
  when `|residual| < threshold`, raises `slip_flag` and rejects the
  update otherwise (the kinematic-bicycle model is wrong under slip,
  so folding it in would corrupt yaw).

  **Breaking surface**:
  - `/brake_pressure` subscription dropped from
    `odometry_filter_node`. Brake authority pulled `α_vx` toward
    zero during heavy braking pre-rewrite; the EKF's Kalman gating
    handles wheel-lockup naturally via covariance. Bridge still
    publishes `/brake_pressure` for replay parity, but no autonomy
    consumer subscribes.
  - `/odom_diag/effective_alpha_vx` topic retired.
  - `OdometryFilter::push_brake()` method, `Params` struct (replaced
    by `EkfParams`), `kAlphaVx*`, `kBetaVyLeak`, `kBrakeLockup*`,
    `kRpmStaleS` all removed.

  **Preserved**: `/odom` (now with non-trivial covariance populated
  from the EKF P matrix), `odom→base_link` TF, lifecycle, QoS,
  frame names, and the `/odom_diag/yaw_residual_rad_s` +
  `/odom_diag/slip_flag` topics.

### Added

- **Auto-pull bag onto host filesystem on session stop** (#498,
  closes the manual `pull-bag.sh` step from #490). When **Record bag
  (mcap)** is ticked, clicking **Stop Session** now:
  1. Finalises the bag in the docker volume (`ifssim_bags`, on
     container ext4 — fast).
  2. Streams the bag tarball out of `dv_pipeline_stack` via the
     Python docker SDK (`container.get_archive`) and extracts it
     into the host's `./bags/<name>/` (bind-mounted to mc_backend
     as `/host_bags/`).
  3. Cleans the volume-side copy via `container.exec_run(rm -rf …)`.

  All inside the StopBag callback. No user action required to get
  the bag onto the host filesystem; `tools/pull-bag.sh` is now the
  manual-recovery path, not the default flow. Session log surfaces
  progress + the final host path. Failure semantics are best-effort:
  if the transfer fails (host disk full, etc.), the bag stays in
  the volume and the user gets a clear log line pointing at the
  manual recovery command.

  Env-gated via `IFSSIM_BAG_AUTO_PULL` (default `1`). Set to `0` to
  disable and keep the manual `pull-bag.sh` flow.

  Why the Python SDK and not the docker CLI: the Linux docker CLI
  inside mc_backend can't pass a Windows host path to `docker cp`
  (the first `:` in `C:/Users/...` gets parsed as a container name,
  and the daemon doesn't recognise `/c/Users/...` as a host mount).
  The SDK uses the daemon's HTTP API directly — no argv parsing.
  Tarball extraction uses Python 3.12+'s safe `filter="data"`
  policy, plus a member-path traversal check for defence in depth.

  **One security note**: the mc_backend container now mounts
  `/var/run/docker.sock` (for the SDK) and `./bags:/host_bags`
  (for the extraction target). docker.sock means anything that
  compromises mc_backend can root the host. The blast radius for
  mc_backend was already broad (it talks RPC to the sim, runs ROS
  actions, holds the API key), so this doesn't materially change
  the threat model for this dev/sim stack. For a hardened
  deployment, set `IFSSIM_BAG_AUTO_PULL=0` AND drop both mounts
  from `docker-compose.yml`.

  The `./bags:/host_bags` bind-mount partially reverses #490's
  retirement of host bind-mounts on the autonomy stack — but it
  only applies to mc_backend (low-traffic) and only at session-stop
  (write-only, not on any hot path), so the 30-90 s startup-time
  cost #490 was targeting doesn't apply here. Recording itself
  still lands in the named volume on container ext4 (fast); this
  bind-mount only takes the finalised tarball at the end.

- **Two-phase mission lifecycle: `SetMission` → `RuntimeControl`**
  (#499, #518). The single `StartMission` action that previously
  combined configure + activate is replaced by an explicit two-step
  protocol on `mission_control_node`:
  - **`SetMission(mission_id)`** — prepare phase. Resolves the
    mission via `MODE_REGISTRY` (single source of truth for
    `mission → ordered (node, behavior)` in
    `pipeline/mode_manager/mode_manager/mode_registry.py`),
    calls each autonomy node's new `~/setup` service with
    `(mode_name, behavior)`, then drives the `configure` lifecycle
    transition. Per-node progress streams back as `Feedback.stage`
    so the operator sees live bring-up state instead of a single
    "starting" wait. Numba JIT (~10-20 s on `cone_detection_node`)
    lands here, not in `RuntimeControl`.
  - **`RuntimeControl`** — activate + run. Once `SetMission` returns
    `success=true`, the client opens `RuntimeControl`;
    `mission_control_node` activates the prepared stack and streams
    throttle/steering/emergency/finished feedback at 40 Hz.
    `sim_supervisor_node` subscribes to the feedback topic and
    relays each frame onto `/fsds/control_command` for the bridge —
    a clean split from before, where the supervisor was the action
    server.

  New packages: `pipeline/node_base/` (Python) and
  `pipeline/node_base_cpp/` (C++) provide `BaseLifecycleNode` with
  the `~/setup` plumbing; every managed autonomy node now inherits
  from it. `pipeline/bringup/` consolidates the launch files
  (`sim_pipeline.launch.py`, `car_pipeline.launch.py`,
  `full_pipeline.launch.py`) — `docker/dv_pipeline_stack/
  pipeline.launch.py` is now a thin include of these.

- **`odometry_filter_node`** (#499 / #518). The IMU+RPM
  complementary filter that previously lived inside
  `sim_supervisor_node` was lifted into a standalone C++
  `BaseLifecycleNode` (`pipeline/odometry_filter_node/`, backed by
  `pipeline/odometry_filter/` for the algorithm library). Same
  `/odom` contract (100 Hz, IMU+RPM+steering+brake_pressure,
  `odom→base_link` TF, identical diagnostics on
  `/odom_diag/*`) — but now part of the standard managed bring-up
  via `activate_mode`, alongside the other four autonomy nodes.
  Matches the real-car split where the uDV's odometry firmware is
  a separate subsystem from the mission interface.

- **`supervisor_cli`** (#518). Terminal client that mirrors the web
  backend's two-phase flow without needing Mission Control:
  `ros2 run sim_supervisor supervisor_cli {set_mission, start_mission, run}`.
  Reproduces production lifecycle from a shell — useful for
  developing missions/behaviors and triaging session-start bugs.
  Mission IDs match the registry (1=trackdrive, 2=autocross,
  3=accel, 4=skidpad, 5=scruti, 0=tear down).

- **`tools/compose-up-and-watch.sh|ps1`** (#518). Wrapper that
  runs `docker compose up -d` then `docker compose watch` in the
  foreground, syncing host edits under `pipeline/*` into named
  src volumes for live Python iteration. Replaces the pre-#490
  bind-mount workflow; `tools/refresh-bridge.sh` is still the
  go-to for C++ / msg / launch / setup.py changes.

- **`DriveController` / `CompositeDriveController` / Stanley** in
  `pipeline/control/control/controllers/`. The control node now
  picks its lateral+longitudinal strategy from the `behavior`
  string passed via `~/setup` — `pure_pursuit` for trackdrive/accel,
  `stanley` for autocross/skidpad/scruti — instead of a runtime
  ROS parameter. Composite wraps both into a single
  `ActuationCommand`-returning interface.

### Changed

- **Project relicensed to GPL-3.0-or-later.** Top-level `LICENSE`
  added with the full GPL-3.0 text. All `package.xml` files under
  `pipeline/` and `ros2/src/` updated from their previous mix of
  MIT (7 packages), Apache-2.0 (4 packages), and GPLv2 (2 packages —
  fs_msgs + ifssim_bridge inherited from upstream FSDS-Sim) to a
  uniform `GPL-3.0-or-later`. Motivation: keep the stack on a single
  copyleft-compatible licence so prospective GPL-3-only SLAM /
  perception dependencies can be linked in without per-package
  licence-conflict audits. Apache-2.0 contributions remain
  attributable through git history; the relicense reflects forward
  distribution only.

- **Documentation overhaul** (#492). Consolidated the three setup
  entry points (`readme.md` → `QUICKSTART.md` → `GETTING_STARTED_DOCKER.md`)
  into one unified path:
  - **New** `docs/SETUP.md` — single first-time-user setup, parallel
    Windows + macOS sections. Covers prereqs, submodule init, sim
    build / download, Docker stack, first Mission Control session,
    troubleshooting. Replaces `QUICKSTART.md` (Mac-only, stale post-#482)
    and `GETTING_STARTED_DOCKER.md` (UE 5.4 + bind-mount workflow,
    both wrong post-v0.1.0 / #490).
  - **New** `docs/OPERATING.md` — daily ops: refresh-bridge cycle,
    recording and retrieving MCAP bags, switching tracks, common
    failures during a session.
  - `docs/autonomy_pipeline.md` → `docs/AUTONOMY.md` (renamed, links
    refreshed for the new doc names).
  - `docs/FUNCTIONALITIES.md` → `docs/REFERENCE.md` (renamed so
    first-time users don't mistake the 1000-line reference for setup).
  - **New** `docs/CONTRIBUTING.md` — branch flow + CI + release
    process. Extracted from `readme.md`'s "Hacking on IFSSIM" section.
  - `readme.md` slimmed to a thin front door (45 lines). All
    contributor flow moved to `CONTRIBUTING.md`.
  - `.github/ISSUE_TEMPLATE/*.yml` + `PULL_REQUEST_TEMPLATE.md`
    repointed at the new doc names. Code comments in
    `docker/`, `pipeline/sim_supervisor/`, `ros2/src/ifssim_bridge/`
    also updated.

### Added (SLAM + EKF hardening, post-v0.1.0 9-state EKF)

A second wave of autonomy-pipeline fixes landed after the initial EKF
rewrite. Each is a small change but they compose into the headline
"`/odom` position drift mid-lap dropped from 56 m to under 2 m on the
60 s autocross course, integrated yaw error from +19° mean down to
+1.3° mean, and the late-corner SLAM yaw-cascade pattern is gone."

- **`/odom` as a SLAM pose prior** (#545). `cone_graph_slam` adds a
  `BetweenFactorPose3` on consecutive poses driven by the latest
  `/odom` delta. Bridges cone-poor windows where the previous IMU-only
  prior gave the optimizer too much freedom and let it snap to
  bad cone factors. Anchors the inter-scan pose so cone DA is
  evaluated against a pose consistent with wheel odometry, not just
  IMU pre-integration.
- **Kinematic-bicycle steering BetweenFactor** (#543). A second pose
  prior derived from `ω = (vx/L)·tan δ`, silent during cornering by
  design (the slip-gate suppresses the factor when the kinematic
  bicycle is wrong) but informative on straights — modestly
  constrains yaw without contributing model error during transients.
- **Cascade-skip recovery with force-accept threshold** (#541). When
  the DA-failure cascade detector skips 5+ consecutive scans of cone
  factors, force-accept the next scan as new-territory exploration.
  Prevents permanent IMU-only drift when the car enters genuinely new
  cone geometry (the detector's "everything looks new" signature is
  ambiguous between cascade and exploration).
- **Under-observed landmark filter** (#536). `/Conos` (the
  map-frame cone publication consumed by `path_planning_node`) now
  excludes landmarks observed fewer than 3 times. Filters single-shot
  perception artifacts and phantom DA-spawn ghosts. Cone count on
  /Conos drops from ~130 to ~100 on a clean autocross run; planner
  sees a more stable map.
- **EKF stationary-calibration invariant restored** (#534). The
  3-second post-activate stationary-calibration window had been
  drifting due to a Phase-3↔4 ordering issue surfaced by the
  `SetMission`/`RuntimeControl` two-phase split. Restored the
  per-sample stationarity gate and added a low-`vx` gate on the
  steering correction (skip when `vx < 3 m/s`, where kinematic-
  bicycle equilibrium hasn't built up).
- **Gyro bias freeze + Joseph-form covariance update + slip-aware
  NHC** (#555). The 9-state EKF's `F[OMEGA, BG_Z] = −1` propagation
  produces non-zero `P[BG_Z, OMEGA]`, so the standard Kalman update
  for **any** non-gyro observation (RPM, kinematic-bicycle steering,
  NHC) leaks into the gyro bias state. Over a 15 s sustained corner
  `bg_z` walked ±1.8 °/s, integrating into 27° of mid-lap yaw error.
  Fix is per-correction Schmidt-Kalman partitioning — explicitly
  zero `K[i]` for every state not legitimately informed by the
  observation, then use Joseph form
  `P = (I-K·H)·P·(I-K·H)ᵀ + K·R·Kᵀ` for the covariance update so P
  stays symmetric and PSD under the modified gain. NHC is now
  always-on with a slip-aware sigma (tight 0.10 m/s when
  `!slip_flag`, loose 0.50 m/s when `slip_flag`) — previously it
  was gated off entirely during slip, letting `vy` run unbounded
  for half of every autocross lap. Net signal-side result:
  `/odom` yaw-rate offset on straights drops from +0.526 °/s to
  +0.046 °/s; peak `/odom`-vs-GT body-frame position drift drops
  from 56 m to 1.4 m; SLAM cone-DA cascade events are gone in the
  cascade-pulse scan.
- **`sigma_bg_walk` 1e-5 → 1e-4** (#539). One-line tune of the
  gyro-bias random-walk process noise. Loosened to let `bg_z` track
  in-run drift instead of frozen-bias semantics. Subsumed for
  practical purposes by #555's Schmidt-Kalman partitioning but
  kept for the case where bias really does drift physically.

### Added (tooling)

- **`tools/refresh-bridge.sh` stale-volume guard** (#548). The
  development workflow rebuilds the `dv_pipeline_stack` image and
  recreates the container — but compose-watch's named source
  volumes persist across both, leading to a silent failure where
  source edits were rebuilt but the running container still saw
  the previous content. Fixed in two parts: drop the named source
  volumes after `compose build`, and verify post-recreate that the
  container's source matches the host (sha256, three canary files
  in odometry_filter + cone_slam). Aborts non-zero if any canary
  doesn't match.
- **`tools/replay.sh` fix-up** (#547). Stale executable names
  (`ros2 run slam Cone_Detection` etc.) updated to the post-#530-
  revert names (`cone_detection cone_detection_node`,
  `cone_slam slam_node`, `path_planning path_planning_node`,
  `control control_node`). Added explicit lifecycle transitions
  since `mode_manager` doesn't run in replay. Topic rename
  `/cone_slam/state` → `/slam/pose` (per #382) applied to the
  comparator and recorder topic lists.
- **Lichtblick layouts: integrated yaw drift + per-axis SLAM-vs-GT
  plots** (#537). Two new userNodes / panels in `clean_bag_replay`
  and `slam_debug` for diagnosing the yaw-integration and per-axis
  drift modes addressed by #555.

### Reverted

- **`slam: two-phase rewrite (#530)` reverted in #531.** The
  architecture couldn't be validated end-to-end on bag `_211619`
  before the EKF-stack work landed (`/odom` drift made the
  rewrite's cone DA cascade indistinguishable from baseline). The
  legacy `cone_graph_slam_node` entry-point retained on dev as the
  shipping SLAM. The two-phase rewrite branch is preserved under
  `origin/feat/slam-two-phase-rewrite` for future revival once
  `/odom` is healthy enough to validate the architecture in
  isolation.

### Fixed

- **mc-backend: stream bag tarball to disk in auto-pull; fix UI
  badge** (#528). Large bags would OOM `mc_backend` when
  `container.get_archive` was buffered in-memory. Now streamed to
  a temp file with chunked extraction. UI badge for "Bag pulled
  to host" now reflects the actual extraction result instead of
  going stale on streaming failures.

### Planned for v0.1.3

- **Cone-floor clipping fix.** Spline-spawned cones currently sit a
  few cm into the asphalt on every track because the spline control
  points are at world Z=+100 but the ground mesh's local-bbox top
  sits a few cm higher. Diagnosis + per-probe evidence in
  `docs/cone_floor_clipping_fix.md`; fix is a line-trace ground-snap
  in the `spline_cones*` BP construction script (~1 h editor work).
  Visual + LiDAR-perf impact (bottom rings of close cones get
  occluded by the floor mesh). Deferred from v0.1.0 / v0.1.1 /
  v0.1.2 because the autonomy logic isn't affected. Tracking: #483.
- **Adaptive cone DA / cascade hardening.** PR #555's signal-side
  cleanup made `/odom` an honest pose source, but the late-corner
  cone-DA still cascades when the car re-enters a region with cones
  visible from a different angle (autocross is open-line, not a
  closed loop — there is no loop closure to lean on here). Pose
  drifts ~1 m past the 1.0 m Euclidean DA gate, all observations
  look new, cascade-skip-recovery dumps duplicate landmarks into
  the persistent map. Experimental branches were explored this
  release cycle (Mahalanobis DA, cascade percentage-gate
  tightening, per-scan new-landmark rate cap); none reliably
  helped on the validation bag. Real fix likely combines a tighter
  proximity-veto envelope that scales with recent pose uncertainty
  plus a cap on new-landmark commits per scan.

## [0.1.1] — 2026-05-14

Small follow-up to v0.1.0 covering one new operator-facing feature
(record MCAP bags from Mission Control), one autonomy observability
improvement (granular lifecycle progress for the StartMission action),
and a meaningful Docker performance win on Windows + macOS hosts.

### Added

- **Record bag on session start** (#486, closes #465). New "Record
  bag (mcap)" checkbox in Mission Control's EventSetup. When ticked,
  the autonomy bring-up triggers a `ros2 bag record -s mcap -a`
  process inside `dv_pipeline_stack` (so it shares the SHM-tuned DDS
  context with the publishers — recording mc_backend-side dropped
  ~96 % of `/lidar/Lidar1` scans because mc_backend forces
  `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for cross-container action calls).
  The recorder lives in a new `pipeline/bag_recorder_node/` colcon
  package and exposes `/bag_recorder/{start,stop}` services
  (`dv_msgs/srv/{StartBag,StopBag}`). Live state surfaces through
  `/api/referee/state` (`bag_name`, `bag_state`, `bag_path`,
  `bag_error`). Stops automatically on session end; the mcap is
  closed and renamed on the SIGINT path.

- **Granular lifecycle progress** (#489, closes #387). `mode_manager`
  now publishes a `/mode_manager/progress` topic emitting one message
  per change_state transition across the autonomy fan-out
  (cone_detection, slam_node, path_planning, control). Mission
  Control's session-start spinner can now show per-node progress
  ("activating cone_detection_node…") instead of the previous opaque
  "bringing up autonomy" wait. Pre-#489 the long
  cone_detection_node configure step (numba JIT, ~10-20 s) looked
  indistinguishable from a hang.

### Changed

- **Docker bag flow: bind mount → named volume** (#490 Part 1).
  `/bags` inside `dv_pipeline_stack` was a host bind mount to
  `./bags/`; on macOS Docker Desktop (virtiofs) and Windows + WSL2
  (9p), the cross-fs `shutil.move` from the in-container `/tmp`
  staging dir to the host-side `/bags` took 20-40 s for multi-GB
  bags, blocking the StopBag service callback and visibly stalling
  the session-stop click. Now a named docker volume `ifssim_bags`
  keeps the move on the container's local ext4; finalisation is
  <1 s. To pull a finalised bag onto the host:

      tools/list-bags.sh                # enumerate bags in the volume
      tools/pull-bag.sh <bag_name>      # docker cp it onto ./bags/<name>/

  `docker cp` uses Docker Desktop's vmcompute stdio pipe (not the
  bind-mount layer) so even the explicit pull beats the implicit
  move it replaced.

- **Source bind mounts removed + `.dockerignore`** (#490 Part 2).
  Pre-#490 every ROS package under `pipeline/` and `ros2/src/` was
  bind-mounted into `dv_pipeline_stack`'s workspace, plus
  `fastdds_profile.xml` and `tools/random-track-generator`. On
  Docker Desktop those traversed 9p / virtiofs on every Python
  import → 30-90 s container startup on Windows. Worse, `docker
  compose build` shipped the full 33 GB working tree (bags + UE5
  artifacts + .git) as build context every time.

  Fix: source is COPY'd into the image at build time (no runtime
  bind-mount); a new repo-wide `.dockerignore` cuts the build
  context to ~200 MB. To pick up a source edit:

      tools/refresh-bridge.sh   # = docker compose build + up -d --force-recreate

  Performance impact reported in PR #490 (smoke-tested macOS, cache-hot):

  - `docker compose build dv_pipeline_stack`: **24 s** (was minutes)
  - `docker compose build mission_control_backend`: **5 s**

  Bind mount **kept**: `./Content/tracks:/tracks` on `mc_backend`,
  because UE5 (the host process, not in a container) reads track
  CSVs from there via the loadTrack RPC — both sides need the same
  physical files.

  `DV_REBUILD_ON_STARTUP` env flag deprecated. It used to trigger
  a `colcon build` against the bind-mounted source on container
  start; not needed any more.

### Notes

- The `random-track-generator` submodule must be initialised
  (`git submodule update --init --recursive tools/random-track-generator`)
  on a clean clone. Pre-#490 the empty bind-mount silently hid the
  issue. Now `mission_control_backend`'s build fails-soft: the COPY
  succeeds with an empty dir and track-generation endpoints 500 at
  runtime.

- For maximum Windows perf, clone the repo inside WSL2's native ext4
  (`~/IFSSIM`) instead of `/mnt/c/Users/<user>/IFSSIM`. Every Docker
  filesystem operation then runs at native Linux speed. No code change
  required — just a `git clone` in a different place.

## [0.1.0] — 2026-05-14 (first release)

### Added

- **/odom Phase 3 — steering_angle + brake_pressure filter inputs**
  (#383). Bridge publishes two new sensor topics at 100 Hz:
  `/fsds/steering_angle` (radians, converted from the plugin's
  normalized [-1, 1] axis via `max_steering_angle_rad`) and
  `/fsds/brake_pressure` (normalized [0, 1] from the controls echo).
  `sim_supervisor_node.OdometryFilter` consumes both:
  kinematic-bicycle yaw cross-check (`ω_pred = (vx/L)·tan(δ)`) for
  slip detection; brake-event α_vx scaling (collapses RPM correction
  weight when brake authority > 30 %) for slip-aware longitudinal
  tracking. Three new diagnostic topics surface the cross-check
  state: `/odom_diag/yaw_residual_rad_s`, `/odom_diag/slip_flag`,
  `/odom_diag/effective_alpha_vx`. 6 new unit tests covering yaw
  residual / brake α-scaling / reset semantics.

### Changed

- **/odom Phase 2 — REP-105 TF restructure** (#382). `sim_supervisor_node`
  now owns `odom→base_link` (broadcast at the 100 Hz filter publish
  rate); `slam_node` stops broadcasting that edge and instead
  publishes `map→odom` as the dynamic drift-correction transform
  (computed at each scan tick as `slam_pose ⊖ latest /odom`, ~10 Hz).
  The chain `map → odom → base_link` resolves to SLAM's absolute
  pose at the leaf regardless of supervisor's dead-reckoning drift
  between SLAM ticks. `/Conos` and `/Path` migrate from `odom` to
  `map` frame; `path_planning_node`'s TF lookup becomes
  `map→base_link`; `/cone_slam/state` is renamed to `/slam/pose`
  (with frame_id `map`) so the topic name doesn't lock us into the
  current iSAM2 backend. Pure-Python math in
  `pipeline/cone_slam/cone_slam/tf_math.py` (`compute_map_to_odom`),
  11 unit tests covering identity, translation/rotation drift,
  yaw-wrap, and the round-trip identity `T_map_odom · T_odom_base ==
  T_map_base`. `/tf_static` is no longer used by the autonomy stack.


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

#### Windows-port + release-prep series (2026-05-13 → 2026-05-14)

- **LiDAR UDP port out of Windows' dynamic range** (#477). Moved the
  legacy chunked-UDP LiDAR port from 51453 → 41500. 51453 lives
  inside Windows' dynamic / ephemeral port range (49152–65535) where
  Docker Desktop's UDP forwarder silently drops bindings — `SendTo`
  returns `ok=1, bytesSent=8032` while zero packets reach the
  container. 41500 is below 49152 AND non-adjacent to 41452 (the
  latter avoids a macOS Docker-Desktop UDP-range-proxy quirk on
  contiguous ports). Same fix applies to both platforms.
- **Plugin Windows-runtime fixes** (#478). Two paired changes guarded
  by `#if PLATFORM_WINDOWS`: `timeBeginPeriod(1)` in
  `FFSDSPluginModule::StartupModule` drops the Windows scheduler tick
  from 15.6 ms → 1 ms so `FPlatformProcess::Sleep` sub-2ms calls
  actually achieve their target cadence; `FSocket::SetNoDelay(true)`
  on per-client RPC sockets kills TCP Nagle so the 400 Hz sensor
  stream doesn't get coalesced into 40-200 ms batches on Win64.
  Brought `/imu` from 12 Hz → 430 Hz on Windows. Mac/Linux unaffected.
- **Windows build tooling** (#479). New `package_windows.sh` mirroring
  `package_mac.sh`: RunUAT.bat-driven Win64 Shipping cook, stages
  tracks + settings.json + GameUserSettings.ini, rewrites
  UECommandLine.txt with absolute -project + windowed + ResX/ResY +
  -ForceRes + FPS cap. New `Config/DefaultGameUserSettings.ini` seeds
  `FullscreenMode=2` (windowed, 1280×720) so a fresh cook doesn't land
  in WindowedFullscreen on first run. `.gitattributes` LF rules for
  `Dockerfile`, `docker-compose*.yml`, `settings.json`, `Default*.ini`
  so Windows clones with `core.autocrlf=true` don't bake CRLF into
  shipping images (the `lichtblick` Caddy port `8080\r` regression
  that bit during testing).
- **Reset button restores start-gate orientation** (#480). `/api/sim/reset`
  now reads the plugin's authoritative start-gate pose via the
  `getStartGatePose` RPC (same data `loadTrack` aligns the car to)
  and teleports with the full 7-tuple `(x, y, z, qw, qx, qy, qz)`.
  Previously dropped the captured quaternion and used position-only
  teleport, leaving the pawn at the spawn XY with whatever yaw it
  had at reset time. Verified: post-reset pose matches post-loadTrack
  pose to 1 mm position / 1.0 quat-dot product.
- **LiDAR TCP transport** (#482). Moved the LiDAR wire path from
  chunked UDP (introduced in #322) to streaming TCP on the existing
  RPC port. New `FFSDSRpcServer::StreamLidar` method mirrors the
  sensor-stream pattern; bridge gains `lidarStreamThread()` that
  reads a fixed 24-byte stream header + total_points×16-byte
  payload. Solves two cross-platform wedge modes the chunked-UDP
  path could not avoid: Docker Desktop's userspace UDP proxy losing
  port bindings under sustained fragmented load (#286, observable
  on Mac without the host-networking toggle AND on Windows
  regardless of toggle), and WSL2 kernel UDP `rcvbuf` overflow
  dropping 28 % of LiDAR chunks on Windows. The original #322
  rationale (~7 MB/s macOS Docker Desktop TCP loopback cap) was
  fixed upstream over the 6+ Docker Desktop releases since then.
  Env-var fallback `IFSSIM_LIDAR_TRANSPORT=udp` preserves the
  chunked-UDP path. Sim's UDP broadcaster is gated off by default —
  bridges using the UDP fallback toggle it on via the new
  `enableLidarUdpBroadcast` / `disableLidarUdpBroadcast` RPCs.
  Verified on Mac (10.021 Hz steady) and Windows (10.0–10.4 Hz over
  sustained 30 s, ~95 k pts/scan).
- **`package_windows.sh` + `package_mac.sh` produce versioned zips**
  (this PR). Both scripts read `ProjectVersion` from
  `Config/DefaultGame.ini` and produce
  `dist/IFSSIM-v<X.Y.Z>-{Mac,Windows-x64}.zip` after staging.
  `dist/` is gitignored. Skip with `SKIP_ARCHIVE=1` during iterative
  cooking.

---

The above accumulator (Added / Changed since v0.1.0 scaffolding) and
the scaffolded feature list below both ship in this same v0.1.0 tag.
Reader can think of v0.1.0 as "everything since the first commit of
IFSSIM as a public project, up to 2026-05-14". Treating it as one
release rather than splitting into 0.0.x increments keeps the SemVer
history honest — the project hadn't been tagged before today.

### Added (scaffolded feature list)

The first cut intended for use outside the ISC Racing Team's internal
dev loop. The sim is feature-complete for FS-DV mission practice; the
items in **Known limitations** below are real but not blockers for
that use case. See `docs/QUICKSTART.md` for first-run instructions.

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
  request/response, sensor stream, and (post-#482) LiDAR stream
  all multiplexed on this single port.
- **UDP broadcaster** on 41452 (sensor frames) + 41500 (LiDAR
  chunks, fallback only). Sensor UDP fanout is benign-but-unused
  at runtime (sensors ride the TCP RPC push). LiDAR UDP is the
  legacy path retained as an escape hatch via
  `IFSSIM_LIDAR_TRANSPORT=udp` on the bridge — production runs
  use TCP because UDP's chunked-fragmentation path has unreliable
  proxy behaviour on every Docker Desktop backend (#286, #482).
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
  and `UECommandLine.txt` patched for windowed mode + 30 FPS cap.
  Zips into `dist/IFSSIM-v<X.Y.Z>-Mac.zip` at release time.
- **`package_windows.sh`** — Win64 Shipping, runs from Git Bash with
  RunUAT.bat, requires a Windows host with VS 2022 (UE5 cannot
  cross-compile to Windows from any other platform). Same set of
  side-effects as the Mac script: tracks/, settings.json,
  GameUserSettings.ini, UECommandLine.txt with absolute -project +
  windowed + ResX/ResY + FPS cap. Zips into
  `dist/IFSSIM-v<X.Y.Z>-Windows-x64.zip` at release time.

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
  Live LiDAR verified at the Hesai datasheet rate (10 Hz).
- **Windows 10/11** with UE5 5.7 — first-class. Verified end-to-end
  on Docker Desktop (WSL2 backend): live LiDAR + sensor stream both
  at nominal rates, Lichtblick reachable, autonomy pipeline drives.
- **Linux** — not in scope for v0.1.0. The build scripts can be
  adapted (RunUAT supports Linux Shipping cooks) but the release
  pipeline isn't wired up and the platform isn't on the tested
  matrix. Tracked for a future release.

[Unreleased]: https://github.com/isc-fs/IFSSIM/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/isc-fs/IFSSIM/releases/tag/v0.1.0
