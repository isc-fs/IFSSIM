# DV Pipeline Rebuild — Planning Doc

**Status:** draft, opened 2026-04-26 by Raul Moran (with Claude as scribe)
**Successor to:** `fix/49-stanley-clean-baseline` (closed in PR #119, the keepers commit)
**Driving issue:** the autonomy pipeline never completes a clean autocross lap (DOO=0) on the simulator. Five-run sweeps under iterated tuning consistently fail at or shortly after the first turn. Root cause traced not to a single bug but to a foundation that is sim-only by construction: ground-truth pose, ground-speed sensor, and a feature-mapping layer that all silently depend on UE5's perfect telemetry. Real-car deployment has none of these.

## 0. Pivot 2026-04-27 — from dense scan-match SLAM to cone-graph SLAM

**The original plan in §3 specified a LiDAR-IMU dense scan-matching SLAM** (GLIM-anchored). After implementation we determined that this whole class of SLAM (GLIM, FAST-LIO, fast_LIMO, KISS-ICP, …) is **structurally wrong for our environment.** Three rounds of integration, each replacing the previous:

1. **GLIM** — diverged at vehicle speeds (76% drift, OOM under load). Aborted 2026-04-26.
2. **FAST_LIO_ROS2** (Ericsii fork) — yaw direction inverted vs. ground truth on every drive (`feat(slam): swap GLIM → FAST_LIO_ROS2`, then realized fundamental issue). Aborted 2026-04-27 morning.
3. **fast_LIMO** (fetty31, ros2-v2.1.0) — survived an "easy" drive at 1.05× distance ratio after extensive tuning, but **catastrophically diverges through every sharp turn** on a slow controlled drive: world-frame flips 180°, position error grows to 60-130 m over 45 s of motion, IKFoM oscillates between two scan-match local minima.

The audit (3 parallel investigations: fast_LIMO source, upstream config comparison, repo issues) made the diagnosis unambiguous: **dense scan-matching SLAM assumes geometrically rich environments** (walls, vegetation, signage) where the scan is over-constrained against the prior map. **FS Driverless tracks are cone-only by competition rules** — sparse landmarks with rotational symmetry — and we can't add features to fix it. The fast_LIMO author's own Issue #13 thread confirms this: FSDS+fast_LIMO drift cannot be fully fixed by timestamps or tuning; fast_LIMO assumes scan-match richness we don't have.

**Real winning DV teams (AMZ, MIT, MUR, Edinburgh) do not use FAST-LIO style SLAM.** They use cone-association graph SLAM with explicit per-cone data association, IMU preintegration factors, and (often) GPS factors. AMZ's documented architecture is FastSLAM 2.0 with color-aware association.

**New plan (replaces §3.2 sensor data path and §4 PR #3 SLAM swap):**

- Build a **GTSAM-based cone-graph SLAM node** — pose nodes, IMU preintegration factors, cone landmark nodes with `BearingRangeFactor`, GPS factors, iSAM2 incremental backend.
- Add the missing **data-association layer** to `pipeline/slam/` (current cone detection re-detects fresh every scan with no cone-id persistence; see audit notes in `Publicar_Mapa`/`actualizar_mapa`).
- Keep cone detection (`Cone_Detection`), color classification (spatial Y-sign + cache), big-orange height separation — those work and don't need to change.
- Drop fast_LIMO entirely. Branch `feat/28-cone-graph-slam` (renamed from `feat/28-glim-localization` to reflect the actual direction).

**What §3 below still applies:**
- §3.1 TF tree convention (`map → odom → base_link`) — same.
- §3.2 base_link rename across the pipeline — already shipped in commit `5ef63f7`, keep.
- §4 PR #4–#6 (motor RPM, cone map slim, controller retune) — unchanged, just unblocked by the new SLAM rather than fast_LIMO.

**Sections §3.2 SLAM box, §4 PR #3, and §5 Open decision #1 are now historical** — read for context, but the answers are: *we tried, those approaches don't work for us, see §0 for what replaces them.*

## 1. Why a rebuild

The 2026-04-26 audit catalogued every "ideal" leak in the pipeline. The findings are not isolated bugs — they're a foundation problem. Specifically:

- **Localization** is `Odometria_perfecta` republishing `/fsds/testing_only/odom` as the `odom→fsds/FSCar` TF. Every downstream module (SLAM map accumulator, path planner, control's lateral feedback) silently consumes that TF.
- **Velocity** is `/fsds/gss` (Ground-Speed Sensor) — a sim-only convenience. Real IFS-08 has no equivalent.
- **Frame names** are sim-specific (`fsds/FSCar`) — the real car needs ROS-standard `base_link` for parity.
- **The cone map's coordinate consistency** is therefore guaranteed only because the underlying TF is a perfect oracle. Replace it with real SLAM and the map jitters; classification cache built today only papers over that.

The tactical fixes in `fix/49` (color cache, stop-approach-cap gate, gate-miss diagnostics) are real and ship — but they only get us a sim that *might* lap clean. They don't get us anywhere on the real car.

This doc plans the foundation swap: GLIM-anchored localization, motor-RPM-derived velocity, ROS-standard frame tree, slimmed cone map. The pipeline emerges as one that runs identically in sim and on the IFS-08, with only the sensor source layer differing.

## 2. Real-car constraints

The IFS-08 carries:

| Sensor | Topic on real car | Sim equivalent | Notes |
|---|---|---|---|
| Hesai ATX 128-ch LiDAR | `/lidar/Lidar1` | `/fsds/lidar/Lidar1` (modeled, 10 Hz, 200k pts/s) | matched |
| BMI088 IMU | `/imu` | `/fsds/imu` (modeled, 400 Hz, with bias drift) | matched |
| GPS | `/gps` | `/fsds/gps` (modeled, 10 Hz, NavSatFix) | matched |
| Inverter motor RPM (CAN) | `/motor_rpm` | needs new bridge publish — sim has it via `getCarState` rpm field | **gap to close in PR #4** |

The IFS-08 does **NOT** carry:
- Ground-speed Doppler sensor (GSS) — the sim's `/fsds/gss` has no analogue.
- Cameras — vision-based perception is out of scope (memo `project_no_cameras_on_real_car.md`). Cone classification stays position-based.
- Wheel-speed encoders — not assured. The bridge will publish motor RPM only; if the real car ends up with wheel-speed too, that's a future addition.

## 3. Architecture target

### TF tree (sim and real, identical)

```
map ─→ odom ─→ base_link ─→ {fsds/Lidar, fsds/IMU, fsds/GPS, ...}
 │              │
 │              └ owned by GLIM (LiDAR-IMU SLAM, 10 Hz)
 │
 └ map fixed-to-world; GLIM may update with loop closure
```

- `map` is the loop-closed global frame.
- `odom` is the dead-reckoning frame; `odom→base_link` updates at IMU rate, no jumps.
- `base_link` replaces `fsds/FSCar` everywhere. Renaming this is the riskiest single change in the rebuild — every TF lookup, every static transform, every Foxglove layout.
- The bridge publishes static transforms `base_link → fsds/Lidar`, `base_link → fsds/IMU`, etc. — same in sim and real. Source of truth: `settings.json` for sim, real-car CAD for hardware.

### Sensor data path (real car identical to sim)

```
LiDAR ─┐
       ├─→ GLIM ─→ /odom (Twist + Pose), /map ─→ TF: map→odom→base_link
IMU ───┘             │
                     │
LiDAR ─→ Cone_Detection ─→ /Conos_raw (vehicle frame)
                                 │
                                 v
                          ConeMap (uses GLIM TF)
                                 │
                                 v
                              /Conos
                                 │
                                 v
                         Plan_Path (uses GLIM TF)
                                 │
                                 v
                              /Path
                                 │
                                 v
Motor RPM ─→ velocity_estimator ─→ Control ─→ /control_command
GLIM /odom ────────────────────────┘ (cross-check)
```

### Node graph: before and after

| Today (`fix/49` baseline) | After rebuild |
|---|---|
| `Odometria_perfecta` (GT) | **GONE** — replaced by GLIM |
| `Odometria` (GSS-fused, unused on dev?) | **GONE** — GSS-dependent |
| `Cone_Detection` | unchanged |
| `Publicar_Mapa` + `mapa.py` (570 lines, accumulator + chain-walker + sign-of-y override + cache) | **REWRITTEN** as ~150-line `ConeMap` against GLIM frame |
| `Publicar_Track` (GT-only viz) | **GONE** — debug-only, can run via `ros2 topic echo` if needed |
| `Plan_Path` | wrapper trimmed to drop TF-via-`fsds/FSCar`; vendored `fsd_path_planning` library unchanged |
| `Control` | TF lookups switch to `base_link`; velocity feedback switches to `/motor_rpm`-derived; iteration noise from this session pruned |
| **NEW: `velocity_estimator`** | tiny node: `v = (rpm/60) × (wheel_circumference/gear_ratio)`, publishes `/velocity` |
| **NEW: `glim_ros2`** | from `koide3/glim`, configured for our sensor set |

Net: **2 new nodes (GLIM + velocity_estimator), 3 nodes removed** (Odometria_perfecta, Odometria, Publicar_Track), 1 rewritten (Publicar_Mapa). Cone_Detection unchanged.

### Velocity sourcing

Primary: `v = (rpm / 60) × (2π × WheelRadius / GearRatio)`.
- IFS-08 settings: `WheelRadius = 0.228 m`, `GearRatio = 2.909` → wheel circumference 1.432 m → `v_m_per_s = rpm / 60 × 1.432 / 2.909 = rpm × 0.00821`.
- At motor 6000 RPM: v = 49.3 m/s. Far above any FSD scenario; no rate concern.
- Latency: motor RPM is on CAN at 100 Hz on the real car; the sim bridge will publish at the same rate. ~10 ms latency end-to-end.

Cross-check: GLIM's `/odom` Twist `linear.x`. Sanity rule (in `velocity_estimator`): if RPM-derived and GLIM-derived diverge by > 20% over a 1 s window, log a warning. Not a hard fault — wheel slip on launch *should* show divergence and that's information, not a bug.

Stale RPM handling: if `/motor_rpm` goes stale (>0.5 s), fall back to GLIM Twist with a one-tick log.

## 4. PR sequencing

Six PRs, each independently mergeable to `dev`. PRs #2–#6 target a single integration branch (`feat/N-dv-pipeline-rebuild`) so the work-in-progress can be reviewed continuously. The naming below uses `feat/N` as a placeholder; actual branch numbers come from the GitHub Actions auto-issue workflow.

### PR #1 — Close `fix/49` (the keepers)

**Branch:** `fix/49-stanley-clean-baseline` (already exists)
**Target:** `dev`
**Scope:** ship the bug fixes that were validated this session, drop the iteration noise.

Files to land:
- `pipeline/slam/slam/slam.py` — color cache (persistent classification across `actualizar_mapa` rebuilds)
- `pipeline/control/control/velocity_control.py` — `_stop_approach_cap` gate-on-orange (was killing v_tgt for any short perceived path)
- `pipeline/control/control/control.py` — `is_stop_area` plumbing for the gate
- `pipeline/path_planning/path_planning/path_planning.py` — `GATE_MISS` + `PATH_RATE` per-second diagnostics, `MIN_AHEAD_PER_SIDE` 2 → 1 (post-cache the threshold can be tighter)
- `pipeline/control/control/controlador_stanley.py` — already cleaned of K_us debris

**Out of this PR:** every K_us / TTL fallback / Stanley speed-gate-lift change has been reverted. Do not re-introduce in this PR.

**Validation:** smoke test — pipeline starts, autonomy enables, `COLOR_CACHE` shows ≥95% hit rate within 5 s of motion. Sim does not have to lap clean for this PR; that's PR #6's bar.

### PR #2 — Rename `ros_stack` → `dv_pipeline_stack`

**Branch:** `feat/N-rename-dv-pipeline-stack`
**Target:** `dev`
**Scope:** mechanical rename, behavior-neutral.

Files touched:
- `docker/ros_stack/` directory → `docker/dv_pipeline_stack/`
- `docker-compose.yml` — service, container, image, build context, volume mounts
- `docker/dv_pipeline_stack/Dockerfile` — `WORKDIR /ros_stack_ws` → `WORKDIR /dv_pipeline_stack_ws`, all `COPY` paths updated
- `docker/dv_pipeline_stack/{bridge.launch.py, pipeline_only.launch.py, entrypoint.sh, teleop.sh, teleop_keyboard.py}` — paths, log strings
- `tools/mission_control/backend/main.py` — wherever it references `ros_stack` container name (TBD; needs a grep)
- `docs/` — any references in existing docs
- Memory entries (`project_glim_next.md`, etc.) — update path refs

**Validation:** `docker compose build && docker compose up -d` succeeds, pipeline starts, autonomy commands flow end-to-end. No behavior change in any controller / planner / SLAM logic. PR diff should be 100% renames + path updates.

### PR #3 — LiDAR-IMU SLAM integration + frame swap

**Branch:** `feat/N-glim-localization` → reused for FAST-LIO2 after the GLIM pivot (2026-04-27); will be renamed at PR-merge time.
**Target:** `dev`
**Scope:** the foundational swap.

**Library choice:** **FAST-LIO ROS2** (Ericsii fork) — pivoted from GLIM after step 6 validation showed GLIM cannot track our sim's LiDAR-IMU stream at sustained vehicle speeds. See `docs/glim_integration.md` §15 (post-mortem) for the full story. FAST-LIO2 has dramatically lighter deps (PCL + Eigen, no GTSAM), is known to run real-time on Raspberry Pi-class CPUs, and per the design doc fallback path was already pre-approved as the GLIM substitute.

Files touched:
- `docker/dv_pipeline_stack/Dockerfile` — install GLIM dependencies (GTSAM, gtsam_points, Eigen, nanoflann), build `glim_ros2`
- `docker/dv_pipeline_stack/pipeline_only.launch.py` — spawn `glim_ros2` node, drop `Odometria_perfecta` and `Publicar_Track` nodes
- `pipeline/odometria/` — **DELETE** (the GT-driven and GSS-fused odom nodes both go away)
- `pipeline/slam/slam/slam.py` — TF lookups switch from `fsds/FSCar` to `base_link`
- `pipeline/path_planning/path_planning/path_planning.py` — same TF rename
- `pipeline/control/control/control.py` — same TF rename, drop `/fsds/testing_only/odom` subscription
- `ros2/src/ifssim_bridge/` — drop `odom→fsds/FSCar` TF publish (GLIM owns it now); publish static `base_link→fsds/Lidar`, `base_link→fsds/IMU` instead; remove `/testing_only/odom` and `/testing_only/track` outside competition mode (or keep them off by default for a debug toggle)
- `Foxglove` layouts in `docs/` — update frame_id references

**GLIM config:** start from `koide3/glim`'s default `glim_ros2.yaml`, point its LiDAR input at `/lidar/Lidar1` and IMU at `/imu`. Tune frame names so its output is `map → odom → base_link`.

**Validation criterion:** with the sim at standstill, GLIM's pose matches `/fsds/testing_only/odom` (still published in dev mode for ground truth) within ±1 cm. After driving a full autocross lap, drift between GLIM's pose and GT < 10 cm.

**Risk:** GLIM build inside Docker. GTSAM is a heavy C++ dep; first build might take 15-30 min. Pin GTSAM version (4.2a9 per koide3 docs).

### PR #4 — Motor-RPM velocity

**Branch:** `feat/N-motor-rpm-velocity`
**Target:** `dev`
**Depends on:** PR #3 (frame swap; not strictly required but cleaner to land after)

Files touched:
- `ros2/src/ifssim_bridge/` — new periodic publisher of `/motor_rpm` (Float32 or std_msgs/Int32) at 100 Hz, sourced from `getCarState`'s `rpm` field
- `pipeline/control/` — new `velocity_estimator.py` node (~50 lines) computing `v = rpm × constant`, publishing `/velocity` (geometry_msgs/Twist or std_msgs/Float32)
- `pipeline/control/control/control.py` — drop `/fsds/gss` subscription, subscribe to `/velocity` instead. Add the GLIM-Twist cross-check (warn-only)
- Bridge: drop `/fsds/gss` publish in non-debug mode (keep behind `competition_mode = false` flag for sim regression)

**Validation:** RPM-derived velocity within 5% of GLIM Twist on a constant-speed straight (manual teleop @ ~5 m/s). At launch (wheel slip), divergence > 20% triggers the cross-check warn — that's correct behavior, not a bug.

### PR #5 — Slim the cone map

**Branch:** `feat/N-cone-map-slim`
**Target:** `dev`
**Depends on:** PR #3 (uses GLIM frame)

Files touched:
- `pipeline/slam/slam/mapa.py` — rewrite. New design: world-frame dict keyed by snapped position (the same snap-radius approach as today's color cache, just unified). DBSCAN goes away (cone_detection already clusters per-frame; the world-frame dedup is enough).
- `pipeline/slam/slam/slam.py` — `Publicar_Mapa` becomes a thin wrapper that transforms detections to map frame and updates the dict. Color cache logic merges in.
- Drop `Publicar_Track`, `BenchMark`.

**Validation:** with GLIM running, cone positions in `/Conos` should be stable (drift < 5 cm tick-to-tick) over a full lap. Color hit rate stays ≥95% throughout the lap. Module size ≤ 200 lines (vs. 570 today).

### PR #6 — Controller cleanup + retune

**Branch:** `feat/N-controller-retune`
**Target:** `dev`
**Depends on:** PR #4 (velocity source) and PR #5 (clean cone map)

Files touched:
- `pipeline/control/control/control.py` — strip the iteration noise (long comment blocks documenting failed K_us / FF / speed-gate experiments). Keep the operative gains as ROS params with their current values; expand comments only on still-load-bearing decisions.
- `pipeline/control/control/controlador_stanley.py` — already trimmed in PR #1; review for any lingering pre-K_us comments
- `pipeline/control/control/velocity_control.py` — same comment-noise pass
- New behavioral changes: NONE in code; this is the retuning PR. Tune `control_gain`, `softening_gain`, `max_normal_acceleration` against a clean drive with PR #1–#5 in place.

**Validation: this is the bar that lets the rebuild ship.** Three consecutive autocross laps with `DOO = 0`, `OC = 0`, `finished = true`. If we can't hit that, something earlier in the chain is still wrong and we go back, not forward.

## 5. Open decisions

1. **GLIM IMU rate.** Bridge publishes IMU at 400 Hz; GLIM examples are usually 100-200 Hz. Worth a quick benchmark in PR #3: does GLIM track the 400 Hz stream cleanly, or does it want downsampled? Decision deferred to PR #3 prototyping.

2. **Frame `base_link` placement.** Standard ROS convention puts `base_link` at the rear-axle midpoint (REP-103). Current `fsds/FSCar` pose is at the vehicle CG (per the Chaos plugin). In PR #3, decide: rename and shift origin to rear axle (more standard, but requires updating sensor offsets in `settings.json`), or rename and keep at CG (less work, less standard). Probably the latter for sim parity.

3. **`/testing_only/*` topics in competition mode.** Currently bridge gates them on `competition_mode`. After the rebuild, are they ever needed? Proposal: keep them publishable for sim regression suites, default off, never wired into the autonomy pipeline.

4. **Mission Control container references.** Need to grep for hardcoded `ros_stack` strings in the MC backend before PR #2. If MC reaches into the container by name (not just by API), the rename ripples there too.

5. **Velocity estimator ownership.** Inline a tiny estimator inside `control.py`, or stand up a separate `velocity_estimator` node? Separate node is cleaner ROS-style and lets the controller stay focused on path-following. Probably a separate node, ~50 lines.

## 6. Out of scope

- **Vision-based cone classification.** No cameras on the real car (`project_no_cameras_on_real_car.md`). Position-based classification + cache stays.
- **Loop-closure–driven map correction.** GLIM provides this internally; we don't surface it as a separate ROS node.
- **DOO penalty model retuning.** Out of scope for the rebuild; FS rules govern.
- **Acceleration / Skidpad event-specific behaviors.** PR #6 validates against autocross only; the trackdrive / skidpad / acceleration suite is its own follow-up.
- **Real-car bring-up.** This rebuild produces a sim that *can* run on the real car. Bringing it up on hardware (CAN integration, motor RPM topic from inverter, real GPS / IMU calibration) is a separate workstream.

## 7. Rollback strategy

Each PR is independently revertable. The riskiest is PR #3 (GLIM + frame swap) — if GLIM doesn't behave, revert that PR alone and the pipeline returns to `Odometria_perfecta`-driven sim mode. PRs #4–#6 only land after #3 is stable, so they don't need their own rollback paths beyond standard `git revert`.

If the entire rebuild proves wrong-tree (e.g., GLIM can't track the IFS-08 LiDAR cleanly even after tuning), the fallback is: keep `Odometria_perfecta` for sim, accept that the real-car deployment story needs a different SLAM library (FAST-LIO, LIO-SAM, Cartographer). PR #1's keepers ship regardless.
