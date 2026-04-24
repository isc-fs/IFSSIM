# fix/10-control-reset — Remaining Issues

Status as of 2026-04-18. Car can be launched from Mission Control (Start Session), pipeline runs end-to-end, but the vehicle does not drive correctly yet.

---

## Update — 2026-04-25

The list below has been partially addressed by fix/26 → fix/37. Updated status:

| # | Item | Status | PR / Note |
|---|------|--------|-----------|
| 1 | Stanley/velocity controller tuning | **deferred** | Pipeline-internal — explicitly out of scope for the sim/plugin/bridge audit |
| 2 | SLAM map quality under motion | **deferred** | Pipeline-internal |
| 3 | Path planner heading | **deferred** | Pipeline-internal |
| 4 | Skidpad / acceleration events untested | **deferred** | Tracked in `project_followups.md` (Acceleration finish-detection) |
| 5 | Lap counting / OC at spawn | **fixed** | #80 (fix/29) — OC suppressed until car moves past spawn; laps echo correct |
| 6 | UE5 reconnect — control commands dropped during gap | **partially fixed** | #84 (fix/31) added truncation warning + robust `finished` parse; bridge graceful-fallback on disconnect still TODO |
| 7 | Lichtblick "dots to the left" — TF mismatch | **fixed** | #92 (fix/35) — bridge now queries `getSensorOffset` from plugin instead of hardcoding stale values |
| 8 | No integration tests | **still open** | No CI runner for the UE side; deferred |

Additionally, the following sim/plugin/bridge gaps were caught during the audit and shipped in the same window (not in the original list):

- Plugin memory-safety: 3 UAFs in RPC server + LiDAR sensor (#81)
- Camera Z axis polarity + IMU noise unit harmonisation to SI (#86)
- seg/plot stubs return errors instead of silently lying (#90)
- MC API-key auth + tightened CORS default (#94)
- MC telemetry WS regressions + Scoring tab null guard (#95)

Original list below preserved for reference.

---

## What works

- **Mission Control one-button flow**: Start Session → stops pipeline → activates RES → sets event/laps → resumes sim → releases RES → starts pipeline automatically.
- **Path planner**: numba cache bug patched (`IndexDataCacheFile.save` swallows `TypeError`), `circle_fit` guarded for collinear cones (det ≈ 0), cone/path guards added (`< 2 cones → return`, `path None or < 2 points → return`).
- **SLAM TF lookup**: switched from stale 30 ms-offset timestamp to `rclpy.time.Time()` (latest available).
- **Sensor TFs**: LiDAR and camera offsets corrected to REP-103 (forward = +X).
- **GSS velocity axes**: `GssVelX = LinearVelocity.X` (forward), `GssVelY = -LinearVelocity.Y` (left-positive).
- **ENU ↔ UE5 quaternion conversion**: `UEQuatToENU = Q90 * UE.Inverse()`.
- **Reset**: position-only teleport (no quaternion); avoids the 90° orientation bug.
- **Stanley steering gate**: steering = 0 when `|velocity| < 0.5 m/s` to prevent spinout on start.
- **Control topic fix**: control node now publishes to `/control_command` (was `/fsds/control_command`, which the bridge never received).

---

## Known remaining issues

### 1. Car does not follow the path correctly
The control loop runs and commands reach UE5 now, but the vehicle does not stay on track. Root causes to investigate:

- **Stanley controller gains**: `control_gain=4.0`, `softening_gain=6.0` may be too aggressive or too weak for the current wheelbase/speed. Needs tuning with the car actually moving.
- **Velocity target too low or too high**: `max_speed=10 m/s`, feedforward from curvature — first real test needed to see if the car reaches a reasonable speed.
- **EMA smoothing factor**: raised from 0.05 → 0.3 in this session; may need further tuning. At 0.05 the first ~60 ticks produced ~0 throttle; at 0.3 it ramps faster but may still be too conservative.
- **Cross-track error penalty** (`Fg * e²`): with `Fg=1.0` and `crosstrack_term` subtracted from target speed, a large initial error could zero the throttle command.

### 2. SLAM map quality unknown under real motion
SLAM was tested with the car stationary. Under autonomous driving:
- Cone association and loop closure have not been validated.
- TF accuracy between `odom` and `map` frames under motion is untested.
- If SLAM drifts, path planner gets garbage cones and publishes a bad path.

### 3. Path planner heading computation
`fsd_path_planning` generates path poses, but the yaw computed for each pose has not been validated against the actual track direction. If the path headings are wrong, Stanley's cross-track correction will steer the wrong way.

### 4. Skidpad / acceleration events untested
Only `trackdrive` has been tested (partially). `skidpad` requires a figure-eight path, `acceleration` a straight-line run — neither has been validated with the pipeline.

### 5. Lap counting and finish detection
`required_laps` is set via Mission Control but the `finished` flag logic and lap counter in the sim have not been tested under autonomous driving. DOO (detection of orange cone) and OC (off-course) penalties are displayed but not validated.

### 6. UE5 reconnect stability
When UE5 is closed and reopened quickly the bridge disconnects and reconnects. This works now, but if the pipeline is running during a reconnect, control commands are dropped for several seconds. No graceful fallback (e.g. zero throttle + brake) is applied during the gap.

### 7. Lichtblick "dots to the left" of car model
In the Lichtblick visualiser the car model appears offset/rotated from the SLAM trajectory. Likely a TF frame convention mismatch between the `base_link` → `fsds/FSCar` static TF and the model orientation. Cosmetic but makes SLAM debugging harder.

### 8. No integration tests
All fixes were validated manually by observing Mission Control telemetry and Docker logs. No automated tests exist for:
- Bridge topic flow (sensor data in → control commands out)
- Path planner output sanity (given known cone positions)
- Control node response to a known path + pose input

---

## Suggested next steps (priority order)

1. **Tune velocity controller**: start with `max_speed=3 m/s`, increase `smoothing_factor` to 0.5 or remove EMA entirely, then observe actual throttle values in Mission Control.
2. **Validate Stanley gains**: log `crosstrack_error` and `limited_steering_angle` in real driving; adjust `control_gain` until the car tracks without oscillation.
3. **Validate SLAM under motion**: drive manually (teleop) one lap, inspect the Lichtblick map — check that cones accumulate correctly and `odom → map` drift is acceptable.
4. **Fix Lichtblick car model orientation**: check the static TF from `base_link` to `fsds/FSCar` and align with the model's forward axis.
5. **Graceful reconnect**: add a watchdog in the bridge that sends `throttle=0, brake=1` when the TCP/UDP connection is lost, and resumes normal control on reconnect.
6. **Add skidpad / acceleration support**: validate path planner output for those event types.
