# IFSSIM — Functionalities Reference

> **ISC Racing Team** | Unreal Engine 5.7 · Chaos Physics · ROS 2 · Python

This document is a comprehensive technical reference for all systems, components, and interfaces in IFSSIM.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Simulation Engine](#2-simulation-engine)
3. [Vehicle Model](#3-vehicle-model)
4. [Sensor Suite](#4-sensor-suite)
5. [RPC Server (TCP API)](#5-rpc-server-tcp-api)
6. [Data Streaming (UDP Push)](#6-data-streaming-udp-push)
7. [ROS 2 Bridge](#7-ros-2-bridge)
8. [Python Client](#8-python-client)
9. [Track System](#9-track-system)
10. [Referee & Competition System](#10-referee--competition-system)
11. [Mission Control](#11-mission-control)
12. [Configuration (settings.json)](#12-configuration-settingsjson)
13. [Coordinate System](#13-coordinate-system)
14. [Autonomy Pipeline](#14-autonomy-pipeline)

---

## 1. Architecture Overview

IFSSIM is a Formula Student Driverless simulator built on Unreal Engine 5.7. The system is layered into four levels of abstraction:

```
┌─────────────────────────────────────────────────┐
│              Unreal Engine 5.7                  │
│  FSDSPlugin (C++)  ·  Chaos Physics  ·  Maps   │
├─────────────────────────────────────────────────┤
│         Networking Layer (port 41451)           │
│    TCP RPC Server  ·  UDP Push Broadcaster     │
├─────────────────────────────────────────────────┤
│              Client Interfaces                  │
│  ROS 2 Bridge  ·  Python Client  ·  REST API   │
├─────────────────────────────────────────────────┤
│             Mission Control                     │
│    FastAPI Backend  ·  React Frontend          │
└─────────────────────────────────────────────────┘
```

**Key processes and ports:**

| Component | Protocol | Port |
|---|---|---|
| TCP RPC Server | TCP text/binary | 41451 |
| UDP Sensor Stream | UDP binary push | 41452 |
| UDP LiDAR Stream | UDP binary push | 41453 |
| Mission Control API | HTTP/WebSocket | 8000 |
| Mission Control UI | HTTP | 3000 |

---

## 2. Simulation Engine

### Unreal Engine 5.7 + Chaos Physics

IFSSIM runs on **Unreal Engine 5.7** using the **Chaos physics engine** (UE5's native physics system, replacing the legacy PhysX). The project is structured as:

- **Game Module:** `Source/Blocks/` — minimal UE5 game module, entry point
- **Plugin:** `Plugins/FSDSPlugin/` — all simulation logic lives here
- **Maps:** `Content/` — `.umap` files for each competition discipline

### Maps

| Map | Description |
|---|---|
| `Acceleration.umap` | Straight-line acceleration run |
| `Skidpad.umap` | Figure-8 skidpad layout |
| `TrainingMap.umap` | Open training area |
| `customMap.umap` | Runtime-loadable custom track |

### Game Mode

`AFSDSGameMode` orchestrates startup: it spawns the vehicle pawn, referee actor, cone spawner, RPC server (port 41451), and UDP broadcaster in sequence. Settings are loaded from `settings.json` before any actors are created.

### Clock Speed

The simulation clock speed is configurable via `settings.json` (`ClockSpeed` field). Set to values below 1.0 for slow-motion testing.

---

## 3. Vehicle Model

### IFS-08 Formula Student Car

The default vehicle (`FSCar`) is modelled after ISC Racing Team's IFS-08 car. All physics parameters are defined in `settings.json` under `VehiclePhysics` and applied at runtime via `AFSDSVehiclePawn`. Compile-time defaults live in `Plugins/FSDSPlugin/Source/FSDSPlugin/Public/FSDSSettings.h` (`FFSDSVehiclePhysics`).

#### Mass, drivetrain, geometry

| Parameter | Value | Unit | Source |
|---|---|---|---|
| Mass | 275 | kg | `Mass` |
| Drivetrain | RWD | — | `Drivetrain` |
| Wheel radius | 0.228 | m | `WheelRadius` |
| Wheel width | 0.190 | m | `WheelWidth` |
| Max steer angle | 28 | ° | `MaxSteerAngle` |
| Wheelbase | 1.627 | m | `Wheelbase` |
| Front track | 1.220 | m | `TrackFront` |
| Rear track | 1.190 | m | `TrackRear` |
| Weight distribution front | 43.8 | % | `WeightDistFront` |
| Centre of gravity height | 0.300 | m | `CoGHeight` |

> **Note on CoG:** runtime `CenterOfMassOverride` is applied (-10 mm in X) but the `FormulaMesh_PhysicsAsset` ships with a forward-biased authored CoM that runtime overrides cannot fully correct (`BodyInstance.COMNudge` saturates non-monotonically). The load-transfer RPC reports the actual Chaos values so autonomy code can consume relative wheel loads correctly even when the absolute distribution is biased.

#### Powertrain

The drive torque comes from a **dedicated `UEmraxMotor` class** modelled after the IFS-08's actual motor: an **EMRAX 228 MV** (LC variant) running at a 400 V bus. Chaos's built-in ICE-style engine is set up at construction (so the wheel solver has a non-zero engine to talk to) but **silenced at runtime** — `SetThrottleInput(0)` is held every tick and the per-wheel drive torque is injected via `SetDriveTorque` with an Additive combine. The Chaos engine never produces drive torque; the EMRAX class is the single source of truth for the motoring side.

What this means for `settings.json`: **the `MotorMaxTorque`, `MotorMaxPower`, `MotorRPM[]`, and `MotorTorque[]` fields are vestigial.** They feed the Chaos engine that's bypassed; tweaking them changes nothing observable on the drive side. The only motor-related fields that still influence behaviour are the regen caps (`MaxRegenTorque`, `MaxRegenPower`), which the pawn forwards to `Motor->P.MaxRegenTorqueNm`/`MaxRegenPowerW`. Everything else lives in `FEmraxMotorParams` defaults in `Plugins/FSDSPlugin/Source/FSDSPlugin/Public/EmraxMotor.h`.

##### EMRAX 228 MV / LC datasheet parameters (class defaults)

| Parameter | Value | Unit | Source |
|---|---|---|---|
| Pole pairs | 10 | — | `PolePairs` |
| Rotor inertia | 0.02521 | kg·m² | `RotorInertia` |
| Motor mass (informational) | 13.5 | kg | `MassKg` |
| Hard mechanical speed limit | 6,500 | RPM | `MaxMechRpm` |
| Torque constant | 0.61 | Nm/A_RMS | `KtNmPerArms` |
| Peak power cap (NX-tech LUT @ 400 V) | 100,000 | W | `MaxPeakPowerW` |
| Peak torque (S2 2-min, datasheet) | 220 | Nm | `MaxPeakTorqueNm` |
| Continuous torque (S1, LC cooling) | 130 | Nm | `ContTorqueNm` |
| Continuous power (S1, LC cooling) | 75,000 | W | `ContPowerW` |
| Current-loop time constant | 0.0015 | s | `CurrentLoopTau` |
| Gear ratio | 2.909 | — | `GearRatio` (settings.json) |
| Drivetrain efficiency | 0.92 | — | `DrivetrainEfficiency` (settings.json) |

##### Peak-torque envelope (motor frame, before gear reduction)

Hardcoded in `EmraxMotor.cpp` as a 64-knot lookup with linear interpolation matching the EMRAX 228 MV NX-tech file's 103-RPM grid:

| RPM | 0 – 4532 | 4635 | 4944 | 5253 | 5562 | 5871 | 6180 | 6489 |
|---|---|---|---|---|---|---|---|---|
| Peak Nm | 220 | 219 | 209 | 199 | 187 | 177 | 164 | 154 |

Constant 220 Nm in the constant-torque region; field-weakening rolloff above ~4,600 RPM. Per tick, the envelope value is further capped by the constant-power boundary `T = min(T_envelope, P_max / ω)` so the 100 kW power limit binds in the field-weakening region as well.

##### Regen (generating) regime

Regen torque is gated by the **battery cell-input current limit**, not by the motor's mechanical envelope. The EMRAX can dump well over 100 kW into a load, but the IFS-08 accumulator (~140 cells) faults out at a few kW of total charge power.

| Parameter | Value | Unit | Source |
|---|---|---|---|
| Max regen power | 6,000 | W | `MaxRegenPower` (settings.json) → `Motor->P.MaxRegenPowerW` |
| Max regen torque | 230 | Nm | `MaxRegenTorque` (settings.json) → `Motor->P.MaxRegenTorqueNm` |

At 6,500 RPM (≈680 rad/s), `MaxRegenPowerW=6000` yields ~8.8 Nm of available regen — the power cap binds at all but the lowest speeds.

The implementation includes a **single-quadrant regen guard**: regen torque is forced to zero whenever the wheel isn't rotating forward. Asking for regen at `ω ≤ 0` would (a) be electrically unsafe on the real inverter, and (b) produce reverse motor torque on a stationary wheel — driving the car backward from rest, which the real IFS-08 controller cannot do.

##### Inverter dynamics, idle creep, thermal derate

- **Current-loop lag.** First-order discrete with `α = clamp(dt/τ, 0, 1)` and `τ = 1.5 ms`. At a 1 kHz physics tick the response is ~5 ticks to settle.
- **Idle creep** (`IdleCreepTorqueNm`, off-throttle floor while `|RPM| < IdleCreepRpmThreshold`). Defaults to **0 Nm**, i.e. disabled — real EMRAX has no idle. The non-zero default that previously lived here was a Chaos workaround for the wheel solver freezing at the `ω=0`, `v=0` degenerate state; that's now fixed upstream by `SleepThreshold=0` in `SetupVehicleMovement`, so the workaround is no longer needed and the previously observed ~0.83 m/s creep on EBS-release went away.
- **I²t thermal derate.** Class fields and budget exist (`OverloadBudgetJ=1e6 J`, `CoolingRateW=8000 W`) but the derate is **currently held at 1.0 (no derate)** in `Step()`. The previous derate path was clamping launch torque below the static-friction-lock threshold and blocking autonomous launches; it'll be reintroduced once the HV-battery class lands and the model has a proper coolant + winding-temp signal rather than the I²t proxy.

##### Brakes

The IFS-08 has **no hydraulic service brake** — retarding torque on the drive (rear) wheels is motor regen only. Front wheels have `MaxBrakeTorque = 0`, retarded only by aero drag and tire scrub. EBS is pneumatic on all four corners and routed through the Chaos handbrake channel (`bAffectedByHandbrake = true` on both wheel classes).

#### Tire model — Pacejka Magic Formula '96

The legacy `FrictionForceMultiplier` flat-mu model on the wheel classes is overridden at `BeginPlay` by `FSDSPacejka::BakeToWheel`, which computes the Chaos `LateralSlipGraph` and `LongitudinalSlipGraph` from a Pacejka MF96 fit. Coefficients (in `settings.json` under `VehiclePhysics.Pacejka`):

| Coefficient | Lateral | Longitudinal |
|---|---|---|
| Stiffness factor B | 10.0 | 12.0 |
| Shape factor C | 1.9 | 1.7 |
| Curvature factor E | -1.5 | -0.5 |

Peak friction `μ_peak` is set globally by `TireMu = 1.4` (unitless, identical front and rear).

The class-default `FrictionForceMultiplier = 1.65` on `FSDSWheelFront` / `FSDSWheelRear` is the cold-start fallback before `BakeToWheel` runs and is never the operating-state value.

#### Suspension and roll dynamics

| Parameter | Value | Unit | Source |
|---|---|---|---|
| Suspension damping ratio | 1.5 | — | `SuspensionDamping` |
| Roll centre height, front | 0.040 | m | `RollCenterFront` |
| Roll centre height, rear | 0.060 | m | `RollCenterRear` |
| Roll stiffness, front | 27,000 | Nm/rad | `RollStiffnessFront` |
| Roll stiffness, rear | 22,000 | Nm/rad | `RollStiffnessRear` |
| Heave stiffness | 227,600 | N/m | `HeaveStiffness` |
| Pitch stiffness | 155,600 | Nm/rad | `PitchStiffness` |

Consumed by `AFSDSVehiclePawn::ComputeTireLoadsParametric` for the parametric load-transfer model (lateral + longitudinal weight transfer routed to per-wheel Fz inputs of the Pacejka fit).

#### Wheels

- **Front (`UFSDSWheelFront`):** Hoosier R20 16.0×7.5-10 (200 mm class radius — overridden by `WheelRadius=0.228 m` from settings at runtime), 19 mm width, max steer 28°, no service brake (`MaxBrakeTorque=0`), pneumatic EBS via handbrake channel.
- **Rear (`UFSDSWheelRear`):** Same tire, no steering, drive wheels (motor torque injected via `SetDriveTorque` with `Additive` combine method), `MaxBrakeTorque` sized at `BeginPlay` to `MaxRegenTorque · GearRatio · DrivetrainEfficiency / 2` per wheel so the brake input channel saturates correctly against the regen ceiling.

#### Aerodynamics

Applied per-tick using `AddForce` on the vehicle mesh:
- **Drag:** `F_drag = 0.5 · ρ · CdA · v²` (opposing velocity), `CdA = 0.9 m²`.
- **Downforce:** `F_down = 0.5 · ρ · ClA · v²` distributed front/rear per `AeroBalanceFront`, `ClA = 1.8 m²`, `AeroBalanceFront = 0.45`.

Air density ρ = 1.225 kg/m³.

#### Vehicle control inputs

```
throttle  ∈ [-1.0, 1.0]   (negative = reverse)
steering  ∈ [-1.0, 1.0]   (negative = left)
brake     ∈ [0.0, 1.0]    (regen demand, capped by MaxRegenTorque + MaxRegenPower)
```

The bridge translates `fs_msgs/ControlCommand{throttle, steering, brake}` directly to `setCarControls`. The brake channel is regen demand; the EBS is its own latched signal (`/signal/ebs`).

#### API control vs manual

When `enableApiControl` is called, keyboard/gamepad input is disabled and the vehicle responds only to `setCarControls` commands. Can be toggled at runtime. The vehicle also boots in EBS-engaged state (mirroring the FS-DV `AS_Off` rule) — the controller must publish `/signal/ebs_reset` (latched) to release it before commands take effect.

---

## 4. Sensor Suite

All sensors are attached to the vehicle pawn and configured via `settings.json`. Sensor positions and rotations are defined in ENU meters relative to the vehicle origin.

### 4.1 LiDAR

**Implementation:** `FSDSLidarSensor` — modelled after the **Hesai ATX_S01** (1-D rotating mirror, hybrid solid-state). Ray-casting runs on the **GPU**: three SceneCaptures share view geometry — depth (`SCS_SceneDepth`), base colour (`SCS_BaseColor`), world-space normal (`SCS_Normal`) — and a compute shader decodes them per ray into a point cloud with per-point intensity. Selected by the `LidarPath` setting (`"gpu"` is the production default; a `"cpu"` fallback using `LineTraceSingleByChannel` parallelised over channels is kept for reference and parity testing).

| Parameter | Default | Configurable | Source |
|---|---|---|---|
| Channels | 116 | ✓ | `NumberOfChannels` |
| Points per second | 1,740,000 | ✓ | `PointsPerSecond` |
| Rotations per second | 10 Hz | ✓ | `RotationsPerSecond` |
| Vertical FOV upper | +5.9° | ✓ | `VerticalFOVUpper` |
| Vertical FOV lower | −12.4° | ✓ | `VerticalFOVLower` |
| Horizontal FOV | −60° to +60° (120°) | ✓ | `HorizontalFOVStart/End` |
| Max range (global) | 30 m | ✓ | `MaxRange` |
| Per-channel max range | 25–30 m (datasheet shape) | ✓ | `PerChannelMaxRangeM` |
| Range noise std (Gaussian) | 0.03 m | ✓ | `RangeNoiseStd` |
| Point dropout rate | 1% | ✓ | `DropoutRate` |
| Mount position (X, Y, Z) | (0.0, 0.0, 1.10) m | ✓ | `X`, `Y`, `Z` |
| Ray-cast back-end | `gpu` | ✓ | `LidarPath` |

**Per-channel max range.** Real Hesai ATX_S01 has per-beam laser-power variance (datasheet App. A.1.1) — outer rings reach less far than central beams (60 m bottom, 198 m centre, 90 m top on the real sensor). The sim uses a 116-element array linearly remapped from the datasheet's 60-198 m envelope into the 25–30 m simulator range budget, preserving the relative shape. Length must equal `NumberOfChannels`; mismatch logs a warning and falls back to the global `MaxRange`. Honoured by both back-ends.

**Why max range = 30 m, not the datasheet 200 m:** cones past 30 m don't matter for FS detection (max corridor width is well under that), and shrinking the per-ray broadphase cuts per-ray Chaos work by roughly 7×. Bump back to 200 for benchmarks against real-car captures.

**Mount.** `(X=0.0, Y=0.0, Z=1.10)` corresponds to the IFS-08's main hoop crossbar height, CoG-aligned, centred. Pre-2026 settings used a hood-mount approximation (`X=0.5, Z=0.9`); cone-detection cluster-height thresholds were rebaselined when the mount moved.

**Wire format.** Point cloud is output as a flat `float[]` array in sensor-local frame (X forward, Y left, Z up — UE5/ENU). Each point is 4 floats `(x, y, z, intensity)` — xyz in metres, intensity in [0, 1]. Sensor packs only emit *hits*, so the wire rate is roughly 65% of attempted rays (e.g. ~1.14 M valid returns at 1.74 M attempted).

**Intensity model (#255).** Mirrors the Hesai ATX-S01 working principle:
```
intensity = ρ_905 × cos(θ_inc) × (R_ref / range)²
```
- **ρ_905** — surface reflectance at 905 nm. Sourced from the `BaseColor` capture's Rec.709 luminance as a placeholder; per-cone-material 905 nm reflectance values (blue ≈ 0.15, yellow ≈ 0.50, orange ≈ 0.65, white-stripe ≈ 0.92) are a follow-up content task.
- **cos(θ_inc)** — angle of incidence between the ray and the world-space surface normal at the hit, sampled from the `Normal` capture (GPU path) or `FHitResult::ImpactNormal` (CPU path).
- **(R_ref / range)²** — Lambert inverse-square term, normalised so a perpendicular surface at `R_ref = 1 m` returns the unmodified reflectance.
- Output clamped to [0, 1]. The CPU fallback uses a placeholder ρ = 0.5 (no per-material lookup).

**Noise model.** Gaussian range noise applied per point in metres. Independent Bernoulli dropout per point.

### 4.2 IMU

**Implementation:** `FSDSImuSensor` — 6-DOF inertial measurement in body frame, parameterised for a **BMI088** (the unit on the real IFS-08).

| Parameter | Default | Configurable | Source |
|---|---|---|---|
| Accelerometer noise std (white, Gaussian) | 0.024 m/s² | ✓ | `AccelNoiseStd` |
| Gyroscope noise std (white, Gaussian) | 0.0035 rad/s | ✓ | `GyroNoiseStd` |
| Accelerometer bias steady-state σ | 0.01 m/s² | ✓ | `AccelBiasStd` |
| Gyroscope bias steady-state σ | 0.0002 rad/s | ✓ | `GyroBiasStd` |
| Accelerometer bias correlation time τ | 100 s | ✓ | `AccelBiasTau` |
| Gyroscope bias correlation time τ | 100 s | ✓ | `GyroBiasTau` |

**BMI088 derivation.** Datasheet noise densities: accel `175 µg/√Hz`, gyro `0.014 °/s/√Hz`. At 400 Hz sample rate (200 Hz Nyquist BW) those become `~0.024 m/s²` and `~0.0035 rad/s` per-sample stddevs, matching the defaults above. Bench-test bias drift for a MEMS IMU at FS race-car operating temperatures is typically ~100 s correlation time.

**Noise model.** Gaussian white noise plus an **Ornstein–Uhlenbeck** bias process. The discrete update each tick is

```
bias[k+1] = bias[k]·exp(-Δt/τ) + σ·√(1 - exp(-2Δt/τ))·N(0,1)
```

so `*BiasStd` is the *long-run* steady-state stddev (the bound), not a drift rate. Earlier versions used a pure random walk (unbounded over long sessions) and `FRandRange(-1,1)` (uniform — gave 1/√3 ≈ 58% of the declared stddev).

**Outputs.** Linear acceleration (m/s²), angular velocity (rad/s), orientation quaternion — all in ENU body frame.

### 4.3 GPS / GNSS

**Implementation:** `FSDSGpsSensor` — converts UE5 world position to geodetic coordinates.

| Parameter | Default | Configurable |
|---|---|---|
| Position noise std (Gaussian) | 0.5 m | ✓ |
| Velocity noise std (Gaussian) | 0.1 m/s | ✓ |
| Reference origin | Map centre | — |

**Coordinate conversion:** ENU position → latitude/longitude using a flat-Earth approximation anchored to a configurable reference origin. Output is WGS84 latitude, longitude, altitude.

The bridge publishes `position_covariance.diag = σ²`. Earlier versions emitted noise from `FRandRange(-1,1)` (uniform), so actual stddev was 0.58× the declared value — the bridge over-stated GPS noise to consumers by √3.

### 4.4 Ground Speed Sensor (GSS)

**Implementation:** `FSDSGssSensor` — vehicle velocity in body frame, modelled after Kistler Correvit SFII.

| Parameter | Default | Configurable |
|---|---|---|
| Velocity noise std (Gaussian) | 0.02 m/s | ✓ |

**Outputs:** Linear velocity vector (vx, vy, vz) in vehicle body frame (ENU convention: X forward, Y left).

### 4.5 Barometer

**Implementation:** `FSDSBarometerSensor` — standard barometric altitude formula.

**Outputs:** Altitude (m), pressure (Pa), temperature (°C).

### 4.6 Magnetometer

**Implementation:** `FSDSMagnetometerSensor` — Earth's magnetic field vector projected into body frame.

**Outputs:** Magnetic field vector (mx, my, mz) in body frame (Tesla).

### 4.7 Distance Sensor

**Implementation:** `FSDSDistanceSensor` — single-ray line trace.

**Outputs:** Distance to nearest obstacle (m), min/max range limits.

### 4.8 Cameras

**Implementation:** `FSDSCameraSensor` — UE5 `USceneCaptureComponent2D`.

| Parameter | Default | Configurable |
|---|---|---|
| Resolution | 785 × 785 px | ✓ |
| FOV | 90° | ✓ |
| cam1 position | X=1.6 m, Z=0.2 m | ✓ |
| cam2 position | X=1.6 m, Y=−0.15 m, Z=0.2 m, Yaw=−10° | ✓ |

The default `settings.json` only configures `ImageType: 0` (Scene RGB) for both cameras; the other image types listed below are supported by the plugin (`EFSDSImageType` enum in `FSDSSettings.h`) and can be requested by adding additional `CaptureSettings` entries per camera.

**Supported image types:**

| Code | Type |
|---|---|
| 0 | Scene (RGB) |
| 1 | Depth Planner |
| 2 | Depth Perspective |
| 3 | Depth Visualisation |
| 4 | Disparity Normalised |
| 5 | Segmentation |
| 6 | Surface Normals |
| 7 | Infrared |

**Output format:** PNG-encoded bytes. Capture is dispatched to the game thread and synchronised with a 3-second timeout.

**Camera settings per image type:**
- Auto-exposure speed, bias, min/max brightness
- Motion blur amount (default 0 — disabled for simulation)
- Target gamma
- Orthographic projection mode with configurable ortho width
- Per-type visual noise (random grain, horizontal wave, distortion)

---

## 5. RPC Server (TCP API)

The simulator runs a **TCP server on port 41451** that accepts newline-terminated text commands and returns JSON or binary responses. Each client connection is handled in its own thread, supporting concurrent clients.

### Connection

```python
import socket
s = socket.socket()
s.connect(("127.0.0.1", 41451))
s.sendall(b"ping\n")
response = s.recv(1024)  # b"true"
```

### Full Command Reference

#### Connection & Control

| Command | Response | Description |
|---|---|---|
| `ping` | `true` | Heartbeat check |
| `enableApiControl` | `true` | Enable API mode, disable manual input |
| `isApiControlEnabled` | `true`/`false` | Query API control state |
| `getSettingsString` | JSON string | Full contents of settings.json |
| `reset` | `true` | Reload current level (full state reset) — note: the server-info advert no longer lists this command, but the handler is still wired up for legacy clients |
| `listCameras` | `["cam1","cam2",...]` | List configured camera names |
| `getSensorOffset <name>` | `x, y, z, roll, pitch, yaw` (m / rad) | Static body-frame offset of a sensor as configured in settings.json. The ROS 2 bridge calls this on init to publish accurate static TFs instead of hardcoding values. |

#### Vehicle State

| Command | Response fields | Description |
|---|---|---|
| `getCarState` | `speed, gear, rpm, maxrpm, x, y, z, vx, vy, vz, qw, qx, qy, qz` | Vehicle kinematics + drivetrain state |
| `getCarControls` | `throttle, steering, brake, handbrake, is_manual_gear, manual_gear, gear_immediate` | Last applied controls |
| `setCarControls <throttle> <steering> <brake>` | `true` | Apply control inputs |
| `simGetVehiclePose` | `x, y, z, qw, qx, qy, qz` | Vehicle pose in ENU metres |
| `simSetVehiclePose <x> <y> <z>` | `true` | Teleport vehicle to ENU position |
| `simGetGroundTruthKinematics` | `px, py, pz, vx, vy, vz, ax, ay, az, wx, wy, wz, qw, qx, qy, qz` | Full ground-truth kinematics (no sensor noise) |

#### Sensors

| Command | Response fields | Description |
|---|---|---|
| `getGpsData` | `lat, lon, alt` | GPS with noise |
| `getImuData` | `ax, ay, az, gx, gy, gz, qw, qx, qy, qz` | IMU with noise |
| `getGroundSpeedSensorData` | `vx, vy, vz` | GSS in body frame |
| `getLidarData` | `points, channels, range` | LiDAR metadata (use binary or UDP for point cloud) |
| `getDistanceSensorData` | `distance, min, max` | Distance sensor |
| `getBarometerData` | `altitude, pressure, temperature` | Barometer |
| `getMagnetometerData` | `mx, my, mz` | Magnetometer |

#### Camera

| Command | Response | Description |
|---|---|---|
| `simGetImage <cam_name> <type>` | `{"size":N,"camera":"cam1","type":0}` | Metadata; use binary for pixels |
| `simGetImageBinary <cam_name> <type>` | Binary: `IMG:<bytes>\n` + PNG data | Raw PNG bytes |

#### Simulation Control

| Command | Response | Description |
|---|---|---|
| `simPause` | `true` | Pause simulation |
| `simResume` | `true` | Resume simulation |
| `simIsPaused` | `true`/`false` | Query pause state |
| `simContinueForTime <seconds>` | `true` | Resume for N seconds then auto-pause |

#### Competition / Referee

| Command | Response fields | Description |
|---|---|---|
| `getRefereeState` | `doo_counter, oc_counter, cones, laps, required_laps, finished, event, lap_times[], cone_positions[]` | Full competition state |
| `setEvent <type> <laps>` | `true` | Set event type and required lap count |
| `loadTrack <filepath>` | `true` | Load a CSV track file at runtime |

#### Scene / Object API

| Command | Response | Description |
|---|---|---|
| `listSceneObjects [filter]` | `["ActorName",...]` | List all actors, optional name filter |
| `getObjectPose <name>` | `px, py, pz, qw, qx, qy, qz` | Get actor pose in ENU |
| `setObjectPose <name> <x> <y> <z>` | `true` | Teleport any scene actor |

#### Streaming Modes

Two commands open the connection in persistent streaming mode (no further requests on that socket):

| Command | Mode | Description |
|---|---|---|
| `streamSensors` | Push binary | GPS+IMU+GSS+Odom at ~100 Hz |
| `streamLidar` | Push binary | LiDAR point cloud at ~10 Hz |

---

## 6. Data Streaming (TCP push + UDP push)

High-frequency sensor data flows out of the plugin on dedicated streaming sockets — separate from the RPC server. The bridge can consume either transport per stream:

- **TCP push streams** opened by the plugin to a fixed bridge endpoint at startup. Reliable; bound by macOS Docker Desktop's TCP loopback throughput cap (~7 MB/s) which limits LiDAR throughput on Mac.
- **UDP push** when the bridge is launched with `lidar_transport:=udp` (or `LIDAR_TRANSPORT=udp` in `docker-compose`). Sender-paced; bypasses macOS Docker's TCP loopback throttle but accepts per-datagram packet loss.
- **Unix Domain Socket (UDS)** for LiDAR specifically, when the plugin can open `/tmp/ifssim_streams/lidar.sock` on a host where the path is shared into the container — bypasses the TCP stack entirely.

The two binary frame layouts are below. **Both halves of the wire — plugin sender and bridge receiver — must agree byte-for-byte.** The structs are `#pragma pack(push, 1)` on both sides; canonical definitions live in `Plugins/FSDSPlugin/Source/FSDSPlugin/Public/RPC/FSDSUdpBroadcaster.h` and `ros2/src/ifssim_bridge/include/udp_receiver.h`.

### Sensor Frame (port 41452, ~100 Hz)

Magic: `0x49465353` ("IFSS"). One unified packet carries everything the bridge fans out into separate `/imu`, `/gps`, `/gss`, `/testing_only/odom`, `/motor_rpm`, `/testing_only/extra_info`, and the controls echo.

```c
struct SensorFrame {
    uint32_t magic;                                 // 0x49465353
    uint32_t frame_id;
    uint64_t timestamp;                             // ns since plugin start

    // GPS
    double   latitude, longitude;                   // WGS84
    float    altitude;                              // m

    // IMU (body frame)
    float    accel_x, accel_y, accel_z;             // m/s²
    float    gyro_x, gyro_y, gyro_z;                // rad/s
    float    orient_x, orient_y, orient_z, orient_w; // quat (ENU body)

    // Ground-speed sensor (body frame, m/s)
    float    gss_vx, gss_vy, gss_vz;

    // Pose / odom (ENU world)
    float    pos_x, pos_y, pos_z;                   // m
    float    pose_qx, pose_qy, pose_qz, pose_qw;
    float    speed;                                 // m/s (signed)
    float    rpm;                                   // motor RPM, post-gearbox-side

    // Referee
    int32_t  doo_counter, oc_counter, lap_count;

    // Controls echo (most recent setCarControls input)
    float    throttle, steering, brake;
};
```

### LiDAR Frame (port 41453, ~10 Hz)

Magic: `0x4C494452` ("LIDR"). Each LiDAR scan is split into chunks; the bridge reassembles into a single point cloud frame.

```c
struct LidarChunkHeader {
    uint32_t magic;             // 0x4C494452
    uint16_t chunk_index;
    uint16_t total_chunks;
    uint32_t frame_id;
    int32_t  points_in_chunk;
    int32_t  total_points;
    int32_t  channels;
    int64_t  lag_ns;            // capture-to-send lag in nanoseconds
};
// Followed by: points_in_chunk × 4 × float32 (x, y, z, intensity) in sensor frame
// (UE5/ENU: X forward, Y left, Z up). UDP path pads to 500 points per chunk
// (500×16 + 24 ≈ 8024 B fits under macOS's 9216 B UDP datagram cap).
```

`lag_ns` is the time elapsed between `LidarSensor->LastTimestamp` (the actual capture instant) and packing time. The bridge subtracts it from `node_->now()` when stamping the ROS message, so `header.stamp` reflects the real capture moment regardless of GPU-readback latency (#238).

---

## 7. ROS 2 Bridge

The `ifssim_bridge` package (`ros2/src/ifssim_bridge/`) connects a ROS 2 stack to IFSSIM. It opens several connections to the plugin in parallel; transports per stream are picked at launch:

| Connection | Default transport | Configurable | Purpose |
|---|---|---|---|
| Sensor stream | TCP push (port 41452) | TCP / UDP | One unified `SensorFrame` carrying GPS + IMU + GSS + odom + RPM + referee + controls echo |
| LiDAR stream | TCP push (port 41453) | TCP / UDP / UDS | LiDAR point cloud chunks |
| Camera client | TCP req/resp (port 41451 RPC) | — | One image per timer tick per camera |
| Command client | TCP req/resp (port 41451 RPC) | — | `setCarControls`, EBS, track queries, settings |

`lidar_transport` is set via the launch parameter or `LIDAR_TRANSPORT` env var. Bridge logs a warning and falls back to `tcp` for unknown values.

### Published Topics

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `gps` | `sensor_msgs/NavSatFix` | ~100 Hz | `frame_id=fsds/GPS`. Covariance diag = `GpsPositionNoiseStd²`. |
| `imu` | `sensor_msgs/Imu` | ~100 Hz | `frame_id=fsds/IMU`. Covariances from `AccelNoiseStd`, `GyroNoiseStd`. |
| `gss` | `geometry_msgs/TwistWithCovarianceStamped` | ~100 Hz | `frame_id=fsds/GSS`. Body-frame velocity. |
| `motor_rpm` | `std_msgs/Float32` | ~100 Hz | Motor RPM (post-gearbox shaft side). |
| `lidar/Lidar1` | `sensor_msgs/PointCloud2` | up to 10 Hz | `frame_id=fsds/Lidar`. See section 4.1 for actual rates by transport. |
| `camera/<name>/compressed` | `sensor_msgs/CompressedImage` | `camera_hz` (default 10) | One topic per configured camera; PNG-compressed. |
| `tire_loads` | `std_msgs/Float32MultiArray` | ~100 Hz | Per-wheel Fz from `ComputeTireLoadsParametric`, order FL/FR/RL/RR. |
| `signal/go` | `fs_msgs/GoSignal` | 1 Hz | Mission name + track identifier. |
| `signal/finished` | `fs_msgs/FinishedSignal` | on event | Latched on the sim-side finish detection. |
| `testing_only/odom` | `nav_msgs/Odometry` | ~100 Hz | `frame_id=odom`, `child=base_link`. Ground-truth pose. *Hidden in competition mode.* |
| `testing_only/track` | `fs_msgs/Track` (latched) | 0.2 Hz | All cone positions. *Hidden in competition mode.* |
| `testing_only/extra_info` | `fs_msgs/ExtraInfo` | 1 Hz | DOO counter, OC counter, lap count. *Hidden in competition mode.* |

> **Topic naming.** The bridge today publishes bare names (no `/fsds/` prefix) and consumers remap or namespace as needed. The autonomy submodule expects a `/fsds/*` namespace per the integration contract in [`autonomy_pipeline.md`](autonomy_pipeline.md); the prefix renames are an open IFSSIM-side work item — until they land, consumers do the prefix on their side.

**Bridge TF behavior.** The bridge does not publish any dynamic TF. Sensor messages carry sensor-local frame_ids (`fsds/IMU`, `fsds/GPS`, `fsds/Lidar`; cameras use the FSDS-internal hierarchical name `fsds/FSCar/<cam_name>` in their image messages but it's not part of the live TF chain) — autonomy nodes consume the sensors directly without TF lookups. The bridge publishes a few static TFs at startup (`base_link → fsds/IMU`, `base_link → fsds/Lidar`, `base_link → fsds/GPS`) on `/tf_static`, populated from the corresponding `getSensorOffset` RPC. The live TF tree (`map → odom → base_link`) is published by `slam_node`; see [`autonomy_pipeline.md`](autonomy_pipeline.md). `/testing_only/odom` is for debugging only — the autonomy must not consume it.

> **`/testing_only/odom` is fully ground-truth.** Both pose and twist are sourced from the vehicle pawn's clean kinematics — no sensor in the loop, no GSS noise. The plugin packs a separate clean body-frame velocity field into `SensorFrame` (`GtVelBodyX/Y/Z`) and the bridge sources `twist.linear` from those fields with a tiny `1e-9` diagonal covariance. The `/gss` topic continues to carry the noisy GSS sensor model where the noise belongs.

### Subscribed Topics

| Topic | Type | Notes |
|---|---|---|
| `control_command` | `fs_msgs/ControlCommand` | Throttle, steering, brake → forwarded to sim via `setCarControls`. |
| `signal/ebs` | `std_msgs/Empty` (latched) | Trigger EBS via the plugin's emergency-brake RPC. |
| `signal/ebs_reset` | `std_msgs/Empty` (latched) | Clear the bridge's EBS-latched gate (otherwise the bridge silently drops `/control_command` until released). |

### Services

| Service | Type | Notes |
|---|---|---|
| `reset` | `fs_msgs/srv/Reset` | Full level reload (calls the plugin's `reset` RPC). |

### Custom Message Types (fs_msgs)

```
Cone.msg          — color (YELLOW=0, BLUE=1, ORANGE_BIG=2, ORANGE_SMALL=3, UNKNOWN=4) + location
Track.msg         — Cone[]
ControlCommand.msg — throttle, steering, brake (float64)
GoSignal.msg      — mission, track (string)
ExtraInfo.msg     — doo_counter, laps (uint32)
FinishedSignal.msg — (empty, signal only)
WheelStates.msg   — FL/FR/RL/RR: rpm, rotation_angle, steering_angle
Reset.srv         — response: bool success
```

### Launch

```bash
ros2 launch ifssim_bridge ifssim_bridge.launch.py \
    host:=localhost \
    port:=41451 \
    mission_name:=trackdrive \
    track_name:=A \
    competition_mode:=false \
    camera_hz:=10
```

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `host` | `localhost` | Simulator IP |
| `port` | `41451` | RPC port |
| `timeout` | `5.0` | Connection timeout (s) |
| `mission_name` | `trackdrive` | Published on `signal/go` |
| `track_name` | `A` | Published on `signal/go` |
| `competition_mode` | `false` | Hides ground-truth topics |
| `camera_hz` | `0` | Camera publish rate (0 = disabled) |

### Build

```bash
cd ros2
colcon build --symlink-install
source install/setup.bash
```

### Noise Covariances

The bridge reads sensor noise parameters directly from `getSettingsString` on startup and populates ROS message covariance matrices accordingly:
- GPS: diagonal `σ_pos²` on position covariance
- IMU: separate diagonal covariances for `angular_velocity` and `linear_acceleration`
- GSS: diagonal `σ_vel²` on twist covariance

---

## 8. Python Client

The `ifssim` Python package (`python/ifssim/`) provides a high-level API compatible with the original FSDS `fsds.FSDSClient`.

### Installation

```bash
cd python
pip install -e .
```

### Usage

```python
from ifssim import IFSSIMClient, CarControls, ImageType

client = IFSSIMClient(ip="127.0.0.1", port=41451)
client.confirmConnection()
client.enableApiControl(True)

# Read state
state = client.getCarState()
print(state.speed, state.kinematics_estimated.position)

# Control
controls = CarControls(throttle=0.5, steering=0.1, brake=0.0)
client.setCarControls(controls)

# Sensors
gps   = client.getGpsData()
imu   = client.getImuData()
gss   = client.getGroundSpeedSensorData()
lidar = client.getLidarData()          # .point_cloud = [x,y,z, x,y,z, ...]
img   = client.simGetImage("cam1", ImageType.Scene)  # PNG bytes

# Competition
ref = client.getRefereeState()
print(ref.doo_counter, ref.laps, ref.lap_times)

# Simulation control
client.simPause()
client.simContinueForTime(5.0)
client.reset()
```

### FSDS Compatibility

`FSDSClient` is an alias for `IFSSIMClient`. Code written against the original FSDS Python client runs without modification:

```python
from ifssim import FSDSClient   # drop-in alias
client = FSDSClient()
```

### Data Types

| Type | Fields |
|---|---|
| `CarState` | `speed, gear, rpm, maxrpm, kinematics_estimated, timestamp` |
| `KinematicsState` | `position, orientation, linear_velocity, angular_velocity, linear_acceleration` |
| `Vector3r` | `x_val, y_val, z_val` + arithmetic operators + `to_numpy_array()` |
| `Quaternionr` | `x_val, y_val, z_val, w_val` + `to_numpy_array()` |
| `CarControls` | `throttle, steering, brake, handbrake, is_manual_gear, manual_gear` + `set_throttle(val, forward)` |
| `LidarData` | `point_cloud: List[float]`, `time_stamp`, `pose` |
| `ImuData` | `linear_acceleration, angular_velocity, orientation, time_stamp` |
| `GpsData` | `gnss.geo_point.latitude/longitude/altitude, time_stamp` |
| `GroundSpeedSensorData` | `linear_velocity, time_stamp` |
| `RefereeState` | `doo_counter, oc_counter, laps, lap_times, required_laps, finished, event, cones[]` |
| `ConePosition` | `x, y, color` (YELLOW=0, BLUE=1, ORANGE_BIG=2, ORANGE_SMALL=3) |
| `ImageType` | `Scene=0, DepthPlanner=1, DepthPerspective=2, Segmentation=5, ...` |

---

## 9. Track System

### CSV Format

Tracks are defined as CSV files in `Content/tracks/`. Each row is one cone:

```
cone_type,x,y
blue,5.2,3.1
yellow,5.2,-3.1
big_orange,0.0,0.0
small_orange,0.0,1.5
```

Cone types: `blue`, `yellow`, `big_orange`, `small_orange`.
Coordinates are in ENU metres relative to track origin.

### Built-in Tracks

| File | Description |
|---|---|
| `acceleration.csv` | Standard acceleration straight |
| `skidpad.csv` | FSG skidpad figure-8 |
| `random_track.csv` | Example random layout |

### Runtime Track Loading

A running simulation can load any CSV track without restarting:

```python
client._text_cmd("loadTrack /absolute/path/to/track.csv")
```

The cone spawner reads the CSV, destroys existing cones, spawns new ones, and re-registers them with the referee.

### Random Track Generation

Requires the external `random-track-generator` repo (set `TRACK_GEN_PATH` env var). Uses Voronoi-based generation with configurable parameters:

| Parameter | Default | Description |
|---|---|---|
| `n_points` | 50 | Number of control points |
| `n_regions` | 30 | Voronoi regions |
| `max_bound` | 150 m | Maximum track extent |

---

## 10. Referee & Competition System

`AFSDSReferee` runs as a persistent actor that tracks the full competition state per session.

### Cone Hit Detection (DOO)

- Physics is enabled on all cones (Chaos rigid bodies, ~1 kg each)
- After a 1-second settle delay at level load, each cone's world position is snapped as its reference
- Every tick, all cones are checked for displacement from their reference position
- A displacement > **15 cm** increments the DOO (Deletion of Opportunity) counter
- Each cone is counted at most once

### Off-Track Detection (OC)

- Checked at 5 Hz (throttled) once the track has > 10 cones
- The car is considered on-track if it is within 6 m of both the nearest blue cone and the nearest yellow cone, AND the sum of those distances is < 9 m
- A transition from on-track → off-track increments the OC (Off-Course) counter
- Hysteresis: counter increments only on entry, not while staying off-track

### Finish Line

- Automatically derived from the positions of the first two `big_orange` cones
- A `BoxComponent` trigger is positioned and sized to span the line between them
- The trigger is 1 m deep, line-width + 1 m wide, 4 m tall

### Lap Timing

1. **First crossing:** starts the timer
2. **Subsequent crossings:** records lap time, resets timer for next lap
3. Minimum lap time: 1 s (acceleration), 3 s (all other events) — prevents false triggers
4. Debounce: the trigger does not re-fire while the vehicle remains inside the zone

### Event Types

| Event | Required Laps | DOO Penalty | OC Penalty |
|---|---|---|---|
| `trackdrive` | Configurable (default 10) | +2.0 s | +10.0 s |
| `autocross` | 1 | +2.0 s | +10.0 s |
| `acceleration` | 1 | +2.0 s | +10.0 s |
| `skidpad` | 4 (2 right + 2 left) | +0.2 s | — |

### Scoring Formulas (FSG 2024 Rules)

```
T_corrected = T_elapsed + DOO × penalty + OC × penalty
Score = max(0, MaxPoints × (1.5 × T_best / T_corrected − 0.5))
```

| Event | Max Points |
|---|---|
| Trackdrive | 200 |
| Autocross | 100 |
| Acceleration | 75 |
| Skidpad | 75 |

---

## 11. Mission Control

Mission Control is a web-based operator interface split into a **FastAPI backend** and a **React/TypeScript frontend**.

### Backend (FastAPI, port 8000)

#### Authentication (optional)

When the backend process has `IFSSIM_MC_API_KEY` set in its environment, every mutating REST call must include the header `X-API-Key: <value>`, and the WebSocket handshake must include `?api_key=<value>` as a query parameter. Calls without a valid key get HTTP 401 (or a 1008 close on the WS).

When the env var is **unset**, no key is required — convenient for solo dev. CORS is similarly env-driven via `IFSSIM_MC_CORS_ORIGINS` (defaults to `http://localhost:3000`).

The frontend stores the key in `localStorage` under `mc_api_key`; on the first 401 it prompts for the value and retries the call.

#### REST API

**Simulation:**

| Endpoint | Method | Description |
|---|---|---|
| `/api/sim/status` | GET | Connection state, map, FPS, pause state |
| `/api/sim/pause` | POST | Pause simulation |
| `/api/sim/resume` | POST | Resume simulation |
| `/api/sim/reset` | POST | Full level reload |

**Event Control:**

| Endpoint | Method | Description |
|---|---|---|
| `/api/event/state` | GET | Full referee state |
| `/api/event/set` | POST | Set event type and lap count |
| `/api/event/start` | POST | Start a session — sends `StartMission` to the autonomy lifecycle. Refuses with HTTP 400 if no track loaded (cones=0). |

The `/api/event/start` flow drives the autonomy lifecycle through the typed mission-management interface: the backend is an `rclpy` Action client of `sim_supervisor_node` and sends `StartMission` with the chosen mission. The supervisor relays to `mission_control_node`, which drives `mode_manager` to bring up the right lifecycle nodes with the right strategy flag. Phase 1 (startup) runs the heartbeat + JIT-warmup window; once it reports `ready`, Phase 2 begins and actuator commands start flowing through the supervisor to the bridge. Sim-only setup steps (track load, sim pause/resume, RES line) stay on the bridge JSON-RPC, called by the same backend in parallel. See [`autonomy_pipeline.md`](autonomy_pipeline.md) for the protocol details.

**RES (Remote Emergency Stop):**

| Endpoint | Method | Description |
|---|---|---|
| `/api/res/activate` | POST | Pulse RES line (pause sim + lock controls). Does not kill the autonomy pipeline |
| `/api/res/release` | POST | Release RES |
| `/api/res/status` | GET | Current RES state |

**Vehicle:**

| Endpoint | Method | Description |
|---|---|---|
| `/api/vehicle/state` | GET | Speed, RPM, gear, position, controls |
| `/api/vehicle/pose` | GET | ENU pose + orientation |
| `/api/vehicle/teleport` | POST `{x,y,z}` | Teleport vehicle |

**Track Management:**

| Endpoint | Method | Description |
|---|---|---|
| `/api/track/list` | GET | All CSV tracks with cone counts |
| `/api/track/{name}/preview` | GET | Matplotlib PNG preview (base64) |
| `/api/track/{name}/load` | POST | Load track into running simulation |
| `/api/track/{name}` | DELETE | Delete track file |
| `/api/track/generate` | POST | Generate random track via external generator |

**Scoring:**

| Endpoint | Method | Description |
|---|---|---|
| `/api/scoring/summary` | GET `?t_best=N` | FSG 2024 scoring breakdown |

**Session Log:**

| Endpoint | Method | Description |
|---|---|---|
| `/api/session/log` | GET | In-memory event log |
| `/api/session/export` | GET | Download as JSON |

#### WebSocket Telemetry

```
ws://localhost:8000/ws/telemetry
```

Pushes JSON at **5 Hz** (200 ms loop):

```json
{
  "speed": 12.4,
  "rpm": 3200,
  "gear": 2,
  "x": 45.2, "y": 12.1, "z": 0.3,
  "throttle": 0.7, "steering": 0.05, "brake": 0.0,

  "regen_torque": 12.5,
  "regen_power": 4200.0,
  "regen_avail_torque": 18.0,
  "regen_max_torque": 240.0,
  "regen_max_power": 80000.0,

  "doo": 1, "oc": 0,
  "laps": 3, "required_laps": 10,
  "finished": false,
  "event": "trackdrive",
  "fps": 60,
  "paused": false,
  "res_active": false,
  "pipeline_enabled": true
}
```

The five `regen_*` fields surface live brake-energy-recovery telemetry (motor-side, pre-gearbox) — `*_avail_torque` is the cap at the current motor ω, `*_max_torque/power` are the hardware ceilings from `settings.json`. `pipeline_enabled` mirrors the `/pipeline_ctrl/enable` flag the bridge watches.

If the backend can't talk to the sim, the loop emits `{"error": "sim_disconnected"}` instead of the schema above.

### Frontend (React + TypeScript + Tailwind, port 3000)

Dark-themed (background `#0a0a0a`, accent `#ffb81c` — ISC Racing gold).

**Six tabs:**

| Tab | Contents |
|---|---|
| **Dashboard** | Sim status, quick controls (pause/resume/reset), RES button |
| **Event** | Event type selector, lap count, start/stop |
| **Tracks** | Track list, cone breakdown, preview image, load/delete, random generate |
| **Telemetry** | Real-time gauges: speed, RPM, throttle bar, brake bar, steering indicator, ENU position, referee state |
| **Scoring** | Live FSG 2024 score computation with penalty breakdown |
| **Log** | Timestamped session event log with JSON export |

**RES button:** Pulsing red animation when active, prominent in the top bar. Cannot be accidentally dismissed.

---

## 12. Configuration (settings.json)

All runtime parameters are set in `settings.json` at the project root. The file is auto-discovered on startup.

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Car",
  "ViewMode": "SpringArmChase",
  "ClockSpeed": 1.0,
  "Vehicles": {
    "FSCar": {
      "VehicleType": "ChaosCar",
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      "AutoCreate": true,
      "VehiclePhysics": { ... },
      "Sensors": { ... },
      "Cameras": { ... }
    }
  }
}
```

### ViewMode options

| Value | Description |
|---|---|
| `SpringArmChase` | Third-person chase camera (default) |
| `FlyWithMe` | Overhead following camera |
| `Manual` | Free-roam camera |
| `NoDisplay` | Headless (no camera) |

### Sensor type codes (for `SensorType` field)

| Code | Sensor |
|---|---|
| 2 | IMU |
| 3 | GPS |
| 6 | LiDAR |
| 7 | GSS |

---

## 13. Coordinate System

IFSSIM uses **ENU (East-North-Up)** coordinates in all external interfaces. Internally, Unreal Engine uses a left-handed system with centimetres.

### Conversion (ENU ↔ UE5)

| ENU | UE5 |
|---|---|
| East (X) | Y-axis |
| North (Y) | X-axis |
| Up (Z) | Z-axis |
| 1 metre | 100 cm |

```cpp
// UE to ENU
FVector ENUPos = FVector(UEPos.Y / 100.f, UEPos.X / 100.f, UEPos.Z / 100.f);

// ENU to UE
FVector UEPos = FVector(ENUPos.Y * 100.f, ENUPos.X * 100.f, ENUPos.Z * 100.f);
```

### Quaternion convention

Orientation quaternions are converted such that heading 0 = North (ENU Y-axis). All angular velocity and linear acceleration vectors undergo the same X↔Y swap.

All coordinates in the TCP API, UDP streams, ROS 2 topics, Python client, and Mission Control are in ENU metres. The only exception is the raw UE5 scene object API (`listSceneObjects`, `getObjectPose`, `setObjectPose`), which uses ENU metres as well after internal conversion.

---

## 14. Autonomy Pipeline

The autonomy stack — perception, SLAM, path planning, control — is the same code on the real car and in sim. It lives in a separate repo and is attached here as a submodule, so this functionality reference deliberately doesn't duplicate it. See [`autonomy_pipeline.md`](autonomy_pipeline.md) for the full architecture, which covers:

- The integration contract between IFSSIM (sim, bridge, Mission Control web, viz) and the autonomy submodule.
- The end-to-end topic graph from `/fsds/lidar/Lidar1` through `/fsds/control_command`.
- Mission management: `sim_supervisor_node` (the simulated micro), `mission_control_node` (DVPC role), `mode_manager_node` (lifecycle orchestrator).
- The two-phase runtime action protocol (startup with JIT warmup → runtime with throttle/steering/emergency/finished).
- The TF tree `map → odom → base_link` and which node owns which frame.
- One open question still being finalised on the submodule side (where `/odom` comes from now that `odometria_node` is being removed).

For the simulator-side topics that *feed* the autonomy stack (sensors, vehicle physics, RPC, Mission Control surface), see the sections above.

---

*ISC Racing Team — IFSSIM*
