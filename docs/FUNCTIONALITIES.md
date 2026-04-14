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

The default vehicle (`FSCar`) is modelled after ISC Racing Team's IFS-08 car. All physics parameters are defined in `settings.json` under `VehiclePhysics` and applied at runtime via `AFSDSVehiclePawn`.

#### Physical Parameters (defaults)

| Parameter | Value | Unit |
|---|---|---|
| Mass | 210 | kg |
| Drivetrain | RWD | — |
| Wheel radius | 200 | mm |
| Wheel width | 190 | mm |
| Max steer angle | 28 | ° |
| Motor max torque | 230 | Nm |
| Motor max power | 80,000 | W |
| Gear ratio | 2.909 | — |
| Drivetrain efficiency | 92 | % |
| Aerodynamic drag (CdA) | 0.95 | m² |
| Aerodynamic downforce (ClA) | 3.0 | m² |
| Aero balance front | 45 | % |
| Tire friction coefficient (μ) | 1.65 | — |
| Weight distribution front | 43.8 | % |
| Centre of gravity height | 344 | mm |
| Suspension damping | 1.5 | — |

#### Motor Torque Curve

| RPM | Torque (Nm) |
|---|---|
| 0 | 230 |
| 1000 | 240 |
| 2000–5000 | 240 |
| 6000 | 200 |
| 6500 | 180 |

#### Wheels

- **Front:** Hoosier 16.0×7.5-10 R20, max steer 28°, friction μ = 1.65, Chaos simulation
- **Rear:** Identical tire spec, no steering, handbrake capable

#### Aerodynamics

Applied per-tick using `AddForce` on the vehicle mesh:
- **Drag:** `F_drag = 0.5 × ρ × CdA × v²` (opposing velocity)
- **Downforce:** `F_down = 0.5 × ρ × ClA × v²` (distributed front/rear per `AeroBalanceFront`)

Air density ρ = 1.225 kg/m³.

#### Vehicle Control Inputs

```
throttle  ∈ [-1.0, 1.0]   (negative = reverse)
steering  ∈ [-1.0, 1.0]   (negative = left)
brake     ∈ [0.0, 1.0]
```

#### API Control vs Manual

When `enableApiControl` is called, keyboard/gamepad input is disabled and the vehicle responds only to `setCarControls` commands. Can be toggled at runtime.

---

## 4. Sensor Suite

All sensors are attached to the vehicle pawn and configured via `settings.json`. Sensor positions and rotations are defined in ENU meters relative to the vehicle origin.

### 4.1 LiDAR

**Implementation:** `FSDSLidarSensor` — multi-channel batch raycasting via UE5's `UKismetSystemLibrary::LineTraceSingle`.

| Parameter | Default | Configurable |
|---|---|---|
| Channels | 128 | ✓ |
| Points per second | 100,000 | ✓ |
| Rotations per second | 10 Hz | ✓ |
| Vertical FOV upper | +3° | ✓ |
| Vertical FOV lower | −16° | ✓ |
| Horizontal FOV | ±60° | ✓ |
| Max range | 100 m | ✓ |
| Range noise std | 2.0 cm | ✓ |
| Point dropout rate | 1% | ✓ |
| Mount position | X=1.4m, Z=−0.2m | ✓ |

Point cloud is output as a flat `float[]` array in sensor-local frame (X forward, Y left, Z up). Each point is 3 floats (x, y, z) in metres.

**Noise model:** Gaussian range noise applied per point. Independent Bernoulli dropout per point with configurable probability.

### 4.2 IMU

**Implementation:** `FSDSImuSensor` — 6-DOF inertial measurement in body frame.

| Parameter | Default | Configurable |
|---|---|---|
| Accelerometer noise std | 0.18 m/s² | ✓ |
| Gyroscope noise std | 0.004 rad/s | ✓ |
| Accelerometer bias std | 0.01 m/s² | ✓ |
| Gyroscope bias std | 0.0002 rad/s | ✓ |

**Noise model:** White Gaussian noise + random-walk bias (accumulated per tick). Bias is seeded at startup and drifts over time.

**Outputs:** Linear acceleration (m/s²), angular velocity (rad/s), orientation quaternion — all in ENU body frame.

### 4.3 GPS / GNSS

**Implementation:** `FSDSGpsSensor` — converts UE5 world position to geodetic coordinates.

| Parameter | Default | Configurable |
|---|---|---|
| Position noise std | 0.5 m | ✓ |
| Velocity noise std | 0.1 m/s | ✓ |
| Reference origin | Map centre | — |

**Coordinate conversion:** ENU position → latitude/longitude using a flat-Earth approximation anchored to a configurable reference origin. Output is WGS84 latitude, longitude, altitude.

### 4.4 Ground Speed Sensor (GSS)

**Implementation:** `FSDSGssSensor` — vehicle velocity in body frame, modelled after Kistler Correvit SFII.

| Parameter | Default | Configurable |
|---|---|---|
| Velocity noise std | 0.02 m/s | ✓ |

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
| cam1 position | X=1.6m, Z=−0.2m | ✓ |
| cam2 position | X=1.6m, Y=−0.15m, Yaw=−10° | ✓ |

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
| `reset` | `true` | Reload current level (full state reset) |
| `listCameras` | `["cam1","cam2",...]` | List configured camera names |

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

## 6. Data Streaming (UDP Push)

For high-frequency sensor data, IFSSIM broadcasts binary frames over UDP without any client request. This is the lowest-latency path and what the ROS 2 bridge uses by default.

### Sensor Frame (port 41452, ~100 Hz)

Magic: `0x49465353` ("IFSS")

```c
struct SensorFrame {
    uint32_t magic;        // 0x49465353
    float    pos_x, pos_y, pos_z;      // ENU position (m)
    float    pose_qx, pose_qy, pose_qz, pose_qw;  // orientation
    float    accel_x, accel_y, accel_z;  // m/s²
    float    gyro_x, gyro_y, gyro_z;     // rad/s
    float    orient_x, orient_y, orient_z, orient_w;  // IMU quat
    float    gss_vx, gss_vy, gss_vz;    // body-frame velocity (m/s)
    float    latitude, longitude, altitude;
};
```

### LiDAR Frame (port 41453, ~10 Hz)

Magic: `0x4C494452` ("LIDR")

```c
struct LidarChunkHeader {
    uint32_t magic;         // 0x4C494452
    int32_t  total_points;
    int32_t  chunk_index;
    int32_t  total_chunks;
};
// Followed by: total_points × 3 × float32 (x, y, z in sensor frame, metres)
```

---

## 7. ROS 2 Bridge

The `ifssim_bridge` package (`ros2/src/ifssim_bridge/`) connects a ROS 2 stack to IFSSIM over the TCP RPC server. It uses **4 persistent TCP connections** simultaneously:

| Connection | Type | Purpose |
|---|---|---|
| Sensor stream | TCP push | GPS, IMU, GSS, Odometry, TF at ~100 Hz |
| LiDAR stream | TCP push | PointCloud2 at ~10 Hz |
| Camera client | TCP req/resp | CompressedImage at configurable Hz |
| Command client | TCP req/resp | Controls, referee queries, settings |

### Published Topics

| Topic | Message Type | Rate | Description |
|---|---|---|---|
| `gps` | `sensor_msgs/NavSatFix` | ~100 Hz | GPS with covariance from settings |
| `imu` | `sensor_msgs/Imu` | ~100 Hz | IMU with covariance from settings |
| `gss` | `geometry_msgs/TwistWithCovarianceStamped` | ~100 Hz | Ground speed in body frame |
| `lidar/Lidar1` | `sensor_msgs/PointCloud2` | ~10 Hz | XYZ point cloud |
| `camera/<name>/compressed` | `sensor_msgs/CompressedImage` | configurable | PNG-compressed image |
| `signal/go` | `fs_msgs/GoSignal` | 1 Hz | Mission + track identifiers |
| `testing_only/odom` | `nav_msgs/Odometry` | ~100 Hz | Ground-truth odometry *(hidden in competition mode)* |
| `testing_only/track` | `fs_msgs/Track` (latched) | 0.2 Hz | All cone positions *(hidden in competition mode)* |
| `testing_only/extra_info` | `fs_msgs/ExtraInfo` | 1 Hz | DOO counter, lap count *(hidden in competition mode)* |

TF frames published: `fsds/map` → `fsds/FSCar` (dynamic), `fsds/FSCar` → `fsds/FSCar/Lidar1`, `fsds/FSCar/<cam_name>` (static, 1 Hz).

### Subscribed Topics

| Topic | Message Type | Description |
|---|---|---|
| `control_command` | `fs_msgs/ControlCommand` | Throttle, steering, brake → forwarded to sim |
| `signal/finished` | `fs_msgs/FinishedSignal` | Mission complete signal |

### Services

| Service | Type | Description |
|---|---|---|
| `reset` | `fs_msgs/srv/Reset` | Full level reload |

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
| `/api/event/start` | POST | Set event and resume in one call |

**RES (Remote Emergency Stop):**

| Endpoint | Method | Description |
|---|---|---|
| `/api/res/activate` | POST | Emergency stop — pause sim + lock controls |
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

Pushes JSON at **10 Hz**:

```json
{
  "speed": 12.4,
  "rpm": 3200,
  "gear": 2,
  "x": 45.2, "y": 12.1, "z": 0.3,
  "throttle": 0.7, "steering": 0.05, "brake": 0.0,
  "doo": 1, "oc": 0,
  "laps": 3, "required_laps": 10,
  "finished": false,
  "event": "trackdrive",
  "fps": 60,
  "paused": false,
  "res_active": false
}
```

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

*ISC Racing Team — IFSSIM*
