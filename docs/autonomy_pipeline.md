# Autonomy pipeline

End-to-end overview of the ROS 2 autonomy stack that drives the simulated car: from raw LiDAR/IMU coming out of UE5 through cone detection, SLAM, path planning, and control — and back to UE5 as a `ControlCommand`. Read this before touching any of `pipeline/`.

For sim-side topics (sensors, vehicle physics, RPC) see [`FUNCTIONALITIES.md`](FUNCTIONALITIES.md). For container layout and how to launch the stack see [`GETTING_STARTED_DOCKER.md`](GETTING_STARTED_DOCKER.md).

## High-level data flow

```
            UE5 sim ──► bridge ──► /imu, /fsds/lidar/Lidar1, /motor_rpm, /fsds/gps
                                              │
                                              ▼
                                    Cone_Detection (slam pkg)
                                              │
                                              ▼  /Conos_raw, /Conos_Orange    (base_link)
                                              │
                                              ▼
                                cone_graph_slam (cone_slam pkg)
                                              │
                                              ▼  /cone_slam/state, /Conos     (odom)
                                              │     +  TF map→odom→base_link
                                              ▼
                                  path_planning (path_planning pkg)
                                              │
                                              ▼  /Path, /path_planning/delaunay   (odom)
                                              │
                                              ▼
                                       control (control pkg)
                                              │
                                              ▼  /control_command  (throttle/regen/steer)
                                              │
                                              ▼
                                  bridge ──► UE5 sim (FSDSPlugin)
```

Three side channels run alongside this main loop:

- **Mission Control** (`tools/mission_control/`) talks to the bridge over the FSDS RPC port (start/stop session, load track, RES, EBS, snapshot referee state). It does not sit on the ROS topic chain — it manipulates the sim and the autonomy lifecycle from the side.
- **`/Conos_Orange`** is a dedicated stream of big-orange (finish-line) cones in `base_link`. The control node consumes it directly to implement the autonomous-stop logic without going through SLAM.
- **EBS / EBS-reset** are latched `std_msgs/Empty` topics (`/signal/ebs`, `/signal/ebs_reset`) the control node publishes for the bridge to act on.

## TF tree

The live TF chain at runtime is:

```
map ──(static, identity)──► odom ──(dynamic, EKF output)──► base_link
```

- **`map → odom`** is published as identity on `/tf_static` by `cone_graph_slam_node`. We don't have GPS-aligned global localisation, so the SLAM odom frame *is* effectively the map for downstream consumers; the static is there mainly so Lichtblick / Foxglove 3D panels can root themselves on `map`.
- **`odom → base_link`** is the dynamic vehicle pose published by `cone_graph_slam_node` from its EKF state at SLAM tick rate.
- The bridge does **not** publish any dynamic TF. Raw sensor messages carry sensor-local `frame_id`s (`fsds/IMU`, `fsds/GPS`, `fsds/Lidar`) that are not part of the live TF chain — the pipeline consumes the sensors directly without TF lookups.
- `/fsds/testing_only/odom` (frame `odom`, child `fsds/FSCar`) is a ground-truth odometry feed published by the bridge for debugging and offline analysis only. The autonomy must not consume it.

## Pipeline packages

### Cone detection — `pipeline/slam/slam/cone_detection_node.py`

- **In:** `/fsds/lidar/Lidar1` (`sensor_msgs/PointCloud2`, frame `fsds/Lidar`).
- **Out:** `/Conos_raw` and `/Conos_Orange` (`visualization_msgs/MarkerArray`, frame `base_link`).
- DBSCAN clustering on the LiDAR point cloud, two-phase L-BFGS-B parametric cone fit with centroid fallback, height threshold to separate big-orange (505 mm) from blue/yellow (325 mm).
- Per-cone metadata is layered onto `Marker.scale`: `scale.x` carries σxy (observation position uncertainty, sentinel −1 = unknown), `scale.z` carries cluster height. Downstream SLAM consumes both.
- A per-second `CONE_FILTER` diagnostic line logs the fall-off through each filter stage (input points → clusters → ≥2 pts → height gate → fit/centroid). Use it when observations look thin — it localises the loss in one scan instead of bisecting through the pipeline.

### Cone-graph SLAM — `pipeline/cone_slam/cone_slam/cone_graph_slam_node.py`

- **In:** `/Conos_raw`, `/imu`, `/motor_rpm`.
- **Out:** `/cone_slam/state` (`nav_msgs/Odometry`, frame `odom`, child `base_link`), `/Conos` (`MarkerArray` in `odom` — landmarks). Publishes TF `map → odom` (static) and `odom → base_link` (dynamic).
- EKF-style state estimation over a graph of cone landmarks. Trigger is each `/Conos_raw` scan; IMU and motor RPM are cached as priors. Big-orange cones are **not** added to the map (they're handled by `/Conos_Orange` directly to avoid colour confusion with the blue/yellow associator).
- Design rationale and per-iteration tuning history are archived in [`history/cone_graph_slam_design.md`](history/cone_graph_slam_design.md) and [`history/cone_graph_slam_progress.md`](history/cone_graph_slam_progress.md).

### Path planner — `pipeline/path_planning/path_planning/`

- **In:** `Conos` (the SLAM-mapped cones in `odom`).
- **Out:** `Path` (`nav_msgs/Path` in `odom`), `/path_planning/delaunay` (`MarkerArray` debug overlay in `odom`).
- Delaunay triangulation over the cone field, best-first search through the candidate midpoint graph from a car-anchored start, PCHIP cubic Hermite spline densification.
- The planner refuses to fabricate a path it can't justify: with only one side of cones visible, only collinear inputs, or no forward visibility, it returns an empty path. The control node then commands zero throttle. This is by design — the autonomy must not drive when it can't see a real corridor.
- Debug overlay is three layers in one MarkerArray, namespaced `delaunay_edges` / `candidates` / `selected`. Subscribe in Lichtblick / Foxglove to see why a tick produced an empty path.
- Tight-corner tuning is tracked in [#180](https://github.com/isc-fs/IFSSIM/issues/180); a C++ port is tracked in [#164](https://github.com/isc-fs/IFSSIM/issues/164).

### Control — `pipeline/control/control/`

- **In:** `Path`, `/cone_slam/state`, `/Conos_Orange`.
- **Out:** `/control_command` (`fs_msgs/ControlCommand` — throttle / regen / steering), `/signal/ebs`, `/signal/ebs_reset`.
- Layout:
  - `state.py` — body-frame state snapshot built from `/cone_slam/state` (the twist's child frame is `base_link`).
  - `reference.py` — path-projection / lookahead utilities.
  - `controllers/pure_pursuit.py` — lateral controller (Pure Pursuit, lookahead-based steering).
  - `controllers/pi_velocity.py` — longitudinal controller (PI on velocity, with a single-quadrant regen guard at v < threshold so we don't push the car backward from rest).
  - `models/bicycle.py` — kinematic bicycle for the lateral controller's geometry.
  - `control_node.py` — the ROS-facing wrapper that wires everything together.
- On init the node publishes `/signal/ebs_reset` (latched) so a fresh autonomy instance can drive without manual EBS clearing. The stop-latch (autonomous finish on big-orange detection) only arms after the car has travelled >30 m from start, so the spawn-line big-orange doesn't trigger an immediate stop.
- The motor controller in UE5 is single-quadrant regen on the EMRAX 228 model (see [`FUNCTIONALITIES.md`](FUNCTIONALITIES.md)) — regen demand on a stationary or backward-rotating wheel is clamped to zero in `EmraxMotor.cpp`, so the controller can issue regen near zero velocity safely.

## Mission Control hooks

`tools/mission_control/backend/main.py` is a FastAPI app that orchestrates the autonomy lifecycle. The user-visible Start Session sequence:

1. Stop the autonomy pipeline (kills the launched processes inside the container).
2. Activate RES (autonomy-disable signal — the bridge holds the car).
3. Set the FSDS event mode (e.g. trackdrive).
4. Resume the sim from pause.
5. Start the autonomy pipeline.
6. Wait ~4.5 s for SLAM to produce a calibrated estimate.
7. Auto-release the EBS so the controller can issue commands.

Guardrails the audit + recent fixes added:

- `event_start` refuses with HTTP 400 if the loaded track has zero cones (silent track-load failures used to leave the event running on stale level state).
- `/api/res/activate` no longer kills the autonomy pipeline; it only pulses the RES line.
- The EMRAX idle creep torque was disabled (was 5 Nm — a Chaos workaround that produced ~0.83 m/s drift whenever EBS was released without an autonomy command).

A faithful FS-DV state machine (T 14.8.x: `AS_Off` / `AS_Ready` / `AS_Driving` / `AS_Finished` / `AS_Emergency`) is tracked in [#173](https://github.com/isc-fs/IFSSIM/issues/173); the current Mission Control sequence is the staged stand-in until that lands.

## When the car won't drive — debugging order

1. Is a track loaded? `/api/track/state` should show `cones > 0`. If not, Start Session would refuse anyway since #61.
2. Are cones being detected? Watch the per-second `CONE_FILTER` log line from `cone_detection_node`. Stage drop-off shows you which gate (min-points, height, fit/centroid) is shedding observations.
3. Is SLAM producing landmarks? `/Conos` non-empty + `/cone_slam/state` ticking. If `/Conos_raw` is high but `/Conos` stays low, SLAM gating is the bottleneck (#177 was an instance).
4. Is the planner emitting a path? Subscribe to `/path_planning/delaunay` in Lichtblick. If the triangulation looks fine but no `selected` midpoints appear, the best-first search is rejecting candidates — check planner constants in `planner.py` (typical culprits: `SEARCH_RADIUS_M`, `MAX_HEADING_DELTA_RAD`, `MIN_TRACK_WIDTH_M`).
5. Is the controller issuing commands? `/control_command` should tick at the controller rate. Zero throttle with a non-empty `/Path` usually means the controller's stop-latch fired or the velocity reference is zero.
6. If `/control_command` looks healthy but the car doesn't move: bridge half-open TCP issue. `docker restart ifssim-dv_pipeline_stack-1` clears it. Tracked as a recurring nuisance; no root fix yet.
