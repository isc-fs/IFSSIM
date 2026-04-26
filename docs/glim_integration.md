# GLIM Integration — Design Doc

**Status:** draft, opened 2026-04-26
**PR:** `feat/28-glim-localization` (PR #3 in `docs/dv_pipeline_rebuild.md`)
**Library:** [koide3/glim](https://github.com/koide3/glim) — LiDAR-IMU SLAM
**Successor to:** `Odometria_perfecta` (the GT-driven TF publisher)

## 1. Goal

Replace the simulator's ground-truth-driven `odom→fsds/FSCar` TF with a real LiDAR-IMU SLAM stack that runs identically in sim and on the IFS-08 car. After this PR the autonomy pipeline never reads `/fsds/testing_only/odom` again — that topic is grandfathered as a sim-only ground-truth reference for validation.

**Non-goals**: vision-based perception (no cameras on the real car), wheel-odom dead-reckoning fallback, multi-LiDAR fusion. GLIM is the single source of truth for pose.

## 2. What GLIM provides

Per [koide3/glim README](https://github.com/koide3/glim):
- **Inputs**: PointCloud2 (any spinning LiDAR — Hesai ATX 128ch is in-spec) + Imu.
- **Outputs**: 6-DoF trajectory at IMU rate, 3D point cloud map with loop closure, TF tree.
- **Backend**: GTSAM factor-graph optimization with periodic loop-closure refinement.
- **ROS 2 wrapper**: `glim_ros2` package, configurable via YAML.

Critically, GLIM owns the `map → odom → base_link` TF tree. Our sim and real car will both consume that tree from a single source.

## 3. Frame tree before / after

### Before (today, on `dev` after PR #122)
```
odom (published by Odometria_perfecta from /fsds/testing_only/odom — GT)
  └─ fsds/FSCar (vehicle body)
```
Static transforms `fsds/FSCar → fsds/Lidar`, `fsds/FSCar → fsds/IMU`, etc. published by the bridge.

### After (this PR)
```
map  (loop-closed, owned by GLIM)
  └─ odom  (dead-reckoned, IMU-rate, owned by GLIM)
       └─ base_link  (vehicle body, owned by GLIM)
              ├─ fsds/Lidar    (static, owned by bridge)
              ├─ fsds/IMU      (static, owned by bridge)
              ├─ fsds/Gps      (static, owned by bridge)
              └─ ...
```
**`fsds/FSCar` → `base_link` rename everywhere.** This is the single biggest mechanical change in this PR — every TF lookup (`slam.py`, `path_planning.py`, `control.py`), every static transform publish in the bridge, every Foxglove layout in `docs/`. Use `git grep "fsds/FSCar"` as the audit anchor.

## 4. Container additions

**Build from source — no `koide3/glim_ros2` prebuilt image, no CUDA.** Two reasons:
- Real-car compute is unlikely to ship with a CUDA-class GPU; a CPU-only stack means the same Docker image runs in sim and on the car.
- Building from source means we own the version pins (GTSAM, gtsam_points, glim, glim_ros2) — no opaque base-image upgrades changing things under us.

We use GLIM's `OdometryEstimationCPU` module (no GPU). This drops the CUDA toolkit, gtsam_points-with-CUDA flag, and CUDA-related apt deps from the Dockerfile entirely. Estimated first-build time **15–25 min** (lower than the CUDA-included path).

`docker/dv_pipeline_stack/Dockerfile` adds, in order (cache-friendly: heaviest layers first):

```dockerfile
# C++ build deps for GTSAM + gtsam_points + glim
RUN apt-get update && apt-get install -y \
    libboost-all-dev libtbb-dev libgoogle-glog-dev \
    libsuitesparse-dev libeigen3-dev libnanoflann-dev \
    libfmt-dev libspdlog-dev \
    cmake build-essential git \
  && rm -rf /var/lib/apt/lists/*

# GTSAM 4.3a0 — gtsam_points 1.2.0 supports both 4.2a9 and 4.3a0, picking newer
RUN cd /tmp && git clone --depth 1 --branch 4.3a0 \
        https://github.com/borglab/gtsam.git && \
    cd gtsam && mkdir build && cd build && \
    cmake -DGTSAM_BUILD_PYTHON=OFF \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
          -DGTSAM_BUILD_TESTS=OFF \
          -DGTSAM_BUILD_UNSTABLE=OFF \
          -DGTSAM_WITH_TBB=ON \
          -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/gtsam

# gtsam_points 1.2.0 — CPU only (no CUDA)
RUN cd /tmp && git clone --depth 1 --branch v1.2.0 \
        https://github.com/koide3/gtsam_points.git && \
    cd gtsam_points && mkdir build && cd build && \
    cmake -DBUILD_DEMO=OFF \
          -DBUILD_WITH_CUDA=OFF \
          -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/gtsam_points

# GLIM core + ROS 2 wrapper, cloned into the colcon workspace src/
# (built alongside our packages by the existing colcon build step)
RUN cd /dv_pipeline_stack_ws/src && \
    git clone --depth 1 https://github.com/koide3/glim.git && \
    git clone --depth 1 https://github.com/koide3/glim_ros2.git
```

Place these RUN blocks **before** `COPY pipeline/ src/` so the heavy GTSAM layer caches across pipeline-source changes. The GLIM clone goes into `src/` so the existing `colcon build --symlink-install` step picks it up.

## 5. GLIM YAML config

A new file `docker/dv_pipeline_stack/glim_ros2.yaml` (mounted into the container or COPY'd to `/dv_pipeline_stack_ws/glim_config.yaml`):

```yaml
# Sensor input
common:
  imu_topic: "/imu"
  imu_frame_id: "fsds/IMU"
  points_topic: "/lidar/Lidar1"
  lidar_frame_id: "fsds/Lidar"

# Frame outputs (GLIM owns this whole subtree)
frame:
  map_frame_id: "map"
  odom_frame_id: "odom"
  base_frame_id: "base_link"

# >>> ESTIMATION BACKEND: CPU module per project decision (2026-04-26).
# OdometryEstimationCPU runs without CUDA; matches real-car compute envelope.
odometry_estimation:
  type: "OdometryEstimationCPU"
  num_threads: 4

# IMU rate handling — IFSSIM bridge publishes at 400 Hz; GLIM examples
# use 100-200 Hz. Configure to accept the higher rate without dropping.
imu:
  imu_frequency: 400.0
  acc_noise: 0.18         # match settings.json AccelNoiseStd
  gyro_noise: 0.004       # match settings.json GyroNoiseStd
  acc_bias_noise: 0.01
  gyro_bias_noise: 0.0002

# LiDAR (Hesai ATX 128ch, 10 Hz)
preprocess:
  min_distance: 1.0
  max_distance: 60.0
  k_correspondences: 8
  num_threads: 4

# Loop closure — keep on for sim parity with real-car operation
loop_closure:
  enabled: true
  min_loop_overlap: 0.7
```

The exact field names follow `glim_ros2`'s schema; this is illustrative pending a closer read of the upstream YAML.

## 6. Topics in / out

| Topic | Direction | Type | Source / sink |
|---|---|---|---|
| `/lidar/Lidar1` | IN | sensor_msgs/PointCloud2 | bridge → GLIM |
| `/imu` | IN | sensor_msgs/Imu | bridge → GLIM |
| `/tf` | OUT | tf2_msgs/TFMessage | GLIM publishes `map→odom→base_link` |
| `/glim/odom` | OUT | nav_msgs/Odometry | for downstream control velocity (PR #4 wires) |
| `/glim/map` | OUT | sensor_msgs/PointCloud2 | for visualization / debug |

## 7. Pipeline launch changes

`docker/dv_pipeline_stack/pipeline_only.launch.py`:

- **DELETE** the `Odometria_perfecta` node block.
- **DELETE** the `Publicar_Track` node block (GT-only, debug viz).
- **ADD** the `glim_ros2` node, parameterized from `glim_ros2.yaml`.
- **DROP** the `REMAP_ODOM` mapping for the control node (control no longer subscribes to `/testing_only/odom`).

The `Cone_Detection`, `Publicar_Mapa`, `Plan_Path`, `Control` nodes stay — but their TF lookups are renamed.

## 8. Bridge changes

`ros2/src/ifssim_bridge/`:

- **Stop publishing** `odom → fsds/FSCar` TF (GLIM owns this now).
- **Publish static transforms** `base_link → fsds/Lidar`, `base_link → fsds/IMU`, `base_link → fsds/Gps`. Static = published once at startup with `tf2_ros::StaticTransformBroadcaster`. Source the offsets from `settings.json` (`Lidar1.X/Y/Z`, etc.).
- **Continue publishing** `/fsds/testing_only/odom` and `/testing_only/track` only when `competition_mode: false` (already gated; no change). These remain available for sim-side validation but aren't consumed by autonomy.

## 9. Pipeline-source TF rename

Single global rename: every reference to `"fsds/FSCar"` becomes `"base_link"`. Files affected (per a quick grep before this PR):

- `pipeline/slam/slam/slam.py` — `tf_buffer.lookup_transform("odom", "fsds/FSCar", ...)` → `... "base_link" ...`
- `pipeline/path_planning/path_planning/path_planning.py` — same
- `pipeline/control/control/control.py` — same
- `tools/trajectory_record.py`, `tools/diagnostics/cone_frame_probe.py`, etc.
- Any Foxglove layout JSON in `docs/`

Use `git grep "fsds/FSCar" | wc -l` as a tracking metric — count goes to zero by the end of this PR.

## 10. Validation

The acceptance gate for this PR is **drift, not lap completion** (lap completion is PR #6's bar):

1. **Static drift**: with the sim car at standstill (RES on, no autonomy), GLIM's `base_link` pose vs `/fsds/testing_only/odom` should differ by < 1 cm over 60 s.
2. **Driving drift**: drive a single autocross lap manually (teleop) at ~5 m/s. At lap end, GLIM's pose vs GT pose should differ by < 10 cm.
3. **TF graph health**: `ros2 run tf2_tools view_frames` should produce the expected `map → odom → base_link → {sensor frames}` tree with no orphans.
4. **Topic flow**: `/glim/odom` publishes at IMU rate (~400 Hz expected, may be lower if GLIM downsamples internally — that's acceptable as long as ≥ 100 Hz).

If (1) or (2) fail by ≥ 2× the threshold, fall back per §11.

## 11. Risks + fallback

| Risk | Likelihood | Mitigation |
|---|---|---|
| GTSAM 4.3a0 build incompatibility with Eigen / Boost shipped in `ros:humble-ros-base` (Ubuntu 22.04) | Medium | Pin to system Eigen via `-DGTSAM_USE_SYSTEM_EIGEN=ON`. If a deeper conflict shows up, fall back to GTSAM 4.2a9 (gtsam_points 1.2.0 supports both) |
| GLIM doesn't track on simulated LiDAR (sim point cloud may differ in density / noise from real Hesai output) | Medium | Tune `min_distance`, `max_distance`, `k_correspondences`. If still bad, try downsampling LiDAR to 64 ch to match GLIM examples better |
| `OdometryEstimationCPU` slower than real-time on full 200k-pts/s scans | Medium | Drop `PointsPerSecond` in `settings.json` from 200000 → 100000 (still 10k pts/scan at 10 Hz, plenty for cone-density geometry). Monitor GLIM's per-scan latency |
| First container build hits Docker layer cache miss every time GLIM source changes | Low | Pin GLIM to a specific commit hash (not `--depth 1`) once we settle on a working version |
| 400 Hz IMU exceeds GLIM's internal queue, drops samples | Medium | Downsample IMU to 200 Hz at the bridge as a per-config option (gated by `competition_mode` so the real car can still publish 400 Hz if it wants) |
| Real-car LiDAR-IMU sync mismatches sim (different timestamp clock) | High when bringing up real car | Out of scope for this PR; real-car bring-up handles its own synchronization |

**Fallback path** if GLIM doesn't work after a week of effort:
- Revert this PR. Restore `Odometria_perfecta` for sim mode.
- Re-evaluate: try [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) (smaller dep footprint, similar behavior) or [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM).
- Real-car deployment still needs LiDAR-IMU SLAM regardless of which library we land on.

## 12. Implementation order (within this PR)

1. **Container build only** — add GLIM deps to Dockerfile, verify image builds clean. No node spawn, no behavior change. Commit.
2. **GLIM node + YAML config** — spawn `glim_ros2` in `pipeline_only.launch.py`, no other autonomy nodes consume it yet. Verify `map → odom → base_link` TF appears. Commit.
3. **Bridge static transforms** — bridge publishes `base_link → fsds/{Lidar,IMU,Gps}` statics. Stop publishing `odom → fsds/FSCar`. Verify TF graph correct. Commit.
4. **Frame rename in pipeline** — bulk rename `fsds/FSCar` → `base_link` in slam, path_planning, control, tools. Verify pipeline runs end-to-end against GLIM TF. Commit.
5. **Delete `Odometria_perfecta` and `Publicar_Track`** — remove the nodes, drop the package files (`pipeline/odometria/` becomes empty / deletable, the slam package loses one node). Verify nothing breaks. Commit.
6. **Validation drive** — single manual lap with GLIM running, log drift vs `/fsds/testing_only/odom`, paste the numbers in the PR. If drift < 10 cm at lap end, ready to merge.

Splitting into 6 small commits keeps the PR reviewable and each step independently revertable.

## 13. What this PR does NOT do

- Replace `/fsds/gss` velocity feedback. That's PR #4 (motor RPM).
- Touch the cone map (`mapa.py`) beyond renaming the TF target frame. That's PR #5.
- Retune the controller. That's PR #6.

## 14. After merge

`project_glim_next.md` memory entry can be removed (it'll be stale). Update `project_immediate_gaps.md` and `project_ifssim.md` to reflect GLIM as the live localization layer.
