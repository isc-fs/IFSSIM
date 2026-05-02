# Cone-Graph SLAM Design

**Status:** draft 2026-04-27
**Authors:** Raul Moran (with Claude as scribe)
**Branch:** `feat/28-cone-graph-slam`
**Replaces:** the GLIM/FAST_LIO/fast_LIMO localization box from `dv_pipeline_rebuild.md` §3.2

## 1. Architecture

```
                  ┌──────────────────┐
   /imu  (400 Hz) │                  │
   ──────────────▶│   cone_graph     │
                  │     _slam        │
   /Conos_raw     │   (rclpy node)   │  ──▶  /tf  : odom → base_link
   (10 Hz)        │                  │  ──▶  /Conos (MarkerArray, world
   ──────────────▶│   GTSAM iSAM2    │              frame, persistent IDs)
                  │   factor graph   │  ──▶  /cone_slam/state (Odometry)
   /gps (10 Hz)   │                  │  ──▶  /cone_slam/landmarks (debug)
   ──────────────▶│                  │
                  └──────────────────┘
```

Replaces both fast_LIMO (TF + odom) AND `Publicar_Mapa` (cone map publish) with a single node.
Existing `Cone_Detection` node continues unchanged — it produces `/Conos_raw`.
`path_planning` continues unchanged — it consumes `/Conos`.

## 2. Factor graph topology

```
   prior         IMU       IMU       IMU
    │            │          │          │
    ▼            ▼          ▼          ▼
   x_0 ── pre ── x_1 ── pre ── x_2 ── pre ── x_3 ── ...
    │            │          │          │
    │     gps    │          │   gps    │
    │     │      │          │     │    │
    │     ▼      │          │     ▼    │
    │           BR1         BR4
    │            ↓           ↓
    L_y17       L_y17       L_y17        ← yellow cone landmark, observed multiple times
    L_b03       L_b03                    ← blue cone landmark
                            L_b04        ← new blue cone added when first seen
```

**Nodes:**
- `x_k` — vehicle pose at scan k. `gtsam.Pose2` for 2D (sufficient for FS — cars don't fly). 3-DOF: x, y, yaw.
- `L_color_id` — cone landmark, `gtsam.Point2`. 2-DOF: x, y in odom frame.

**Factors (edges):**
- `PriorFactor(x_0, [0,0,0], σ_anchor)` — anchor first pose at origin with tight covariance.
- `BetweenFactor(x_{k-1}, x_k, ΔT, σ_imu_pre)` — IMU preintegration delta. Computed from `gtsam.PreintegratedImuMeasurements` between consecutive scan stamps.
- `BearingRangeFactor2D(x_k, L_*, bearing, range, σ_obs)` — cone observation. Bearing = atan2(y, x), range = √(x²+y²) of the cone in base_link frame at scan time.
- `GPSFactor(x_k, [lat, lon → ENU], σ_gps)` — loose GPS constraint (large covariance, ~3 m for sim BMI088-class GPS noise). Optional, only when GPS fix is good.

**Why 2D, not 3D?** FS Driverless cars don't change altitude meaningfully. 3D Pose adds 3 DOF for no information. Sticking to 2D halves graph size and triples optimizer speed.

## 3. Lifecycle / state machine

```
   [INIT_WAITING_IMU]
           │
           │  first IMU msg arrives
           ▼
   [INIT_CALIBRATING]   (3 s; estimate gyro bias, gravity-align via stationary mean)
           │
           │  bias estimates converged
           ▼
   [SLAM_RUNNING]       (steady-state)
           │
           │  /pipeline_ctrl/disable signal
           ▼
   [SHUTTING_DOWN]
```

**INIT_CALIBRATING (3 s):**
- Accumulate IMU samples
- Estimate accel and gyro biases as the mean over the window (assuming stationary)
- Compute initial gravity vector (should be ~[0, 0, −9.81] in body frame for level car)
- If gravity vector deviates significantly from −Z, reject (car was moving) and restart calibration
- Publish nothing yet

**SLAM_RUNNING:**
- For each scan (`/Conos_raw` callback):
  1. Pre-integrate IMU samples accumulated since the last scan into a `PreintegratedImuMeasurements` delta
  2. Add new pose node `x_k` to the graph (initial estimate = previous pose composed with IMU delta)
  3. Add `BetweenFactor(x_{k-1}, x_k)` from the IMU delta
  4. **Data association** (see §4): match each cone observation to existing landmarks
  5. For each matched obs → add `BearingRangeFactor` between `x_k` and the matched landmark
  6. For each unmatched obs → add a new landmark to the graph (initial position = obs transformed by current pose), then add the factor
  7. (Optional) If a fresh GPS measurement arrived since last scan → add `GPSFactor(x_k)`
  8. Run `iSAM2.update()` once (fast — ~ms)
  9. Publish `/tf` (odom → base_link) from updated `x_k` estimate
  10. Publish `/Conos` from current landmark estimates (with persistent IDs encoded in marker.id)

## 4. Data association

Per AMZ §3.2, the recipe:

```
for each observation o in scan:
    color = classify_by_y_sign(o)    # +y = blue, -y = yellow, height > 0.30 = big-orange
    candidates = {L : color(L) == color  AND  range(L → predicted_pose) < 30 m}

# Build cost matrix [|obs| × |candidates|]
for i, o in enumerate(observations):
    for j, L in enumerate(candidates):
        d = mahalanobis(o, L, predicted_pose, Σ_pose, Σ_landmark)
        C[i, j] = d if d < χ²_threshold else +∞

# Hungarian assignment minimizes total Mahalanobis cost
matches = scipy.optimize.linear_sum_assignment(C)

# Filter out gated-out matches (those that hit +∞)
matches = [(i, j) for (i, j) in matches if C[i, j] < +∞]
```

**Color gating** is the heaviest filter — 4-way disambiguation (blue / yellow / orange / big-orange).
**Mahalanobis gating** with χ² = 9.21 (95% confidence, 2-DOF) handles measurement noise.
**Hungarian** is one scipy call.

If an observation has *no* candidate after gating → emit a new landmark.

**Color classifier:** initially keep the existing spatial Y-sign rule (already in `slam.py:_lookup_or_classify`). Add big-orange detection from observation height (`scale.z > 0.30 m` per existing convention).

## 5. Files to add (new ROS2 package `pipeline/cone_slam/`)

```
pipeline/cone_slam/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/cone_slam
└── cone_slam/
    ├── __init__.py
    ├── cone_graph_slam_node.py    # main rclpy node, lifecycle, callbacks
    ├── factor_graph.py            # GTSAM graph builder, iSAM2 wrapper
    ├── data_association.py        # color + Mahalanobis + Hungarian
    ├── imu_preintegrator.py       # accumulator between scans, returns ΔT factor
    ├── landmark_db.py             # cone ID assignment, persistent map
    └── color_classifier.py        # spatial Y-sign + big-orange height
```

**Outside this package:**
- `docker/dv_pipeline_stack/Dockerfile` — install GTSAM + Python bindings (already had GTSAM 4.3a0 in the image during the GLIM era; bringing it back is a known recipe)
- `docker/dv_pipeline_stack/pipeline_only.launch.py` — replace `fast_limo_multi_exec` Node with `cone_graph_slam_node`, remove `Publicar_Mapa` Node

## 6. Decisions to make before coding

1. **Python or C++ implementation?**
   - **Python**: matches the rest of the pipeline, easier to iterate, GTSAM has Python bindings. ~3-4 weeks to working sim.
   - **C++**: better real-car perf, harder to iterate, more boilerplate. ~5-6 weeks.
   - **Pitch**: Python first, port to C++ when sim works (or never if Python is fast enough — at 10 Hz scan rate and small graph, Python is probably fine).

2. **Pose representation: Pose2 (3-DOF) vs Pose3 (6-DOF)?**
   - Pose2: simpler, faster, sufficient for flat tracks
   - Pose3: more general, handles slopes, larger graph
   - **Pitch**: Pose2 first; revisit if track has hills.

3. **Where does color classification happen?**
   - In `Cone_Detection` (modify the existing detection node to publish color in `/Conos_raw`)
   - In `cone_graph_slam` (replicate the spatial Y-sign rule using observations and pose)
   - **Pitch**: in `cone_graph_slam` to keep `Cone_Detection` unchanged; replicate the rule (it's 5 lines).

4. **Big-orange handling.**
   - Big-orange cones mark start/finish line (special — only ~4 cones per track)
   - Treat them as a separate landmark color? Or as regular landmarks with a "is_big_orange" flag?
   - **Pitch**: separate color class (4 colors total: blue, yellow, orange, big-orange). Lets us add big-orange-specific logic later (e.g., "lap detection from re-observing big-orange").

5. **GPS factor or not in v0?**
   - GPS gives an absolute position prior — useful for long drives, helps anchor the map
   - But our sim GPS is noisy and FS rules vary on GPS use
   - **Pitch**: Skip GPS in v0 (keep the SLAM purely IMU + cones). Add GPSFactor in v1 if drift becomes a problem on long drives.

6. **What's the "loop closure" scenario?**
   - When the car completes a lap and re-observes the start cones, we want big global correction
   - Simplest: just rely on data association to re-match landmarks on the second lap. iSAM2 will absorb the constraint.
   - **Pitch**: nothing special needed; if data association is robust, loop closure is automatic.

## 7. Validation plan

**Use the same offline-replay harness we built for fast_LIMO** (`tools/replay.sh`). It already feeds bag data into a SLAM node and prints LIMO-vs-GT comparison. Just point it at our new node instead of fast_LIMO.

**Acceptance bar (sim):**
- Standstill: <0.1 m position drift over 60 s (fast_LIMO already passed this)
- Slow drive (≤2 m/s, gentle turns): <2 m position error at any point during a 60 s drive
- Aggressive drive (≤4 m/s, hard turns): <5 m position error at any point during a 60 s drive
- Loop closure: when the car returns to spawn after a full lap, position error should be <1 m (GraphSLAM superpower over filter SLAM)

If we hit these, we're done; if not, the failure mode tells us what to debug (DA failures → improve gating; drift on straights → tighten IMU covariances; etc.).

## 8. Out of scope for this branch

- **Multi-session / re-loading saved maps.** Each run starts fresh.
- **Cone color from camera/RGB.** Stays spatial-Y-sign + height (matches `project_no_cameras_on_real_car.md`).
- **Track topology inference** (start-line detection, lap detection). Path planning may want this; not our problem here.
- **Real-time guarantee on real car.** Python implementation may need port to C++ before deployment, but sim parity is the bar for this branch.

## 9. Sequencing

This will be a multi-PR effort. Suggested order:

1. **PR A (this branch):** scaffold the package, IMU preintegrator + bias init + pose graph with `BetweenFactor` only (no cones yet). Validate against bag — should match GT well at standstill, drift gracefully on motion (no scan-match correction yet).
2. **PR B:** add cone observation factors + data association layer. Validate against bag — should hit the slow-drive acceptance bar.
3. **PR C:** add `GPSFactor`, tune covariances. Validate aggressive drive.
4. **PR D:** strip fast_LIMO, livox_msgs_shim, Sophus, GTSAM-from-source from Dockerfile (use the install we restore). Replace `Publicar_Mapa` in launch. Revert throttle cap.
5. **PR E:** loop-closure regression test, real-car prep notes.

PRs A–C land first to validate. D is pure cleanup once we trust C. E is paperwork.
