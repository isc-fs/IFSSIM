# Cone-Graph SLAM — Progress & Handoff

**Last updated:** 2026-04-27
**Branch:** `feat/28-cone-graph-slam`
**Companion design doc:** [`cone_graph_slam_design.md`](cone_graph_slam_design.md)

This file tracks the day-by-day state of the cone-graph SLAM build so a fresh chat can resume without re-reading the entire conversation transcript. Append new sections at the bottom; do not rewrite history.

---

## TL;DR for whoever picks this up next

- **PR A** landed (`a2f60f2`): GTSAM/iSAM2 plumbing, IMU preintegration, anchored pose graph. Yaw tracks GT within 4° over 70 s.
- **PR B step 1** landed (`895ec0b`, `2767f1c`): cone factors, color-gated nearest-neighbor DA, Huber robust loss, `relinearizeSkip=10`. Standstill <0.04 m, mid-drive 0.4–1.6 m, **cascade at t≈52 s**.
- **PR B step 2a** in working tree (this commit): range-dependent observation sigmas + 25 m range cap. Standstill unchanged, mid-drive unchanged, **cascade pushed to t≈63 s** (+11 s).
- **The remaining cascade is structural**, not data-association: it triggers when a scan window has 0–1 cones and iSAM2 freely rotates the global frame to find a cheaper minimum. Cone-side tuning cannot fix it.
- **Recommended next step: PR C — GPS factor** (`/gps` is already on the bus). Adds a global anchor every scan that no empty-scan window or bad association can overpower. ~30 lines.

---

## 1. Where the code lives

```
pipeline/cone_slam/
├── package.xml
├── setup.py
└── cone_slam/
    ├── __init__.py
    ├── color_classifier.py        # Y-sign + height threshold
    ├── cone_graph_slam_node.py    # main rclpy node — state machine + pubs/subs
    ├── data_association.py        # color gate → distance gate → Hungarian
    ├── factor_graph.py            # GTSAM iSAM2 wrapper, X/V/B/L symbols
    ├── imu_preintegrator.py       # BMI088 noise model + bias estimation
    └── landmark_db.py             # persistent cone IDs, color-locked at first sight
```

Built into `ifssim-dv_pipeline_stack:latest` via `COPY pipeline/ src/` in `docker/dv_pipeline_stack/Dockerfile`. The gtsam wheel pins to `4.2.0` in the same Dockerfile.

`tools/replay.sh` accepts a 3rd arg `[fast_limo|cone_slam]`. With `cone_slam` it spawns `Cone_Detection` (provides `/Conos_raw`) alongside `cone_graph_slam`.

## 2. Resumption checklist for a new chat

1. Confirm working tree is clean: `git status` — should be on `feat/28-glim-localization`.
2. Confirm the image is up-to-date with the cone_slam tree:
   ```
   docker run --rm --entrypoint=/bin/bash ifssim-dv_pipeline_stack:latest \
     -lc "ls /dv_pipeline_stack_ws/install/ | grep cone_slam"
   ```
   If empty: `docker compose build dv_pipeline_stack && docker compose up -d --force-recreate dv_pipeline_stack`.
3. Reproduce the current baseline:
   ```
   tools/replay.sh clean_drive_75s 80 cone_slam
   ```
   Expected: standstill <0.05 m, drive tracks within ~1–2 m to t≈63 s, cascade after.
4. Pick up from §6 ("Open work") below.

## 3. PR-by-PR results matrix

All numbers from `tools/bags/clean_drive_75s` (75 s recorded bag with ≥3 s standstill, BMI088 noise, GT odometry recorded inline). Replayed via `tools/replay.sh`.

| PR | Standstill | Mid-drive | Cascade onset | Notes |
|----|-----------|-----------|---------------|-------|
| baseline (fast_LIMO) | 0.02 m | 0.5–2 m | none in 75 s window | reference impl |
| **PR A** (`a2f60f2`) | <0.05 m | drift, ~3 m by 25 s | n/a — no cones | IMU-only pose graph |
| **PR B step 1** (`895ec0b`+`2767f1c`) | <0.03 m | 0.4–3 m to t≈25 s | **t≈52 s** → 30 km err | first version with cone factors |
| **PR B step 2a** (this commit) | <0.04 m | 0.4–1.6 m to t≈50 s | **t≈63 s** → 15 km err | range-dep sigmas + 25 m cap |

Per-second pose-cmp tables saved to `tools/bags/<bag>/replay_pose_cmp_cone_slam.txt`.

## 4. What PR B step 2a actually changed

### 4a. Range-dependent observation sigmas (`factor_graph.py`)

The bearing-range factor's noise model now scales with measurement distance:

```python
range_sigma   = 0.05 + 0.005 * range_m   # 5 cm + 5 mm/m
bearing_sigma = 0.02 + 0.001 * range_m   # ≈1.1° + linear growth
```

Why: cluster point count ∝ 1/d² (MUR's `num_expected_points(d)` rule, AMZ §3.2), so centroid variance grows with range. The previous constant `0.20 m` lied to iSAM2 — a 25 m cone with true ±1 m error reported as ±0.2 m got ≈25× the weight it deserved, which is what was driving the late-drive yaw snap on PR B step 1.

### 4b. Range cap at 25 m (`cone_graph_slam_node.py`)

`_observations_from_markers` drops any observation with `body_x² + body_y² > 25²` before it reaches data association. `MAX_OBSERVATION_RANGE_M = 25.0`.

Why: universal practice across FSD teams (EUFS 20 m, QUTMS 25 m, MUR ≈25 m, KIT19d 42 m). At 25 m the Hesai ATX gives ≈3–5 rays per cone; the centroid variance dominates over the bearing-range factor's information content. Filter here (not in `Cone_Detection`) so `/Conos_raw` stays untouched for visualization/debug.

## 5. Cone detection audit findings (the input to PR B step 2a)

Three parallel agents surveyed: (1) our local `pipeline/slam/`, (2) AMZ Driverless (Kabzan et al. 2019), (3) other FSD teams (MUR, EUFS, QUTMS, KIT19d, TUM, MIT).

### 5a. Our pipeline (`pipeline/slam/cone_detection*.py`)

- PointCloud2 → RANSAC ground (thr 0.05 m, 50 iter) → DBSCAN (eps=0.3 m, min_samples=2) → **two-phase apex+height fit with L-BFGS-B, c=5.5 d=0.35 hardcoded** → publish `/Conos_raw`.
- **No per-cone covariance.** Point estimates only. Downstream sigmas were hardcoded constants.
- **Fixed cone-shape constants** `c=5.5, d=0.35` are tuned for small cones; big-orange (0.50 m vs 0.32 m) systematically mis-fits phase 1.
- **All markers published magenta** (R1,G0,B1) — color is a downstream re-classification (`color_classifier.py` does Y-sign + height threshold).

### 5b. AMZ Driverless (the gold standard)

LiDAR-only pipeline (camera is independent, fused only at SLAM):
1. Motion compensation
2. **Himmelsbach ground removal** (polar sector/bin — much more robust on slopes than RANSAC)
3. Euclidean clustering on ground-removed cloud
4. **Cone reconstruction trick**: cluster on ground-filtered cloud, then recover full cone (apex + base) by carving a cylindrical region from the **unfiltered** cloud
5. **Geometric-prior validation**: analytic `expected_point_count(distance, sensor_resolution)` vs actual cluster count; outliers rejected
6. Color from intensity via 32×32 CNN; reliable to ~5 m

Reported: **0.20 m RMSE over 230 m track**. Position trusted to ~15 m.

### 5c. Other teams — universal patterns

| Team | Ground filter | Detection | Range cap | Validation |
|------|--------------|-----------|-----------|-----------|
| MUR | RANSAC | Cluster centroid | varies | `num_expected_points(d) ∝ 1/d²` (only public range-aware filter) |
| QUTMS | RANSAC | Cluster centroid | 25 m | 15-pt cap, 0.10 m radius cap |
| EUFS | none | Grid pattern-match | 20 m | — |
| KIT19d | none | DBSCAN (lets ground go to noise) | 42 m | — |
| TUM/MIT | RANSAC | Cluster centroid | ~30 m | — |

Three universal patterns we deviated from:
1. **Everyone uses cluster centroid.** Nobody fits a parametric cone shape. Our two-phase L-BFGS-B fit is unique.
2. **Everyone range-caps at 20–35 m.** We had no cap until PR B step 2a.
3. **Nobody publishes per-cone covariance** scaling with range. Was the universal gap until PR B step 2a took the simplest version (linear-in-range).

## 6. Open work — what to do next

### 6a. PR C: GPS factor (RECOMMENDED FIRST)

**Goal:** add `gtsam.GPSFactor(X(k), [x, y, 0], gps_noise)` every scan to give the optimizer a global (x, y) anchor.

**Why this fixes the cascade:** the failure at t≈63 s on PR B step 2a happened in the SLAM log around step 470:
```
step=460  obs=2  new=2  assoc=0  pose=(+50.26,-33.88, yaw=-139.5°)
step=470  obs=0  new=0  assoc=0  pose=(+77.12,-40.89, yaw=-18.6°)   ← empty scan
step=480  obs=1  new=1  assoc=0  pose=(-132.83,+175.30)             ← cascade
```

Step 470 has zero cones; the optimizer runs only on IMU + bias factors. At the next relinearization (every 10 updates per `relinearizeSkip=10`) iSAM2 finds a globally-cheaper trajectory by rotating the world frame. Once the pose snaps wildly, every existing cone reprojects out of the data-association gate, every observation becomes a "new" landmark, and the optimizer has nothing left to anchor on.

A GPS factor at every scan stops this dead. Even a loose σ=1 m anchor pins the global frame and makes any rotation expensive enough to dominate Huber-capped cone residuals.

**Sketch:**
```python
# In ConeGraphSlamNode.__init__:
self.create_subscription(NavSatFix, "/gps", self._on_gps, sensor_qos)
self._latest_gps: Optional[NavSatFix] = None

# In _on_cones, after stage_imu_factor:
if self._latest_gps is not None:
    enu = self._gps_to_enu(self._latest_gps)  # subtract calibration origin
    self._graph.stage_gps_factor(enu_x, enu_y, sigma=1.0)

# In FactorGraph:
def stage_gps_factor(self, enu_x: float, enu_y: float, sigma: float) -> None:
    noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([sigma, sigma, 1e6]))
    self._new_factors.add(gtsam.GPSFactor(
        X(self._k), gtsam.Point3(enu_x, enu_y, 0.0), noise))
```

ENU calibration origin: take the GPS fix at the moment we exit `INIT_CALIBRATING` (car is stationary at world origin per pose anchor) and subtract it from every subsequent fix.

**Memory note (from `project_no_gss_on_real_car`)**: the existing `/fsds/gss` is a no-go because real IFS-08 has no ground-speed sensor. **GPS is fine** — it's a different sensor and IFS-08 has it. The existing IFSSIM bridge already publishes `/gps` (saw the foxglove channel in the replay log).

### 6b. PR B step 3 (deferred): Mahalanobis gating

Switch `data_association.py` from Euclidean distance gating to Mahalanobis with each landmark's iSAM2-reported covariance. Lower priority because:
- It addresses *bad associations once they happen*; doesn't help when there are *no* observations (which is the failure mode on this bag).
- Needs to plumb landmark covariance from iSAM2 (`marginalCovariance(L(id))`) through to the DA layer; non-trivial.

### 6c. PR D: cleanup (after PR C lands clean)

- Revert any throttle cap on the controller side (was applied while debugging cone_slam).
- Strip fast_LIMO from `Dockerfile` (clone, build, config copy).
- Replace `pipeline_only.launch.py` SLAM choice with cone_slam.
- Delete `pipeline/cone_slam` legacy fast_LIMO config.

## 7. Things that are settled and shouldn't be re-litigated

- **Pose3 internally, not Pose2.** GTSAM's `ImuFactor` is defined for `Pose3 + Vector3 velocity` only. Forcing Pose2 means hand-integrating IMU and losing efficient covariance propagation. We accept the extra DOF; project to 2D when publishing TF.
- **GTSAM 4.2.0 mixed API.** `setRelinearizeThreshold` is a method; `relinearizeSkip` is a property (no setter). Both are used in `factor_graph.py`. Don't try to make them consistent.
- **IMU at rest reads +g (specific force = -gravity).** `accel_bias = mean - (0,0,+9.81)`, NOT `-9.81`. Wired into `imu_preintegrator.py`.
- **IMU subscription depth must be ≥ 2000.** rclpy single-threaded executor cannot service `_on_imu` while `_on_cones` is running an iSAM2 update. With queue=10 the preintegrator sees ≈3–5 samples per 100 ms scan instead of ≈40.
- **MarkerArray has no top-level header.** Pull the stamp from the first non-DELETEALL marker in `_stamp_msg()`.
- **Cone color is locked at first observation.** Re-classifying every scan defeats the point of persistent landmark IDs.

## 8. The replay harness — how to read its output

`tools/replay.sh <bag> <duration> [fast_limo|cone_slam]`:
- Spins an ephemeral container off `ifssim-dv_pipeline_stack:latest`, isolated on `ROS_DOMAIN_ID=42` so it doesn't interfere with a live `dv_pipeline_stack` container.
- For `cone_slam`: starts `Cone_Detection` (provides `/Conos_raw`), waits 3 s for numba JIT, starts `cone_graph_slam`, then `ros2 bag play`.
- Streams `tools/pose_cmp.py` output: per-second `SLAM xy yaw | GT xy yaw | dist_err`. Saved to `<bag>/replay_pose_cmp_<impl>.txt`.

Failure-mode reading guide:
- **Standstill error growing slowly** → IMU bias estimation is off (check `_finish_calibration` log line for accel_bias z near zero).
- **Mid-drive error ~constant** → tracking working; IMU+cones are mutually consistent.
- **Sudden jump >5 m in one second** → iSAM2 found a cheaper global rotation. Either (a) bad cone association at that timestamp, or (b) sparse-observation window unanchored. Look at the matching SLAM log line for `obs=`/`new=`/`assoc=` counts.
- **Cascade (err doubling every second)** → optimizer has reached a regime where every scan's predicted pose is so far off that no observation matches; every cone becomes a new landmark; nothing left to constrain the graph.

## 9. Useful one-liners

```bash
# Verify cone_slam is in the latest image:
docker run --rm --entrypoint=/bin/bash ifssim-dv_pipeline_stack:latest \
  -lc "ls /dv_pipeline_stack_ws/install/cone_slam/lib/cone_slam/"

# Tail SLAM log mid-replay:
docker exec ifssim-replay-<pid> tail -f /tmp/slam.log

# AST-check edits without container rebuild:
docker run --rm --entrypoint=/usr/bin/python3 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$(pwd):/repo:ro" ifssim-dv_pipeline_stack:latest \
  -c "import ast; [ast.parse(open(p).read(), p) for p in ['/repo/pipeline/cone_slam/cone_slam/factor_graph.py','/repo/pipeline/cone_slam/cone_slam/cone_graph_slam_node.py']]; print('OK')"

# Compare runs:
diff tools/bags/clean_drive_75s/replay_pose_cmp_fast_limo.txt \
     tools/bags/clean_drive_75s/replay_pose_cmp_cone_slam.txt
```

---

## Appendix A: bag inventory

```
tools/bags/
├── azim_drive_75s        # azimuth-only drive, no real motion
├── clean_drive_75s       # PRIMARY validation bag — clean trackdrive, ≥3 s standstill, full lap
├── inst_drive_75s        # instant-throttle drive
├── iter1_drive_60s       # iter-1 fast_LIMO tune
└── slow_drive_75s        # low-speed drive
```

`clean_drive_75s` is the canonical replay target. Other bags exist for stress testing (high yaw rate, instant throttle, etc.) and should be brought online once `clean_drive_75s` runs cascade-free for the full 75 s.
