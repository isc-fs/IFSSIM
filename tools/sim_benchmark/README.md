# Sim Benchmark Toolkit

This folder contains simulation-only benchmarking utilities and must stay
outside `pipeline/` so it is not carried into the car submodule.

## Scripts

- `capture_benchmark_bag.py` — records simulator-only topics plus a manifest.
- `run_perception_benchmark.py` — offline perception replay + sim GT comparison (latched `/testing_only/track` layout + odom at LiDAR stamp, FOV-gated matching).
- `perception_metrics.py` / `perception_report.py` — matching, error stats, detailed HTML (BEV plots, histograms).
- `run_slam_benchmark.py` — offline SLAM replay vs sim GT (gated track cones, pose error vs `/testing_only/odom`).
- `run_onboard_replay.py` — live pipeline replay of an **onboard** bag (no sim GT) + HTML report of pipeline outputs.
- `slam_metrics.py` / `slam_report.py` — GT cone injection, trajectory and error plots.
- `control_benchmark_node.py` — online GT control error harness.
- `../track_driver.py` — GT pure-pursuit driver (uses `pipeline/control` Pure Pursuit).
- `generate_report.py` — aggregates JSON/CSV outputs, optional HTML report.

## Typical flow

1. Launch normal sim stack (`sim_pipeline.launch.py`) and simulator.
2. Record a reproducible bag (includes sim GT track for perception eval):
   - `python tools/sim_benchmark/capture_benchmark_bag.py --duration-s 90`
   - Pipeline must be running for `/testing_only/track`; `/odom` is optional (rebuilt offline from IMU/RPM)
3. Run offline module benchmarks (auto re-exec in `ifssim-dv_pipeline_stack` when host has no ROS).
   From the **repo root** (no `cd`):

   ```bash
   python run_perception_benchmark.py results/capture/<bag_name> --profile --profile-frames 80 --bev-samples 8
   ```

   Or from `tools/sim_benchmark/`:

   - `python run_perception_benchmark.py results/capture/<bag_name>`
   - `python tools/sim_benchmark/run_slam_benchmark.py <bag_path>` → `results/slam/<strategy>_<ts>/report.html`
     (needs `/testing_only/track`, `/odom`, `/imu`; see capture notes below)
   - Onboard / car bags (no `/testing_only/*`):
     `python tools/sim_benchmark/run_onboard_replay.py results/capture/<bag_name>`
     Plays `/imu` `/lidar_points` `/motor_rpm` `/steering_angle` into the live
     autonomy nodes. Add `--report` to record pipeline outputs and write
     `results/onboard/<mission>_<ts>/report.html` (detection counts, odom/SLAM
     trajectories, map, autonomy vs pilot steering). Without `--report` there
     is no second bag. There is no precision/recall — there is no GT.
     `--duration-s 30` clips a long bag; `--rate 1.0` keeps control timing.

     To watch in Lichtblick / Foxglove while it plays:

     ```bash
     python tools/sim_benchmark/run_onboard_replay.py results/capture/<bag_name> --live
     ```

     The replay container publishes `foxglove_bridge` on **ws://localhost:8766**
     (8766 so it does not collide with the sim stack on 8765). Open
     http://localhost:8080 → Open connection → Foxglove WebSocket → that URL.
     Layout `lichtblick/onboard_live.json` (Perception / Filter / Map / Path /
     Control / Odom / IMU / Mission / Performance tabs). Perception BEV shows
     `/lidar_points/above_ground` (RANSAC outliers, what clustering sees);
     perspective shows `/lidar_points/ground` (full rotated crop, including
     ground inliers). Both share the `base_link` plane with `/Conos_raw`.
     Diagnostic clouds/Float32s are subscription-gated so unused viz does not
     burn rotate/pack/bridge CPU.
     The bag starts as soon as Lichtblick connects (or after `--live-wait-s`,
     default 20 s, if nobody connects). `--loop` repeats until Ctrl-C.
   - Bag and results paths must live under `tools/sim_benchmark/`. Use `--no-docker` inside a sourced ROS shell to run locally.

   **Perception GT alignment.** GT odom is looked up at the LiDAR `header.stamp`, which is
   now the **absolute UE sim capture time** of the scan (bridge Option 2 — the plugin tags
   each scan with `SimCaptureNs`). That stamp is immune to GPU readback / DDS buffering /
   chunk-reassembly latency, so GT lines up with the cloud by construction — no manual
   offset. (Previously the LiDAR cloud was ~300 ms stale and *drifting*, which needed a
   per-run `--gt-scan-center-frac` + `--max-frames 500` workaround; both are gone. The
   `diagnose_perception_timing.py` sweep stays — run it on a fresh bag to confirm the best
   offset sits near 0.) Canonical run with profiling:

   ```bash
   python run_perception_benchmark.py results/capture/<bag_name> \
     --profile --profile-frames 80 --bev-samples 8
   ```
4. Run online control benchmark (CSV centerline or topic source):
   - `python tools/sim_benchmark/control_benchmark_node.py --centerline-csv <track.csv>`
   - Mission Control (Docker): results land in `tools/sim_benchmark/results/` via
     `IFSSIM_BENCHMARK_RESULTS_ROOT=/benchmark_results` (writable mount; `/ifssim_tools` is read-only).
5. Open `results/perception/<run>/report.html` in a browser. Each perception run includes:
   - **Detection vs sim GT**: precision, recall, position error (mean/median/p95)
   - **BEV plots**: GT (blue) vs prediction (red), match lines, ego marker (sample frames)
   - **Histograms / time series**: error distribution, recall and error over time
   - **Performance**: latency and cone counts per frame
   - **Pipeline profile** (optional): stage table, stacked bar chart, `profile.json` /
     `profile_stages.csv` via
     `python tools/sim_benchmark/run_perception_benchmark.py <bag> --profile --profile-frames 80`
     (add `--ransac-ablation` for RANSAC subsample A/B). Older `report.html` files
     without a profile section can be refreshed:
     `python tools/sim_benchmark/generate_report.py`

   Bags recorded before `/testing_only/track` was added only get latency charts; re-capture to enable GT plots.

### SLAM benchmark bag topics

| Topic | Purpose |
|-------|---------|
| `/imu`, `/motor_rpm`, `/testing_only/odom` | Required; `/odom` recorded or **synthesized** from sensors (same `OdometryFilter` as sim_supervisor) |
| `/steering_angle`, `/brake_pressure` | Used when synthesizing `/odom` (default capture topics) |
| `/testing_only/track` | Latched full-track layout → body-frame GT cones (FOV/range gated) |
| `/lidar/Lidar1` | Cone update cadence (~10 Hz); without it, updates fall back to sparse `/testing_only/track` |

Perception (`/Conos_raw`) is **not** used by the SLAM benchmark; use `run_perception_benchmark.py` for that.

**SLAM report outputs** (under `results/slam/<strategy>_<timestamp>/`):

| File | Content |
|------|---------|
| `pose_steps.csv` | Per-event log: EKF filter, IMU-only predict (incl. `imu_only_vx`), wheel DR (`wheel_vx`), SLAM vs GT on each IMU/RPM, supervisor `/odom`, GT odom, and cone commits |
| `samples.csv` | SLAM pose at each cone injection |
| `trajectory.csv` | GT vs SLAM at `/testing_only/odom` rate |
| `map_cones.csv` | GT track layout + final SLAM landmarks (aligned frame) |
| `report.html` | Charts: EKF filter (full), IMU-only predict (minimal), SLAM, separate GT vs IMU-only trajectory, filter+SLAM trajectory, map match, worst moments |

Re-run `run_slam_benchmark.py` on an existing bag to regenerate reports with `pose_steps.csv` (no re-capture needed).

6. Optional: `python tools/sim_benchmark/generate_report.py` refreshes all `report.html` files and writes a top-level index.
