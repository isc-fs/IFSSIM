# Sim Benchmark Toolkit

This folder contains simulation-only benchmarking utilities and must stay
outside `pipeline/` so it is not carried into the car submodule.

## Scripts

- `capture_benchmark_bag.py` — records simulator-only topics plus a manifest.
- `run_perception_benchmark.py` — offline perception replay + sim GT comparison (latched `/testing_only/track` layout + odom at LiDAR stamp, FOV-gated matching).
- `perception_metrics.py` / `perception_report.py` — matching, error stats, detailed HTML (BEV plots, histograms).
- `run_slam_benchmark.py` — offline SLAM replay vs sim GT (gated track cones, pose error vs `/testing_only/odom`).
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
   - Bag and results paths must live under `tools/sim_benchmark/`. Use `--no-docker` inside a sourced ROS shell to run locally.

   **Perception GT alignment — `--gt-scan-center-frac` (don't forget this).** The LiDAR
   cloud is delivered ~300 ms *stale* (DDS buffering + chunk reassembly of the 1.4 MB
   cloud; `lag_ns` only corrects the plugin capture→send portion, not the receive side).
   So GT odom looked up at the LiDAR header stamp is from ~3 scan-periods too late, and
   every matched cone shows a phantom `v·Δt` forward bias (~0.5 m at speed → `mean_err`
   0.65 m). It **defaults to `0.0` (no correction)** — omitting the flag makes a good
   detector look broken. Pass a negative value to rewind GT to the true capture moment.
   The lag is **not constant — it drifts upward over a long bag** (~70 ms early → ~430 ms
   late over a 154 s capture, as the LiDAR delivery backlog grows). A single `center_frac`
   can only match it at one instant, so the full 1541-frame bag scores ~0.5 m at *any*
   value, while a ~500-frame slice (where the lag is stable) scores ~0.12 m at `-2.5`.
   **So pin `--max-frames 500`** (≈first 50 s) to keep a single offset valid — dropping it
   runs the full bag and the back two-thirds drift out of alignment (error → 1 m, recall →
   0.57). For a full-bag eval you need a time-varying `lag(t)` (not yet built) or a shorter
   capture. Canonical run with profiling:

   ```bash
   python run_perception_benchmark.py results/capture/sim_benchmark_20260527_135117 \
     --gt-scan-center-frac -2.5 --max-frames 500 --profile --profile-frames 80 --bev-samples 8
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
