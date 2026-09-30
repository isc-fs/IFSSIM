# Sim Benchmark Toolkit

This folder contains simulation-only benchmarking utilities and must stay
outside `pipeline/` so it is not carried into the car submodule.

## Scripts

- `capture_benchmark_bag.py` — records simulator-only topics plus a manifest.
- `run_sim_bag_benchmark.py` — runs every ground-truth benchmark (perception and SLAM) on one simulator bag as one session; `--only` / `--skip` pick benchmarks. See [Simulator bag benchmarks in one command](#simulator-bag-benchmarks-in-one-command).
- `run_provenance.py` — records which code produced each run (`provenance.json` + diffs). See [Which code produced a run](#which-code-produced-a-run).
- `pipeline_overrides.py` — applies pipeline parameter overrides (`--pipeline-overrides`) and records the values in effect. See [Running from a spec](#running-from-a-spec-bench-run).
- `specs/` — preset run specs for `bench-run`.
- `run_mission_benchmark.py` — the pipeline drives the missions in the simulator, scored by the referee; defaults in `missions.yaml`. See [Mission benchmarks](#mission-benchmarks).
- `run_perception_benchmark.py` — offline perception replay + sim GT comparison (latched `/testing_only/track` layout + odom at LiDAR stamp, FOV-gated matching).
- `perception_metrics.py` / `perception_report.py` — matching, error stats, detailed HTML (BEV plots, histograms).
- `perception_sanity_plots.py` — visual check of one scan per bag (sim or real): interactive 3D ground removal, DBSCAN clusters with per-cluster gate verdicts, and a single-cluster cone-fit view. `python tools/sim_benchmark/perception_sanity_plots.py --bag <sim_bag> --bag <real_bag>`
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

## Simulator bag benchmarks in one command

`run_sim_bag_benchmark.py` runs the perception and SLAM benchmarks on one simulator
bag and keeps both results together as one session:

```bash
python tools/sim_benchmark/run_sim_bag_benchmark.py results/capture/<bag>            # everything
python tools/sim_benchmark/run_sim_bag_benchmark.py <bag> --only perception --profile
python tools/sim_benchmark/run_sim_bag_benchmark.py <bag> --skip perception --motion-model imu
python tools/sim_benchmark/run_sim_bag_benchmark.py <bag> --dry-run                   # print the commands
```

```text
results/sim_bag/<bag>_<ts>/
  session.json              which benchmarks ran, with which options, exit codes, durations
  provenance.json, *.diff   the code state (next section)
  perception/base_<ts>/     the perception benchmark's usual output, report.html included
  slam/trackdrive_<ts>/     the SLAM benchmark's usual output
```

- The ground-truth gating options (`--gt-range-m`, `--gt-hfov-deg`, `--gt-min-range-m`,
  `--gt-scan-period-ms`) go to both benchmarks, so they score against the same cones.
- Common options have their own flags (`--profile`, `--max-frames`, `--strategy`,
  `--motion-model`, `--resynth-odom`). Anything else goes through `--perception-args "..."` or
  `--slam-args "..."`.
- A benchmark that fails does not stop the others (`--stop-on-error` to change that). The
  session is then marked `partial`.
- It refuses a bag with no `/testing_only/track` and `/testing_only/odom`: that is a bag from
  the car, which has no ground truth.
- When a tracking server is configured, the session is uploaded once, at the end (the two
  benchmarks do not upload on their own). `bench-view` shows it as one report under
  **Simulator bag benchmarks** ([IFS-DV-BENCHWEB](https://github.com/isc-fs/IFS-DV-BENCHWEB)).

## Which code produced a run

Every run folder made by `common.make_run_dir` gets a `provenance.json`: the IFSSIM and
`pipeline` commits and branches, whether either tree had uncommitted changes, the Docker image,
and the command. When there are uncommitted changes (untracked files included, up to 512 KB
each) they are saved next to it as `ifssim.diff` / `pipeline.diff`, so the run can be rebuilt
from its commit plus the diff.

- `label` is what people read: `a64350a` for a clean commit, `a64350a-dirty.3f2c1a9e` when
  something was uncommitted. The suffix is the `code_id`, a hash of both commits and both
  diffs, so the same code always gets the same id. Two runs with the same id are reruns; a
  fix, committed or not, gives a new id.
- The container has no `.git`, so `maybe_reexec_in_docker` records it on the host and hands it
  over through `IFSSIM_PROVENANCE_DIR` (a staging folder under `results/.provenance/`, removed
  after the run).
- Recording never fails a benchmark: what it cannot read is recorded as unknown, and any
  error is printed as a warning.
- Commits that no remote has (as of the last `git fetch`) are saved as
  `<repo>.unpushed.bundle`, and the run warns you to push them. `git fetch <bundle> HEAD`
  gets them back on any machine.
- The Docker image is recorded with its registry digest when it was pulled. A pulled image's
  digest is part of the `code_id`. A locally built image can't be matched across machines,
  so the run warns about it.
- `python tools/sim_benchmark/run_provenance.py` prints what a run started now would record.

## Mission benchmarks

`run_mission_benchmark.py` runs acceleration, skidpad, autocross and trackdrive with the pipeline
driving, scored by the sim's referee: per mission, each track, each repeat (seed). It drives each run
the way the Mission Control panel does (load the track, `resetScenario <seed>`, start the event,
follow the referee, stop), over Mission Control's HTTP API and the sim's RPC, so it needs no ROS.

**Which pipeline** (`--stack`): by default (`own`) the benchmark starts its own pipeline container,
`bench-dv-stack`, with a copy of this checkout's `pipeline/` mounted in and rebuilt at start-up
(`DV_REBUILD_ON_STARTUP`), so the runs use this checkout's pipeline commit, whatever the image was
built from. The image is `$IFSSIM_DV_IMAGE` (the launcher's worker sets it), else `--image`, else the
one `docker compose build` makes. The first start takes a few minutes (colcon build, Numba warm-up);
the container is removed at the end and its log kept as `results/mission/stack_*.log`.
`--stack running` uses the `dv_pipeline_stack` already running instead (quick tries; the recorded
commit is then only this checkout's, which may not be what that container runs).

Only one pipeline may drive the sim: with `--stack own` the benchmark refuses while a compose
`dv_pipeline_stack` runs, or stops it for the benchmark and starts it again after with
`--replace-stack`.

**Where the pipeline runs** (`--pipeline-on`):

- `bench_pc` (default): on this computer. To try it on any computer, without the latte panda:
  start the sim and Mission Control, then
  ```bash
  python3 tools/sim_benchmark/run_mission_benchmark.py --missions acceleration
  python3 tools/sim_benchmark/run_mission_benchmark.py --missions trackdrive --laps 3 --repeats 3 --start-noise
  ```
- `latte_panda`: the benchmark's container runs only the sim bridge here, and the latte panda runs
  the autonomy against it (`ros2 launch bringup sim_pipeline.launch.py`). Give the commands that
  start and stop it there, e.g. over ssh: `--panda-start`, `--panda-stop`, and `--panda-sha` to
  record which commit it runs. Without them it refuses to start.

Each run is a folder under `results/mission/<mission>_<track>/<commit>/seed<k>/` (`manifest.json`
with the referee state, `results.json`, `laps.csv`, `events.csv`, a coarse `telemetry.csv`), the
layout the tracker's simulator pages read. Settings: `missions.yaml` (tracks, repeats, start pose
noise, laps, timeouts), changed per run with the flags or `--mission-overrides <file.json>`.

**Pipeline parameter overrides** (`--pipeline-overrides <file.json>`, `{node: {param: value}}`; the
`pipeline:` section of a run spec): checked against the pipeline's `bringup/config/params.yaml`
(an unknown node or parameter, or a wrong type, stops the benchmark before anything runs), merged
into the copy mounted into the stack, and recorded in each run's `manifest.json` (`params`).
`cone_detection` is `cone_detection_node`; a dotted name is a nested parameter. They need
`--stack own` on the bench PC, and a pipeline commit with `params.yaml` (IFS09-DV-PIPELINE #8).

**Which code ran:** each run records where the pipeline ran and how its commit is known: this
checkout's (`--stack own`), the latte panda's (`--panda-sha`), or unsure (`--stack running`).

## Running from a spec (`bench-run`)

A **run spec** is a YAML file that says which benchmarks to run, on which bags, with which
settings and pipeline parameter overrides. `bench-run` (in [IFS-DV-BENCHWEB](https://github.com/isc-fs/IFS-DV-BENCHWEB), a checkout next to this one) runs one on this
machine, one job at a time. What it can run is listed in `bench.yaml` at the repository root.
Presets are in `specs/`.

```bash
cd ../IFS-DV-BENCHWEB                                    # next to IFSSIM, or set IFSSIM_ROOT
uv run bench-run --list                                   # benchmarks, settings, bags, presets
uv run bench-run sim-bag --dry-run                        # the merged spec and the commands
uv run bench-run sim-bag-quick --set benchmarks.sim_bag.bags=<bag>
uv run bench-run sim-bag --set pipeline.cone_detection.residual_gate_mse=0.05 \
                         --set pipeline.slam_node.motion_model=imu
```

```yaml
benchmarks:
  sim_bag:
    bags: all                 # or a list of folder names under results/capture/
    only: [perception, slam]
    repeats: 1
    settings: {gt_range_m: 20.0, max_frames: 200}
pipeline:                     # parameter overrides, checked against the pipeline's own names
  cone_detection: {residual_gate_mse: 0.05}   # ConeDetectionConfig fields
  slam_node: {motion_model: imu}              # ConeGraphSlamNode ROS parameters
sweep:                        # one job per value
  pipeline.cone_detection.residual_gate_mse: [0.02, 0.05, 0.1]
```

The benchmarks take the overrides as `--pipeline-overrides <file.json>`
(`pipeline_overrides.py`). A misspelled parameter stops the run before it starts. Each run
folder gets `params/<component>.json` (every value in effect, and which were overridden), and
`spec.json` when `bench-run` started it. Runs with the same code but other settings get another
**spec id**, so the viewer doesn't show them as reruns. The format:
`bench_tracking/launch/spec.py` in IFS-DV-BENCHWEB.

## Uploading to the team server

Benchmarks don't upload anything themselves. Upload finished runs by hand with
`bench-track sync` (in [IFS-DV-BENCHWEB](https://github.com/isc-fs/IFS-DV-BENCHWEB)), which sends every finished run the server does not
have yet, once. The server and login come from `~/.config/ifssim-bench/tracking.env`.
Setting up the server and a machine: IFS-DV-BENCHWEB's `DEPLOY.md`.

Benchmarks are meant to run on the central machine, launched from the web page, on PRs
and on commits (not built yet; design in
`docs/history/2026-09-28_benchmark-launcher-design.md` in IFS-DV-BENCHWEB).
Running them on your own machine is the fallback.

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
