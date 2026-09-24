# Benchmark experiment tracking — design proposal

Status: **design proposal**, followed by an evaluation build (see the update below)
Date: 2026-09-23
Scope: `tools/sim_benchmark/` (onboard bag replay today), GT perception/SLAM/control
benchmarks, and future simulator-in-the-loop (SIL) benchmarks.
Code references are to branch `feat/516-onboard-bag-replay` (HEAD 2c27956 + uncommitted edits).

> **Update:** an evaluation build of all three trackers on real replay data + mock SIL data is in
> `tools/sim_benchmark/tracking/`; results in [`2026-09-23_tracker-evaluation.md`](2026-09-23_tracker-evaluation.md).

Decisions already made and not reopened here: **W&B hosted** as the tracker, **ClearML
self-hosted** as the fallback, a **thin in-house harness** with local files as the source of
truth, **upload on the host after the run**, **bags never uploaded**.

---

## 1. What exists today (and what that means for the design)

| Fact (from the code) | Consequence |
|---|---|
| `maybe_reexec_in_docker` (`common.py`) ends with `raise SystemExit(subprocess.call(cmd))`. The host process exits as soon as the container does. | A post-run upload hook has to live in `maybe_reexec_in_docker`, between the `subprocess.call` and the `SystemExit`. |
| `make_run_dir` runs **inside** the container and picks the timestamp there. The host never learns which run dir was created. | The host has to choose the run id **before** launching the container and pass it in (see §5.3). Otherwise it would have to guess "newest dir". |
| The container mounts `tools/sim_benchmark` and `pipeline/*` **read-only, without `.git`**. | Git provenance must be captured **on the host**. The container can't do it. |
| The code under test is the host's `pipeline/` submodule, mounted over the installed packages via `PYTHONPATH=/dev_*`. The image contributes ROS, `fs_msgs`, and the C++ nodes. | Provenance needs **two SHAs plus dirty state** (IFSSIM and `pipeline`) and the **Docker image ID**. `tools/scenario_runner/run_scenario.py::git_sha` already does the SHA+dirty part. Reuse it. |
| Both repos are usually dirty (right now: 9 files in IFSSIM and 10 in `pipeline`). | A dirty flag isn't enough. Upload the **`git diff` patch** of both repos as a file, or a "dirty" run can't be reproduced. |
| Nodes start with default params plus a mission behaviour (`MISSION_BEHAVIORS`). No param file is passed in. | Snapshot the effective params with `ros2 param dump` per node after `activate`, inside the container, into `params/<node>.yaml`. That's the only reliable record. |
| `results.json` has container paths (`/results/...`). `duration_s` is the **requested clip** (0 = full bag), not a measured duration. `strategy` duplicates `mission`. | The adapter renames these (§4). The local file keeps its current keys so `report.html` / `generate_report.py` keep working. |
| Run dir sizes (onboard): CSVs ~0.8 MB, `logs/` ~0.5 MB, `report.html` ~0.1 MB, `samples.json` 3.5 MB, `replay_bag/` 23 MB. Source bag: 7.5 GB. | Upload the CSVs, logs, report and params. **Don't** upload `samples.json`: it duplicates the CSVs. The `replay_bag` (pipeline *outputs*, 23 MB) is an open question (Q7). |
| `results/onboard/`: 9 dirs. **Only 4 have `results.json`**: 20260921_144527, 20260922_{170038,172257,173826}. The other 5 are `--live` / no-`--report` runs with just `logs/`. All 4 replay the **same bag** (`manual_20260920_154527_indexed`). | The backfill is 4 runs, which is still useful: they're a same-bag set for validating the overlay panels. The 5 log-only dirs are skipped. |
| The SIL building blocks already exist: sim RPC (`tools/mission_control/backend/sim_client.py`: `load_track`, `set_event`, `get_referee_state`, `teleport`), `resetScenario <seed>` (restores RNG, referee, pose), `simContinueForTime`, and `tools/scenario_runner/run_scenario.py` (N seeds, per-run manifest with both SHAs + referee state). | The SIL benchmark runner should **extend `scenario_runner`**, not start over. The referee state (`Laps[]`, `DooCounter`, `OffTrackCounter`, `bFinished`, `RequiredLaps`) is the race-metric source. |

---

## 2. Architecture

```
            ┌──────────── host ────────────────────────────────────────────┐
 CLI ──►    │ 1. capture provenance (git SHAs, diffs, image id, bag id)    │
            │ 2. choose run_id, write provenance.json into run dir         │
            │ 3. docker run … (IFSSIM_RUN_ID=…)  ──► container writes:     │
            │        results.json, *.csv, report.html, logs/, params/      │
            │ 4. if --track: bench_tracking.upload(run_dir) ──► sink(s)    │
            └──────────────────────────────────────────────────────────────┘
                                    │
                     Tracker (interface)  ──►  WandbBackend   (now)
                                          ──►  ClearMLBackend (fallback)
                                          ──►  NullBackend    (default / tests)
```

* **Upload is a separate, re-runnable step**: `python tools/sim_benchmark/track.py upload <run_dir>`.
  The post-run hook only calls it. The same command handles backfill and retrying a failed upload.
* **Failure isolation**: the upload runs after the benchmark exit code is known, catches every
  exception, prints a warning, and leaves the benchmark exit code unchanged. A run that failed to
  upload gets `tracking.json = {"status": "failed", "error": …}` so `track.py upload --pending`
  can find it later.
* **Idempotency**: after a successful upload the run dir gets
  `tracking.json = {"backend": "wandb", "run_id": …, "url": …, "uploaded_at": …, "schema_version": …}`.
  A second upload is skipped unless `--force`, which creates a new tracker run and tags the old
  one `superseded`. W&B can't overwrite logged history, so a forced upload always means a new run.
* **Offline machines** (GPU sim boxes): the W&B backend honours `WANDB_MODE=offline`. `wandb sync`
  (or `track.py sync`) pushes later. ClearML has an equivalent offline mode.

---

## 3. Per-run logging schema

Every benchmark kind produces the same five sections. The adapter (§5.2) maps each benchmark's
local files onto them.

### 3.1 Config (immutable, filterable)

| Key | Type | Source | Notes |
|---|---|---|---|
| `schema_version` | int | harness | Starts at 1. Bumped when names or meanings change. |
| `job_type` | str | CLI | `onboard_replay`, `perception_gt`, `slam_gt`, `control_gt`, `control_closed_loop`, `pipeline_e2e` |
| `code.ifssim.sha` / `.branch` / `.dirty` | str/str/bool | host git | Diff uploaded as `provenance/ifssim.diff` |
| `code.pipeline.sha` / `.branch` / `.dirty` | str/str/bool | host git (submodule) | Diff uploaded as `provenance/pipeline.diff`. The pipeline is the code under test. |
| `code.image.id` / `.tag` | str | `docker image inspect` | `IFSSIM_DV_IMAGE`. `null` for `--no-docker`. |
| `code.native_filter` | bool | host | Whether `_native/odometry_filter_py*.so` was mounted (C++ EKF vs Python fallback). |
| `scenario.id` | str | harness | Stable hash of the scenario fields below. Used as the join key for "same scenario". |
| `scenario.kind` | str | CLI | `bag` or `sim` |
| `scenario.bag.name` / `.id` / `.duration_s` / `.topics` | str/str/float/list | bag metadata + hash | `.id` = content hash (§3.6) |
| `scenario.mission` | str | CLI | `trackdrive`, `autocross`, `accel`, `skidpad` |
| `scenario.behaviors` | dict | `MISSION_BEHAVIORS[mission]` | node → behaviour |
| `scenario.rate` / `.clip_s` | float | CLI | `clip_s` = current `duration_s` (0 = full bag) |
| `scenario.track.name` / `.id` | str | SIL only | Track CSV name + content hash (W&B Artifact) |
| `scenario.sim.build` / `.plugin_sha` / `.seed` / `.noise` / `.event` / `.laps` | … | SIL only | `noise` = the `FSDSSettings` noise-std block |
| `params` | dict | `params/<node>.yaml` | Flattened `node.param`. Filterable, so it works for sweeps. |
| `env.user` / `.host` / `.gpu` / `.started_at` / `.wall_s` | … | host | |
| `provenance.complete` | bool | harness | `false` for backfilled runs (§8) |

### 3.2 Summary scalars

Canonical names from §4. One value per run. These drive the runs table, parallel-coordinates
panels, baselines and alerts. Where a baseline exists (§6.3), the harness also logs
`<metric>.delta_vs_baseline` for the headline metrics.

### 3.3 Series (native W&B line plots, overlay across runs for free)

Each series family gets its **own step metric** through `define_metric`, because topics run at
different rates (control ~40 Hz, LiDAR ~10 Hz):

| Family | Step metric | Keys | Source |
|---|---|---|---|
| perception | `t/perception_s` | `perception/n_conos_raw` | `perception.csv` |
| control | `t/control_s` | `control/throttle`, `control/steering_rad`, `control/brake` | `control.csv` |
| steering | `t/control_s` | `control/steer_residual_rad` (autonomy − pilot) | computed |
| SIL (later) | `track/s_m` (distance along centerline) | `control/cross_track_m`, `control/heading_err_rad`, `vehicle/speed_mps`, … | §7.3 |

Time is **seconds since the first pipeline output** (as `_shift_times` already does), not
wall time, so same-bag runs line up.
Downsampling: W&B keeps full history but samples line plots for display. The harness logs at native
rate up to about 10k points per series and decimates beyond that. The full-resolution CSVs are
always uploaded as files.

### 3.4 Tables (for plots native panels can't draw)

| Table | Columns | Used by |
|---|---|---|
| `trajectory` | `t_s, source ∈ {odom, slam, gt}, x, y, frame` | XY route overlay (custom Vega chart) |
| `map_cones` | `x, y, color, source ∈ {slam, gt}, frame` | Cone map overlay |
| `laps` (SIL) | `lap, time_s, doo, off_track, finished` | Lap comparison |
| `events` (SIL) | `t_s, s_m, kind, detail` | First-failure time and cause |

`frame` records the alignment (§6.4): `raw`, `start_pose`, or `fit:<ref_run>`. Tables are
decimated to ≤ 2 000 rows for plotting. The full CSV is still attached.

### 3.5 Files

`report.html` (also logged as `wandb.Html`, so it renders in the run page), all `*.csv`, `logs/`
(tar.gz), `params/`, `provenance/*.diff`, `provenance.json`, `results.json`. Not uploaded:
`samples.json`, source bags, and (pending Q7) `replay_bag/`.

### 3.6 Bag identity

A full SHA-256 of a 7.5 GB bag takes ~15–30 s. That's fine once, but not on every run. Proposal:

* `bag_id = sha256(metadata.yaml ‖ for each storage file: name, size, sha256)[:16]`, computed once
  and cached in `<bag>/.bench_id.json` together with each file's `(size, mtime)`. Recomputed only
  if those change.
* The run records `bag.name` (human) + `bag.id` (identity). "Same bag" means same `bag.id`, not
  same name. The `_indexed` / restamped variants get different ids, which is correct.
* The shared-storage path is a **registry lookup** (`bag.id → path`), not stored in the run
  (see Q4).

---

## 4. Metric naming convention

**Pattern:** `<domain>/<quantity>[_<stat>][@<qualifier>]_<unit>`

* **domain**: `perception`, `slam`, `odom`, `planning`, `control`, `race`, `vehicle`, `latency`,
  `source`, `run`
* **stat** (optional): `mean`, `median`, `p95`, `max`, `rms`, `rate` (a fraction), `n` (a count, as prefix: `n_…`)
* **qualifier** (optional): range/condition, e.g. `@20m`, `@lap`
* **unit** (mandatory, last): `_m`, `_s`, `_ms`, `_rad`, `_mps`, `_hz`, `_frac`. Counts have no unit (`n_` prefix).
* lower_snake_case. No abbreviations beyond `n`, `p95`, `rms`, `odom`, `slam`.

A **metric registry** (`tools/sim_benchmark/metrics.yaml`) lists each canonical name with
`direction` (`min` / `max` / `none`), `unit`, a description, and optionally
`regression_threshold`. Baseline deltas, the colour of deltas in Reports, and alerts all read from
it, and the harness warns on any metric not in the registry.

### 4.1 Mapping of current keys

| Benchmark | Current `results.json` key | Canonical name | dir |
|---|---|---|---|
| onboard | `mean_conos_raw` | `perception/n_cones_mean` | none |
| onboard | `median_conos_raw` / `max_conos_raw` | `perception/n_cones_median` / `_max` | none |
| onboard | `empty_detection_rate` | `perception/empty_frame_rate_frac` | min |
| onboard | `n_conos_raw`, `n_conos`, `n_odom`, `n_slam`, `n_path`, `n_cmd` | `run/n_msgs_conos_raw`, … | none |
| onboard | `odom_path_length_m` / `slam_path_length_m` | `odom/path_length_m` / `slam/path_length_m` | none |
| onboard | `odom_slam_end_gap_m` | `slam/end_gap_vs_odom_m` | min |
| onboard | `mean_abs_steer_residual_rad` | `control/steer_residual_mean_abs_rad` | min |
| onboard | `n_map_cones` | `slam/n_map_cones` | none (compare vs track) |
| onboard | `last_path_poses` | `planning/n_last_path_poses` | none |
| onboard | `source_duration_s`, `source_imu`, … | `source/duration_s`, `source/n_imu`, … (**config**, not metrics) | — |
| perception | precision / recall / match err | `perception/precision_frac`, `perception/recall_frac`, `perception/recall@20m_frac`, `perception/match_err_mean_m`, `…_p95_m` | max/max/max/min/min |
| slam | pose error stats | `slam/pos_err_rms_m`, `slam/yaw_err_rms_rad`, `odom/pos_err_rms_m`, `slam/map_match_err_mean_m` | min |
| control | `mean_cross_track_err_m`, `max_…`, `mean_heading_err_rad` | `control/cross_track_mean_m`, `control/cross_track_max_m`, `control/cross_track_rms_m`, `control/heading_err_mean_abs_rad` | min |
| SIL | referee | `race/lap_time_best_s`, `race/lap_time_mean_s`, `race/n_laps`, `race/finished` (0/1), `race/n_doo`, `race/n_off_track`, `race/penalty_s`, `race/first_failure_s_m` | min/min/max/max/min/min/min/max |
| all | node timing | `latency/cone_detection_p95_ms`, `latency/slam_p95_ms`, `latency/control_p95_ms` | min |

(The exact perception/SLAM source keys come from the nested dicts in `perception_metrics.py` /
`slam_metrics.py`. The adapter maps them in phase 3. The list above is the **target vocabulary**
for the team to agree on: Q6.)

`race/penalty_s` depends on the FS rules version (e.g. DOO = 2 s, OC = 10 s). The weights go in
the registry, not the code.

---

## 5. Harness

### 5.1 Tracker interface (signatures only)

```python
# tools/sim_benchmark/tracking/tracker.py
class Tracker(Protocol):
    def start(self, *, job_type: str, name: str, group: str | None,
              tags: Sequence[str], config: Mapping[str, Any],
              run_id: str | None = None, notes: str = "") -> RunHandle: ...
    def define_series(self, family: str, *, step_key: str) -> None: ...
    def log_series(self, family: str, rows: Iterable[Mapping[str, float]]) -> None: ...
    def log_summary(self, metrics: Mapping[str, float | int | None]) -> None: ...
    def log_table(self, name: str, columns: Sequence[str],
                  rows: Iterable[Sequence[Any]]) -> None: ...
    def log_file(self, path: Path, *, name: str | None = None,
                 kind: Literal["report", "data", "log", "provenance"] = "data") -> None: ...
    def log_html(self, name: str, path: Path) -> None: ...
    def use_artifact(self, kind: Literal["track", "sim_build", "bag_ref"],
                     name: str, digest: str, *, path: Path | None = None) -> None: ...
    def alert(self, title: str, text: str, level: Literal["info", "warn", "error"]) -> None: ...
    def finish(self, *, status: Literal["ok", "failed"] = "ok") -> TrackingRecord: ...

@dataclass(frozen=True)
class TrackingRecord:            # written to <run_dir>/tracking.json
    backend: str; run_id: str; url: str | None; uploaded_at: str; schema_version: int

def get_tracker(backend: str | None = None) -> Tracker: ...
    # backend from --track=<name> or IFSSIM_TRACKER; "wandb" | "clearml" | "null"
```

Backends: `WandbTracker`, `ClearMLTracker` (phase ≥ 2, only if needed), `NullTracker` (records
calls in memory for the adapter unit tests).

### 5.2 Adapters (one per benchmark kind; pure functions of the run dir)

```python
# tools/sim_benchmark/tracking/adapters.py
@dataclass
class RunBundle:
    job_type: str; name: str; group: str; tags: list[str]
    config: dict; summary: dict
    series: dict[str, tuple[str, list[dict]]]      # family -> (step_key, rows)
    tables: dict[str, tuple[list[str], list[list]]]
    files: list[tuple[Path, str]]                  # (path, kind)
    html: Path | None

def load_onboard(run_dir: Path) -> RunBundle: ...
def load_perception_gt(run_dir: Path) -> RunBundle: ...
def load_slam_gt(run_dir: Path) -> RunBundle: ...
def load_control_gt(run_dir: Path) -> RunBundle: ...
def load_sim(run_dir: Path) -> RunBundle: ...          # phase 3
def detect_kind(run_dir: Path) -> str: ...             # from results.json["module"]

def upload(run_dir: Path, tracker: Tracker, *, baseline: RunRef | None = None,
           force: bool = False) -> TrackingRecord | None: ...
```

The adapters only read files, so everything (keys, units, decimation, deltas) is unit-testable
against the existing run dirs, with no network.

### 5.3 Provenance and host hooks

```python
# tools/sim_benchmark/tracking/provenance.py
def capture_provenance(*, image: str | None) -> dict: ...         # host only; SHAs, branches, diffs, image id, user/host
def bag_identity(bag_dir: Path) -> dict: ...                      # cached content hash (§3.6)
def write_provenance(run_dir: Path, prov: dict, diffs: dict[str, str]) -> None: ...

# common.py changes (described, not implemented):
#   make_run_dir(root, module, strategy, run_id: str | None = None)
#       honours IFSSIM_RUN_ID so host and container agree on the dir name.
#   maybe_reexec_in_docker(...):
#       before docker run:  run_id = new_run_id(); prov = capture_provenance(...)
#                           pass -e IFSSIM_RUN_ID=run_id
#       after docker run:   write provenance.json; if --track: upload(run_dir) (never raises)
#   --no-docker path: the same two calls, wrapped around main().
```

Why provenance is captured **before** the container starts: a replay takes minutes, and the
developer may edit code in the meantime. The record should show the state that ran.

The param snapshot (`params/<node>.yaml`) is the one thing that has to happen **inside** the
container (`ros2 param dump /<node>` after `activate`, in `replay_pipeline`).

### 5.4 CLI surface

* Existing benchmark scripts: add `--track[=wandb|clearml]` (opt-in, Q2), `--tag`, `--baseline`,
  `--notes`. These flags are consumed on the host and removed from the argv passed to the container.
* `track.py upload <run_dir>… [--force] [--backend …]`
* `track.py upload --pending` (retry failed or never-uploaded runs)
* `track.py backfill results/onboard` (§8)
* `track.py set-baseline <tracker_run_url|run_dir>` (§6.3)
* `track.py sync` (push offline runs)

---

## 6. W&B project organisation

### 6.1 Entity / project

One project: `<team-entity>/ifssim-bench` (entity name: Q1). One project keeps cross-kind
comparisons possible, e.g. a commit's perception and closed-loop results side by side.

### 6.2 Run identity fields

| Field | Value | Example |
|---|---|---|
| `job_type` | benchmark kind | `onboard_replay` |
| `name` | `<scenario_short>/<pipeline_sha>/<ts>` | `manual_20260920_154527/a64350a-dirty/0922T173826` |
| `group` | `<scenario.id>@<pipeline_sha>[-dirty-<diffhash6>]` | repeats of one scenario at one commit |
| tags | `baseline`, `nightly`, `sweep`, `backfill`, `branch:<name>`, `dirty`, `superseded`, free-form via `--tag` | |

"Scenario × commit" as the group gives mean/min/max bands for SIL repeats for free. Comparing
commits on one scenario then means **filtering by `config.scenario.id`**, not grouping. Every
saved workspace view starts from that filter.

### 6.3 Baselines

* A baseline is a **run** (bag scenarios) or a **group** (SIL, N repeats), tagged `baseline` +
  `baseline:<scenario.id>`. Only one is active per scenario: `set-baseline` removes the tag from the
  previous one.
* On upload the harness looks up the active baseline for the run's `scenario.id` and logs
  `<metric>.delta_vs_baseline` for registry metrics with a direction. For same-bag runs it also
  logs `perception/n_cones_delta_vs_baseline` as a series (this run's count minus the baseline's,
  resampled onto this run's timeline). That covers the "detection count over time with delta"
  comparison inside W&B.
* Who promotes a baseline, and when, is Q3.

### 6.4 Workspace panels (saved view "Onboard replay — same bag")

| Section | Panel | Mechanism |
|---|---|---|
| Summary | Runs table: config.code.pipeline.sha, tags, headline metrics + `delta_vs_baseline` | native |
| Routes | **XY overlay**: odom and SLAM of the selected runs, colour = run, dash = source | custom Vega chart over `trajectory` table |
| Map | **Cone map overlay**: `map_cones` per run, shape = source | custom Vega chart over `map_cones` table |
| Perception | `perception/n_conos_raw` vs `t/perception_s`, overlaid. Second panel: delta vs baseline | native line plot |
| Control | throttle / steering / brake, overlaid. Steer residual. | native line plot |
| Scalars | bar chart per headline metric across runs. Parallel coordinates for params → metrics | native |
| Report | embedded `report.html` of the focused run | `wandb.Html` media panel |

**Cross-bag alignment** (for overlays from different bags):

* Same `bag.id` → overlay `frame=raw` directly.
* Different bag → the harness also stores `frame=start_pose` rows: each trajectory is rotated and
  translated so its first pose is at the origin, heading +x. Odom is already close to this, but SLAM
  and GT are not. The chart title shows the frame, and a Vega transform filters by the selected
  frame. Nothing is silently aligned.
* Optional `frame=fit:<ref>`: 2-D rigid best-fit (Umeyama on map cones, nearest-neighbour matched)
  against a reference run, computed at upload when `--align-to` is given. It's labelled the same
  way. Only worth building if the team actually overlays different bags often.

**Spike to do first (phase 1 exit criterion):** confirm that a custom Vega chart over a
per-run table overlays correctly across multiple selected runs in the workspace and in a Report,
with the 4 backfilled same-bag runs. The whole W&B choice rests on this. If it fails, ClearML's
plot comparison is evaluated before building further.

### 6.5 Reports

Saved comparisons are W&B Reports, built from the panels above with a pinned run set:
"PR #NNN vs baseline", "Weekly nightly summary", "Sweep result X". The PR template gets an optional
"Benchmark report" link field.

### 6.6 Artifacts

| Artifact type | Name | Contents | Why |
|---|---|---|---|
| `track` | `track-<name>` | the track CSV (tiny) | exact track version per SIL run |
| `sim_build` | `sim-<build_id>` | **reference only** (build id, plugin SHA, path on the GPU box). The binary isn't uploaded. | lineage |
| `bag_ref` | `bag-<bag.id>` | **reference only** (`add_reference` to a shared-storage URI), plus `metadata.yaml` | lineage without uploading 7.5 GB |
| `baseline_set` | `baselines` | JSON map `scenario.id → run id` | makes baseline promotion auditable |

---

## 7. Simulator-in-the-loop benchmarks (later phases)

### 7.1 Scenario specification

A scenario is a small YAML file under `tools/sim_benchmark/scenarios/`. Its hash is `scenario.id`:

```yaml
name: trackdrive_fsg23_nominal
event: trackdrive          # → set_event
laps: 10
track: Content/tracks/track_20260404_013723.csv   # → load_track; hashed → track artifact
noise: { gyro_std: 0.002, accel_std: 0.05, lidar_range_std: 0.02 }   # FSDSSettings noise block
seeds: [1, 2, 3, 4, 5]     # repeats
timeout_s: 400
pipeline: { mission: trackdrive, params_overrides: {} }
record: { series: [control, perception], video_on_failure: true }
```

The runner is an extension of `tools/scenario_runner/run_scenario.py`: `load_track` →
`set_event` → for each seed `resetScenario <seed>` → run until `bFinished` / timeout / failure →
`get_referee_state` → write a run dir in the same layout as today (`results.json` + CSVs +
`provenance.json`). It then goes through the same `track.py upload`. **The tracker never drives
the sim.**

### 7.2 One run per repeat, one group per (scenario, commit)

* Each seed is its own W&B run (`job_type=pipeline_e2e` / `control_closed_loop` / `perception_sil`),
  in the group `<scenario.id>@<sha>`.
* After the last seed, the runner uploads **one aggregate run** (`job_type=aggregate`, same
  group) with `race/finish_rate_frac`, `race/lap_time_mean_s ± ci95`, `race/n_doo_mean`,
  `race/dnf_causes` (table), and a per-seed table. Baselines, alerts and sweeps compare on
  aggregate runs, because W&B groups don't aggregate summary scalars natively.

### 7.3 Distance-indexed series

Series are logged against `track/s_m`, the arc length of the car's projection onto the track
centerline (`common.load_csv_centerline` already builds the centerline from the track CSV), plus a
`lap` column. Then "cross-track error at the hairpin" lines up across runs and seeds even when lap
times differ. Time-indexed copies are kept only for control signals.

### 7.4 Failures

* First-failure detection in the runner: DOO/OC counter increments (referee), `bFinished` not
  reached by `timeout_s`, stationary for > X s, node died (lifecycle state). Each is logged as an
  `events` row with `t_s`, `s_m`, `kind`, `detail`, and `race/first_failure_s_m` is recorded.
* **Video only on failure**: a ring buffer of the last ~20 s of a chase camera, dumped to mp4 and
  logged as `wandb.Video` only when a failure event fires. Needs a sim-side capture path (Q5).

### 7.5 Sweeps

* A W&B Sweep (Bayes or grid) over `params.*` overrides. **Objective = the aggregate metric over the
  scenario's seeds** (e.g. `race/lap_time_mean_s` with `race/finish_rate_frac == 1` as a constraint,
  implemented as a penalty).
* One sweep trial = one W&B run that runs all seeds internally and logs the aggregate plus a per-seed
  table. W&B sweep agents own one run per trial, so per-seed child runs don't fit.
* **One `wandb agent` per GPU machine, sequential** (one sim per machine). More machines means more
  agents on the same sweep id. With ClearML, the same thing is a `clearml-agent` queue per GPU box.

### 7.6 Nightly regression

* A GPU machine runs a systemd timer / cron at night: pull `dev` + the `pipeline` submodule, build,
  run the **regression suite** (a list of scenario YAMLs plus bag replays), upload with tag
  `nightly`.
* The harness compares each aggregate against the active baseline group with the registry's
  `regression_threshold` (and, for SIL, a simple significance check on the N seeds: Welch t-test
  or bootstrap CI, so noise doesn't page anyone).
* A regression triggers `tracker.alert(level="warn"|"error")` → W&B Alert → Slack/email. The nightly
  Report link goes in the alert text.

---

## 8. Backfill of existing runs

* `track.py backfill tools/sim_benchmark/results/onboard` uploads every dir with a `results.json`
  (4 today), tagged `backfill`, with `provenance.complete = false`.
* Missing fields are **left null, not guessed**: `code.*` (unknown), `params` (unknown), `image`
  (unknown). `bag.id` **can** be computed now, since the bag is still on disk. Optionally, a
  `code.pipeline.sha_guess` from `git log --before=<run ts>` goes in a separately named field,
  never in `code.pipeline.sha` (Q8).
* The 5 log-only dirs (`--live` / no `--report`) are skipped. Their `replay_bag` (where present) could
  be re-reported with `--skip-replay --report` first if anyone cares.

---

## 9. ClearML fallback mapping

| Concept | W&B | ClearML |
|---|---|---|
| run | Run | Task |
| job_type / group / tags | `job_type` / `group` / tags | task type + project subfolder / tag / tags |
| config | `run.config` | `task.connect(dict)` (hyperparameters) |
| summary | `run.summary` | `logger.report_single_value` |
| series | `define_metric` + `log` | `report_scalar(title, series, value, iteration)` (iteration = step) |
| tables | `wandb.Table` + Vega | `report_table` + `report_scatter2d` per run. Overlay via the compare view's plots. |
| files / html | `log_artifact` / `wandb.Html` | `upload_artifact` / `report_media` |
| artifacts | Artifacts | Datasets / artifacts |
| sweeps | Sweeps | HyperParameterOptimizer + `clearml-agent` queues |
| alerts | `run.alert` | Monitoring service / Slack integration |

Only `WandbTracker` is built until cost rules W&B out. The adapters and local files don't change.
History can be migrated by re-running `track.py upload --backend clearml` over the run dirs, which
is why local files stay the source of truth.

---

## 10. Phased rollout

| Phase | Deliverables | Exit criteria |
|---|---|---|
| **1. Harness + onboard replay** | `tracking/` package (interface, W&B + Null backends, onboard adapter, provenance, bag id), `IFSSIM_RUN_ID` plumbing in `common.py`, param dump in `run_onboard_replay.py`, `--track` flag, `metrics.yaml` (onboard subset), adapter unit tests on fixture run dirs | A `--report --track` replay shows up in W&B with full provenance. The **Vega overlay spike (§6.4) passes**. A killed network doesn't change the replay's exit code. |
| **2. Backfill + workspace/Reports** | `track.py backfill/upload --pending/set-baseline`, the 4 backfilled runs, saved workspace view, first Report template, baseline delta logging, perception/SLAM/control GT adapters | The team can open one link and compare any two onboard replays' routes, cone maps, detection-count delta and control signals. |
| **3. SIL benchmarks** | scenario YAML + extended `scenario_runner`, sim adapter, distance-indexed series, aggregate runs, track/sim_build artifacts, failure events (video if Q5 allows), first sweep | N-seed group for one scenario at two commits compared in one Report with finish rate and lap time CI. |
| **4. Nightly regression + alerts** | GPU-box timer, regression suite list, significance-aware comparison, W&B Alerts → Slack, weekly Report | A deliberately regressed param triggers an alert the next morning. An unchanged commit triggers none over a week (no false positives). |

---

## 11. Open questions for you

1. **Plan / entity**: are we eligible for the W&B academic plan, or do we need a paid Team plan?
   What's the entity name, and who is admin?
2. **Upload trigger**: opt-in `--track` (recommended to start), or automatic on every `--report` run?
3. **Baseline convention**: which run per bag/scenario is the baseline (e.g. latest `dev` merge,
   hand-picked), and who is allowed to promote a new one?
4. **Shared bag storage**: where do bags live (NAS path, S3/MinIO bucket, a GPU box)? That decides
   the `bag_ref` URI scheme and the `bag.id → path` registry.
5. **Sim automation**: how scenarios should be specified (is the YAML in §7.1 right?), how fast
   `resetScenario` + track load really is between repeats, whether the sim can run headless and
   unattended, how many GPU machines are available for it, and whether a sim-side chase-cam
   ring buffer for failure clips is feasible.
6. **Core metric vocabulary**: agree the canonical names and directions per benchmark type (§4.1
   is a proposal), especially what counts as a "failure" and the penalty weights for `race/penalty_s`.
7. **`replay_bag` upload**: the 23 MB pipeline-*output* bag is small and makes a run fully
   re-reportable/inspectable in Foxglove later. Upload it as an artifact (~23 MB/run of W&B storage),
   or keep it local like source bags?
8. **Backfill SHA guess**: record a best-guess `pipeline` SHA from the commit timestamp (in a
   separately named field), or leave it strictly unknown?
9. **Data sensitivity**: is it acceptable to upload `pipeline` diffs, params and logs to hosted W&B
   (competition-code confidentiality)? If not, diffs stay local and only their hash is uploaded, or
   that alone pushes toward self-hosted ClearML.
10. **Doc location** (resolved): the branch was rebuilt on `dev` as `feat/517-bench-tracking-viewer`,
    and this doc lives in `docs/history/` with the other design write-ups.
