# bench_tracking: experiment tracking for sim_benchmark

This is an evaluation harness. It pushes the **same** benchmark data into **W&B, MLflow and ClearML**
and builds the same in-depth dashboards in each, so the trackers can be compared on real
content. See `docs/history/2026-09-23_benchmark-tracking-design.md` (design) and
`docs/history/2026-09-23_tracker-evaluation.md`
(what each tracker could and couldn't do).

Nothing here runs a benchmark:

| Data | Source | Kind |
|---|---|---|
| Bag benchmarks | existing `results/onboard/<mission>_<ts>/` with `--report` (results.json, CSVs, samples.json, report.html, logs) | real, `job_type=onboard_replay` |
| Live run reports | existing `results/onboard/…` from `--live` without `--report`: node logs only | real, `job_type=live_replay` |
| Simulator benchmarks | `bench-track mock-sim`: point-mass lap model on real track CSVs | **MOCK**, `sim_e2e` / `sim_aggregate` / `sim_sweep_trial` |

## bench-view: the viewer on top of MLflow (prototype)

MLflow stores the runs; `bench-view` is the UI for looking at them. It is a Dash app that reads
straight from the MLflow server: `search_runs` for the catalog, and each run's `bundle/` artifact
(full-resolution Parquet, written by `store.py`) for the plots. MLflow's own UI keeps working
alongside it.

```bash
./deploy/mlflow/run_server.sh &                         # if not running
uv run bench-track attach-bundles --replay-root $R --sim-root ../results/sim_mock   # runs imported before bundle/ existed
uv run bench-view                                       # http://127.0.0.1:8050  (--uri to point at another MLflow)
```

The app is organised in three steps:

1. **What kind of report**: the left navigation lists Bag benchmarks, Live runs, Sim benchmarks,
   Nightly and Parameter sweeps. The sections of the open page appear under it.
2. **Which runs**: the selection bar says what is on screen ("Showing report ● Tue 22 Sep 17:38").
   *Change…* and *+ Compare* open the run browser: runs grouped by day, with their key numbers
   coloured against the baseline, and *Open* / *+ Compare* on each row. Type to filter.
   - One run is one report.
   - Two or more runs are a comparison: the bar shows them as chips (✕ removes one) with an
     *Overlay* / *Side by side* switch, and *⇄ Swap* when side by side.
3. **The charts**: full width, with the title and a one-line explanation above each one, and the
   main ones first.
   - Each time chart has a stats line under it (median · p95 · min · max per run), as in the bag
     reports.
   - The minor charts sit behind "More charts".

**Playback, as in Foxglove/Lichtblick:** all time charts share one playhead. Hovering a chart moves it,
and so do the bar at the bottom (▶, 1–10×, drag), space, ←/→ and a click on the map. Every chart draws
the line and shows each run's value at t in its header. On *Trajectory*:
- A sticky route map moves each run's marker along an 8 s trail.
- A "moment" panel lists the events within ±2 s.
- The signal panels are Foxglove-style and include a SLAM-state strip (mapping / localisation bands
  plus event symbols).
- Zooming any time chart zooms the rest and highlights that stretch on the map.

All of this runs in the browser (`assets/sync.js`), so it has no server round trips.

The baseline is always black. ★ *Make baseline* stores it as the MLflow tag `bench.pinned_baseline`.
The URL holds the selection (`/bag/slam?mode=side&r=…&vs=…`), so any view can be shared as a link.

Runs with no `bundle/` still load, rebuilt from MLflow's metric history (integer steps, decimated)
and its `tables/*.json` artifacts. That takes ≈6 s per replay run instead of <0.2 s.

## Setup

```bash
cd tools/sim_benchmark/tracking
uv sync --extra all --extra dev          # Python 3.12 venv with wandb, mlflow, clearml, dash (viewer)

# MLflow (local): http://127.0.0.1:5005
./deploy/mlflow/run_server.sh &

# ClearML (self-hosted, Docker): http://localhost:8090
cp deploy/clearml/.env.example deploy/clearml/.env      # set random keys
docker compose -f deploy/clearml/docker-compose.yml --env-file deploy/clearml/.env up -d

# W&B (hosted): needs an account
uv run wandb login                        # optional: export WANDB_ENTITY=<team>
```

## Commands

```bash
R=/path/to/IFSSIM/tools/sim_benchmark/results          # holds onboard/ and capture/
uv run bench-track mock-sim                             # -> ../results/sim_mock (90 seed runs)
uv run bench-track summary    --replay-root $R --sim-root ../results/sim_mock
uv run bench-track import     --backend clearml --replay-root $R --sim-root ../results/sim_mock
uv run bench-track dashboards --backend clearml --replay-root $R --sim-root ../results/sim_mock
uv run bench-track purge      --backend clearml --only replay      # delete + forget uploads
uv run pytest -q tests
```

The first run hashes each bag once (≈10 s per 7.5 GB, cached in `~/.cache/bench_tracking`).
`--fast-bag-id` skips that. Upload ids are kept in `.state/<backend>.json`, so a re-import
skips runs that are already there. `--write-records` also writes `tracking.json` into each run dir
(off by default, because imports read run dirs from other checkouts).

## Layout

```
bench_tracking/
  bundle.py        RunBundle / Series / Table / Event: backend-neutral run description
  metrics.yaml     metric registry: canonical names, unit, direction, tolerance, headline
  registry.py      registry access + delta/regression verdicts
  logparse.py      node logs -> series/events/scalars (CONE_FILTER, SLAM_LAT/PROF/OBS, PATH_RATE, control)
  provenance.py    git state, bag metadata + content id, scenario id
  geometry.py      interpolation, start-pose / ICP alignment, track loading + s/lateral projection
  adapters/        replay.py (onboard + live run dirs), sim.py (sim run dirs + seed aggregation)
  mock_sim.py      MOCK SIL generator (scenario × commit × seed, nightly, sweep)
  compare.py       baselines, deltas, delta series, bootstrap CIs
  figures.py       plotly figures: per-run deep dives + cross-run comparisons
  suite.py         load everything, aggregate, attach baselines
  backends/        wandb_backend.py, mlflow_backend.py, clearml_backend.py (one upload(bundle) each)
  dashboards/      common.py (shared dashboard content), wandb_dash / mlflow_dash / clearml_dash
  store.py         RunBundle <-> bundle/ dir (Parquet + JSON), the artifact bench-view reads
  viewer/          bench-view: data.py (MLflow catalog + bundle cache), replay.py / sim.py (figures),
                   pages.py (what each page shows per mode, run browser),
                   app.py (shell, selection state, callbacks), assets/ (style.css, sync.js: playhead, zoom, browser)
  cli.py           bench-track
deploy/            mlflow/run_server.sh, clearml/docker-compose.yml
tests/             backend-free tests
```

## What gets logged per run (all backends)

- **config**: code (both repos' SHAs/dirty, image), scenario (bag id or track/seed/noise), params,
  env, provenance completeness. Imported replays are marked `provenance.complete=false`.
- **summary**: every registry metric, plus `<metric>.delta` vs the scenario baseline and
  `compare/n_regressions`.
- **series**: replay-time (`t/replay_s`), log-time (`t/log_s`), track-distance (`track/s_m`,
  `track/s_lap_m`), sim-time (`t/sim_s`) and per-lap (`lap`) families.
- **tables**: trajectory, map cones, laps, per-seed, perception-vs-range, lifecycle, warn catalogue,
  events, baseline comparison.
- **figures**: 10–14 per-run plotly deep dives (route with pipeline events, SLAM compute profile,
  cone funnel, track map with penalties, cross-track heatmap, failure snapshot…).
- **files**: CSVs, results.json, logs, report.html (never bags).
