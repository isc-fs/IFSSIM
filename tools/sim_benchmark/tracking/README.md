# bench_tracking: experiment tracking for sim_benchmark

> New here? [HOW_IT_WORKS.md](HOW_IT_WORKS.md) explains the whole flow, from a benchmark run to the viewer.
> Setting up the team server: [DEPLOY.md](DEPLOY.md).

It puts benchmark runs into **MLflow** and shows them in **bench-view**. MLflow was chosen after
the same data was tried in W&B, MLflow and ClearML. See
`docs/history/2026-09-23_benchmark-tracking-design.md` (design) and
`docs/history/2026-09-23_tracker-evaluation.md` (the comparison).

Nothing here runs a benchmark. It reads the results folders the benchmarks write:

| Data | Source | Kind |
|---|---|---|
| Bag benchmarks | existing `results/onboard/<mission>_<ts>/` with `--report` (results.json, CSVs, samples.json, report.html, logs) | real, `job_type=onboard_replay` |
| Live run reports | existing `results/onboard/…` from `--live` without `--report`: node logs only | real, `job_type=live_replay` |
| Simulator bag benchmarks | `results/sim_bag/<bag>_<ts>/` from `run_sim_bag_benchmark.py`, and older `results/perception/…` + `results/slam/…` runs | real, `job_type=sim_bag` |
| Simulator benchmarks | `bench-track mock-sim`: point-mass lap model on real track CSVs | **MOCK**, `sim_e2e` / `sim_aggregate` / `sim_sweep_trial` |

## bench-view: the viewer on top of MLflow

MLflow stores the runs; `bench-view` is the UI for looking at them, and for launching new ones.
It is a Dash app that reads straight from the MLflow server: `search_runs` for the catalog, and
each run's `bundle/` artifact (full-resolution Parquet, written by `store.py`) for the plots.
MLflow's own UI keeps working alongside it.

The site has two halves, switched at the top of the side panel:

- **Results** (`/bag`, `/simbag`, …): everything below.
- **Launch** (`/launch`): *New run* (benchmarks, bags, code, a YAML with settings and
  parameter overrides; the jobs it makes are shown before launching), *Queue* (running,
  waiting, finished; cancel), and a page per job with its log and links to its results.
  Jobs go to a queue that `bench-worker` works through (`bench_tracking/launch/`,
  DEPLOY.md "Launching benchmarks"). Locally the queue is a SQLite file, so a laptop can run
  the page and a worker when the central machine is down.

```bash
./deploy/mlflow/run_server.sh &                         # if not running
uv run bench-view                                       # http://127.0.0.1:8050  (--uri to point at another MLflow)
```

What each page, section and chart shows, and what data is behind it, is in
[VIEWER_GUIDE.md](VIEWER_GUIDE.md).

The app is organised in three steps:

1. **What kind of report**: the left navigation lists Bag benchmarks, Live runs, Simulator bag
   benchmarks, Sim benchmarks, Nightly and Parameter sweeps. The sections of the open page appear under it. « (or `[`)
   collapses it to a thin rail so the plots get the full width.
2. **Which runs**: the selection bar says what is on screen ("Showing report ● Tue 22 Sep 17:38").
   *Change…* and *+ Compare* open the run browser: runs grouped by day (by bag for simulator
   bag sessions, where reruns of identical code are marked and dimmed), with their key numbers
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
uv sync --extra all --extra dev          # Python 3.12 venv with mlflow and dash (viewer)

# the team server: ~/.config/ifssim-bench/tracking.env (DEPLOY.md)
# or a private local MLflow: http://127.0.0.1:5005
./deploy/mlflow/run_server.sh &
```

## Commands

```bash
R=/path/to/IFSSIM/tools/sim_benchmark/results          # holds onboard/ and capture/
uv run bench-track sync --results-root $R             # upload every finished run the server lacks
uv run bench-track sync --results-root $R --dry-run   # what is on the server, what would go up
uv run bench-track mock-sim                           # -> ../results/sim_mock (90 seed runs)
uv run bench-track import  --sim-root ../results/sim_mock   # mock simulator runs (sync does not read them)
uv run bench-track summary --replay-root $R --sim-root ../results/sim_mock   # print, upload nothing
uv run bench-track purge   --only replay              # delete runs this machine uploaded
uv run bench-run --list                               # what bench.yaml offers (see ../README.md)
uv run bench-run sim-bag --dry-run                    # a spec's jobs and commands
uv run bench-run sim-bag --queue                      # queue them for bench-worker instead
uv run bench-worker                                   # run queued jobs here (the central machine: DEPLOY.md)
uv run pytest -q tests
```

`--sim-bag-root` reads the sessions `run_sim_bag_benchmark.py` writes (`$R/sim_bag/*`). It also
pairs the perception and SLAM runs made one at a time before that runner existed
(`$R/perception/*` + `$R/slam/*` of the same bag, started within 5 minutes) into one session each.

The first run hashes each bag once (≈10 s per 7.5 GB, cached in `~/.cache/bench_tracking`).
`--fast-bag-id` skips that. Before uploading, each run is looked up on the server by its run key
(the `bench.run_key` tag), so nothing is uploaded twice. `.state/mlflow*.json` (one per server)
remembers what this machine uploaded. `--write-records` also writes `tracking.json` into each run dir
(off by default, because imports read run dirs from other checkouts).

## Layout

```
bench_tracking/
  bundle.py        RunBundle / Series / Table / Event: backend-neutral run description
  metrics.yaml     metric registry: canonical names, unit, direction, tolerance, headline
  registry.py      registry access + delta/regression verdicts
  logparse.py      node logs -> series/events/scalars (CONE_FILTER, SLAM_LAT/PROF/OBS, PATH_RATE, control)
  provenance.py    reads provenance.json (code state recorded at run time), bag metadata + content id, scenario id
  geometry.py      interpolation, start-pose / ICP alignment, track loading + s/lateral projection
  adapters/        replay.py (onboard + live run dirs), sim.py (sim run dirs + seed aggregation),
                   sim_bag.py (simulator bag sessions: perception + SLAM vs ground truth)
  mock_sim.py      MOCK SIL generator (scenario × commit × seed, nightly, sweep)
  compare.py       baselines, deltas, delta series, bootstrap CIs
  figures.py       plotly per-run deep dives, uploaded with each run for MLflow's own UI
  suite.py         load everything, aggregate, attach baselines
  backends/        base.py (the interface), mlflow_backend.py (bundle -> MLflow run, run-key lookup)
  config.py        reads ~/.config/ifssim-bench/tracking.env (server, login)
  store.py         RunBundle <-> bundle/ dir (Parquet + JSON), the artifact bench-view reads
  viewer/          bench-view: data.py (MLflow catalog + bundle cache), replay.py / sim.py / simbag.py (figures),
                   launch.py (the Launch pages: new run, queue, job),
                   pages.py (what each page shows per mode, run browser),
                   app.py (shell, selection state, callbacks), assets/ (style.css, sync.js: playhead, zoom, browser)
  launch/          running benchmarks: manifest.py (the repo's bench.yaml), spec.py (run specs: merge,
                   validate, spec_id, jobs), run.py (bench-run), queue.py (the job queue), submit.py
                   (spec -> pinned code -> jobs), checkout.py (refs, worktrees, image), worker.py
                   (bench-worker). Knows the benchmarks only through bench.yaml
  cli.py           bench-track
deploy/            central/ (the team server, DEPLOY.md), mlflow/run_server.sh (a private local MLflow)
tests/             tests that need no server
```

## What gets logged per run

- **config**: code (both repos' SHAs/dirty, image, `code.id`, `code.label`) from the run's
  `provenance.json` (`tools/sim_benchmark/run_provenance.py`), scenario (bag id or
  track/seed/noise), params, env, provenance completeness. Runs made before provenance was
  recorded are marked `provenance.complete=false` and `code.label=unknown`.
- **summary**: every registry metric, plus `<metric>.delta` vs the scenario baseline and
  `compare/n_regressions`.
- **series**: replay-time (`t/replay_s`), log-time (`t/log_s`), track-distance (`track/s_m`,
  `track/s_lap_m`), sim-time (`t/sim_s`) and per-lap (`lap`) families.
- **tables**: trajectory, map cones, laps, per-seed, perception-vs-range, lifecycle, warn catalogue,
  events, baseline comparison.
- **figures**: 10–14 per-run plotly deep dives (route with pipeline events, SLAM compute profile,
  cone funnel, track map with penalties, cross-track heatmap, failure snapshot…).
- **files**: CSVs, results.json, logs, report.html, provenance.json and its diffs, `spec.json`
  and `params/` when the run had a spec (never bags).
- **code.spec_id**: `default`, or the id of the settings and parameter overrides it ran with.
  Reruns are runs with the same code id *and* spec id.
