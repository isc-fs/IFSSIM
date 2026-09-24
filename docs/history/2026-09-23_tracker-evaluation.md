# Tracker evaluation: W&B vs MLflow vs ClearML on IFSSIM benchmark data

Status: **evaluation build, for the tracker decision** · 2026-09-23
Code: `tools/sim_benchmark/tracking/` (see its README). Design: [`2026-09-23_benchmark-tracking-design.md`](2026-09-23_benchmark-tracking-design.md).

The same data went into each tracker through one harness, with the same dashboard content:

| Data | Runs | Real? |
|---|---|---|
| Onboard bag replays (`--report`) | 4 | real (`results/onboard`, bag `manual_20260920_154527`) |
| Live replays (`--live`, logs only) | 5 (2 aborted before play) | real, everything from node logs |
| SIL scenario × commit × seed matrix | 3 × 2 × 5 = 30 seeds + 6 aggregates | **mock** |
| SIL nightly | 8 nights × 3 seeds + 8 aggregates | **mock** |
| SIL sweep | 12 trials × 3 seeds + 12 aggregates | **mock** |

125 uploads per tracker. Each run has 30–60 scalars (+ deltas vs baseline), 6–12 series families
(up to 3 000 points per series), 4–9 tables, 10–14 interactive per-run figures, and its files.
Four dashboards were built per tracker: bag replay, sim matrix, nightly, sweep.

## Status per tracker

| | MLflow 3.16 (local) | ClearML 2.x server (self-hosted, Docker) | W&B 0.30 (hosted) |
|---|---|---|---|
| Upload | done, 125 runs | done, 125 runs | done, 125 runs |
| Dashboards | built, checked in browser | built, checked in browser | built and saved (5 views, 4 Reports); read back via API; **visual check pending** (needs a logged-in browser) |
| Where | http://127.0.0.1:5005 → experiment `ifssim-bench/dashboards` | http://localhost:8090 → Reports | https://wandb.ai/sjrom4-universidad-pontificia-comillas/ifssim-bench (Workspace "views" menu, Reports tab) |

The server accepted the W&B custom Vega chart presets (registered through the undocumented
`upsertView` `vega2-panel` type) and the saved reports reference them. What's still open is
whether they *render* correctly with the merged multi-run tables. That needs a look in the browser.

## Requirement-by-requirement

The brief's must-have: **comparisons live inside the tracker**. That means routes and cone maps
overlaid, detection count over time with delta, control signals, and scalar deltas.

| Requirement | W&B | MLflow | ClearML |
|---|---|---|---|
| Time/distance series overlaid across selected runs | Native. `define_metric` step per family, float x (s, m, lap). | Metric history only. **Integer steps**, so x is encoded (ms, dm), and the chart axis says "step". | Native in compare view and embeddable in Reports. **Integer iterations**, so x is encoded (0.1 s, 1 m) and the unit goes in the title. |
| XY route / cone-map overlay | Custom Vega chart over the merged per-run tables. *Needs the preset check.* | **Not possible natively**: precomputed plotly HTML artifact only. | **Not natively for XY** (scalars only). Precomputed plotly embedded in a Report. |
| Cone count Δ vs baseline | Series logged by the harness. Native panel overlays it. | Harness series, no overlay UI | Harness series, embedded live widget ✔ (checked) |
| Scalar deltas vs baseline | **Native** (`baseline_run`, delta columns) + harness `.delta` | Harness only (`.delta` metrics, HTML table) | Harness only (runs table on the dashboard task) |
| Seeds of one group: mean ± band | **Native** (group by run group, mean/min–max) | No. Nested runs give a tree, not an aggregate chart. | No. Per-task colours only, so 10 seeds of 2 commits can't be coloured by commit (seen in the lap-time widget). |
| Dashboards defined in code, reproducible | Workspace views + Reports (`wandb-workspaces`) | **None.** A "dashboard" is a run whose artifact is an HTML page. | Reports via the server REST API (`reports.create` / `update`; not in the Python SDK). Worked, with two quirks (below). |
| Per-run deep dive (route with events, SLAM profile, funnel, track map…) | `wandb.Plotly` media | plotly HTML artifacts | `report_plotly` (Plots tab) |
| Tables | `wandb.Table` (queryable, joinable in Weave panels) | JSON table artifacts | `report_table` (plot-style table) |
| Sweeps | W&B Sweeps + agents | none (external Optuna) | HPO + `clearml-agent` queues (also dispatches sim runs to GPU boxes) |
| Alerts on regression | `run.alert` → Slack/email | none | needs the services agent / custom monitor |
| Artifacts / lineage (tracks, bag references) | `track` / `bag` (reference-only) artifacts | plain artifacts | Datasets / artifacts |
| Offline machines | offline mode + `wandb sync` | local server | local server / offline mode |

## Measured

| | MLflow | ClearML | W&B |
|---|---|---|---|
| Upload time, 125 runs | 1 209 s (9.7 s/run) | 622 s (5.0 s/run) | 1 032 s (8.3 s/run) |
| Storage for 125 runs | 1.9 GB SQLite (**6.8 M metric rows**, one row per series point) + 192 MB artifacts | 680 MB Elasticsearch + 216 MB Mongo + 62 MB files | hosted quota |
| Server footprint | 1 process | 6 containers, ≈3.5 GB RAM (Elasticsearch 2.9 GB) | none |

## Quirks found while building

- **MLflow 3.16's UI is GenAI-first.** Experiments open under "Evaluations" with Traces, Judges and
  Prompts in the sidebar. The classic runs/compare view is behind the "Model training" toggle.
  Chart layouts built in the compare view are UI state, not code.
- **MLflow's SQL store scales with series points.** At nightly cadence it needs Postgres and
  retention rules, or series have to move to artifacts.
- **ClearML `reports.create` silently ignores the `report` body.** It has to be set with a follow-up
  `reports.update`. Markdown headings need blank lines around the embedded iframes.
- **ClearML `type=single` widget with several tasks renders empty.** Removed. The runs table covers it.
- **ClearML re-renders embedded plotly figures.** A scatter with a *categorical* y axis lost its
  rows (fixed by using numeric y + tick labels). Other figures may need the same care.
- **ClearML colours per task.** There is no "colour by commit/group" in the compare view or widgets.
- **W&B presets are undocumented from code.** If `upsertView(type: vega2-panel)` is refused, the
  route/map overlays fall back to plain scatter (no line order, no shape by source).

## What the dashboards already show on the real replays

These come from the backfilled runs (no code SHAs, so they can't be attributed to a change yet):

- In **every** played replay the SLAM map jumps from ≈18 to 80–170 landmarks at t≈45–60 s. That's
  the same window as the data-association skips, cascade recoveries and the switch to
  localisation mode. 0922T173826 then drifts to a **48 m** SLAM−odom gap while its odom stays
  consistent (route overlay).
- Startup (first node Ready → last node Active) grew from 30 s (21 Sep) to 53–66 s (22 Sep), and the
  time from odometry-node activation to the first `/odom` grew from 34 s to 72–89 s.
- DBSCAN guard trips are constant (35–36 per run, ≈13–15 k points dropped each): a systematic
  condition in this bag, not noise.
- 0921T144527 and 0922T170038 predate the CONE_FILTER log line. Their funnel panels are empty
  rather than wrong.

## Recommendation (for your decision)

1. **W&B remains the best fit for the stated requirement**, *if* the two server-side checks pass
   (custom XY preset renders, merged tables overlay). It is the only one with native group
   bands, native baseline deltas, float x axes and code-defined dashboards on a documented SDK.
   Presets were accepted and the dashboards are saved; the check left is visual.
2. **ClearML is a solid, verified fallback.** Series overlays and code-built Reports worked end to
   end on our data, and its agent queues fit sim dispatch. The costs are no XY overlays,
   no group colouring, integer x axes, an undocumented reports API, and a 3.5 GB-RAM server to run.
3. **MLflow doesn't meet the "comparisons inside the tracker" requirement.** Every comparison
   is a static HTML artifact. It is also the slowest and heaviest store for our series-heavy runs.

## Follow-up: MLflow for storage, our own viewer on top (prototype)

All three native UIs turned out clunky for this data. The charts are small and crowded, and comparing
runs is awkward. So the prototype splits the job: **MLflow stores, `bench-view` shows**
(`tools/sim_benchmark/tracking/bench_tracking/viewer/`, README section "bench-view").

- **Stays in MLflow**: runs, params, summary metrics, tags, artifacts, lineage to experiments, its UI and
  API, and access through the server. Each run gains one artifact, `bundle/`: the full-resolution run as
  Parquet (≈1.3 MB for an onboard replay, ≈0.2 MB for a seed). The viewer reads that instead of the
  metric history, which gives float x axes, every sample, and a load time under 0.2 s instead of ≈6 s.
  The metric history is still written for MLflow's own UI. Dropping it would also remove the
  6.8 M-row SQLite cost noted above.
- **The viewer is organised** as *kind of report → section → mode*:
  - The left navigation picks the kind (bag benchmarks, live runs, sim, nightly, sweeps) and a section
    of it (summary, perception, SLAM…).
  - The toolbar sets how many reports are on screen: one report on its own, an overlay of up to four,
    or two side by side, chart for chart, with the same axis ranges and linked zoom.
  - Charts are full width, one per row, like the HTML report, and the minor ones sit behind a fold.
  - Runs are picked in a run browser: runs by day, with key numbers coloured against the baseline, and
    Open / + Compare on each row. The number of runs picked sets the mode.
  - Time charts share a Foxglove-style playhead: hover, scrub, play or click the map. It drives a line and
    value readouts on every chart, the map markers and an event log, all in the browser.
  - The first version had one sidebar shared by every collection, AG-grid run pickers and a two-column
    chart grid. The second had dropdown pickers and a mode switch. Both were replaced after feedback
    that picking runs was unintuitive and the charts crowded.
- **What it shows**:
  - For replays: a route map linked to synchronised time lanes (hover, zoom-window and click-to-jump),
    metric tables with Δ coloured by the registry tolerance, an events explorer and a log-pattern pivot.
  - For sim: seed-level bootstrap scorecards, a track map linked to the along-track profiles, nightly
    trends and sweep grids (click a night or a trial to open it).
  - The whole view is in the URL, so it can be shared as a link.
- **Writes back** one thing, the baseline choice, as the MLflow tag `bench.pinned_baseline`.
- **Costs**: about 2 500 lines of Python in the prototype, which reuse `figures.py`, `compare.py` and the
  registry. It is one more service to run next to the MLflow server (a Flask process; the Dash dev server
  here, gunicorn for a team deployment). It has no auth of its own, so it goes behind the same network
  or proxy as MLflow.
- **Not done yet**: a production WSGI setup, caching the catalog for thousands of runs (pagination or
  filters in `search_runs`), per-PR static report export, and a Foxglove deep link from a timeline moment.

Checked:
- 10 unit tests, including a bundle round-trip.
- Every section of the five kinds, in each of the three modes, rendered headlessly with no browser errors.
- In a scripted browser: hover-linking, the zoom window, linked zoom in side-by-side, mode switches that
  keep the selection, opening a trial from the sweep grid, and the URL state.

## Re-running the W&B leg

```bash
cd tools/sim_benchmark/tracking
uv run wandb login                                  # and export WANDB_ENTITY=<team> if not personal
R=<IFSSIM checkout>/tools/sim_benchmark/results      # holds onboard/ and capture/
uv run bench-track import     --backend wandb --replay-root $R --sim-root ../results/sim_mock
uv run bench-track dashboards --backend wandb --replay-root $R --sim-root ../results/sim_mock
```
