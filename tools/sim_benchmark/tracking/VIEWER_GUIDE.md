# bench-view guide: what each page shows

A tour of the viewer's report types, their sections and the charts in each. For how to run it
and how it works inside, see the [README](README.md#bench-view-the-viewer-on-top-of-mlflow-prototype).

The side panel on the left picks the **report type**, and under it the **section** of that page.
Collapse it with the « button (or press `[`) to give the plots the full width; the choice is
remembered in your browser.

## Report types at a glance

| Type | One report is… | Data in the local store (Sep 2026) | Real? |
|---|---|---|---|
| [Bag benchmarks](#bag-benchmarks) | one offline replay of a recorded bag (`--report`) | 4 replays of `manual_20260920_154527` (21–22 Sep); the 21 Sep one is the baseline | ✅ real |
| [Live runs](#live-runs) | one bag played into the live stack (`--live`) | 5 runs of the same bag (22–23 Sep), 2 of them aborted | ✅ real |
| [Simulator bag benchmarks](#simulator-bag-benchmarks) | one session of `run_sim_bag_benchmark.py`: perception and SLAM on one simulator bag, scored against the simulator's ground truth | 5 sessions from 23 Sep (5 different bags), paired from separate perception and SLAM runs; their code was not recorded | ✅ real |
| [Sim benchmarks](#sim-benchmarks) | one pipeline commit across the whole scenario matrix | 2 commits: `a64350a` (dev, baseline) and `c0ffee1` (`feat/adaptive-lookahead`). Each runs 3 scenarios with several seeds | ❌ mock |
| [Nightly](#nightly) | one night | 8 nights on the nominal scenario; nights 4 and 5 are deliberately bad | ❌ mock |
| [Parameter sweeps](#parameter-sweeps) | one parameter combination (a trial) | 12 trials over `control.lookahead_gain` × `control.max_lat_acc`, 3 seeds each | ❌ mock |

The run data is not in the repository: it lives in the local MLflow store.
The sim pages show made-up data from `mock_sim.py`, because no sim benchmarks have been run yet.
Their charts are real, but the numbers are not.

## Picking and comparing runs

The same controls work on every page:

- **One run** is the default: the latest report of that type.
- **Change…** opens the run browser. Runs are grouped by day, their key numbers are coloured
  against the baseline, and each row has *Open* and *+ Compare*. Type to filter.
- **+ Compare** overlays up to 3 more runs. The first one added is the **reference** for every
  "Δ" / "difference to the reference" chart.
- **Side by side** puts exactly two runs in columns with matched axes. *⇄ Swap* flips them.
- **★ Make baseline** pins a run as the baseline (drawn in black, used to colour the browser).
- The URL holds the whole selection, so any view can be shared as a link.

On time charts, a single **playhead** is shared by the whole page. Hover any chart, drag the
playback bar, press space to play, or use ←/→ (shift for 10 s). Each chart header shows every
run's value at that moment, and zooming one time chart zooms them all.

---

## Bag benchmarks

**Real data.** These are offline replays run with `--report`. They record sampled topics, the
pose (odometry and SLAM), the final SLAM map and the node logs.

### Summary

- **Metric tiles** for the headline metrics of the report.
- When comparing: a **comparison table** grouped by domain, plus a **verdict line** that flags
  any metric that moved beyond its registry tolerance.
- *All metrics* folds out the full list.

### Trajectory

The Foxglove-style view:

- **Route map**: odometry (solid) and SLAM (dotted) paths, with the map cones. Each run has a
  marker and an 8 s trail that follow the playhead. Click the map to jump there.
- **Moment panel**: the events within ±2 s of the playhead.
- **Stacked time panels** on a shared time axis. *Panels & map* chooses which to show:
  - cone detections per scan;
  - cones Δ vs the reference (1 s mean);
  - SLAM map size (landmarks);
  - SLAM − odometry gap;
  - SLAM processing time;
  - SLAM associated / observed ratio;
  - speed;
  - steering (dashed grey = the pilot in the bag);
  - throttle;
  - /Path publish rate;
  - pipeline events (the SLAM-state strip).

### Perception

| Chart | What it shows |
|---|---|
| Cones detected per scan | Raw cone detections per LiDAR scan, 1 s mean |
| Cones detected: difference to the reference | Each run minus the reference run; below zero = fewer cones |
| Detection funnel | Mean per scan through the cone filter: DBSCAN clusters → pass the shape test → accepted, plus cones dropped for range (from the `CONE_FILTER` log line) |
| Cones per scan: distribution | Violin of detections per scan, with box and mean |
| Left / right balance | Share of accepted cones on the left, 3 s mean; 0.5 = balanced |
| DBSCAN guard trips | Each point is a trip; y = points dropped to keep clustering bounded |
| Cone-detection node rate | Processing rate of the node, from its log |
| Points per scan entering the filter | LiDAR points into the filter, 2 s mean |

### SLAM

| Chart | What it shows |
|---|---|
| SLAM state | Mapping → localisation band per run, with the events SLAM logged |
| SLAM map size | Landmarks in the map over time |
| SLAM − odometry position gap | How far the SLAM pose is from odometry (a drift indicator) |
| SLAM processing time | Per update, 1 s mean |
| Where SLAM spends its time | Mean cost of each stage per update, stacked; the tick marks the p95 of the total |
| Final SLAM map | Landmarks at the end of the run, all runs overlaid in the same frame |
| SLAM − odometry heading difference | Yaw disagreement between SLAM and odometry |
| SLAM processing time: distribution | Cumulative distribution per run (log x); dotted line = p95 |
| Age of the cone message | How old the cone message was when SLAM processed it |
| Associated / observed cones | Fraction of observed cones matched to landmarks per update |
| New landmarks per update | Landmarks added per update |
| Pose correction per update | Size of the pose correction SLAM applied |
| Rejected pose jumps | Cumulative count of pose jumps SLAM refused |

### Control

| Chart | What it shows |
|---|---|
| Speed | Odometry speed, 0.5 s mean |
| Steering command | Autonomy steering; dashed grey = what the pilot did in the bag |
| Autonomy − pilot steering | Positive = the stack steers more to the left than the pilot did |
| Throttle | Throttle command, 0–1 |
| Steering residual: distribution | Histogram of autonomy − pilot; the legend gives the RMS per run |
| Steering rate | 0.5 s mean |
| Path length seen by control | Length of the path control is following |
| /Path publish rate | How often planning publishes a path |
| Path-planning callback rate | How often the planning callback runs |
| Poses in each /Path message | Size of each published path |
| Distance travelled | From the control log |

### Events & logs

- **SLAM state and pipeline events**: one row per run. The band is the SLAM mode and the symbols
  are events; hover one for its log line.
- **Event counts**: a matrix of event types × runs.
- **Node startup**: ○ ready · ● active · ★ first output, with 0 at the start of bag play.
- **Event and warning tables**, sortable.

### Everything

All the sections above on one page.

---

## Live runs

**Real data.** The bag is played into the live stack with `--live`. **Only node logs are
recorded**: there is no pose and no sampled topics.

The sections are the same as for [Bag benchmarks](#bag-benchmarks), with these differences:

- *Trajectory* becomes **Timeline**: the time panels without the route map.
- Charts that need the pose or sampled topics show "no data". Examples are the SLAM − odometry
  gap and autonomy − pilot steering.
- Where possible a chart falls back to a log source. For example, cones come from the
  `CONE_FILTER` log (cones accepted by the filter), and speed from the control status line.

---

## Simulator bag benchmarks

**Real data.** A bag recorded in the simulator carries the true track and the true pose, so
perception and SLAM are scored against the truth instead of against each other.
`run_sim_bag_benchmark.py` runs both on one bag; one report is that session. Sessions made with
`--only`/`--skip`, or where a benchmark failed, show what did run and are marked as failed.

### Which code, and reruns

Every session records the code it ran (`provenance.json`, see `tools/sim_benchmark/README.md`):

- The report is named by its **code label** and time: `a64350a · 25 Sep 11:00` for a clean
  commit, `a64350a-dirty.3f2c1a9e · 26 Sep 09:00` when something was uncommitted (the diff is
  saved with the run).
- The run browser groups sessions **by bag**. Two sessions of identical code (same commits, same
  uncommitted changes) are reruns: the newest says *latest of N runs of this code*, the older
  ones are dimmed and say *earlier run of this code*. A fix, committed or not, is new code and
  gets its own label.
- Nothing is overwritten or hidden: every session stays in MLflow, and any two can be compared.

### Summary

- **Facts**: bag, benchmarks that ran, code label, pipeline and IFSSIM commits, and which repos
  had uncommitted changes.
- **Metric tiles**:
  - perception precision and recall;
  - cone detection time p95;
  - SLAM position error (RMS and p95);
  - landmarks in the map, and the share of true cones that made it into the map;
  - EKF odometry error.
- Comparing: the comparison table and verdict line, as for bag benchmarks.

### Trajectory

The Foxglove-style view, with the **true pose as the solid line** and SLAM dotted. The map also
draws the true cones (rings, in their colour). Time is counted from the session's first sample,
because the benchmarks stamp samples with the simulator's clock and that clock starts at a
different value in every bag. The pose starts about 3 s after perception, because SLAM needs a few
scans first. Time panels:

- true speed;
- cones detected per scan;
- recall per scan;
- detection position error;
- cone detection time;
- SLAM position and heading error;
- EKF odometry error.

### Perception

| Chart | What it shows |
|---|---|
| Recall per scan | Detected ÷ true cones in view (range and field-of-view gated), 1 s mean |
| Cones detected vs cones in view | Solid = detected, dashed = true cones the sensor could see |
| Position error against range | Every matched detection; median per metre of range, p90 dotted |
| Detection position error over time | Mean over each scan's matched cones |
| Position error: distribution | Cumulative distribution of all matched cones, with the p95 |
| Missed and false detections per scan | True cones not detected, and detections with no cone there |
| Cone detection time | Per scan, offline on the machine that ran it |
| Where cone detection spends its time | Mean per stage (only with `--profile`) |
| LiDAR points per scan | Points into cone detection |

### SLAM & odometry

| Chart | What it shows |
|---|---|
| SLAM position error vs the true pose | Over time; the saw-tooth is drift between SLAM updates, corrected at each scan |
| Position error by pose source | SLAM, EKF odometry, wheel odometry, IMU only and the bag's `/odom`: mean error (bar) and p95 (whisker), log scale |
| Final SLAM map vs the true track | True cones as rings, each run's landmarks as triangles |
| EKF odometry error vs the true pose | Over time |
| SLAM heading error vs the true pose | Over time |
| SLAM position error: distribution | Cumulative distribution, with the p95 |
| True speed | From the simulator's pose |

### Everything

All the sections above on one page.

---

## Sim benchmarks

**Mock data.** One report is one pipeline commit, run on every scenario of the matrix
(nominal trackdrive, high-noise trackdrive, autocross) with several seeds each.

### Summary

- **One commit**: a facts card (commit, branch, commit message, when it ran,
  scenarios, seeds per scenario, and links to each scenario in MLflow) and a **key metrics** table,
  mean ± standard deviation over seeds, one column per scenario. The key metrics are:
  - lap time, mean and best;
  - finish rate;
  - cones hit;
  - off-tracks;
  - cross-track RMS and max;
  - mean speed.
  *All metrics* folds out the rest.
- **Comparing**: a **scorecard** against the reference commit, per scenario and metric. Each
  row shows both values, the difference, its confidence interval and the probability that it is
  better, with a verdict (better / worse / no change) and a count of each at the top.
  *Only show changes* hides the rows that did not move.

### Laps & reliability

| Chart | What it shows |
|---|---|
| Lap times | Every completed lap of every seed, one panel per scenario (lap 1 is left out for multi-lap events: standing start) |
| Reliability | Finish rate, cones hit, off-tracks and DNFs per scenario, mean over seeds; bar = std |
| Mean lap time per seed | One point per seed |
| Cones hit per seed | One point per seed |
| Cross-track RMS per seed | One point per seed |

### Along the track

- **Track map** coloured by a metric you choose, along the scenario's centreline.
- **Profiles along the lap**, all on distance along the lap:
  - mean \|cross-track\| (dotted = max over seeds);
  - mean speed (dotted = max);
  - mean SLAM error.
- When comparing, two extra lanes: **Δ cross-track** and **Δ speed** vs the reference.
- Hovering a profile moves a cursor on the map.

### Perception & compute

| Chart | What it shows |
|---|---|
| Perception recall vs range | Share of cones detected against distance, pooled over seeds; line style = scenario |
| End-to-end latency | Cumulative distribution pooled over seeds; dotted line = p95 |
| Latency per pipeline stage | Mean per stage, stacked |
| Perception precision per seed | One point per seed |
| CPU per seed | One point per seed |

### Everything

All four sections together.

---

## Nightly

**Mock data.** One report is one night: an aggregate over seeds on the nominal scenario.

- **Trend** (extra first section): small-multiple lines, one point per night, against the
  baseline. By default they show:
  - lap time;
  - finish rate;
  - cones hit;
  - cross-track RMS;
  - perception recall;
  - end-to-end latency p95.

  You can pick other metrics. Nights 4 and 5 are deliberately bad, to show what a regression
  looks like. Click a night to open it.
- **Summary, Laps & reliability, Along the track, Perception & compute, Everything**: the same as
  [Sim benchmarks](#sim-benchmarks), applied to one night.

---

## Parameter sweeps

**Mock data.** One report is one trial, a point on the controller parameter grid
(`control.lookahead_gain` × `control.max_lat_acc`), with three seeds.

- **Explore** (extra first section). Click a point in any chart to open that trial.
  - **Lap time over the grid**: a heatmap of mean lap time for each parameter pair.
  - **Pareto**: lap time against cones hit, coloured by cross-track RMS; faster and cleaner is
    bottom left.
  - **Parallel coordinates**: every trial as a line across the parameters and the results.
- **Summary, Laps & reliability, Along the track, Perception & compute, Everything**: the same as
  [Sim benchmarks](#sim-benchmarks), applied to one trial.
