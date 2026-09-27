# How benchmark tracking works

This guide is for someone who has never used the benchmark tracker. It
explains what it is for, the words people use around it, and what happens
from the moment you run a benchmark to the moment the whole team can see the
result. You don't need to know how it was built.

**Contents**

1. [What problem this solves](#1-what-problem-this-solves)
2. [Words you will meet](#2-words-you-will-meet)
3. [The journey of one benchmark run](#3-the-journey-of-one-benchmark-run)
4. [What you need to do](#4-what-you-need-to-do)
5. [Reading what you see](#5-reading-what-you-see)
6. [Common questions](#6-common-questions)
7. [Behind the scenes (for people changing the system)](#7-behind-the-scenes-for-people-changing-the-system)

---

## 1. What problem this solves

We change the driverless software all the time: perception (finding cones),
SLAM (knowing where the car is and building a map), control. To know whether
a change made things better or worse, we run **benchmarks**. A benchmark plays
recorded sensor data through the software and measures how well it did.

Before this system:

- each benchmark wrote an HTML report into a folder on whoever's computer ran it;
- nobody else could see it unless it was sent to them;
- it was hard to know exactly which version of the code produced a report,
  especially if the code had changes that were never committed;
- comparing two reports meant opening two files side by side.

Now:

- **every run records exactly which code produced it**, automatically;
- **every run is sent to one shared server**, automatically;
- **anyone on the team can open, compare and discuss any run** in one web
  page, the viewer.

You keep running benchmarks the same way you always did; the rest happens
by itself.

## 2. Words you will meet

| word | meaning |
|---|---|
| **Benchmark** | A script that runs part of our software on recorded data and measures the result, e.g. "how many cones did perception find, and how far off were they?" |
| **Bag** | A recording of everything the car's sensors produced during a drive (a ROS 2 "rosbag"). Bags can be many gigabytes. They **never** leave the computer they are on; only results are shared. |
| **Simulator bag** | A bag recorded in our simulator. The simulator knows the *truth* (where every cone really is, where the car really is), so a simulator bag lets us score the software against the right answer. |
| **Ground truth** | That "right answer" from the simulator. Bags from the real car don't have it. |
| **Run** | One execution of a benchmark. It produces a **results folder** with the numbers, charts and an HTML report. |
| **Session** | For simulator bags: the perception benchmark and the SLAM benchmark run together on one bag, kept together as one result. |
| **Commit** | A saved version of the code in git. Each has an id like `a64350a`. |
| **Uncommitted changes** | Edits you made to the code but did not commit yet. A run made with them is not reproducible from the commit alone, so they are saved with the run. |
| **Push / unpushed** | Pushing sends your commits to GitHub. A commit you have not pushed exists only on your computer. |
| **Docker / image** | Most benchmarks run inside a Docker **container**, a ready-made environment with ROS and the rest of the software stack installed. The **image** is the template it is made from. Different images can contain different software. |
| **MLflow** | An open-source tool for storing experiment results. It is our **database of runs**. It has its own website too, but you will mostly use the viewer instead. |
| **Central machine / server** | One computer that runs MLflow for the whole team, so everyone sends their runs to the same place. |
| **Tailscale** | A private network for the team. The central machine can only be reached by people connected to it, from anywhere, without it being on the public internet. |
| **Viewer (`bench-view`)** | Our web page for looking at runs: reports, charts, comparisons. It reads everything from MLflow. |
| **Baseline** | The run that others are compared against, e.g. "the current good version". Differences are shown as better or worse than it. |

## 3. The journey of one benchmark run

```
  ① you run a benchmark
        │
  ② it writes down which code it is running
        │
  ③ it does its work and saves a results folder on your computer
        │
  ④ when it finishes, the results are sent to the central server
        │
  ⑤ anyone on the team opens them in the viewer
```

### ① You run a benchmark

Exactly as before, for example:

```bash
python tools/sim_benchmark/run_sim_bag_benchmark.py results/capture/<some simulator bag>
```

The benchmark scripts and their options are described in
[`../README.md`](../README.md).

### ② It writes down which code it is running

Before doing anything else, the benchmark takes a snapshot of the code and
saves it in the run's folder:

- which **commit** of our two code repositories was checked out (IFSSIM, and
  `pipeline`, which holds the driverless software);
- any **uncommitted changes**, saved as a file (a "diff"), so they are not
  lost;
- any **unpushed commits**, saved as a file, so someone else can get them
  even though they are not on GitHub;
- which **Docker image** it ran in;
- which **computer** it ran on, and who ran it.

From this it makes a short **code label**, the name you will see
everywhere:

- `a64350a`: the code was exactly commit `a64350a`;
- `a64350a-dirty.3f2c1a9e`: commit `a64350a` **plus uncommitted changes**.
  The part after `dirty.` identifies those exact changes. Change one line and
  the label changes too.

This is what makes "I ran it twice" and "I fixed something and ran it again"
look different, even if you never committed the fix.

You may see two warnings at this step. Neither stops the benchmark:

- *"… commits that are on no remote. Push them …"*: you have commits that are
  only on your computer. The run keeps a copy, but pushing them is better.
- *"image … was built locally …"*: you built the Docker image yourself, so
  nobody can tell whether it matches theirs. That's fine for your own
  experiments. For results you want to compare with other people's, use a
  published image (see [6](#6-common-questions)).

### ③ It does its work and saves a results folder

The benchmark runs as it always has and writes its results under
`tools/sim_benchmark/results/` on your computer: numbers, CSV files, an HTML
report. Nothing about this part changed.

### ④ The results are sent to the central server

When the benchmark ends, the results are uploaded to the team's server by
themselves. The upload step is called **sync**. It is careful:

- **Nothing is sent twice.** If the server already has a run, it is skipped,
  even if it was uploaded from someone else's computer.
- **Nothing is lost when you are offline.** If the server cannot be reached,
  the results stay on your computer, and the next benchmark you run (or a
  manual sync) sends everything that is missing.
- **Nothing half-finished is sent.** A run that is still going waits for the
  next sync.
- **Bags are never sent**, only results.

If your computer isn't set up for uploading yet (section 4), benchmarks still
work; the results simply stay on your computer.

### ⑤ Anyone opens them in the viewer

The viewer is a web page. On the central machine it is always running, at
the team's address on Tailscale. You can also run it on your own computer
(section 4).

It has one tab per kind of benchmark (bag benchmarks, live runs, simulator
bag benchmarks, and so on). There you can:

- open any run as a report with charts;
- pick two or more runs and see them **overlaid** or **side by side**;
- see which code produced each run, and whether it was a rerun;
- set the **baseline** that runs are compared against. Everyone sees the same
  choice.

What every tab and chart shows is explained in
[`VIEWER_GUIDE.md`](VIEWER_GUIDE.md).

## 4. What you need to do

### Once: connect your computer to the team server

1. **Join the team's Tailscale network**, if you haven't already (ask whoever
   manages it).
2. **Ask the admin for a login.** They'll give you a user name and a password.
3. **Create the file** `~/.config/ifssim-bench/tracking.env` on your
   computer:

   ```ini
   MLFLOW_TRACKING_URI=https://<server address from the admin>:8443
   MLFLOW_TRACKING_USERNAME=<your user name>
   MLFLOW_TRACKING_PASSWORD=<your password>
   ```

   This file stays on your computer. **Never put it in the repository**: the
   repository is public.
4. **Check that it works:**

   ```bash
   cd tools/sim_benchmark/tracking
   uv run bench-track sync --dry-run
   ```

   It lists which of your runs the server already has and which it would
   upload. It doesn't send anything.

### Every day

- **Run benchmarks as usual.** They upload when they finish.
- **To look at results**, open the viewer at the team address, or run it
  locally:

  ```bash
  cd tools/sim_benchmark/tracking
  uv run --extra viewer bench-view      # then open http://127.0.0.1:8050
  ```

### Sometimes

| you want to… | do this |
|---|---|
| send results made while offline, right now | `uv run bench-track sync` in `tools/sim_benchmark/tracking` |
| keep one simulator bag session off the server | add `--no-upload` to `run_sim_bag_benchmark.py` |
| stop uploading altogether for a while | add `IFSSIM_AUTO_UPLOAD=0` to your `tracking.env` |

Setting up the central machine itself (for the admin) is in
[`DEPLOY.md`](DEPLOY.md).

## 5. Reading what you see

**Code labels.** `a64350a` is clean committed code. `a64350a-dirty.3f2c1a9e`
means committed code plus local changes. Two runs with the *same* label ran
the *same* code.

**Reruns.** When the same code ran more than once on the same bag, the viewer
says so: the newest is "latest of N runs of this code", and the older ones
are dimmed. Reruns show how much results vary from run to run on their own,
which is worth knowing before calling a difference an improvement.

**Badges and run details.** Each run's details show:

- the commits;
- whether there were uncommitted changes (with the saved diff);
- whether there were unpushed commits;
- the Docker image, and whether it was built locally;
- the computer it ran on.

**"Code not recorded".** Runs made before this system existed don't know
their code. They are still shown, marked that way.

**Better or worse.** Every number has a direction (for errors lower is
better, for recall higher is better). The viewer uses it to show changes
against the baseline as improvements or regressions.

## 6. Common questions

**Can I break anything by running a benchmark?**
No. Recording the code and uploading can't make a benchmark fail. If either
has a problem, it prints a warning and the benchmark carries on.

**I ran a benchmark with a bug, fixed it, and ran it again. Does the first one overwrite the second?**
No, nothing is ever overwritten. The fix gives a different code label, so
both runs are kept, and it's clear which is which.

**Can someone reproduce my run exactly?**
Yes. The run's details list the commits; the saved files give back your
uncommitted changes and unpushed commits. For the Docker image, runs made
with a published image can be matched exactly. Pick one by setting
`IFSSIM_DV_IMAGE=ghcr.io/isc-fs/ifssim-dv_pipeline_stack:sha-<commit>` before
running. A locally built image cannot be matched.

**Two of us have a copy of the same results. Will they appear twice?**
No. Each run has an identity that doesn't depend on whose computer it is on,
and the server checks it before accepting an upload.

**Why do my timing results differ from the central machine's?**
Processing times depend on the computer. Accuracy numbers can be compared
between computers; times should be compared on the same computer. The run
details show which computer it was. Automatic per-computer comparisons are
not built yet.

**Who can delete runs?**
Only the admin. Everyone else can upload runs and set baselines.

**What if the server is down?**
Your results wait on your computer. The next benchmark or sync sends them.

**Are bags uploaded?**
Never. Only results.

---

## 7. Behind the scenes (for people changing the system)

Everything above is what users see. This part is for people who work on the
tracker itself.

### The pieces

```
 benchmark script ──► results folder ──► adapter ──► bundle ──► MLflow ──► viewer
 (records code)       (on disk)          (reads a     (one common  (server)   (reads bundles)
                                          folder)      shape)
```

- **Recording the code.** Every benchmark creates its folder with
  `common.make_run_dir`, which calls `run_provenance.record()`. That writes
  `provenance.json`, plus `<repo>.diff` and `<repo>.unpushed.bundle` when
  needed. The container has no git history, so `maybe_reexec_in_docker`
  captures on the host and passes the files in through a staging folder
  (`results/.provenance/`, env var `IFSSIM_PROVENANCE_DIR`).
  - The **code id** hashes both commits, both diffs and a registry image
    digest.
  - The **label** is the pipeline's short commit, plus `-dirty.<code id>` when
    anything was uncommitted.
- **Adapters** (`bench_tracking/adapters/`) turn each kind of results folder
  into a **RunBundle** (`bundle.py`): `config`, `summary` metrics, `series`
  over time, `tables`, `events` and `files`.
  - Metric names and their better or worse direction are registered in
    `metrics.yaml`.
  - Simulator bag times are shifted to start at 0, because the simulator clock
    starts at a different value in every bag.
- **Grouping.** The *scenario id* identifies what was measured (the bag, and
  the options that change the measurement). The *group*,
  `scenario_id@code_id`, collects reruns.
- **Baselines.** `compare.py` picks them: a baseline pinned in the viewer,
  else one tagged at import, else the earliest finished run of the scenario.
  It also computes the deltas.
- **Upload.**
  - `auto_upload.py` runs `bench-track sync` on the host after each
    benchmark, but only if a server is configured and `IFSSIM_AUTO_UPLOAD` is
    not `0`. The simulator bag runner switches it off for its child
    benchmarks and uploads the session at the end.
  - `sync` checks the server's health endpoint first, and skips runs still in
    progress (no results yet, changed in the last 6 hours).
  - It then asks MLflow for each run's **run key**: the capture id from
    `provenance.json`, else the folder's last two path parts, stored as the
    `bench.run_key` tag. It uploads only what is missing.
  - A local `.state/mlflow@<server>.json` remembers what this machine
    uploaded, which limits what `purge` may delete.
- **MLflow layout.**
  - One experiment per kind: `ifssim-bench/replay`, `/sim-bag`,
    `/sim-matrix`, `/sim-nightly`, `/sim-sweep`.
  - Params hold the flattened config; tags hold the ids and flags.
  - Metrics hold the summary numbers, and each series as metric history.
  - The `bundle/` artifact holds the full bundle as Parquet, which is what
    the viewer reads.
- **Viewer** (`bench_tracking/viewer/`). It lists runs from tags and params
  (`data.py`), downloads a run's bundle when it is opened, and draws pages
  (`pages.py`) and charts (`replay.py`, `simbag.py`, `sim.py`). The playhead
  runs in the browser (`assets/sync.js`), and the whole view state is in the
  URL. The same app runs locally (`bench-view`) or under gunicorn
  (`wsgi()`).
- **Server** (`deploy/central/`). Docker Compose runs Postgres, MLflow with
  logins, and the viewer, all on 127.0.0.1 and shared over Tailscale.
  - Every login gets EDIT permission: upload and pin, not delete.
  - The admin creates the experiments, so only the admin can delete.
  - Details are in [`DEPLOY.md`](DEPLOY.md).

### Where the code lives

| file | job |
|---|---|
| `tools/sim_benchmark/common.py` | creates run folders; hooks for provenance and upload |
| `tools/sim_benchmark/run_provenance.py` | records the code: commits, diffs, unpushed bundles, image |
| `tools/sim_benchmark/auto_upload.py` | decides whether to upload, runs `bench-track sync` |
| `tools/sim_benchmark/run_sim_bag_benchmark.py` | runs a simulator bag session |
| `tracking/bench_tracking/adapters/` | results folder → bundle, one module per kind of run |
| `tracking/bench_tracking/bundle.py`, `store.py` | the bundle, the run key, the Parquet format |
| `tracking/bench_tracking/compare.py`, `metrics.yaml` | baselines, deltas, metric names and directions |
| `tracking/bench_tracking/cli.py` | `bench-track`: `sync`, `import`, `admin`, `purge` |
| `tracking/bench_tracking/config.py` | reads `~/.config/ifssim-bench/tracking.env` |
| `tracking/bench_tracking/backends/mlflow_backend.py` | writes a bundle to MLflow; looks up run keys |
| `tracking/bench_tracking/viewer/` | the viewer |
| `tracking/deploy/central/` | the central server |
| `tracking/deploy/mlflow/run_server.sh` | a private local MLflow for trying things out |

### Other documents

| to… | read |
|---|---|
| run the benchmarks, and their options | [`../README.md`](../README.md) |
| set up or look after the central server | [`DEPLOY.md`](DEPLOY.md) |
| understand every tab and chart in the viewer | [`VIEWER_GUIDE.md`](VIEWER_GUIDE.md) |
| use the `bench-track` commands, or see what is logged per run | [`README.md`](README.md) |
