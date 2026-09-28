# Benchmark launcher: running benchmarks from the web, on PRs and on commits

Status: **design proposal**
Date: 2026-09-28
Scope: `tools/sim_benchmark/` (the runners), `tools/sim_benchmark/tracking/` (bench-track,
bench-view, the central deployment).
Follows [`2026-09-23_benchmark-tracking-design.md`](2026-09-23_benchmark-tracking-design.md). Code
references are to branch `feat/517-bench-tracking-viewer` (HEAD 3424cde), plus
`run_onboard_replay.py` from `feat/516-onboard-bag-replay` (2c27956).

## 0. The change of plan

The tracking work so far assumed **everyone runs benchmarks on their own machine** and every
machine uploads to a central MLflow. The plan is now:

- Benchmarks run **on the central machine**. They are launched three ways:
  1. from the web page;
  2. on pull requests;
  3. as small checks on commits.
- A local run is the **fallback** for when the central machine is down. It should still work,
  but it will be rare.
- The web page has **two main tabs**: *Launch* (start benchmarks, watch the queue) and *Results*
  (today's bench-view).
- Launching from the web means choosing benchmarks in a form and **uploading a YAML** with extra
  settings and hyperparameters.

This document settles what the form sets and what the YAML sets (§3). It also says how a launched
run gets from the page to MLflow (§2, §5–§8) and what of the current upload machinery goes away
(§10).

Decisions not reopened here:
- MLflow is the system of record, and bench-view reads it.
- Bags are never uploaded.
- `provenance.json` records the code of every run.
- The central machine is reached only through Tailscale.

---

## 1. What exists today (and what it means for the design)

| Fact (from the code) | Consequence |
|---|---|
| Three runners, each a CLI: `run_onboard_replay.py` (feat/516), `run_sim_bag_benchmark.py` (which calls `run_perception_benchmark.py` and `run_slam_benchmark.py`), and `bench-track mock-sim`. There is no real simulator-in-the-loop runner yet. | The launcher drives the existing CLIs. It doesn't reimplement them. A "benchmark" in the form is one runner. Simulator benchmarks appear once a real runner exists. |
| The runners re-exec themselves in Docker (`maybe_reexec_in_docker`, `common.py`) and mount the host's `pipeline/` over the image. | The worker needs Docker, the bags and a checkout of the code. It runs **on the host**, not in the compose stack (§6). |
| **Pipeline parameters reach the code in three different ways:** (1) Replay starts real nodes with `ros2 run … --ros-args -p use_sim_time:=true` (`_start_nodes`). (2) The SLAM benchmark builds `ConeGraphSlamNode` in-process and calls `set_parameters` (`slam_metrics.py:751`). (3) Perception uses the `ConeDetectionConfig` dataclass (26 fields). It has no ROS parameters, and `cone_detection` declares none. | The YAML can't just be "a ROS params file". §4 defines one `pipeline:` section and says how each runner applies it. The perception benchmark can take overrides without touching the pipeline (§4). Only the replay node needs a small change in the pipeline repo. |
| ROS parameters with defaults in code: control 24, cone_slam 26, odometry filter 17. No parameter files exist. | Every override is a difference from code defaults. A run's parameters are fully described by `code_id` + the overrides. |
| `publish-pipeline-image.yml` publishes `ghcr.io/isc-fs/ifssim-dv_pipeline_stack:sha-<short>` only on pushes to `dev`/`main`, and only when `docker/`, `ros2/src/` or `pipeline` changed. | A PR commit usually has **no image of its own**. Most PRs only change `pipeline/` Python, which is mounted from the host, so the image of the branch point is correct for them. §6 says when to build instead. |
| `provenance.code_id` hashes both commits, both diffs and the image digest. The viewer marks runs with equal `code_id` as reruns of identical code. | Two web runs with the same code but different gains would be wrongly shown as reruns. Runs need a **`spec_id`** too, and the viewer groups by `(code_id, spec_id)` (§3.4). |
| Uploads today: `auto_upload.py` runs `bench-track sync` after every run, sync scans the results folders, and each person has an MLflow login. | That machinery exists for many machines uploading at random times. With one machine running the benchmarks, the worker knows exactly which run finished and uploads just that one. Most of `auto_upload` goes (§10). |
| The repo `isc-fs/IFSSIM` is **public**. | GitHub must never run code on the central machine: **no self-hosted runner** (a PR from a fork could run anything). The central machine asks GitHub for work instead (§7). |

---

## 2. Architecture

```
  browser ──► Tailscale serve (HTTPS, adds Tailscale-User-Login)
                 │
                 ▼
  ┌────────── central machine ────────────────────────────────────────────────────┐
  │ bench-view (gunicorn, compose)                                                │
  │   Results tab  ── reads ──────────────────────────────► MLflow ──► Postgres   │
  │   Launch tab   ── writes a job (spec + who) ──► Postgres `bench_jobs`         │
  │                                                   ▲                           │
  │ GitHub poller (in the worker) ── PR label / push ─┘ (creates jobs)             │
  │                                                                               │
  │ bench-worker (systemd, on the host, one job at a time)                        │
  │   1. take the next job   2. worktree at the pinned SHAs   3. pick the image   │
  │   4. run each benchmark in the spec (existing CLIs, Docker)                   │
  │   5. upload those runs to MLflow   6. report back (job row, GitHub status)    │
  └───────────────────────────────────────────────────────────────────────────────┘
```

- **One job at a time.** Timing metrics are only comparable if runs don't compete for CPU or GPU.
  A single queue on one machine also removes the open "per-machine timing baselines" problem.
- **The job table is the interface.** The Launch tab, the GitHub poller and the CLI all do the same
  thing: insert a row with a spec. Only the worker runs anything.

---

## 3. The run spec, and where the form stops and the YAML starts

### 3.1 The rule

> **The form chooses *what* runs *on what*. The YAML sets *how the code under test behaves*.**

A setting is a **form field** when it is chosen from a list the central machine knows and can
check before launch:
- which benchmarks;
- which bags, from the ones on disk;
- which code, from branches, PRs and commits;
- which baseline, from the runs in MLflow.

A setting is **YAML** when it is a free-form value read by the pipeline or by a benchmark:
- gains, thresholds and gates;
- noise models and timeouts;
- sweep ranges.

These values are many, change often, need diffs and review, and belong in version control.

### 3.2 One spec, two editors

There is **one spec format** (YAML, `schema: 1`). The form isn't a separate input: it edits the
same keys. On launch:

1. The page starts from a **preset** (a spec file from `tools/sim_benchmark/specs/`) or from the
   spec of an earlier run ("Rerun / edit").
2. An uploaded YAML is merged on top.
3. The form's fields are merged on top of that. **The form wins.** A field the YAML also set is
   marked "overridden by the form".
4. The page shows the **merged spec** and the jobs it will create. The jobs are created when you
   press *Launch*.
5. The merged spec is stored with the job and uploaded with every run as `spec.yaml`.

The dividing line is therefore only which keys have form widgets. It can move later without
changing the format. The same files serve the other two ways to run:
- **PRs and commits:** checked-in specs, `specs/pr.yaml` and `specs/commit-smoke.yaml`.
- **Local runs:** `bench-run <spec.yaml>` (§9).

### 3.3 The schema

```yaml
schema: 1
name: loop-closure gate retune          # form
notes: after #88; expect fewer false closures   # form

code:                                   # form: branch, PR or commit; pinned to SHAs at launch
  ifssim: dev
  pipeline: feat/88-loop-closure
  image: auto                           # auto | sha-<short> | build   (§6)

benchmarks:                             # form: which runners, on which inputs
  sim_bag:
    bags: [trackdrive_test_bag_01, skidpad_02]
    only: [perception, slam]            # optional; default all
    repeats: 1
    settings:                           # YAML: the runner's own knobs (CLI flags today)
      gt_range_m: 20.0
      gt_hfov_deg: 120.0
      max_frames: null
  onboard_replay:
    bags: [manual_20260920_154527_indexed]
    mission: trackdrive                 # form (a fixed list)
    repeats: 3
    settings:
      rate: 1.0
      duration_s: 0

compare_to: pinned                      # form: pinned | merge-base | <MLflow run id>

pipeline:                               # YAML: overrides of pipeline defaults, for every benchmark
  cone_detection:                       # ConeDetectionConfig fields
    residual_gate_mse: 0.05
  slam_node:                            # ROS parameters, by node name (run_onboard_replay NODES)
    motion_model: imu
  control_node:
    lookahead_min: 3.0

sweep:                                  # YAML, optional: one job per combination
  pipeline.cone_detection.residual_gate_mse: [0.02, 0.05, 0.1]
```

| Key | Set by | Checked against |
|---|---|---|
| `name`, `notes` | form | nothing |
| `code.*` | form (YAML may give a default) | GitHub/`git ls-remote`, resolved to SHAs |
| `benchmarks.<name>` (present or not), `bags`, `only`, `mission`, `repeats` | form | the runner list, the bag inventory on the central machine, `MISSION_BEHAVIORS` |
| `benchmarks.<name>.settings` | YAML | the runner's argument parser (unknown key = error) |
| `compare_to` | form | MLflow |
| `pipeline.*` | YAML | code defaults, at job start (§4): an unknown parameter fails the job before it runs anything |
| `sweep` | YAML | keys must exist in the rest of the spec |

**One place per value.** A pipeline parameter lives only under `pipeline:`, never under a runner's
`settings`. Today `run_sim_bag_benchmark.py --motion-model` sets `slam_node`'s `motion_model`. In
the spec that is `pipeline.slam_node.motion_model`, and the runner flag is filled in from it.

The spec is validated in one place, `bench_tracking/launch/spec.py`, against the repository's
manifest (§3.5). The page, the worker and `bench-run` all call it. *(Built: this replaced the
`spec.schema.json` first planned here, because most checks need the manifest: which bags exist,
which settings a benchmark takes.)*

### 3.5 The manifest: what the launcher knows about a repository

*(Added while building step 2.)* The launcher is going to move to a repository of its own, so it
must not know how IFSSIM lays out its benchmarks. Everything it needs is in **`bench.yaml` at the
repository root**:

- where results go, where preset specs are, which folders hold bags;
- which repositories `code:` can pin;
- per benchmark: the command, its settings (name, type, flag), its parts, and which pipeline
  component each part takes overrides for.

The boundary between the two sides is: a command line, the `--pipeline-overrides` JSON file
(`tools/sim_benchmark/pipeline_overrides.py` describes it), `$BENCH_SPEC` (the job's `spec.json`,
copied into the run folder with `provenance.json`), and the results folders the tracker already
reads. The format is in `bench_tracking/launch/manifest.py`.

### 3.4 Identity: `spec_id`

`spec_id` = hash of the merged spec, **without** `name`, `notes`, `compare_to`, `code`, `repeats`
and `sweep`. `code` is already in `code_id`. A sweep job carries its own resolved values. So
`spec_id` changes exactly when the benchmark inputs or parameters change.

- It is stored as the MLflow tag `bench.spec_id`, next to `code.id`.
- The viewer groups reruns by `(code_id, spec_id)`.
- A spec with no `pipeline:` overrides and default settings gets the fixed id `default`. Old runs
  are treated as `default`.

---

## 4. How `pipeline:` overrides reach the code

| Runner | Today | With overrides |
|---|---|---|
| `run_onboard_replay.py` (real nodes) | `ros2 run <pkg> <exe> --ros-args -p use_sim_time:=true` | Write `params/<node>.yaml` (`<node>: {ros__parameters: {...}}`) and add `--params-file` to each node that has overrides. |
| SLAM benchmark (in-process node) | `node.set_parameters([Parameter("motion_model", …)])` | The same call, with every key from `pipeline.slam_node`. |
| Perception benchmark (dataclass) | `strategy.CONE_DETECTION_CONFIG or ConeDetectionConfig()` | `dataclasses.replace(cfg, **pipeline.cone_detection)`. Unknown field = error. |
| `cone_detection_node` in replay | same dataclass; no ROS parameters | **Needs a pipeline change:** one parameter (e.g. `config_overrides`, a YAML string) applied with `dataclasses.replace` at configure time. |

After activation, each runner dumps the effective parameters (`ros2 param dump`, or the dataclass as
JSON) into `params/` in the run folder. This was already proposed in the 2026-09-23 design (§1).
It is the check that the overrides took effect, and the viewer shows it.

---

## 5. Jobs and the queue

A table `bench_jobs` in the existing Postgres, in a separate database `bench` next to `mlflow` and
`mlflow_auth`:

| column | |
|---|---|
| `id`, `created_at`, `started_at`, `finished_at` | |
| `trigger` | `web`, `pr`, `commit`, `cli` |
| `requested_by` | Tailscale login, or `github:<user>` |
| `spec` | the merged spec (JSON) |
| `code` | the resolved SHAs and image |
| `state` | `queued` → `running` → `done` / `failed` / `cancelled`; `upload_pending` if MLflow was down |
| `priority` | web and PR 10, commit 0; ties in creation order |
| `runs` | the MLflow run ids it produced |
| `log_path` | the worker's log for the job, streamed to the page |
| `github` | repo, PR, head SHA, status/comment ids (for reporting back) |

- **Sweeps and repeats** expand into one job per combination at launch, so the queue shows real
  progress and one failure doesn't lose the rest. The jobs share a `batch_id`.
- **Cancel:** a queued job just changes state. A running one gets SIGTERM on its process group,
  and the partial run folder is kept but not uploaded.
- **Timeout** per benchmark (default 2 h, `settings.timeout_s` to change it).
- **Worker restart:** jobs left `running` by a dead worker are marked `failed (worker restarted)`.
  They are never resumed.
- **Newer commit on the same PR:** queued jobs for the old head are cancelled.

Postgres is only reachable inside the compose network today. It gets published on
`127.0.0.1:5432` so the worker on the host can reach it.

---

## 6. The worker

`bench-worker`, a systemd service on the central machine (in `tracking/`, a new console script).
It runs on the host, not in compose, because it needs Docker, the bags, git and the GPU. Giving a
container the Docker socket would be root on the host anyway.

For each job:

1. **Code.** `git worktree add` at the pinned IFSSIM SHA, then check the pipeline submodule out at
   its pinned SHA. Web and CI runs always use **pushed** commits, so they are clean: no diffs, no
   unpushed bundles. Worktrees are removed after the job; the fetched objects stay cached.
2. **Image** (`code.image`):
   - `auto`: if the job's IFSSIM range changes `docker/dv_pipeline_stack/` or `ros2/src/` compared
     with `dev`, build the image locally and tag it `bench-local:<sha>`. Otherwise use
     `sha-<short>` of the `dev` commit the branch starts from (the newest published one at or
     before it).
   - `sha-<short>`: that tag.
   - `build`: always build.

   Built images get the warning `provenance.py` already gives for locally built images. The run
   records exactly which image was used.
3. **Run.** Each benchmark in the spec, through its existing CLI, with `IFSSIM_DV_IMAGE`, the
   settings as flags and the overrides as in §4. `IFSSIM_AUTO_UPLOAD` goes away (§10), so nothing
   uploads behind the worker's back. The results go to the usual `results/` folders on the central
   machine.
4. **Upload** just those runs through the existing backend, with the tags `bench.spec_id`,
   `bench.job_id`, `bench.trigger` and `bench.requested_by`, plus `spec.yaml`. If MLflow is down,
   the job becomes `upload_pending` and is retried every 5 minutes.
5. **Compare and report.** Deltas against `compare_to`, then the job row, plus a GitHub commit
   status and PR comment for `pr` and `commit` jobs (§7).

---

## 7. PRs and commits

The central machine **asks** GitHub every minute (the GitHub API, a fine-grained token with
read access to the repo and write access to commit statuses and PR comments). Nothing from GitHub
connects in, and no GitHub Actions runner ever runs on the machine. This matters because the repo
is public.

| Trigger | When | Spec | Reported |
|---|---|---|---|
| **PR** | the label `benchmark` is added to a PR **from a branch of this repo** (fork PRs are ignored; a member can push the branch to the repo to benchmark it), and again for each new head while the label stays on | `specs/pr.yaml` from the PR's own head, so a PR can change what it's benchmarked on | a commit status `bench/pr` (pending → success/failure) linking to the comparison in bench-view, and one PR comment updated in place with the headline deltas |
| **Commit smoke** | each push to `dev`, and each new head of a PR with the label `benchmark` | `specs/commit-smoke.yaml`: one short bag, perception + SLAM, `max_frames` capped; a few minutes | a commit status `bench/smoke` |

- **Baseline** for PRs: `compare_to: merge-base`. That means the newest run of the same spec on
  the `dev` commit the PR branches from. If there is none, a `dev` job is queued first.
- **Pass/fail:** `registry.py` already gives regression verdicts with per-metric tolerances
  (`metrics.yaml`). A status fails when a headline metric regresses beyond its tolerance.
  Timing metrics only warn until there's enough history to set their tolerances.

---

## 8. Access

- **The page:** anyone on the tailnet. `tailscale serve` adds `Tailscale-User-Login` to every
  request from a tailnet user. The Launch tab records it as `requested_by` and needs no password.
  The services listen on `127.0.0.1` only, so only local processes could forge that header.
- **Launch permissions:** everyone on the tailnet can launch and cancel their own jobs. A short
  `admins` list in the worker's config can cancel anyone's. Tailnet ACLs decide who reaches the
  machine at all.
- **MLflow logins** shrink to three: `admin` (deletes), `worker` (uploads), and `viewer` (reads,
  pins baselines). A person who uploads local fallback runs by hand (§9) gets their own login,
  as today.

---

## 9. Running locally, when the central machine is down

```bash
bench-run specs/pr.yaml --code-from-checkout     # the same spec, run against this checkout
bench-track sync --results-root tools/sim_benchmark/results   # later, by hand
```

- `bench-run` is the worker's per-job step 3 without the queue. It validates the spec, applies
  `pipeline:` overrides and runs each benchmark through its CLI.
- By default it uses the checkout as it is, dirty or not. `provenance.json` records the diffs and
  unpushed commits exactly as today. That's the case those features were built for.
- Nothing uploads automatically. `bench-track sync` (with the duplicate check by `bench.run_key`)
  uploads the finished runs when you choose. Local runs are tagged `bench.trigger=local` so the
  viewer can tell them apart.

---

## 10. What is removed, what stays

| | |
|---|---|
| **Removed** | `tools/sim_benchmark/auto_upload.py` and its calls in `common.py` (`maybe_reexec_in_docker`, `make_run_dir`) and `run_sim_bag_benchmark.py`; `--no-upload`; `IFSSIM_AUTO_UPLOAD`; `tracking.env` as the switch that turns uploads on; the "every benchmark uploads when it finishes" parts of DEPLOY.md and HOW_IT_WORKS.md; `admin.sh add-user` for everyone (it stays for the rare local uploader) |
| **Simplified** | `sync`'s "still running" detection stays for local runs, but the worker never uses it |
| **Kept** | `provenance.json` with diffs, unpushed bundles and image identity; `bench.run_key` and the duplicate check; `bench-track sync` and `purge`; the central compose stack, Postgres, backups and Tailscale serve; bench-view unchanged apart from the new tab |

---

## 11. The web page

Two top-level tabs. Today's navigation (Bag benchmarks, Live runs, Simulator bag benchmarks…)
moves under *Results*.

**Launch**
- **New run:**
  1. preset or "rerun" as the starting point;
  2. benchmarks (checkboxes);
  3. inputs per benchmark (bag picker from the machine's inventory, mission, repeats);
  4. code (branch/PR/commit fields with the resolved SHA shown);
  5. baseline;
  6. YAML upload or edit box;
  7. the merged spec with form overrides marked, and the number of jobs it makes;
  8. *Launch*.
- **Queue:** running job with a live log tail, queued jobs in order (cancel), recent finished
  jobs with links to their results.
- **Job page:** spec, code, image, log, runs produced, comparison link.

**Results**
- Today's bench-view.
- Each run shows its spec (`spec.yaml`, `params/`) and trigger.
- The run browser groups reruns by `(code_id, spec_id)` and can filter by trigger and PR.

Launch uses Flask routes on the Dash server (`/api/jobs`, `/api/bags`, `/api/refs`). The same
routes are the CLI's (`bench-run --remote`) way to queue jobs without the page.

---

## 12. Order of work

1. **Removals** (§10) and doc updates. One commit.
2. **Spec** *(done)*: the manifest (`bench.yaml`), the merge/validate module, `spec_id`, the
   overrides in the perception, SLAM and simulator-bag runners (§4), `params/` dumps, `bench-run`
   for local runs, `spec_id` in the tracker and the viewer. Bag replay follows when
   `run_onboard_replay.py` (feat/516) is on this branch.
3. **Pipeline change**: `cone_detection_node` accepts config overrides (pipeline repo, its own PR).
4. **Queue + worker** *(done)*: `bench` database, `bench-worker` (code, image, run, upload),
   systemd unit, DEPLOY.md. The queue is SQLAlchemy over Postgres on the central machine and
   SQLite elsewhere, so the same page and worker run on a laptop as the fallback. The runners
   now mount bags and results that live outside the checkout under test (`common.DockerPaths`).
5. **Launch tab** *(done)*: form, merged-spec preview, queue, job page; `Tailscale-User-Login`.
   The site has a Results | Launch switch; Launch lives under `/launch` in the same app.
6. **GitHub**: poller, `specs/pr.yaml` and `specs/commit-smoke.yaml`, statuses and the PR comment,
   merge-base baselines.

1 and 2 are useful on their own. 4 depends on 2. 5 and 6 each depend on 4.

---

## 13. Open questions

1. **Bags on the central machine.** The bag picker needs an inventory: which folder, who copies
   bags there, and whether `tools/pull-bag.sh` is the way. This is also the undecided "bags"
   question from the tracking design.
2. **The PR spec's contents.** Which bags and benchmarks make a PR check useful in, say, under
   30 minutes? Which make the commit smoke check useful in under 5?
3. **Simulator benchmarks.** Only the mock exists. The spec reserves `benchmarks.sim_e2e`, but its
   settings are defined once a real runner exists.
4. **Disk.** Results folders on the central machine grow with every job. Keep them N days after
   upload, since MLflow has the bundle? The reports and logs are already uploaded.
5. **Who can launch.** Is "anyone on the tailnet" right, or should launching need a tailnet group?
6. **Blocking merges.** Should `bench/pr` become a required status on `dev`, or stay advisory until
   the tolerances are trusted?
