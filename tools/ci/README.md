# tools/ci: the sim check

The scripts behind `.github/workflows/sim-pr.yml`. The workflow only checks the
code out and calls them, so the same commands run the check on your own Linux
machine with an NVIDIA GPU, or on the next box that takes over from `gpu-1080`.

| Script | Does |
|---|---|
| `sim_build.sh` | Packages the sim (Linux, Development) when its build inputs changed since the shared `dev` build, else reuses that build. |
| `sim_pr_check.sh` | The check: a headless start-up check, the car characterisation, the missions in `sim_check.json` through this checkout's pipeline, then `sim_report.py` and the track maps (`sim_plot.py`). |
| `sim_report.py`, `sim_plot.py`, `sim_failures.py` | The report, compared with the last `dev` run's, and a map per mission. |
| `sim_publish.py` | Posts the report on the PR and in the `sim-report` issue (CI only). |
| `gen_tracks.py`, `track_geometry.py` | The random autocross tracks. |

## Where things live: `CI_HOME`

Everything that outlives one run is under one folder, `CI_HOME`
(default `/build/ifssim-ci`, as on `gpu-1080`):

```
$CI_HOME/
  builds/dev -> dev-<commit>   the shared dev build (sim_build.sh, PUBLISH=true)
  baseline/report.json         the last dev run's report, what PRs are compared with
  ci-sim.owner                 marks a sim this check started (see sim_pr_check.sh)
```

The engine (`UE5_ROOT`) and the shader cache can be anywhere.

## Running the check on your machine

You need Unreal Engine 5.8 built from source (`UE5_ROOT`), Docker, Git LFS
content in the checkout, and a few GB free. Use a checkout of the commit you
want to check, with its submodules:

```bash
export CI_HOME=$HOME/ifssim-ci
export UE5_ROOT=$HOME/UnrealEngine

# 1. The sim: packages it once, then reuses it while Source/, Plugins/, Config/
#    and Content/ (tracks aside) don't change.
STAGE=$PWD/Saved/StagedBuildsCI PUBLISH=true tools/ci/sim_build.sh

# 2. The images the check uses: the pipeline stack and Mission Control's backend.
docker build -f docker/dv_pipeline_stack/Dockerfile -t ifssim-dv_pipeline_stack:local .
COMPOSE_PROJECT_NAME=ifssim-ci docker compose build mission_control_backend

# 3. The check. Without a baseline the report has no "vs dev" column.
IFSSIM_BUILD=$CI_HOME/builds/dev/IFSSIM.sh \
DV_IMAGE=ifssim-dv_pipeline_stack:local IFSSIM_DV_IMAGE=ifssim-dv_pipeline_stack:local \
RESULTS=$CI_HOME/runs/$(date +%Y%m%d_%H%M%S) \
  tools/ci/sim_pr_check.sh
```

The report is `$RESULTS/report/summary.md` (and `report.json`, `maps/`). A
failing autocross track can be driven again with `RANDOM_SEEDS=<seed>`; the
report lists each track's seed. `REPEATS=<n>` changes acceleration's runs.

Without a container wrapper (`/usr/local/bin/ifssim-sim`, which `gpu-1080` uses
to share its GPU with a render farm), the check starts `IFSSIM_BUILD` itself.
Set `IFSSIM_SIM` to use another launcher.

A full check takes about 11 minutes; a first `sim_build.sh` about 25 (the
engine compiles its shaders once, then keeps them in its cache).

### Docker installed as a snap

Ubuntu's snap Docker gives containers the GPU only with `--runtime=nvidia`
(`--gpus all` fails), and has a `/tmp` of its own: a host folder under `/tmp`
is mounted as an empty one. The benchmark keeps its temporary copies under the
results folder for that reason, so put `RESULTS` under your home, not `/tmp`.

## Setting up a box for the runner

`setup_bench_host.sh` turns a clean Ubuntu 24.04 LTS into a GPU runner: the
NVIDIA driver (held), Docker CE with the NVIDIA Container Toolkit (not the snap),
Vulkan, `gh`, Tailscale, a `runner` user without sudo, `CI_HOME`, and the Actions
runner registered to the org with only its labels (`self-hosted,Linux,X64,gpu`).
Each step can run on its own and again; `--dry-run` prints what it would do.

```bash
sudo tools/ci/setup_bench_host.sh --dry-run
sudo tools/ci/setup_bench_host.sh                       # everything but the registration
sudo RUNNER_TOKEN=<token> tools/ci/setup_bench_host.sh runner
```

Unreal Engine is built by hand (it needs an Epic-linked GitHub account) into
`$CI_HOME/UE_5.8`, the `UE5_ROOT` the runner's `.env` names. The bench PC's rig
descriptor is `rigs/bench-pc.yaml`.
