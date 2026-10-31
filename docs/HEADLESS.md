# Headless runs

`tools/headless_run` runs the simulator with no window, for unattended
benchmark runs: one run, or runs on repeat overnight. It owns the processes;
the mission benchmark (`tools/sim_benchmark/run_mission_benchmark.py`) owns
the missions.

A run:

1. starts the sim offscreen (`-RenderOffscreen`) and waits until it answers;
2. makes sure Mission Control's backend sees it (`--mc-up` starts the backend);
3. runs the mission benchmark, which loads the tracks, starts the missions,
   follows the referee and scores each run;
4. watches the sim throughout: if it dies, the run stops and says so;
5. stops the sim and writes the record.

```bash
cd tools

# Does the sim start offscreen on this machine, and does the GPU LiDAR render?
python3 -m headless_run check --sim ../Saved/StagedBuilds/Linux/IFSSIM.sh

# One run. Everything after -- goes to run_mission_benchmark.py.
python3 -m headless_run run --sim ../Saved/StagedBuilds/Linux/IFSSIM.sh --mc-up \
    -- --missions acceleration,skidpad --repeats 2

# Continuous: a fresh sim each cycle, until Ctrl-C or --cycles N.
python3 -m headless_run loop --sim ../Saved/StagedBuilds/Linux/IFSSIM.sh --mc-up \
    -- --missions all --start-noise
```

`--sim` takes a packaged build (`IFSSIM.sh`, `IFSSIM.app`, `IFSSIM.exe`), or
the editor (`UnrealEditor`, with `--project IFSSIM.uproject`) on a dev box,
or `attach` for a sim that's already running. `--settings-dir` picks the
folder whose `settings.json` the sim reads. The benchmark's own options
(missions, repeats, start noise, pipeline overrides, `--stack`) are documented
in `run_mission_benchmark.py`.

## What it writes

In `tools/sim_benchmark/results/` (or `--results-root`):

| Path | What |
|---|---|
| `headless/<stamp>/run.json` | the run: outcome, timings, sim command, the benchmark's runs |
| `headless/<stamp>/sim.log`, `benchmark.log` | the two processes' output |
| `headless/index.jsonl` | one line per run, for a night's summary |
| `mission/...` | the benchmark's own results, as before |

| Outcome | Meaning | `run` exit code |
|---|---|---|
| `finished` | every mission finished | 0 |
| `not_finished` | some mission timed out or failed: the pipeline's result | 1 |
| `sim_failed_to_start`, `sim_crashed`, `backend_unreachable`, `benchmark_error`, `run_timeout` | the machine failed, not the pipeline | 3 |
| `interrupted` | Ctrl-C or SIGTERM; everything was stopped | 130 |

`loop` keeps going through `not_finished`, which is a result, and stops
after `--max-infra-failures` (default 3) machine failures in a row, which
don't fix themselves overnight.

## Why offscreen and not "no rendering"

The LiDAR is rendered: a scene capture draws depth and a compute shader turns
it into points. So a headless sim still needs a GPU and a renderer, with no
window. `-nullrhi` would start, and then produce no LiDAR. `check` fails
when the LiDAR never produces a frame.

The sim runs in real time at a fixed 60 Hz. Running faster than real time
isn't supported (`ClockSpeed` in `settings.json` has no effect).

## On a Linux GPU machine

- An NVIDIA GPU with the proprietary driver and Vulkan (`vulkaninfo` lists
  the GPU). No X server or display is needed for `-RenderOffscreen`.
- A Linux build: `package_linux.sh`, or the `package-linux` workflow's
  artifact.
- Docker, for Mission Control's backend and the benchmark's pipeline
  container. On Linux the backend reaches the sim on `127.0.0.1`, which
  `--mc-up` sets (`IFSSIM_HOST`).
- To run every night: a systemd timer or cron entry calling
  `python3 -m headless_run loop --cycles N ...`, which exits on its own.

## Status

- Tested on macOS (UE 5.8.4, editor build, `-game -RenderOffscreen`): ready
  in about 15 s, 60 fps, and the GPU LiDAR renders. The process handling (crash,
  no backend, a taken port, give-up in a loop) is covered by
  `headless_run/tests` with a fake sim.
- Not yet run on a Linux GPU machine, nor through a whole benchmark run.
- Not yet in a container. The plan: the Linux build in an image on the NVIDIA
  container runtime (`--gpus all`,
  `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility`), as a compose
  profile next to the backend.
