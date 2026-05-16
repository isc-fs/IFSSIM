# Operating

Daily ops, once you've got through [SETUP.md](SETUP.md). The
recurring tasks: editing source, recording bags, switching tracks,
diagnosing common failures during a session.

---

## Source-edit cycle

The Docker stack used to bind-mount the source tree so a Python edit
went live with a `docker compose restart`. **As of v0.1.1 (#490) it
doesn't anymore** — source is COPY'd into the image at build time.
This saves 30-90 s on container startup (no per-import 9p traversal
on Windows/macOS) and cuts the docker build context from 33 GB to
~200 MB.

There are two ways to pick up a source edit:

**Live (Python-only, fastest path):**

```bash
tools/compose-up-and-watch.sh dv_pipeline_stack
```

Runs `docker compose up -d` then `docker compose watch` in the
foreground. `compose watch` syncs your edits under `pipeline/*` into
named volumes mounted at `/dv_pipeline_stack_ws/src/<pkg>` inside the
container; the image's `colcon build --symlink-install` already
points the install tree at those sources, so the next time a
lifecycle node restarts (e.g. the next `SetMission` call), it
imports your edited code. Ctrl+C stops the watch but leaves the
containers running.

**Full rebuild (C++, msgs/srvs/actions, launch, entrypoint, FastDDS profile):**

```bash
tools/refresh-bridge.sh
```

Does `docker compose build dv_pipeline_stack` (BuildKit cache makes
incremental builds hit in seconds) then
`docker compose up -d --force-recreate dv_pipeline_stack` (drops any
wedged DDS / UDP-proxy state). Run it any time you edit:

- `pipeline/*/src/*.cpp` or anything under `pipeline/dv_msgs/{srv,msg,action}/`
- `ros2/src/*` (fs_msgs, ifssim_bridge)
- Any launch file under `docker/dv_pipeline_stack/` or `pipeline/bringup/launch/`
- A package's `setup.py` (new entry-points or packages)
- `entrypoint.sh`
- `fastdds_profile.xml`

The named src volumes are seeded from the image on **first creation only**.
If `setup.py` changes don't show up after `tools/refresh-bridge.sh`, it's
because the volumes survived the recreate — drop them with
`docker volume rm ifssim_dv_pipeline_<pkg>_src` and start again.

### Mission Control edits

For backend or frontend changes:

```bash
docker compose build mission_control_backend mission_control_frontend
docker compose up -d --force-recreate mission_control_backend mission_control_frontend
```

### UE 5 plugin edits

The plugin under `Plugins/FSDSPlugin/` is **inside the sim binary** —
no Docker involvement. Re-cook the sim:

```bash
# Windows:
bash package_windows.sh

# macOS:
./package_mac.sh
```

Then relaunch. C++ changes need the recook; pure-content edits
(materials, blueprints, maps) can be tested in the editor first
with `Play in Editor` and only re-cooked once verified.

---

## Recording and retrieving MCAP bags

The bag flow ships as of v0.1.1 (#486 + #490). Recording is a
checkbox in Mission Control; retrieval is one CLI command.

### Recording

In **Mission Control → Event** tab, tick **Record bag (mcap)** before
clicking **Start Session**. Recording starts when the autonomy
finishes its bring-up window (so the recording covers the actual
session, not the warm-up phase). Recording stops automatically when
you click **Stop Session** or when the run ends naturally.

Live state surfaces in the session log:

```
record_bag: started <bag_name> → /bags/<bag_name>/
record_bag: stopped <bag_name>
```

And in the API:

```bash
curl -s http://localhost:8000/api/referee/state | jq .bag_state
# → "recording" during the session, "stopped" or "none" otherwise
```

The recorder runs **inside the dv_pipeline_stack container** so it
shares the SHM-tuned DDS context with the publishers. Recording from
the mc_backend container instead drops ~96% of `/lidar/Lidar1` scans
because mc_backend forces UDPv4 transport (so its action client works
cross-container) and Fast DDS's UDP fragmentation reassembly drops
multi-fragment messages. The v2 architecture (#465) moved the
recorder where the data is.

### Retrieving — default: auto-pull on stop

Since #498 (v0.1.2+), when you click **Stop Session** the finalised
bag is **automatically copied to your host filesystem** and removed
from the docker volume. You'll see this in the session log:

```
record_bag: stopped <name> → /bags/<name>/                ← inside container
record_bag: auto-pulled <name> → /home/.../IFSSIM/bags/<name>
```

The bag lands at `./bags/<name>/` on the host. The volume gets
cleaned in the same step, so it never accumulates.

`docker cp` does the move via Docker Desktop's vmcompute stdio
pipe — fast on Windows / macOS (~1-2 s for a multi-GB bag, vs the
20-40 s a bind-mount `shutil.move` would take). The host's
`./bags/` is gitignored.

To disable auto-pull (e.g. you want bags to stay in the volume for
some reason), set `IFSSIM_BAG_AUTO_PULL=0` in your shell before
`docker compose up`. Then bags stay in the docker volume and you
retrieve them manually:

```bash
tools/list-bags.sh                # see what's in the volume
tools/pull-bag.sh <bag_name>      # docker cp it to ./bags/<bag_name>/
```

These two CLI helpers also serve as the **manual recovery** path
if auto-pull failed for any reason (host disk full, docker socket
unreachable, etc.). The session log surfaces the failure and tells
you which command to run.

### Why the indirection (host bind-mount → named volume + auto-pull)

Pre-v0.1.1 the recorder wrote directly to a host-bind-mounted
`./bags/`, which meant the final `shutil.move` from the in-container
`/tmp` staging directory to `/bags/` was a cross-filesystem copy
across virtiofs (macOS) or 9p (Windows + WSL2). For a multi-GB bag
that took 20-40 s and blocked the StopBag service callback, visibly
stalling the session-stop click in Mission Control.

The current flow (named docker volume + `docker cp` auto-pull) keeps
the move on the container's local ext4 (finalisation < 1 s), then
uses Docker Desktop's vmcompute stdio pipe for the host-side copy
(~1-2 s vs 20-40 s). All inside the same StopBag callback, no user
action required.

### Playing back a bag

Once pulled to `./bags/<name>/`, play it back to drive an offline
analysis pipeline:

```bash
# Through the live bridge container (so any autonomy nodes you have
# active will consume it)
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && cd /workspace && \
   ros2 bag play /bags/<bag_name>'
```

Or use `pipeline/cone_slam/scripts/replay_slam.py` for a deterministic
SLAM-only replay (skips the autonomy lifecycle bring-up).

For visualisation: open Lichtblick at <http://localhost:8080> and
connect to the bag via the WS bridge — same UI as live.

---

## Tracks and missions

### Loading a track

**Mission Control → Track Manager** tab:

- **Standard tracks** — `acceleration.csv` and `skidpad.csv` ship in
  `Content/tracks/`. Click **Load**; the bridge's `loadTrack` RPC
  spawns the cones in UE5 and sets the appropriate event type.
- **Generated tracks** — the **Generate** panel runs the
  `random-track-generator` submodule with a chosen length/curvature
  profile and writes the CSV next to the standard ones. Generated
  tracks land at `random_track.csv` by default; rename before commit
  if you want to keep one.

### Picking a mission

Once a track is loaded, **Event** tab lets you choose:

- **Acceleration** — 75 m straight line. Stop condition: cross the
  finish gate (single big-orange pair).
- **Skidpad** — figure-8 with 4 laps. Stop condition: lap count.
- **Autocross** — single lap of any track. Stop condition: complete a
  lap.
- **Trackdrive** — multiple laps of an autocross-shaped track. Stop
  condition: lap count.

Match the track to the mission — Skidpad on an acceleration track
won't end cleanly (no figure-8 to count laps on).

### Driving manually

If no autonomy node is attached, the car sits still after Start
Session. To drive manually, focus the sim window and use:

- **W / S** — throttle / brake
- **A / D** — steer
- **Space** — handbrake

Manual control is only available when the autonomy isn't taking
priority (i.e. before Start Session, or in `Stopped` state).

### Manual mission control without Mission Control

If you want to reproduce the production lifecycle from a shell —
useful while developing a new mission/behavior or triaging a
session-start bug — `supervisor_cli` is an `rclpy` action client
that mirrors the web backend's two-phase flow:

```bash
docker compose exec dv_pipeline_stack bash -lc '. /opt/ros/humble/setup.bash && \
  . /dv_pipeline_stack_ws/install/setup.bash && \
  ros2 run sim_supervisor supervisor_cli <cmd>'
```

Commands:

- `set_mission <id>` — Phase 1 (`SetMission`) prepare/configure. IDs:
  `1=trackdrive`, `2=autocross`, `3=accel`, `4=skidpad`, `5=scruti`,
  `0=tear down`.
- `start_mission [--wait]` — Phase 2 (`RuntimeControl`) activate +
  run. Blocks until terminated when `--wait` is given.
- `run [--wait] <id>` — `set_mission` followed by `start_mission`
  in one shot.

The web backend uses the same actions on the same node; either path
is equivalent.

---

## Diagnosing common failures during a session

### "The car drove a few seconds then crashed into a cone wall"

Autonomy cascade. Most likely cone_slam's data association lost
track of where it was. Confirm:

```bash
docker compose logs dv_pipeline_stack | grep -iE "cascade|skip cone"
```

If you see "skip cone factors: DA-failure spike" — that's the
documented cascade. The current default mitigations (proximity veto,
spike detector, sanity check) push cascade onset to ~63 s on most
tracks but don't close it. The root-cause work is tracked at
[#447](https://github.com/isc-fs/IFSSIM/issues/447) and
[#485](https://github.com/isc-fs/IFSSIM/issues/485).

### Mission Control session log stops updating mid-session

WebSocket dropped. Refresh the browser tab; the log will resume
streaming live state from `/api/referee/state` and you won't lose
the session itself (the autonomy keeps running).

### `Stop Session` doesn't react immediately

Stop Session cancels the active `RuntimeControl` goal on
`mission_control_node` and then issues `SetMission(mission_id=0)` to
tear down. The cancel honours at the next feedback tick (~25 ms);
the tear-down then walks each `BaseLifecycleNode` through `deactivate`
→ `cleanup` in reverse registry order. If you click Stop during the
10-20 s cone_detection JIT warm-up (which lands inside `SetMission`,
not `RuntimeControl`), the cancel preempts the in-flight configure
and the stack returns to unconfigured. Patience.

### Foxglove / Lichtblick tab pegs the CPU

Browser-based 3D viz on a 15 MB/s LiDAR stream eats 30-40% of a
core. Two options:

1. **Use the native Lichtblick / Foxglove Studio desktop app** —
   3-5× cheaper than the browser version. Connect to
   `ws://localhost:8765`.
2. **Subscribe to the subsampled `/lidar/Lidar1/viz` topic instead
   of `/lidar/Lidar1`** — set `LIDAR_VIZ_DECIMATION=4` in
   `docker-compose.yml`'s `dv_pipeline_stack.environment` and
   `tools/refresh-bridge.sh`. The autonomy stack always sees the
   full cloud.

### Container won't come back after `docker compose down`

If `docker compose down -v` was used (note the `-v`), it dropped the
named volumes — including `ifssim_ifssim_bags`. Bags from previous
sessions are gone. Don't use `-v` unless you want that.

Without `-v`, a normal `down` → `up -d` cycle keeps everything;
recover with:

```bash
docker compose up -d
```

---

## Quick reference

### Common one-liners

```bash
# Tail the bridge log (most useful single command for "is it working?")
docker compose logs -f dv_pipeline_stack

# List ROS topics
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && ros2 topic list'

# See a topic's rate
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && ros2 topic hz /imu --window 200'

# See the live lifecycle state of an autonomy node
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && ros2 lifecycle get /slam_node'

# Drive the two-phase mission lifecycle from a shell, no web UI:
#   1=trackdrive, 2=autocross, 3=accel, 4=skidpad, 5=scruti, 0=tear down
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && . /dv_pipeline_stack_ws/install/setup.bash && \
   ros2 run sim_supervisor supervisor_cli run --wait 1'

# Force-restart everything (sim stays up)
docker compose down && docker compose up -d
```

### Port reference

| Port | Service | Notes |
|---|---|---|
| 3000 | Mission Control frontend | Web UI |
| 8000 | Mission Control backend | REST + WebSocket |
| 8080 | Lichtblick | Web 3D viz |
| 8765 | foxglove_bridge | WS for native Foxglove / Lichtblick desktop |
| 41451 | UE5 (host) | TCP — RPC + sensor stream + LiDAR stream (since v0.1.0) |
| 41452 | UE5 (host) | UDP sensor stream (legacy, off by default) |
| 41453 | UE5 (host) | UDP LiDAR stream (legacy, off by default) |

### Where things live on the host

| What | Windows | macOS |
|---|---|---|
| Sim binary | `Saved\StagedBuilds\Windows\IFSSIM.exe` | `Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app` |
| Sim user data | `%LOCALAPPDATA%\IFSSIM\` | `~/Library/Application Support/Epic/IFSSIM/` |
| Tracks | `Content/tracks/*.csv` | same |
| Bags (on disk) | `./bags/<name>/` (auto-pulled on stop, or `pull-bag.sh` if disabled) | same |
| Bags (transient) | `ifssim_ifssim_bags` docker volume — empty in steady state when auto-pull is on | same |

### Environment knobs

A few that get asked about; full list in `docker-compose.yml`.

| Var | Default | Effect |
|---|---|---|
| `LIDAR_VIZ_DECIMATION` | `4` | Subsampled `/lidar/Lidar1/viz` for browser viz. 0 disables; ≥2 publishes every Nth point. |
| `IFSSIM_MC_API_KEY` | empty | When set, MC backend requires `X-API-Key` header on mutating endpoints. Empty = dev mode, open. |
| `IFSSIM_BAG_AUTO_PULL` | `1` | Auto-pull finalised bag onto host filesystem on StopBag (`docker cp` from the volume to `./bags/<name>/`, then `rm` the volume copy). Set to `0` to disable; bag stays in the volume and you retrieve manually via `tools/pull-bag.sh`. |
| `OPENBLAS_NUM_THREADS` / `MKL_NUM_THREADS` / `OMP_NUM_THREADS` | `2` | BLAS thread caps. Capped because cone_detection's per-call RANSAC spawned 8-10 threads per call and saturated CPU. |
| `DV_PLANNER_CAPTURE` | empty | Path-planning JSONL dump path. For offline replay against failing scenes. |
| `DV_SLAM_LANDMARK_CAPTURE` | empty | cone_slam landmark-creation JSONL dump. For DA cascade triage. |

---

## What's next

- **[AUTONOMY.md](AUTONOMY.md)** — autonomy integration contract.
- **[REFERENCE.md](REFERENCE.md)** — full technical reference.
- **[CONTRIBUTING.md](CONTRIBUTING.md)** — if you want to send a PR.
