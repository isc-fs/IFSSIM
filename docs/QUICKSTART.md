# Quickstart

You have 15 minutes. Goal: a Formula Student car driving on a track,
with telemetry visible in your browser and a LiDAR cloud you can
inspect. **No autonomy stack required for this guide** — the sim runs
standalone for manual driving and as the test bench for someone
else's autonomy code.

If you want to run the autonomy stack alongside, finish this
quickstart first, then continue with [`docs/autonomy_pipeline.md`](autonomy_pipeline.md).

## Prerequisites

| Thing | Why | How |
|---|---|---|
| **macOS 14+** | The current release ships a Mac binary. Linux/Windows scripts exist but the pre-built artefact is Mac-only for now. | — |
| **Docker Desktop** | The ROS 2 bridge + Mission Control backend run in a container. | <https://www.docker.com/products/docker-desktop> — give it ≥ 4 GB RAM. |
| **`git` and `git lfs`** | This repo uses LFS for binary assets (cone meshes, vehicle textures). Without LFS the cooked sim won't have proper materials. | `brew install git git-lfs && git lfs install` |
| *(optional)* **Lichtblick desktop** or **Foxglove Studio desktop** | Native pointcloud / 3D visualisation. Browser equivalents work but burn 3–5× more CPU on a 10 Hz LiDAR stream. | <https://github.com/lichtblick-suite/lichtblick/releases> |

## 1. Get the sim

Until self-hosted runners produce signed Release artefacts, build
from source:

```bash
git clone https://github.com/isc-fs/IFSSIM.git
cd IFSSIM
git lfs pull   # ~500 MB of binary assets
```

You'll need **UE5 5.7** installed to cook the sim. Default Mac
location: `/Users/Shared/Epic Games/UE_5.7/`. If you have it
elsewhere, export `UE5_ROOT=…` before the next step.

```bash
./package_mac.sh
```

This takes ~2 minutes on a recent Mac (M-series). Output lands at
`Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app`. The script also
stages 10 example tracks as a sibling `tracks/` directory and
patches the binary's `UECommandLine.txt` for windowed mode + 60 FPS
cap.

## 2. Bring up the bridge stack

The bridge is a containerised ROS 2 node that translates between the
sim's RPC + UDP wire layer and standard ROS 2 topics. Mission Control
is the orchestrator UI on top.

```bash
./tools/refresh-bridge.sh
```

This stops any existing bridge container, recreates it from the
current source, and waits for the healthcheck. Takes ~30 seconds.
Use it any time you change bridge code, the launch file, or
container plumbing — `docker compose restart` alone isn't enough
(macOS Docker Desktop accumulates wedged UDP-proxy state).

Sanity check the bridge is up:

```bash
docker compose ps dv_pipeline_stack
# STATUS should say "(healthy)"
```

## 3. Launch the sim

```bash
open Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app
```

A windowed UE5 client opens at the default map. The sim's RPC server
binds to TCP port 41451 (commands) and the UDP broadcaster to
41452 (sensors) + 41453 (LiDAR). The bridge container connects to
those ports through Docker Desktop's loopback.

Within a few seconds you should see in the bridge logs:

```bash
docker compose logs -f dv_pipeline_stack | grep ifssim_bridge
# [INFO] [ifssim_bridge]: IFSSIM connected (TCP push model)
# [INFO] [ifssim_bridge]: LiDAR transport: UDP (listening on 51453)
```

That's the green light.

## 4. Open Mission Control

Mission Control is the single UI you use to drive the FS-DV
lifecycle: load a track, set the event type (Trackdrive / Skidpad /
Acceleration / Autocross), trigger Start, watch the car. It's a
small React app talking to a FastAPI backend over a WebSocket.

```bash
cd tools/mission_control
./run.sh
```

The UI launches at `http://localhost:3000`. The backend's at
`http://localhost:8000`.

## 5. Drive

In the Mission Control UI:

1. **Track Manager** tab → pick a track (e.g. **`skidpad.csv`**) →
   click **Load**. The cones spawn in the sim.
2. **Event** tab → pick an event type matching the track (e.g.
   **Skidpad** → 4 laps). Click **Start Session**.
3. The sim activates the emergency stop (RES) for ~4.5 s while
   anything in the autonomy lifecycle warms up, then releases EBS
   and hands control over.
4. Without an autonomy node connected, the car will sit still —
   that's expected. To **drive manually**, focus the sim window
   and use **WASD** + **Space** for handbrake.
5. Watch telemetry update live in the **Dashboard** tab.

To bring an autonomy stack online, see
[`autonomy_pipeline.md`](autonomy_pipeline.md). The bridge publishes
all the standard topics
(`/imu`, `/gps`, `/gss`, `/lidar/Lidar1`, `/testing_only/odom`, …)
that an FS-DV pipeline expects.

## 6. (Optional) Visualise sensors

A native Lichtblick / Foxglove Studio desktop app gives the smoothest
experience. Connect to:

```
ws://localhost:8765
```

That's the foxglove_bridge WebSocket inside the dv_pipeline_stack
container. Subscribe to:

- **`/lidar/Lidar1`** — full-density 95 k-point cloud, 10 Hz
- **`/lidar/Lidar1/viz`** — opt-in 1/Nth subsampled cloud (off by
  default; enable with `LIDAR_VIZ_DECIMATION=4 ./tools/refresh-bridge.sh`
  to drop browser CPU)
- **`/imu`** — 400 Hz IMU
- **`/testing_only/odom`** — ground-truth pose
- **`/camera/cam{1,2}/compressed`** — front-left / front-right
  cameras

If the browser tab pegs at 35 % CPU, see the "Native viewer or
subsampled topic" note in [`FUNCTIONALITIES.md`](FUNCTIONALITIES.md)
§7.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `IFSSIM Bridge: Connection failed to host.docker.internal:41451` repeating | Sim isn't running, or Docker Desktop's loopback is wedged | Make sure the `.app` is running. If it is, `./tools/refresh-bridge.sh` to recreate the bridge container — fixes wedged UDP proxies. |
| `/lidar/Lidar1` exists but `ros2 topic hz` shows zero | No track loaded, so the pawn isn't ticking sensors | Load a track via Mission Control. |
| `package_mac.sh` errors on `BuildCookRun` with "broken references" | A known broken Blueprint (`spline_cones_mini_orange`) | The repo's `DefaultGame.ini` already scopes the cook around it; if the error persists, ensure you're on the latest `dev` and re-`git lfs pull`. |
| Mission Control's "Stop Session" doesn't react | Backend is in the middle of the 4.5 s SLAM warm-up window | Wait for it. Phase 3 of `event_start` rejects mid-flight resets cleanly; you'll see the abort in the session log. |
| Foxglove tab CPU is pegged | Browser-based viewer deserialising 15 MB/s of pointcloud | Switch to native Lichtblick / Foxglove Studio desktop, or set `LIDAR_VIZ_DECIMATION=4` and subscribe to `/viz`. |

## Where to go next

- **[`FUNCTIONALITIES.md`](FUNCTIONALITIES.md)** — every sensor, RPC
  method, ROS topic, configuration knob, and Mission Control
  endpoint, in one place.
- **[`autonomy_pipeline.md`](autonomy_pipeline.md)** — adding the
  autonomy stack (perception → SLAM → planning → control). The same
  code that runs on the real car runs against this sim.
- **[`GETTING_STARTED_DOCKER.md`](GETTING_STARTED_DOCKER.md)** —
  deeper into the Docker pipeline + the optional autonomy
  containers.
- **`README.md` "Hacking on IFSSIM"** — the contributor flow:
  branch model, CI gate, code conventions.
