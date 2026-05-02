# Getting Started — Docker Pipeline

This guide covers everything needed to get the IFSSIM Docker stack running alongside UE5.

---

## Prerequisites

| Requirement | Version |
|---|---|
| Git | 2.x or later |
| Git LFS | 3.x or later |
| Docker Desktop | 4.x or later (Windows / macOS) or Docker Engine + Compose plugin (Linux) |
| Docker Compose | v2 (bundled with Desktop, or `docker-compose-plugin` on Linux) |
| Unreal Engine | 5.4+ |

> **Linux users**: install Docker Engine via your distro's package manager and add your user to the `docker` group. Docker Desktop is optional.

---

## 1. Clone the repository

This repository uses **Git LFS** for large binary assets (maps, meshes). Make sure LFS is installed before cloning:

```bash
# Install Git LFS (once per machine)
git lfs install

# Clone
git clone git@github.com:isc-fs/IFSSIM.git
cd IFSSIM
```

If you already cloned without LFS, run `git lfs pull` inside the repo to fetch the missing assets.

---

## 2. Build the containers

From the repository root:

```bash
docker compose build
```

This builds the custom services (Lichtblick is pulled from the registry, not built locally):
- **`mission_control_backend`** — FastAPI server (port 8000), handles sim commands, scoring, track management
- **`mission_control_frontend`** — React dashboard served via Nginx (port 3000)
- **`dv_pipeline_stack`** — ROS 2 Humble pipeline with sensors, SLAM, control nodes (port 8765)
- **`lichtblick`** — Web-based ROS 2 visualizer (port 8080), connects to the Foxglove bridge

> First build takes several minutes. Subsequent builds are cached.

---

## 3. Start the containers

```bash
docker compose up -d
```

Verify all three are running:

```bash
docker compose ps
```

All services should show `Up`.

---

## 4. Launch UE5

Open the project via the Epic Games Launcher, or double-click `IFSSIM.uproject` in your file manager.

Wait for the editor to finish loading and the plugin to compile, then press **Play**.

> The RPC server (port 41451) only starts when the simulation is in Play mode. Mission Control will show **Disconnected** until then.

---

## 5. Open Mission Control

Navigate to **http://localhost:3000** in your browser.

Once UE5 is in Play mode, the dashboard connects automatically and shows:
- Live telemetry (speed, position, throttle, steering, brake)
- Event state (laps, DOO, OC, finished flag)
- RES status

---

## 6. Load a track

Go to the **Track** tab in Mission Control:

- **Standard Tracks** — `acceleration.csv` and `skidpad.csv` ship with the sim. Clicking **Load** spawns the cones in UE5 and sets the event type automatically.
- **Generated Tracks** — use the generator panel to create a random track, then load it.

> When UE5 launches with the custom map, it starts with an empty track. Load a track from Mission Control before starting an event.

---

## 7. Run an event

1. Load a track (step 6)
2. Go to the **Event** tab
3. Select the event type (acceleration, skidpad, autocross, trackdrive)
4. Click **Configure Event**, then **Start Session**
5. The car will have API control enabled — drive it with your autonomous pipeline or via the ROS stack

---

## 8. RES (Remote Emergency Stop)

The **Stop Session** button in the Event tab activates the RES immediately, cutting throttle and applying full brake. **Release RES** re-enables API control.

---

## 9. Stopping

```bash
docker compose down
```

UE5 can be closed normally from the editor. Pressing **Stop** (ending Play mode) is safe and does not crash the sim.

---

## Rebuilding after code changes

### Pipeline Python edits (slam, cone_slam, path_planning, control)

**No rebuild needed.** `pipeline/` and `ros2/src/` are bind-mounted into `dv_pipeline_stack` over the image's baseline, and `colcon build --symlink-install` (run at image build time) made the install tree's Python entries symlinks back to the source. So host edits are live the moment you toggle the pipeline:

```bash
# Edit pipeline/path_planning/path_planning/planner.py on the host, then:
curl -X POST http://localhost:8000/api/event/start  # restarts the pipeline
# new code is in effect — no docker cp, no rebuild
```

If the pipeline is already running, the simplest way to pick up an edit is to flip the pipeline-control flag:

```bash
docker compose exec dv_pipeline_stack bash -c 'rm /pipeline_ctrl/enable; sleep 2; touch /pipeline_ctrl/enable'
```

### Pipeline changes that *do* need a rebuild

- **C++ source** (`ros2/src/ifssim_bridge/`) — recompile.
- **`.msg` files** (`ros2/src/fs_msgs/msg/`) — regenerate bindings.
- **`setup.py` changes** in any pipeline package — re-link entry points.
- Adding a new package.

Two ways to rebuild:

```bash
# 1. Rebuild on the next container restart (clean — uses the regular entrypoint)
DV_REBUILD_ON_STARTUP=true docker compose up -d --force-recreate dv_pipeline_stack

# 2. Build inside the running container without restart (faster for iteration)
docker compose exec dv_pipeline_stack bash -c \
  "cd /dv_pipeline_stack_ws && colcon build --symlink-install --packages-select <pkg>"
# then restart the pipeline (toggle /pipeline_ctrl/enable as above)
```

### Mission Control rebuild

Backend or frontend Python/TypeScript changes still require an image rebuild (no bind-mount on those services yet):

```bash
# Backend only
docker compose build mission_control_backend
docker compose up -d --no-deps mission_control_backend

# Frontend only
docker compose build mission_control_frontend
docker compose up -d --no-deps mission_control_frontend

# All
docker compose build && docker compose up -d
```

### UE5 plugin

C++ plugin changes (files under `Plugins/`) take effect automatically the next time UE5 launches — no Docker rebuild needed.

---

## Port reference

| Port | Service | Description |
|---|---|---|
| 3000 | Frontend | Mission Control dashboard |
| 8000 | Backend | REST API + WebSocket telemetry |
| 8080 | Lichtblick | Web-based ROS 2 visualizer |
| 8765 | ROS stack | Foxglove WebSocket bridge |
| 41451 | UE5 (host) | RPC server (TCP) |
| 41452–41453 | ROS stack | UDP sensor bridge |

---

## Troubleshooting

**Mission Control shows Disconnected after pressing Play**
Make sure UE5 finished compiling the plugin. Check the UE5 output log for `FSDS RPC: Server starting on port 41451`.

**Containers fail to reach UE5**
The backend reaches UE5 via `host.docker.internal` (resolved automatically on Windows and macOS by Docker Desktop). On Linux this alias is not set up by default — add `--add-host=host.docker.internal:host-gateway` to your run command or set it in `docker-compose.yml` under `extra_hosts`.

**Track loads but no cones appear**
The RPC `loadTrack` response will contain an error field. Check the Mission Control session log tab for details.

**Frontend shows stale data after a backend rebuild**
Hard-refresh the browser (`Ctrl+Shift+R` / `Cmd+Shift+R`) to clear the cached assets.

**Git LFS assets missing after clone**
Run `git lfs pull` inside the repository root to download all tracked binary files.
