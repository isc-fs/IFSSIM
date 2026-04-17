# Getting Started — Docker Pipeline

This guide covers everything needed to get the IFSSIM Docker stack running alongside UE5.

---

## Prerequisites

| Requirement | Version |
|---|---|
| Docker Desktop | 4.x or later |
| Docker Compose | v2 (bundled with Desktop) |
| Unreal Engine | 5.4+ |
| macOS | Sonoma or later (Apple Silicon or Intel) |

---

## 1. Clone the repository

```bash
git clone git@github.com:isc-fs/IFSSIM.git
cd IFSSIM
```

---

## 2. Build the containers

From the repository root:

```bash
docker compose build
```

This builds three services:
- **`mission_control_backend`** — FastAPI server (port 8000), handles sim commands, scoring, track management
- **`mission_control_frontend`** — React dashboard served via Nginx (port 3000)
- **`ros_stack`** — ROS 2 Humble pipeline with sensors, SLAM, control nodes (ports 8765, 8888)

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

Open the project in the Epic Games Launcher or directly:

```bash
open IFSSIM.uproject
```

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

Backend or frontend Python/TypeScript changes require a rebuild before they take effect in the containers:

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

C++ plugin changes (files under `Plugins/`) take effect automatically the next time UE5 launches — no Docker rebuild needed.

---

## Port reference

| Port | Service | Description |
|---|---|---|
| 3000 | Frontend | Mission Control dashboard |
| 8000 | Backend | REST API + WebSocket telemetry |
| 8765 | ROS stack | Foxglove WebSocket bridge |
| 8888 | ROS stack | rosboard web viewer |
| 41451 | UE5 (host) | RPC server (TCP) |
| 41452–41453 | ROS stack | UDP sensor bridge |

---

## Troubleshooting

**Mission Control shows Disconnected after pressing Play**
Make sure UE5 finished compiling the plugin. Check the UE5 output log for `FSDS RPC: Server starting on port 41451`.

**Containers fail to reach UE5**
On macOS, the backend reaches UE5 via `host.docker.internal`. Verify Docker Desktop has "Allow the default Docker socket to be used" enabled in Settings → Advanced.

**Track loads but no cones appear**
The RPC `loadTrack` response will contain an error field. Check the Mission Control session log tab for details.

**Frontend shows stale data after a backend rebuild**
Hard-refresh the browser (`Cmd+Shift+R`) to clear the cached assets.
