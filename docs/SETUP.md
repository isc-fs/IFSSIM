# Setup

The full first-time-user path from a clean machine to a driving sim
with the autonomy stack ready. **Windows and macOS** in parallel —
follow the column for your OS at each step. Linux is supported but
not first-class; see [Linux](#linux) at the bottom.

The end-to-end time is **20–45 minutes**, dominated by:

1. UE 5.7 install (~20 min, only first time)
2. Building the sim (~5 min cold, ~1 min incremental)
3. First Docker image build (~5–10 min cold, seconds after)

You don't need to be a Linux person, a UE5 person, or a ROS person to
get through this. You do need patience for the engine install.

---

## What you're installing

When you finish you'll have, on **one machine**:

- **UE 5.7** running the sim natively (the Windows / Mac `IFSSIM.exe` /
  `.app` binary), publishing sensor data over loopback TCP/UDP
- **Docker Desktop** hosting four containers — bridge, mission-control
  backend, mission-control frontend, Lichtblick visualiser
- **Mission Control web UI** at `http://localhost:3000` driving the
  session lifecycle (load track, pick mission, start, stop, record)

That's the whole stack. Everything else (autonomy nodes, recorder
node, foxglove bridge) lives inside `dv_pipeline_stack`.

---

## 0. Prerequisites

| Tool | Windows | macOS |
|---|---|---|
| **Git** | [git for windows](https://git-scm.com/download/win) | `xcode-select --install` or `brew install git` |
| **Git LFS** | bundled with git for windows; run `git lfs install` once | `brew install git-lfs && git lfs install` |
| **Docker Desktop ≥ 4.34** | [docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop) — enable WSL2 backend | [docker.com](https://www.docker.com/products/docker-desktop) — Apple Silicon native build |
| **UE 5.7** | [Epic Games Launcher](https://www.epicgames.com/store/en-US/download) → Library → "+" → Unreal Engine 5.7 | same — installs to `/Users/Shared/Epic Games/UE_5.7/` |
| **(Windows only) Visual Studio 2022** | [visualstudio.microsoft.com](https://visualstudio.microsoft.com/) Community is free. Install with the **"Game development with C++"** workload, plus the optional **"Unreal Engine installer"** subworkload | — |
| **(macOS only) Xcode** | — | App Store, full Xcode (not just CLT) so UBT can sign |

Docker Desktop: give it **≥ 6 GB RAM** in Settings → Resources. The
`dv_pipeline_stack` container peaks at ~2.5 GB during cone-detection
JIT compile + LiDAR streaming. Less than 4 GB and OOM-kill triggers
mid-session.

---

## 1. Clone the repository

```bash
git clone https://github.com/isc-fs/IFSSIM.git
cd IFSSIM
git lfs install              # one-time per machine; safe to repeat
git lfs pull                 # ~500 MB of binary assets (cones, materials, …)
git submodule update --init --recursive
```

The `git submodule update --init --recursive` is **not optional** —
`tools/random-track-generator` is a submodule and the
`mission_control_backend` Docker image bakes it in at build time.
Skipping this step makes track generation 500 at runtime; pre-v0.1.1
it silently appeared to work because the bind-mount hid the empty
directory.

### Windows: where you clone matters

If your Docker Desktop is set up with the **WSL2 backend** (the
default since Docker Desktop 4.x), you have a choice:

| Clone location | Docker performance |
|---|---|
| `C:\Users\<you>\Documents\Github\IFSSIM` (Windows filesystem) | OK. Cross-filesystem ops use 9p; ~30-90 s container start, slow `docker compose build`. v0.1.1's `.dockerignore` + named bag volume mitigate the worst of it. |
| `\\wsl$\Ubuntu\home\<you>\IFSSIM` (WSL2 ext4) | **2-5× faster.** All Docker FS ops are native Linux. Recommended for any serious autonomy work. |

To use the WSL2 path: open Ubuntu (or your WSL distro), `cd ~`,
`git clone ...`. UE5 launching the sim is unaffected — the binary
runs natively on Windows; only the Docker pipeline benefits.

If you don't know which backend Docker is using:
**Docker Desktop → Settings → General** — "Use the WSL 2 based engine"
should be on.

---

## 2. Build (or download) the sim

Two ways. **Pick one.**

### Option A — download a release binary (faster, recommended)

Go to [Releases](https://github.com/isc-fs/IFSSIM/releases/latest),
download the platform zip:

- Windows: `IFSSIM-v0.1.1-Windows-x64.zip`
- macOS: `IFSSIM-v0.1.1-Mac.zip`

Unzip into the repo root. The contents must end up at:

- Windows: `Saved/StagedBuilds/Windows/IFSSIM.exe` (+ a `tracks/`
  sibling directory + a `settings.json` next to the exe + a
  `UECommandLine.txt`)
- macOS: `Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app` (+ same
  siblings)

That's it; skip to step 3.

### Option B — build from source

You'll need UE 5.7 installed (see prerequisites). From the repo root:

```bash
# Windows (in Git Bash, not PowerShell — package_windows.sh is bash):
bash package_windows.sh

# macOS:
./package_mac.sh
```

The script does a clean `BuildCookRun` then stages the result. ~5 min
cold, ~1 min on a re-cook. Output lands at
`Saved/StagedBuilds/{Windows,Mac}/...` — same layout as Option A.

If `UE_5.7` is somewhere unusual, override:

```bash
# Windows:
UE_ROOT="/c/path/to/UE_5.7" bash package_windows.sh

# macOS:
UE_ROOT="/path/to/UE_5.7" ./package_mac.sh
```

To skip producing the distribution zip (the last step), set
`SKIP_ARCHIVE=1`. Saves ~30 s during dev iterations.

#### "Access to the path 'IFSSIM.exe' is denied"

A previous sim instance is holding the exe. Close it
(`Stop-Process -Name IFSSIM*` in PowerShell, or close the window).
Then re-run.

---

## 3. Launch the sim

| Windows | macOS |
|---|---|
| Double-click `Saved\StagedBuilds\Windows\IFSSIM.exe` | `open Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app` |

The first launch on Windows triggers a **Windows Firewall prompt**
for TCP port 41451 — click **Allow**. Same prompt fires once for
the LiDAR TCP port. On macOS no firewall prompt — the
sandbox/entitlements package_mac.sh applies covers this.

A windowed UE5 client opens at 1280×720 (configurable via env vars
in `package_*.sh`). You'll see a flat training map with no track —
that's expected; the track gets loaded from Mission Control in
step 5.

The sim binds:

- **TCP 41451** — RPC + sensor + LiDAR streams (all TCP since v0.1.0)
- **UDP 41452/41453** — legacy UDP sensor + LiDAR ports, **off by
  default since v0.1.0** but reservable via the
  `enableLidarUdpBroadcast` RPC if you really want UDP.

---

## 4. Bring up the Docker stack

From the repo root:

```bash
docker compose build              # ~5-10 min cold, seconds after
docker compose up -d              # starts all 4 containers
```

After ~10 s, `docker compose ps` should show all four `Up (healthy)`:

```
NAME                                  STATUS
ifssim-dv_pipeline_stack-1            Up (healthy)
ifssim-lichtblick-1                   Up
ifssim-mission_control_backend-1      Up (healthy)
ifssim-mission_control_frontend-1     Up
```

If `dv_pipeline_stack` is stuck on "starting": that's the bridge
trying to reach the sim. If the sim is up (step 3) it'll connect
within a few retry cycles. Watch:

```bash
docker compose logs -f dv_pipeline_stack | grep -i "connected\|stream"
```

Look for these in order:

```
IFSSIM Bridge: Connected to host.docker.internal:41451
ifssim_bridge: IFSSIM connected (TCP push model)
ifssim_bridge: Sensor stream connected
ifssim_bridge: LiDAR transport: TCP (streamLidar, PR-#482)
odometry_filter_node: /odom first publish — IMU+RPM filter calibrated
```

That's the green light: bridge connected, sensors streaming,
complementary filter calibrated and publishing `/odom` at 100 Hz.

---

## 5. First session in Mission Control

Open <http://localhost:3000> in a browser.

1. **Track Manager** tab → pick a track (e.g. `skidpad.csv`) → click
   **Load**. The cones spawn in the sim window.
2. **Event** tab → pick a mission matching the track:
   - `acceleration.csv` → Acceleration
   - `skidpad.csv` → Skidpad
   - any random track → Autocross or Trackdrive
3. *(Optional)* Tick **Record bag (mcap)**. This captures the full
   sensor + autonomy state to an MCAP bag inside the bridge container.
   See [OPERATING.md § Recording bags](OPERATING.md#recording-and-retrieving-mcap-bags)
   for the retrieve step.
4. Click **Configure Event**, then **Start Session**.

When you click Start Session you'll see (in the session log):

```
mode_manager: configuring cone_detection_node…   ← ~10-20 s (numba JIT)
mode_manager: activating cone_detection_node
mode_manager: configuring slam_node…
mode_manager: activating slam_node
mode_manager: configuring path_planning_node…
mode_manager: activating path_planning_node
mode_manager: configuring control_node…
mode_manager: activating control_node
```

The 10-20 s cone_detection_node configure step is **expected** —
that's the numba JIT compile. Pre-v0.1.1 this looked like a hang;
since #489 the spinner shows per-node progress.

Once all four are `active`, the supervisor releases EBS and the car
starts driving (if an autonomy stack is wired up) or sits still
(if not — in which case you can **drive manually** by focusing the
sim window and using **WASD** + **Space** for handbrake).

---

## 6. Verify everything's healthy

If steps 4 and 5 went clean, you're done. But for the record:

```bash
# all 4 containers up
docker compose ps

# bridge is publishing sensors
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && ros2 topic hz /imu --window 100'
# should print ~400 Hz

# LiDAR
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && ros2 topic hz /lidar/Lidar1 --window 30'
# should print ~10 Hz

# odom (complementary filter)
docker compose exec dv_pipeline_stack bash -lc \
  '. /opt/ros/humble/setup.bash && ros2 topic hz /odom --window 100'
# should print ~100 Hz
```

If you see those three rates, the data plane is solid.

---

## Troubleshooting

### `IFSSIM Bridge: Connection failed to host.docker.internal:41451` (looping)

The sim isn't reachable from the bridge container. Two causes:

1. **Sim not running** — most common. Re-launch via the IFSSIM
   binary (step 3).
2. **`host.docker.internal` not resolving on Linux** — Linux Docker
   doesn't set this alias by default. The compose file injects
   `extra_hosts: ["host.docker.internal:host-gateway"]` to handle
   this; if that's not working, your Docker version is too old.
   Upgrade to ≥ 4.34.

If the sim is up and the bridge still fails after 30 s of retries,
the sim crashed at some point during your build — sim crashes leave
the TCP socket dangling for ~60 s. Kill and relaunch:

```powershell
# Windows:
Get-Process -Name IFSSIM* | Stop-Process -Force
```

```bash
# macOS:
killall IFSSIM-Mac-Shipping || true
```

Then re-launch.

### `BuildCookRun ... Access to the path 'IFSSIM.exe' is denied`

A previous sim is holding `IFSSIM.exe`. See the
[Option B troubleshooting note](#access-to-the-path-ifssimexe-is-denied)
above.

### Docker build is extremely slow on Windows

Two things:

1. Make sure `.dockerignore` is in your tree — `ls -la .dockerignore`
   should show it. It cuts the docker build context from 33 GB to
   ~200 MB. Added in v0.1.1 — if you're on an older clone, `git pull`.
2. If you're on `/mnt/c/...`, consider moving the clone into WSL2's
   ext4 (`~/IFSSIM`). See [§1 — Windows: where you clone matters](#windows-where-you-clone-matters).

### `docker compose ps` shows `dv_pipeline_stack` `Restarting`

The container is crash-looping. Check `docker compose logs --tail=200
dv_pipeline_stack`. The two common causes:

1. **Out of memory** — Docker Desktop's memory cap is below 4 GB.
   Settings → Resources → bump to ≥ 6 GB.
2. **Stale Fast DDS shared-memory droppings** — usually self-heals
   on entrypoint (which `rm -f /dev/shm/fastrtps_*`) but if the
   container exited mid-init you can wipe Docker's tmpfs by
   recreating: `tools/refresh-bridge.sh`.

### Track generation returns 500

The `tools/random-track-generator` submodule wasn't initialised
before the mc_backend image was built. Fix:

```bash
git submodule update --init --recursive tools/random-track-generator
docker compose build mission_control_backend
docker compose up -d --force-recreate mission_control_backend
```

### Mission Control shows "Disconnected"

Backend can't reach the bridge's RPC port. Same root cause as the
bridge-connection-failed loop above — make sure the sim is running
on port 41451.

---

## What's next

- **[OPERATING.md](OPERATING.md)** — daily ops: refreshing the bridge
  after a source edit, recording and retrieving MCAP bags, switching
  tracks, picking missions.
- **[AUTONOMY.md](AUTONOMY.md)** — wiring your own autonomy code in:
  the topics IFSSIM publishes, the runtime action contract,
  TF layout.
- **[REFERENCE.md](REFERENCE.md)** — exhaustive technical reference:
  sensor noise models, RPC API, ROS topics, vehicle physics
  parameters.

---

## Linux

Linux is supported (we ship a Linux package script, the Docker stack
runs natively, all the ROS deps are Linux-first) but not first-class —
no pre-built release zip, no self-hosted CI runner. Setup is the same
as macOS / Windows except:

- UE 5.7 has to be built from source on Linux. Clone
  [EpicGames/UnrealEngine](https://github.com/EpicGames/UnrealEngine)
  (requires Epic GitHub account linkage), build with
  `./Setup.sh && ./GenerateProjectFiles.sh && make` (~1 hour on a
  16-core box).
- Set `UE_ROOT=/opt/UE_5.7` (or wherever you built it) before running
  `./package_linux.sh`.
- Docker Engine + Compose plugin instead of Docker Desktop. Add your
  user to the `docker` group. `host.docker.internal` is mapped via
  `extra_hosts` in `docker-compose.yml`.

The autonomy code itself doesn't care which OS the sim runs on —
it's a docker container talking to a TCP socket on the host.
