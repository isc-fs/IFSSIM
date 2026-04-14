# Docker Containerization Plan — feat/15-docker

## Goal

Package the full IFS driverless stack into a single Docker Compose stack.  
UE5 (IFSSIM) runs natively on the host. Everything else — ROS2 bridge, the IFS07-DV pipeline, Mission Control — lives in containers.

---

## Repo layout

```
IFSSIM/                   ← this repo (simulator + bridge + mission control)
IFS07-DV/                 ← pipeline repo (slam, control, path planning, …)
  src/
    slam/
    odometria/
    path_planning/
    control/
    coche_urdf/
    yolo/            (optional — not active by default)
```

The Docker Compose file lives in IFSSIM but mounts / copies IFS07-DV source at build time. The two repos will be siblings on the developer's machine.

---

## Full system architecture

```
┌─────────────────── HOST ──────────────────────────────────────────┐
│                                                                   │
│  UE5 / IFSSIM (native — Mac or Linux)                             │
│    TCP  :41451  RPC server                                        │
│    UDP  :41452  sensor push  ─────────────────────────────────┐   │
│    UDP  :41453  LiDAR push   ─────────────────────────────────┤   │
│                                                               │   │
└──────────────────────┬────────────────────────────────────────────┘
                       │ host.docker.internal (Mac) / host network (Linux)
                       │
┌──────────────────────▼────── DOCKER COMPOSE ──────────────────────┐
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │  ros2_bridge                          (ros:humble-base)  │    │
│  │  Builds: IFSSIM/ros2/src (fs_msgs + ifssim_bridge)       │    │
│  │  Publishes:  /gps  /imu  /gss  /lidar/Lidar1             │    │
│  │              /testing_only/odom  /testing_only/track      │    │
│  │              /camera/cam1/compressed                      │    │
│  │  Subscribes: /control_command  /signal/finished           │    │
│  └──────────────────────────┬───────────────────────────────┘    │
│                             │ ROS2 DDS (shared bridge network)    │
│  ┌──────────────────────────▼───────────────────────────────┐    │
│  │  pipeline                             (ros:humble-base)  │    │
│  │  Builds: IFS07-DV/src (slam, odometria, path_planning,   │    │
│  │          control, coche_urdf, yolo)                      │    │
│  │                                                          │    │
│  │  Nodes launched:                                         │    │
│  │    Odometria_perfecta  → TF odom→fsds/FSCar              │    │
│  │    Cone_Detection      ← /lidar/Lidar1 (remapped)        │    │
│  │    Publicar_Mapa       ← Conos_raw → Conos               │    │
│  │    Publicar_Track      ← /testing_only/track (remapped)  │    │
│  │    Plan_Path           ← Conos, publishes Path           │    │
│  │    Control             ← Path, /gss, /odom               │    │
│  │                          publishes /control_command       │    │
│  └──────────────────────────┬───────────────────────────────┘    │
│                             │ HTTP :8000                          │
│  ┌──────────────────────────▼───────────────────────────────┐    │
│  │  mission_control_backend          (python:3.12-slim)     │    │
│  │  FastAPI + uvicorn on :8000                              │    │
│  └──────────────────────────┬───────────────────────────────┘    │
│                             │ HTTP/WS reverse proxy              │
│  ┌──────────────────────────▼───────────────────────────────┐    │
│  │  mission_control_frontend              (nginx:alpine)    │    │
│  │  Vite-built React/Tailwind on :3000                      │    │
│  └──────────────────────────────────────────────────────────┘    │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

---

## Topic remapping: FSDS → IFSSIM

IFS07-DV was written against the FSDS bridge, which uses `/fsds/` prefixes. IFSSIM uses cleaner names. Remapping is done in the pipeline container's launch file — no pipeline source code changes needed.

| IFS07-DV expects (FSDS) | IFSSIM publishes | Direction |
|---|---|---|
| `/fsds/lidar/Lidar1` | `/lidar/Lidar1` | bridge → pipeline |
| `/fsds/testing_only/odom` | `/testing_only/odom` | bridge → pipeline |
| `/fsds/testing_only/track` | `/testing_only/track` | bridge → pipeline |
| `/fsds/gss` | `/gss` | bridge → pipeline |
| `/fsds/control_command` | `/control_command` | pipeline → bridge |
| `/fsds/cameracam1/image_color` | `/camera/cam1/compressed` | bridge → pipeline ⚠️ |

⚠️ **Camera format mismatch**: FSDS publishes raw `sensor_msgs/Image`; IFSSIM publishes `CompressedImage`. Two options:
1. Add a decompression relay node in the pipeline container (lightweight, no source change)
2. Add a raw image publisher to `FSDSCameraSensor` in IFSSIM (more work, better long-term)

**Plan**: implement option 1 first (relay node in launch file using `image_transport` republish), revisit option 2 later.

---

## IFS07-DV pipeline analysis

### Packages (all `ament_python`)

| Package | Node executable | What it does | Key deps |
|---|---|---|---|
| `slam` | `Cone_Detection` | LiDAR → raw cones (Numba JIT, needs warmup) | numba, opencv, numpy, fs_msgs |
| `slam` | `Publicar_Mapa` | Raw cones → persistent cone map | numpy |
| `slam` | `Publicar_Track` | Real cone positions from simulator | fs_msgs |
| `odometria` | `Odometria_perfecta` | `/testing_only/odom` → TF `odom→fsds/FSCar` | tf2_ros, fs_msgs |
| `path_planning` | `Plan_Path` | Cone map → `Path` (Stanley target points) | scipy, scikit-learn, transforms3d, fsd_path_planning (bundled) |
| `control` | `Control` | `Path` + GSS → `ControlCommand` at 40 Hz | fs_msgs, transforms3d |
| `coche_urdf` | (launch) | Vehicle URDF + joint state for RViz | xacro, joint-state-publisher |
| `yolo` | `Yolo` | Camera → cone detections (not active by default) | ultralytics, opencv, cv_bridge |

### Startup timing constraints
- `Cone_Detection` and `Control` both use Numba JIT — first call compiles, takes ~20s
- `full_pipeline.py` already handles this with `sleep 20` prefix on the Control node
- The pipeline launch file must preserve these delays

### Python dependencies (pip, beyond what ros:humble provides)
```
numba>=0.59
numpy==1.24  # numba requires specific numpy version
opencv-python>=4.11
scipy>=1.8
scikit-learn>=1.6
transforms3d>=0.4
fsd_path_planning  # bundled in path_planning package, install with pip install -e
```
YOLO (optional):
```
ultralytics  # pulls torch — large image, only if yolo is enabled
```

### System packages (apt, beyond ros:humble-base)
```
ros-humble-cv-bridge
ros-humble-image-transport
ros-humble-xacro
ros-humble-joint-state-publisher
ros-humble-tf2-ros
ros-humble-tf2-geometry-msgs
python3-pip
clang-12 clang++-12 libc++-12-dev libc++abi-12-dev  # for numba llvmlite
```

---

## Files to create

```
IFSSIM/
├── docker/
│   ├── ros2_bridge/
│   │   ├── Dockerfile
│   │   └── entrypoint.sh
│   ├── pipeline/
│   │   ├── Dockerfile
│   │   ├── entrypoint.sh
│   │   └── pipeline.launch.py       ← wraps full_pipeline.py with topic remaps
│   ├── mission_control_backend/
│   │   └── Dockerfile
│   └── mission_control_frontend/
│       ├── Dockerfile
│       └── nginx.conf
├── docker-compose.yml
├── docker-compose.linux.yml         ← host networking override for Linux
└── .env.example
```

---

## Step-by-step implementation plan

### Step 1 — `docker/ros2_bridge/Dockerfile`

```dockerfile
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-tf2-ros \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-image-transport \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ros2/src/ src/
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

ENV IFSSIM_HOST=host.docker.internal
ENV IFSSIM_PORT=41451
ENV MISSION_NAME=trackdrive
ENV TRACK_NAME=A

COPY docker/ros2_bridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

`entrypoint.sh`:
```bash
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec ros2 launch ifssim_bridge ifssim_bridge.launch.py \
  host:=$IFSSIM_HOST port:=$IFSSIM_PORT \
  mission_name:=$MISSION_NAME track_name:=$TRACK_NAME
```

### Step 2 — `docker/pipeline/Dockerfile`

```dockerfile
FROM ros:humble-ros-base

# System deps
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    clang-12 clang++-12 libc++-12-dev libc++abi-12-dev \
  && rm -rf /var/lib/apt/lists/*

# Python deps (numba needs specific numpy, install first)
RUN pip install --no-cache-dir \
    "numpy==1.24.*" \
    "numba>=0.59" \
    opencv-python-headless \
    scipy scikit-learn transforms3d

# Build ROS2 pipeline packages
WORKDIR /pipeline_ws
# IFS07-DV source is bind-mounted or copied at build time
# (see ARG IFS07DV_PATH in docker-compose)
COPY IFS07DV_SOURCE/ src/
RUN . /opt/ros/humble/setup.sh && \
    pip install --no-cache-dir -e src/path_planning/ && \
    colcon build --symlink-install

COPY docker/pipeline/entrypoint.sh /entrypoint.sh
COPY docker/pipeline/pipeline.launch.py /pipeline_ws/pipeline.launch.py
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

`entrypoint.sh`:
```bash
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /pipeline_ws/install/setup.bash
exec ros2 launch /pipeline_ws/pipeline.launch.py
```

### Step 3 — `docker/pipeline/pipeline.launch.py`

This replaces `IFS07-DV/full_pipeline.py` and adds topic remappings:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

REMAP_LIDAR   = ('/fsds/lidar/Lidar1',           '/lidar/Lidar1')
REMAP_ODOM    = ('/fsds/testing_only/odom',       '/testing_only/odom')
REMAP_TRACK   = ('/fsds/testing_only/track',      '/testing_only/track')
REMAP_GSS     = ('/fsds/gss',                     '/gss')
REMAP_CMD     = ('/fsds/control_command',         '/control_command')

def generate_launch_description():
    return LaunchDescription([
        Node(package='odometria', executable='Odometria_perfecta',
             remappings=[REMAP_ODOM]),

        Node(package='slam', executable='Cone_Detection',
             remappings=[REMAP_LIDAR]),

        Node(package='slam', executable='Publicar_Mapa'),

        Node(package='slam', executable='Publicar_Track',
             remappings=[REMAP_TRACK]),

        Node(package='path_planning', executable='Plan_Path'),

        Node(package='control', executable='Control',
             prefix=["bash -c 'sleep 20; $0 $@' "],
             remappings=[REMAP_GSS, REMAP_ODOM, REMAP_CMD]),

        # image_transport republish: CompressedImage → raw Image for YOLO
        # Uncomment when YOLO is enabled:
        # Node(package='image_transport', executable='republish',
        #      arguments=['compressed', 'raw'],
        #      remappings=[('in/compressed', '/camera/cam1/compressed'),
        #                  ('out', '/fsds/cameracam1/image_color')]),
        # Node(package='yolo', executable='Yolo'),
    ])
```

### Step 4 — `docker/mission_control_backend/Dockerfile`

```dockerfile
FROM python:3.12-slim

WORKDIR /app
COPY python/ /ifssim_client/
RUN pip install --no-cache-dir /ifssim_client

COPY tools/mission_control/backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY tools/mission_control/backend/ .

ENV IFSSIM_HOST=host.docker.internal
ENV IFSSIM_PORT=41451

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Step 5 — `docker/mission_control_frontend/Dockerfile` (multi-stage)

```dockerfile
FROM node:22-alpine AS builder
WORKDIR /app
COPY tools/mission_control/frontend/package*.json ./
RUN npm ci
COPY tools/mission_control/frontend/ .
RUN npm run build

FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html
COPY docker/mission_control_frontend/nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
```

`nginx.conf`:
```nginx
server {
    listen 80;
    root /usr/share/nginx/html;
    location /api/ { proxy_pass http://mission_control_backend:8000/api/; }
    location /ws {
        proxy_pass http://mission_control_backend:8000/ws;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
    location / { try_files $uri $uri/ /index.html; }
}
```

### Step 6 — `docker-compose.yml`

```yaml
services:

  ros2_bridge:
    build:
      context: .
      dockerfile: docker/ros2_bridge/Dockerfile
    environment:
      - IFSSIM_HOST=${IFSSIM_HOST:-host.docker.internal}
      - IFSSIM_PORT=${IFSSIM_PORT:-41451}
      - MISSION_NAME=${MISSION_NAME:-trackdrive}
      - TRACK_NAME=${TRACK_NAME:-A}
    extra_hosts:
      - "host.docker.internal:host-gateway"
    networks: [ros_net]

  pipeline:
    build:
      context: .
      dockerfile: docker/pipeline/Dockerfile
      # IFS07-DV source must be at ../IFS07-DV relative to IFSSIM
      # or set IFS07DV_PATH env var
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    networks: [ros_net]
    depends_on: [ros2_bridge]

  mission_control_backend:
    build:
      context: .
      dockerfile: docker/mission_control_backend/Dockerfile
    ports: ["8000:8000"]
    environment:
      - IFSSIM_HOST=${IFSSIM_HOST:-host.docker.internal}
    extra_hosts:
      - "host.docker.internal:host-gateway"
    networks: [ros_net]

  mission_control_frontend:
    build:
      context: .
      dockerfile: docker/mission_control_frontend/Dockerfile
    ports: ["3000:80"]
    depends_on: [mission_control_backend]
    networks: [ros_net]

networks:
  ros_net:
    driver: bridge
```

### Step 7 — `docker-compose.linux.yml` (host networking override)

```yaml
services:
  ros2_bridge:
    network_mode: host
  pipeline:
    network_mode: host
  mission_control_backend:
    network_mode: host
  mission_control_frontend:
    network_mode: host
```

Usage on Linux: `docker compose -f docker-compose.yml -f docker-compose.linux.yml up`

### Step 8 — `.env.example`

```
# Simulator host (use host.docker.internal on Mac/Windows, 127.0.0.1 on Linux with host networking)
IFSSIM_HOST=host.docker.internal
IFSSIM_PORT=41451

# Mission config
MISSION_NAME=trackdrive
TRACK_NAME=A

# ROS2
ROS_DOMAIN_ID=0

# Path to IFS07-DV repo (relative to IFSSIM root, used at build time)
IFS07DV_PATH=../IFS07-DV
```

---

## Mac-specific: UDP inbound problem

UE5 pushes sensor UDP packets to a configured `target_ip`. On Mac, Docker containers are in a Linux VM — `127.0.0.1` from UE5's perspective goes to the host, not the container.

**Fix**: add `target_ip` to `settings.json` and wire it through `FSDSUdpBroadcaster`. When containerized on Mac, set `IFSSIM_TARGET_IP` to the Docker bridge IP (typically `172.17.0.x`), or use `host.docker.internal` from inside the container and rely on port mapping.

Concrete plan:
1. Add `"TargetIP": "127.0.0.1"` to `settings.json` under `Vehicles.FSCar`
2. Read it in `FFSDSSettings` and pass to `FFSDSUdpBroadcaster::Start()`
3. Document: on Mac+Docker, set `TargetIP` to the bridge container's IP, or use `0.0.0.0` broadcast mode

On Linux with `network_mode: host` this issue does not exist.

---

## Implementation order

1. `docker/ros2_bridge/Dockerfile` + `entrypoint.sh` — build + smoke test (bridge connects to UE5)
2. `docker/pipeline/Dockerfile` + `pipeline.launch.py` — build + verify numba compiles inside container
3. `docker/mission_control_backend/Dockerfile` — build + verify FastAPI starts
4. `docker/mission_control_frontend/Dockerfile` + `nginx.conf` — build + verify UI loads
5. `docker-compose.yml` + `docker-compose.linux.yml` + `.env.example`
6. IFSSIM `settings.json` + `FSDSUdpBroadcaster`: add configurable `TargetIP`
7. End-to-end test: UE5 editor → Play → `docker compose up` → car drives
8. `docs/DOCKER.md` — developer quickstart

## Open questions to resolve during implementation

- **IFS07-DV source in Docker context**: the pipeline Dockerfile needs IFS07-DV source. Options: (a) git clone at build time (needs auth for private repo), (b) build context includes both repos (requires running compose from a parent dir), (c) submodule. **Recommendation**: add IFS07-DV as a git submodule inside IFSSIM at `pipeline/` — cleanest, keeps single-repo workflow.
- **YOLO weights file**: `src/yolo/weights/best.pt` is ~6MB, tracked in git. Keep as-is (small enough). If it grows, move to Git LFS.
- **Numba cache across restarts**: numba caches compiled functions in `__pycache__`. Mount a named volume at `/pipeline_ws/src/slam/__pycache__` and `/pipeline_ws/src/control/__pycache__` so cache persists across container restarts and ~20s warmup only happens on first ever run.
