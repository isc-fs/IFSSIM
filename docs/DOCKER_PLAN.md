# Docker Containerization Plan — feat/15-docker

## Goal

Package everything except UE5 into a single Docker Compose stack.
UE5 runs natively on the host (editor or packaged build) and communicates with the container over localhost TCP/UDP. The container exposes ROS2, Mission Control backend + frontend, and the Python client.

---

## Architecture

```
┌─────────────────── HOST ────────────────────────────────────┐
│                                                             │
│  UE5 / IFSSIM (native — Mac or Linux)                       │
│    └─ TCP  :41451  (RPC server)                             │
│    └─ UDP  :41452  (sensor stream → container)              │
│    └─ UDP  :41453  (LiDAR stream → container)               │
│                                                             │
└───────────────┬─────────────────────────────────────────────┘
                │ host networking (--network=host on Linux)
                │ host.docker.internal (Mac/Windows)
                │
┌───────────────▼──────── DOCKER COMPOSE ─────────────────────┐
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  ros2_bridge  (ros:humble-ros-base)                  │   │
│  │    colcon-built ifssim_bridge + fs_msgs              │   │
│  │    publishes: /gps /imu /gss /lidar /camera /tf …   │   │
│  │    subscribes: /control_command /signal/finished     │   │
│  └───────────────────────┬─────────────────────────────┘   │
│                          │ ROS2 DDS (shared network)         │
│  ┌────────────────────────▼────────────────────────────┐   │
│  │  mission_control_backend  (python:3.12-slim)         │   │
│  │    FastAPI + uvicorn on :8000                        │   │
│  │    WebSocket telemetry, scoring, event control       │   │
│  └───────────────────────┬─────────────────────────────┘   │
│                          │ HTTP/WS :8000                     │
│  ┌────────────────────────▼────────────────────────────┐   │
│  │  mission_control_frontend  (nginx:alpine)            │   │
│  │    Vite-built React/Tailwind static bundle           │   │
│  │    served on :3000, proxies /api + /ws → :8000       │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  Shared volume: /ros2_ws  (built workspace, read-only)      │
└─────────────────────────────────────────────────────────────┘
```

### Networking note

| Platform | Strategy |
|---|---|
| Linux | `network_mode: host` — containers share host interfaces, UDP just works |
| Mac / Windows | Use `host.docker.internal` as the simulator address; map UDP ports with `ports:` |

The `IFSSIM_HOST` env var (default `host.docker.internal`) controls where the bridge points.

---

## Services breakdown

### 1. `ros2_bridge`
- Base image: `ros:humble-ros-base`
- Installs: `colcon`, `ros-humble-tf2-ros`, `ros-humble-sensor-msgs`, `ros-humble-nav-msgs`
- Builds `fs_msgs` + `ifssim_bridge` via colcon at image build time
- Entrypoint: `ros2 launch ifssim_bridge ifssim_bridge.launch.py host:=$IFSSIM_HOST`
- Env vars: `IFSSIM_HOST`, `IFSSIM_PORT`, `MISSION_NAME`, `TRACK_NAME`
- Depends on: nothing (connects to host UE5 directly)

### 2. `mission_control_backend`
- Base image: `python:3.12-slim`
- Installs `requirements.txt` (fastapi, uvicorn, websockets, matplotlib)
- Copies `tools/mission_control/backend/`
- Also installs the `ifssim` Python client from `python/`
- Entrypoint: `uvicorn main:app --host 0.0.0.0 --port 8000`
- Env vars: `IFSSIM_HOST`, `IFSSIM_PORT`
- Ports: `8000:8000`

### 3. `mission_control_frontend`
- **Build stage**: `node:22-alpine` — runs `npm ci && npm run build`
- **Serve stage**: `nginx:alpine` — serves the `dist/` folder
- Nginx config: serves static on `:3000`, proxies `/api` and `/ws` to `backend:8000`
- Ports: `3000:80`
- Depends on: `mission_control_backend`

---

## Files to create

```
docker/
├── ros2_bridge/
│   └── Dockerfile
├── mission_control_backend/
│   └── Dockerfile
├── mission_control_frontend/
│   ├── Dockerfile
│   └── nginx.conf
docker-compose.yml
docker-compose.linux.yml        ← override for Linux (host networking)
.env.example                    ← IFSSIM_HOST, ports, mission config
```

---

## Step-by-step implementation plan

### Step 1 — `ros2_bridge` Dockerfile
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

Entrypoint sources `install/setup.bash` then does `exec ros2 launch ...`.

### Step 2 — `mission_control_backend` Dockerfile
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

### Step 3 — `mission_control_frontend` Dockerfile (multi-stage)
```dockerfile
# --- build stage ---
FROM node:22-alpine AS builder
WORKDIR /app
COPY tools/mission_control/frontend/package*.json ./
RUN npm ci
COPY tools/mission_control/frontend/ .
RUN npm run build

# --- serve stage ---
FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html
COPY docker/mission_control_frontend/nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
```

### Step 4 — nginx.conf
Serves static assets and proxies API + WebSocket to backend:
```nginx
server {
    listen 80;
    root /usr/share/nginx/html;
    index index.html;

    location /api/ {
        proxy_pass http://mission_control_backend:8000/api/;
    }
    location /ws {
        proxy_pass http://mission_control_backend:8000/ws;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
    location / {
        try_files $uri $uri/ /index.html;
    }
}
```

### Step 5 — `docker-compose.yml`
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
      - "host.docker.internal:host-gateway"  # Linux compat

  mission_control_backend:
    build:
      context: .
      dockerfile: docker/mission_control_backend/Dockerfile
    ports:
      - "8000:8000"
    environment:
      - IFSSIM_HOST=${IFSSIM_HOST:-host.docker.internal}
      - IFSSIM_PORT=${IFSSIM_PORT:-41451}
    extra_hosts:
      - "host.docker.internal:host-gateway"

  mission_control_frontend:
    build:
      context: .
      dockerfile: docker/mission_control_frontend/Dockerfile
    ports:
      - "3000:80"
    depends_on:
      - mission_control_backend
```

### Step 6 — Linux override (`docker-compose.linux.yml`)
On Linux, host networking makes UDP ports transparent without mapping:
```yaml
services:
  ros2_bridge:
    network_mode: host
  mission_control_backend:
    network_mode: host
  mission_control_frontend:
    network_mode: host
```
Usage: `docker compose -f docker-compose.yml -f docker-compose.linux.yml up`

### Step 7 — `.env.example`
```
IFSSIM_HOST=host.docker.internal
IFSSIM_PORT=41451
MISSION_NAME=trackdrive
TRACK_NAME=A
```

---

## Mac-specific notes

Docker Desktop on Mac does **not** forward UDP ports with `--network=host` (the Linux VM intercepts them). The workaround:

- The bridge connects **out** from the container to UE5 on the host (TCP on :41451, UDP listener for push streams on :41452/:41453)
- For outbound TCP (RPC): `host.docker.internal` works fine
- For inbound UDP (sensor/LiDAR push from UE5 to container): UE5 needs to broadcast to the container's address, not `127.0.0.1`. Two options:
  1. **Preferred**: make `FSDS_TARGET_IP` in settings.json configurable, default `127.0.0.1` but overridable to `host.docker.internal` from container side
  2. **Alternative**: run a UDP relay on the host that forwards :41452/:41453 into the container

The plan implements option 1: add `target_ip` to `settings.json` (already has a `host` field concept) and read it in `FSDSUdpBroadcaster`. When running in Docker on Mac, set `IFSSIM_TARGET_IP` to the container's IP.

---

## Implementation order

1. `docker/ros2_bridge/Dockerfile` + `entrypoint.sh`
2. `docker/mission_control_backend/Dockerfile`
3. `docker/mission_control_frontend/Dockerfile` + `nginx.conf`
4. `docker-compose.yml` + `docker-compose.linux.yml`
5. `.env.example`
6. Update `FSDSUdpBroadcaster` to read `target_ip` from `settings.json`
7. Update `settings.json` to add `target_ip` field
8. Build + smoke test each service individually
9. Full stack test: UE5 editor → Play → `docker compose up`
10. Document in `docs/DOCKER.md`
