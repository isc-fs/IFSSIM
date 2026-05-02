# Docker Containerization Plan — feat/15-docker

## Goal

One `docker compose up` that brings up the full IFS driverless sim stack on any machine (Mac, Windows, Linux) regardless of local tooling. UE5/IFSSIM runs natively on the host; everything else runs in containers.

---

## Architecture

```
HOST (Mac / Windows / Linux)
  UE5 / IFSSIM
    TCP  :41451  ──────────────► dv_pipeline_stack (outbound from container)
    UDP  :41452  ──────────────► dv_pipeline_stack (Docker port-mapped)
    UDP  :41453  ──────────────► dv_pipeline_stack (Docker port-mapped)

DOCKER COMPOSE
  dv_pipeline_stack                         ports: 41452/udp, 41453/udp
    ifssim_bridge   → ROS2 topics
    slam            ← /lidar/Lidar1
    odometria       ← /testing_only/odom
    path_planning   ← Conos
    control         → /control_command

  mission_control_backend           port: 8000
  mission_control_frontend          port: 3000
```

**Why single `dv_pipeline_stack` container:**
- DDS multicast works on localhost — no per-platform networking config
- Docker Desktop (Mac/Windows) + port mapping handles UDP forwarding from host transparently
- Linux works the same way — no `network_mode: host` override needed
- Truly one `docker-compose.yml`, no platform variants

`host.docker.internal` resolves to the host on Mac/Windows natively. On Linux we add `extra_hosts: host.docker.internal:host-gateway` and it's identical.

---

## Repo layout

```
IFSSIM/
├── pipeline/               ← IFS07-DV packages copied here (no yolo)
│   ├── slam/
│   ├── odometria/
│   ├── path_planning/
│   ├── control/
│   └── coche_urdf/
├── ros2/src/               ← bridge packages
│   ├── fs_msgs/
│   └── ifssim_bridge/
├── tools/mission_control/
│   ├── backend/
│   └── frontend/
├── docker/
│   ├── dv_pipeline_stack/
│   │   ├── Dockerfile
│   │   ├── entrypoint.sh
│   │   └── pipeline.launch.py
│   ├── mission_control_backend/
│   │   └── Dockerfile
│   └── mission_control_frontend/
│       ├── Dockerfile
│       └── nginx.conf
├── docker-compose.yml
└── .env.example
```

---

## Topic remapping: FSDS → IFSSIM

IFS07-DV was written against the FSDS bridge (`/fsds/` prefixes). IFSSIM drops them. Remapping is done in `pipeline.launch.py` — no pipeline source changes needed.

| Pipeline expects (FSDS) | IFSSIM publishes | Notes |
|---|---|---|
| `/fsds/lidar/Lidar1` | `/lidar/Lidar1` | slam/Cone_Detection |
| `/fsds/testing_only/odom` | `/testing_only/odom` | odometria, control |
| `/fsds/testing_only/track` | `/testing_only/track` | slam/Publicar_Track |
| `/fsds/gss` | `/gss` | control |
| `/fsds/control_command` | `/control_command` | control → bridge |

Camera (YOLO not included): FSDS raw `Image` vs IFSSIM `CompressedImage` — irrelevant for now.

---

## Implementation steps

### 1. `docker/dv_pipeline_stack/Dockerfile`
### 2. `docker/dv_pipeline_stack/entrypoint.sh`
### 3. `docker/dv_pipeline_stack/pipeline.launch.py`
### 4. `docker/mission_control_backend/Dockerfile`
### 5. `docker/mission_control_frontend/Dockerfile` + `nginx.conf`
### 6. `docker-compose.yml`
### 7. `.env.example`
### 8. Update `settings.json` + `FSDSUdpBroadcaster` for configurable TargetIP
### 9. Smoke test each service, then full end-to-end test
### 10. `docs/DOCKER.md` quickstart

---

## Open items

- **IFS07-DV source**: copied directly into `pipeline/` for now. Will become a submodule when the pipeline gets its own repo.
- **Numba cache**: mount named Docker volume at `/dv_pipeline_stack_ws/src/slam` and `/dv_pipeline_stack_ws/src/control` so JIT cache persists across restarts (avoids 20s warmup every time).
- **YOLO**: excluded. Can be added back as an opt-in service later.
