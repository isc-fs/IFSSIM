# Autonomy pipeline

End-to-end overview of the autonomy stack: from raw LiDAR/IMU coming out of UE5 through perception, SLAM, path planning, and control — and back to UE5 as a `ControlCommand`. Read this before touching any of the autonomy nodes.

For sim-side topics (sensors, vehicle physics, RPC) see [`FUNCTIONALITIES.md`](FUNCTIONALITIES.md). For container layout and how to launch the stack see [`GETTING_STARTED_DOCKER.md`](GETTING_STARTED_DOCKER.md).

The DV pipeline (perception, SLAM, path planning, control, mission management) lives in a separate repo and is attached here as a submodule. The point of the split is that **the same code runs on the real car and in sim** — IFSSIM owns everything that fakes the real-car environment for it (sim, sim ↔ ROS bridge, Mission Control web, visualisation, Docker glue), and the autonomy submodule sees the same ROS interface either way. This is the central organising principle of the architecture and shows up in every section below.

## Component ownership

| Side | Owner | Lives in |
|---|---|---|
| Sim (UE5 + FSDSPlugin) | IFSSIM | this repo, `Plugins/FSDSPlugin/` |
| Sim ↔ ROS bridge | IFSSIM | this repo, `ros2/src/ifssim_bridge/` |
| Mission Control web (frontend + backend) | IFSSIM | this repo, `tools/mission_control/` |
| Visualisation (Lichtblick + layouts) | IFSSIM | this repo, `lichtblick/`, container `lichtblick` |
| Mission management (`sim_supervisor`, `mission_control`, `mode_manager`) | DV pipeline | submodule |
| Autonomy lifecycle nodes (`perception`, `slam`, `path_planning`, `control`) | DV pipeline | submodule |
| `coche_urdf` + `robot_state_publisher` | DV pipeline | submodule |
| Docker / compose / launch glue | IFSSIM | this repo, `docker/` |

### Roles inside mission management

- **`sim_supervisor_node`** simulates the role the IFS-08 uDV (the on-vehicle microcontroller) plays on the real car. In sim it sources the `GO` signal and sinks the `FINISHED` signal; on the real car the physical uDV plays that role. This node only exists in sim.
- **`mission_control_node`** receives the selected mission, drives the autonomy lifecycle through `mode_manager`, exchanges heartbeats with the supervisor during startup, and forwards control commands from the autonomy stack to the supervisor (in sim) or to the uDV over USB CDC running microROS (real car). Because the uDV is a microROS endpoint, the DVPC↔uDV exchange is **already a ROS 2 client-server interaction on the real car** — the only thing that changes between sim and real is the underlying DDS transport (intra-process in sim, USB-CDC-framed microROS in real).
- **`mode_manager_node`** brings up the correct lifecycle nodes for the mission and passes them the mission-specific flag/strategy so each node knows which behaviour to run.
- **Autonomy lifecycle nodes** internally use a strategy pattern keyed on the mission flag, so the same node binary handles trackdrive / autocross / accel / skidpad with different inner behaviours.

`mission_control_node` and `sim_supervisor_node` are **always co-resident in sim** — the supervisor doesn't host the DVPC role itself. The autonomy stack's view of the world is therefore the same in sim and on the real car: identical ROS 2 client calls into a microROS endpoint, with only the underlying DDS transport differing. `mission_control_backend` (the web stack) only ever targets the supervisor's `StartMission` action; it never short-circuits straight to `mission_control_node`.

Odometry lives inside `slam_node` (or in a thin fusion library it imports) rather than as a separate node — see [Open questions](#open-questions) for the exact placement decision, which has consequences for how DA degradation in cone-only sections is recoverable.

## End-to-end graph

The runtime control loop mirrors the real-car path: the autonomy stack does **not** publish actuator commands directly to the sim. `control_node` sends commands to `mission_control_node`, which forwards them to `sim_supervisor_node` (the simulated uDV), which publishes them to the bridge. On the real car the same `mission_control_node` forwards to the physical uDV over USB CDC running microROS — semantically identical ROS 2 client calls, only the transport changes. The sim path is one redirect longer than strictly necessary on purpose, so a bug in the chain shows up in sim before it shows up at the test track.

```mermaid
flowchart TB
    classDef sim fill:#1f3a52,stroke:#5fa3d6,color:#e8f0ff
    classDef bridge fill:#3a2a52,stroke:#a778d6,color:#f0e8ff
    classDef mc fill:#52382a,stroke:#d6925f,color:#ffe8d8
    classDef miss fill:#523a52,stroke:#d678d6,color:#ffe8ff
    classDef autn fill:#2a4a2a,stroke:#7dc97d,color:#e8ffe8
    classDef infra fill:#3a3a3a,stroke:#a8a8a8,color:#e8e8e8
    classDef viz fill:#3a3a3a,stroke:#a8a8a8,color:#e8e8e8,stroke-dasharray:4 4

    %% =====================================================================
    %% IFSSIM-OWNED HALF
    %% =====================================================================
    subgraph UE5["IFSSIM (host) — owned by this repo"]
        plugin["<b>FSDSPlugin</b><br/><i>C++</i><br/>RPC server :41451<br/>UDP sensor push"]
    end
    class UE5 sim

    subgraph BridgeContainer["🐳 ifssim_bridge_stack — owned by this repo"]
        bridge["<b>ifssim_bridge</b><br/><i>C++</i><br/>UDP recv + JSON-RPC<br/>publishes /fsds/* topics<br/>subscribes /fsds/control_command"]
    end
    class bridge bridge

    subgraph MCWeb["🐳 mission_control_(frontend|backend) — owned by this repo"]
        mc_frontend["<b>frontend</b> (React)"]
        mc_backend["<b>backend</b> (FastAPI)"]
    end
    class mc_frontend,mc_backend mc

    subgraph VizContainer["🐳 lichtblick — owned by this repo"]
        fox["<b>foxglove_bridge</b><br/>ws://:8765"]
    end
    class fox,VizContainer viz

    %% =====================================================================
    %% DV PIPELINE SUBMODULE
    %% =====================================================================
    subgraph PipelineSubmodule["🐳 dv_pipeline_stack — DV pipeline submodule"]

        subgraph Mission["Mission management"]
            sup["<b>sim_supervisor_node</b><br/>(sim-only)<br/>simulates uDV (microROS endpoint)"]
            mcn["<b>mission_control_node</b>"]
            mm["<b>mode_manager_node</b><br/>lifecycle orchestrator"]
        end
        class sup,mcn,mm miss

        subgraph Auton["Autonomy pipeline (LifecycleNodes)"]
            cd["<b>cone_detection_node</b><br/><i>perception pkg</i>"]
            slam["<b>slam_node</b><br/><i>slam pkg</i>"]
            plan["<b>path_planning_node</b>"]
            ctrl["<b>control_node</b><br/>40 Hz timer<br/>(slew-limited, PR #308)"]
        end
        class cd,slam,plan,ctrl autn

        subgraph Infra["Infrastructure"]
            rsp["<b>robot_state_publisher</b><br/>+ joint_state_publisher<br/>coche_urdf, 200 Hz TF"]
        end
        class rsp infra
    end

    %% =====================================================================
    %% SIM SENSORS: bridge → submodule
    %% =====================================================================
    plugin -- "UDP :51453 (LiDAR)<br/>UDP :41452 (other sensors)" --> bridge
    bridge -- "<b>/fsds/lidar/Lidar1</b><br/>PointCloud2 @ 10 Hz" --> cd
    bridge -- "<b>/fsds/imu</b><br/>sensor_msgs/Imu @ 400 Hz" --> slam
    bridge -- "<b>/fsds/motor_rpm</b><br/>Float32 @ 80 Hz<br/>(velocity prior)" --> slam
    bridge -- "<b>/fsds/testing_only/odom</b><br/>(diagnostic only — GT-aligned)" --> slam
    bridge -- "<b>/fsds/imu</b><br/>(filter prediction)" --> sup
    bridge -- "<b>/fsds/motor_rpm</b><br/>(filter correction)" --> sup
    bridge -- "<b>/fsds/steering_angle</b><br/>(kinematic-bicycle cross-check, #383)" --> sup
    bridge -- "<b>/fsds/brake_pressure</b><br/>(slip-event α scaling, #383)" --> sup

    %% =====================================================================
    %% AUTONOMY DATAFLOW (internal to submodule)
    %% =====================================================================
    cd -- "/Conos_raw MarkerArray" --> slam
    slam -- "<b>/Conos MarkerArray</b><br/>(map frame)<br/>+ TF map→odom" --> plan
    slam -- "<b>/slam/pose</b><br/>(absolute pose, ~10 Hz, map frame)<br/>+ TF map→odom" --> ctrl
    sup -- "<b>/odom</b><br/>nav_msgs/Odometry @ 100 Hz<br/>+ TF odom→base_link<br/>(IMU+RPM dead-reckoning)" --> ctrl
    sup -. "<b>/odom</b><br/>(for map→odom drift correction)" .-> slam
    plan -- "<b>/Path</b><br/>(map frame)" --> ctrl
    rsp -. "TF (URDF joints + base)" .-> slam
    rsp -. "TF" .-> ctrl

    %% =====================================================================
    %% MISSION MANAGEMENT — STARTUP (Phase 1) and RUNTIME (Phase 2) ACTIONS
    %% =====================================================================
    sup <-- "<b>StartMission [Action]</b><br/>Phase 1 — startup<br/>heartbeat + ready/failed<br/>(JIT-warm window)" --> mcn
    sup <-- "<b>RuntimeControl [Action]</b><br/>Phase 2 — running<br/>throttle / steering<br/>+ emergency + finished" --> mcn
    mcn -- "activate_mode [Srv]<br/>+ mission flag" --> mm
    mm -. "change_state [Srv]<br/>(lifecycle)" .-> cd
    mm -. "change_state" .-> slam
    mm -. "change_state" .-> plan
    mm -. "change_state" .-> ctrl
    ctrl -- "<b>/ctrl/cmd_internal</b><br/>fs_msgs/ControlCommand @ 40 Hz" --> mcn
    ctrl -- "<b>/ctrl/emergency</b><br/>std_msgs/Bool (latched)" --> mcn
    slam -- "<b>/slam/finished</b><br/>std_msgs/Bool (latched)" --> mcn

    %% =====================================================================
    %% SIM-SIDE OUTPUT: sim_supervisor → bridge → FSDS
    %% =====================================================================
    sup -- "<b>/fsds/control_command</b><br/>fs_msgs/ControlCommand<br/>(simulating uDV→bridge)" --> bridge
    sup -- "/signal/ebs, /signal/ebs_reset" --> bridge
    bridge -- "JSON-RPC :41451<br/>(throttle/regen/steer + EBS<br/>+ loadTrack/setMode/...)" --> plugin

    %% =====================================================================
    %% MISSION CONTROL WEB ↔ MISSION MANAGEMENT
    %% =====================================================================
    mc_frontend -- "REST" --> mc_backend
    mc_backend -- "StartMission [Action]<br/>(rclpy client)" --> sup
    mc_backend -- "JSON-RPC<br/>(track load, sim pause, RES)" --> bridge

    %% =====================================================================
    %% VIZ
    %% =====================================================================
    bridge -. "all /fsds/*" .-> fox
    cd -. "/Conos_raw, /Conos_Orange" .-> fox
    slam -. "/Conos, /slam/pose, /cone_slam/gt_*, TF" .-> fox
    sup -. "/odom" .-> fox
    plan -. "/Path, /path_planning/debug" .-> fox
    ctrl -. "/control/*" .-> fox
```

Solid arrows are ROS topics, RPC, or Action exchanges; dashed arrows are TF, services, or visualisation side-channels. Bidirectional `<-->` arrows represent ROS Actions (request + heartbeats + result).

Colour key: blue = sim, purple = bridge, orange = Mission Control web, magenta = mission management ROS nodes, green = autonomy lifecycle nodes, grey = infrastructure / viz.

## The integration contract

This is the API surface between IFSSIM and the DV pipeline submodule. **Any change here is a breaking interface change** and must be coordinated.

### Topics IFSSIM publishes (bridge → submodule)

All sensor topics are namespaced under `/fsds/...` — the prefix is what tells the submodule "this is a simulated sensor, not real hardware." On the real car these come from real driver nodes under different names; the bridge is what fakes the FSDS-namespaced surface in sim.

| Topic | Type | Frame | Rate | Notes |
|---|---|---|---|---|
| `/fsds/lidar/Lidar1` | `sensor_msgs/PointCloud2` | `fsds/Lidar` | 10 Hz | LiDAR point cloud — fields `x`, `y`, `z`, `intensity` (FLOAT32). Intensity follows the Hesai ATX-S01 working principle (ρ × cos(θ) × (R_ref/r)²); see `FUNCTIONALITIES.md` §4.1. |
| `/fsds/imu` | `sensor_msgs/Imu` | `fsds/IMU` | ~400 Hz | 6-DoF IMU. Consumed by `slam_node` (preintegration) AND `sim_supervisor_node` (filter prediction step) post-feat/360. |
| `/fsds/motor_rpm` | `std_msgs/Float32` | — | ~80 Hz | Drive-axle RPM. Primary longitudinal velocity input — both `slam_node` (velocity prior) and `sim_supervisor_node` (filter correction step) consume it. The IFS-08 doesn't have GSS, so RPM + IMU + steering + brake_pressure is the full real-car odometry input set. |
| `/fsds/steering_angle` | `std_msgs/Float32` | — | ~100 Hz | **Phase 3 (#383).** Front-wheel angle in radians, converted in the bridge from the plugin's normalized [-1, 1] axis input via `max_steering_angle_rad` (default 0.5). Consumed by `sim_supervisor_node` for the kinematic-bicycle yaw cross-check (`ω_pred = (vx/L)·tan(δ)`); residual published on `/odom_diag/yaw_residual_rad_s`. |
| `/fsds/brake_pressure` | `std_msgs/Float32` | — | ~100 Hz | **Phase 3 (#383).** Commanded brake authority [0, 1]. Echoed from `SensorFrame.brake` (the controls echo back from UE5). Consumed by `sim_supervisor_node` to scale the OdometryFilter's `α_vx` toward zero during brake events (drive wheels potentially locked → RPM unreliable). Effective α published on `/odom_diag/effective_alpha_vx`. |
| `/fsds/gss` | `geometry_msgs/TwistWithCovarianceStamped` | `fsds/GSS` | ~100 Hz | **Published but not consumed.** The IFS-08 has no ground-speed sensor; the bridge keeps the topic for backward-compat with code that hasn't migrated yet. New consumers must not depend on it. |
| `/fsds/gps` | `sensor_msgs/NavSatFix` | `fsds/GPS` | ~10 Hz | **Published but not consumed.** Reserved for future global-localisation work (e.g. GPS-aligned `map` frame); no autonomy node subscribes today. |
| `/fsds/testing_only/odom` | `nav_msgs/Odometry` | `odom` (child `base_link`) | ~80 Hz | **Diagnostic only.** Ground-truth pose + clean body-frame velocity. Autonomy must not consume it on the production path; consumed only by the GT-as-SLAM diagnostic (`pipeline/cone_slam/scripts/gt_pose_relay.py`) and `slam_node`'s GT-aligned residual publisher (`/cone_slam/gt_aligned`, `/cone_slam/gt_error_m`). |

### Topics the submodule publishes back (sim_supervisor → bridge)

The submodule's autonomy stack does **not** publish actuator commands directly to the bridge. Commands flow through `mission_control_node` and `sim_supervisor_node` first, so the sim path matches the real-car `DVPC → uDV (microROS over USB CDC)` chain. The bridge only ever sees commands from `sim_supervisor_node`.

| Topic | Type | Publisher | Notes |
|---|---|---|---|
| `/fsds/control_command` | `fs_msgs/ControlCommand` | `sim_supervisor_node` | Throttle / regen / steering. The bridge subscribes; `control_node` does **not** publish here directly — its output flows through `/ctrl/cmd_internal` → RuntimeControl action → supervisor (#384). |
| `/signal/ebs` (latched) | `std_msgs/Empty` | `sim_supervisor_node` | EBS trigger. Latched on the rising edge of RuntimeControl `Feedback.emergency`. Post-#384 the autonomy never publishes here directly. |
| `/signal/ebs_reset` (latched) | `std_msgs/Empty` | `sim_supervisor_node` | EBS reset on supervisor activate (mirrors uDV power-up). Post-#384 the autonomy never publishes here directly. |

The bridge tolerates startup-time absence of `/fsds/control_command` until Phase 1 of the runtime action protocol reports `ready` — actuator commands only start flowing after the autonomy lifecycle has fully come up.

### Autonomy-internal actuator topics (control_node → mission_control_node)

These topics live inside the autonomy submodule and exist to drive the RuntimeControl action's Feedback frames. The supervisor never subscribes to them directly — `mission_control_node` aggregates them and surfaces them as RuntimeControl `Feedback` per tick.

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/ctrl/cmd_internal` | `fs_msgs/ControlCommand` | `control_node` | `mission_control_node` | Per-tick throttle / steering / brake from the controller, 40 Hz. Renamed from `/control_command` in #384 so the bridge can never accidentally re-subscribe to the autonomy's pre-aggregation source. |
| `/ctrl/emergency` (latched) | `std_msgs/Bool` | `control_node` | `mission_control_node` | EBS request. `mission_control_node` surfaces a rising edge as `Feedback.emergency=true`; the supervisor then latches `/signal/ebs`. Default-false latched at controller activate so a late mcn subscription sees a defined state. |
| `/slam/finished` (latched) | `std_msgs/Bool` | `slam_node` | `mission_control_node` | Mission-completion signal. Surfaced as `Feedback.finished=true`; mission_control then terminates the RuntimeControl action with `outcome="finished"`. Currently a stub (always false) — wiring the actual lap-min-distance + big-orange detector to flip it is a follow-up. |

### Runtime action protocol (sim_supervisor ↔ mission_control)

The control loop is a two-phase ROS Action exchange between `sim_supervisor_node` and `mission_control_node`:

**Phase 1 — startup.** The supervisor sends an Action goal carrying the chosen mission (`trackdrive`, `autocross`, `accel`, `skidpad`). The mission controller drives `mode_manager` to bring up the right lifecycle nodes with the right strategy flag, sends periodic heartbeats back to the supervisor (so a crashed startup is detectable), then reports `ready` or `failed`. The startup window is also where Numba JIT compile for the planner runs — designed deliberately to absorb the first-call compile delay that would otherwise let the autonomy enter the runtime phase before the planner is ready.

If the operator changes the mission during this window (e.g. switches accel → skidpad), the in-flight startup is cancellable and the system can re-enter Phase 1 for the new mission without a full restart.

**Phase 2 — runtime.** Once Phase 1 reports `ready`, the supervisor opens a `RuntimeControl` Action against `mission_control_node` and `mission_control_node` streams `Feedback` frames back at 40 Hz, sourced from:

- `throttle`, `steering` — `/ctrl/cmd_internal` (`fs_msgs/ControlCommand`) from `control_node`.
- `emergency` — latched `/ctrl/emergency` (`std_msgs/Bool`) from `control_node` (any node detecting an unrecoverable state would publish here).
- `finished` — latched `/slam/finished` (`std_msgs/Bool`) from `slam_node` (e.g. on big-orange detection past the lap-min-distance gate).

The supervisor's `RuntimeControl` feedback callback republishes each frame onto `/fsds/control_command` for the bridge to forward to FSDS, and latches `/signal/ebs` on the rising edge of `Feedback.emergency`. The action terminates with `outcome="finished"` / `"emergency"` / `"cancelled"`; the cancelled path is what runs when the operator presses Stop Session in Mission Control (the web backend sends StartMission with mission=""). On the real car, the same Action runs between `mission_control_node` and the physical uDV via microROS over USB CDC, and `sim_supervisor_node` doesn't run.

### Mission-control-web → mission management

The Mission Control FastAPI backend calls into the supervisor's Action interface to drive the lifecycle, and into the bridge's JSON-RPC for sim-side actions:

| Surface | Type | Source | Target | Purpose |
|---|---|---|---|---|
| Start mission | `Action StartMission` (rclpy client) | `mission_control_backend` | `sim_supervisor_node` | Kicks off Phase 1; supervisor relays to `mission_control_node`. |
| Track load, sim pause/resume, RES | JSON-RPC | `mission_control_backend` | `ifssim_bridge` | Sim-only sandbox controls. |

Bridge JSON-RPC (track load, sim pause/resume, sim-side RES, sensor probe) stays on the IFSSIM side. Sim-only commands belong on the IFSSIM side; mission state belongs on the submodule side via the supervisor.

**Lifecycle orchestration mechanism.** The submodule's nodes start with the container in the standard ROS 2 `unconfigured` lifecycle state — `docker compose up` brings the autonomy launch up, no flag-file watcher, no `subprocess.Popen` from the backend. `mission_control_backend` never spawns or kills processes; all transitions are driven by `StartMission` Action calls into `sim_supervisor_node`, which fans out `change_state` services through `mode_manager_node` to each lifecycle node. This mirrors how the real car works (processes always running under systemd-equivalent; the physical uDV — a microROS endpoint over USB CDC — sources `GO`/`FINISHED` into the same lifecycle services that the simulator-side supervisor feeds in sim). Stopping a session is `deactivate` + `cleanup`, not `kill`.

## TF tree

```mermaid
flowchart LR
    classDef dyn fill:#2a4a2a,stroke:#7dc97d,color:#e8ffe8

    map(("map")) -- "dynamic — SLAM tick rate ~10 Hz<br/>drift correction<br/>(/tf)" --> odom(("odom"))
    odom -- "dynamic — 100 Hz<br/>dead-reckoning<br/>(/tf)" --> base(("base_link"))

    class map,odom,base dyn
```

Post-#382 (Phase 2 of the /odom split):

- **`map → odom`** is the dynamic drift-correction transform, broadcast by `slam_node` at scan rate (~10 Hz). Computed at each tick as `slam_pose ⊖ latest /odom` so the chain `map → odom → base_link` resolves to SLAM's absolute pose at the leaf, regardless of how far supervisor's dead-reckoning has drifted between SLAM ticks. Pure-Python math lives in `pipeline/cone_slam/cone_slam/tf_math.py` (`compute_map_to_odom`); see also `test/test_map_to_odom.py`.
- **`odom → base_link`** is the dynamic dead-reckoning transform, broadcast by `sim_supervisor_node` at 100 Hz from the IMU+RPM complementary filter. On the real car this is owned by the uDV firmware.
- The map-frame `slam_pose` and `odom-frame` supervisor pose are both 2D in practice (yaw-only quaternion); the chain composition stays well-conditioned across yaw-wrap.
- `base_link` is the canonical vehicle frame everywhere — the bridge already roots its sensor static TFs there (`base_link → fsds/IMU`, `base_link → fsds/Lidar`, `base_link → fsds/GPS`), and the autonomy stack uses the same name. No `fsds/FSCar` aliasing.
- The bridge does **not** publish any dynamic TF. Sensor messages carry sensor-local `frame_id`s (`fsds/IMU`, `fsds/GPS`, `fsds/Lidar`) that are not part of the live TF chain — autonomy nodes consume the sensors directly without TF lookups.
- `robot_state_publisher` + `joint_state_publisher` (in `coche_urdf`) publish URDF joint TFs at 200 Hz for visualisation and any downstream consumer that needs articulation.
- **Pre-#382 (legacy)**: `slam_node` owned both edges — `odom → base_link` dynamic at scan rate, `map → odom` static identity. /tf_static was used. Phase 2 retired both: `odom → base_link` moved to `sim_supervisor` at higher rate, `map → odom` became the dynamic drift-correction transform. /tf_static is no longer used by the autonomy stack.

## Open questions

Architectural choices still being finalised. Listed here because they have downstream consequences worth being deliberate about.

| # | Question | Owner |
|---|---|---|
| Q1 | **Where does `/odom` come from?** ✅ **Resolved across three phases.** Phase 1 (feat/360): `sim_supervisor_node` owns the `/odom` topic; filter is IMU + motor RPM only (no GSS — the IFS-08 doesn't have one). Phase 2 (feat/392): TF ownership moved — supervisor takes `odom→base_link`, slam_node publishes `map→odom` for drift correction, `/Conos` and `/Path` migrate to map frame, `/cone_slam/state` renamed to `/slam/pose`. Phase 3 (feat/394 — this PR): `/fsds/steering_angle` and `/fsds/brake_pressure` added to the bridge; filter consumes them as kinematic-bicycle yaw cross-check + brake-event α_vx scaling. Diagnostics on `/odom_diag/yaw_residual_rad_s`, `/odom_diag/slip_flag`, `/odom_diag/effective_alpha_vx`. Implementations in `pipeline/sim_supervisor/sim_supervisor/odometry.py` + `tf_math.py`. | DV pipeline |
| Q2 | **IMU consumption rate inside the OdometryFilter.** Currently subscribes at the BMI088 native rate (400 Hz, deep queue) and integrates every sample. Publish rate to `/odom` is decoupled at 100 Hz. Open question: would downsampling IMU to 100 Hz at the subscription level (matching publish rate) lose meaningful filter quality? Bias estimation during the 3 s stationary window benefits from full-rate sampling; the steady-state predict step likely doesn't need it. **Action:** quantify before tightening — bag a real drive, replay through both 400 Hz and 100 Hz versions of the filter, compare /odom-vs-GT residual. | DV pipeline |

## Diagnostic tools

Co-located with the autonomy stack rather than under `tools/`, so they're easy to find when reading the SLAM code.

- **`pipeline/cone_slam/scripts/gt_pose_relay.py`** — a standalone ROS node that replaces `slam_node` by republishing `/fsds/testing_only/odom` under the same node-name and topic contract. Diagnostic for isolating "is SLAM the bottleneck or the consumers?" without changing any consumer. The local finite-difference velocity path inside the relay is a holdover from the earlier asymmetric-twist behaviour and can be retired once the bridge fix lands; it doesn't hurt anything to leave in place as a safety net.
- **`pipeline/cone_slam/scripts/replay_slam.py`** — offline replay of a captured rosbag through the SLAM node. Deterministic reproduction for failure analysis without a running sim.
- **`SLAM_OBS` per-second log line** in `slam_node` — live obs/assoc/new/skip counters; the cleanest way to see a DA cascade in real time.
- **`tools/refresh-bridge.sh`** — full container teardown + recreate when Docker UDP wedges or DDS state goes stale on macOS.

## See also

- [`FUNCTIONALITIES.md`](FUNCTIONALITIES.md) — sim-side topics, sensors, vehicle physics, RPC.
- [`GETTING_STARTED_DOCKER.md`](GETTING_STARTED_DOCKER.md) — container layout and how to launch the stack.
- LiDAR per-point intensity follows the Hesai ATX-S01 working principle (`ρ × cos(θ) × (R_ref/r)²`), implemented in #318-style coordinated wire-format change. Per-cone-material 905 nm reflectance tuning is a follow-up content task (separate from the simulator-side principle).
