# DVPC bench HIL-replay — session handoff (2026-07-13)

Honest state of the bench hardware-in-the-loop rosbag-replay work on the real
IFS-08 car (DVPC `isc@192.168.2.2`). **Read the "Verified vs Assumed" split
carefully — a lot of this session's changes are educated guesses that have not
been validated against the physical car.**

---

## 0. What the bench does

Replay a sim-recorded rosbag's **sensors** (`/imu` + `/lidar_points`, re-stamped
to now) into the real car with the **wheels on stands**, motor + steering +
steering-sensor live. Goal: reproduce what the sim did and validate the autonomy
+ actuation stack without a track. The sim bag's pipeline output (path, cones,
odom, GT) is the reference; the car re-runs the pipeline on the same sensors.

**Bottom line after this session: the car has NOT completed a single full replay,
and the core steering question (do the wheels go where we command?) is still
unmeasured.** See §4 and §6.

---

## 1. Operating the DVPC (all still true)

- **SSH:** `sshpass -p isc ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password -o NumberOfPasswordPrompts=1 isc@192.168.2.2`. `sshpass` is on the Mac. The link drops often — retry. Avoid `()` in remote `bash -lc` and nested heredocs; write scripts locally + `scp`.
- **Pipeline on car:** `/home/isc/dv_ws/src/IFS08-DV-PIPELINE` @ `prerun/rosbag-onboard`. Auto-update is **OFFLINE**, so local source edits **survive power cycles** (confirmed). `dv restart` reloads; `dv race` brings the pipeline up after a boot (umbilical holds it at boot).
- **Build on car:** `colcon build --base-paths src --packages-select <pkg> --symlink-install`. **Must** pass `--base-paths src` — an `install.staging/` tree otherwise causes a "Duplicate package names" error. Python pkgs are symlink-installed (edit = live after `dv restart`); C++ (`odometry_filter`) needs a real compile.
- **Live params:** `ros2 param get` only works with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (the pipeline runs cyclonedds; a default-RMW shell silently returns nothing). Topics are visible on the default RMW, param *services* are not.
- **Bag recovery:** a hard-killed recorder leaves corrupt metadata (`Exception on parsing info file: invalid node; first invalid key: "version"`). Fix: `rm <bag>/metadata.yaml && ros2 bag reindex <bag> -s mcap`.
- **Foxglove bridge:** `systemd-run --user --unit=foxglove -p Restart=always -p RestartSec=2 --setenv=HOME=/home/isc /home/isc/foxglove_launch.sh`. `foxglove_launch.sh` **must** source `ros2_ws/install/local_setup.bash` + `dv_ws/install/local_setup.bash` **and** set `RMW=rmw_cyclonedds_cpp` to see the pipeline graph. `bench_replay.sh` now auto-launches it (STEP 0). ⚠ **Known unresolved:** `fs_msgs` schema still doesn't resolve in the bridge, so `/ctrl/cmd_internal` won't decode in Lichtblick — but `/ctrl/cmd` (Twist) does and carries the same command.
- **Local mcap analysis (no ROS on Mac):** `pip install mcap mcap-ros2-support`; `mcap_ros2.reader.read_ros2_messages`. Note `m.log_time` is a **datetime** — use `.timestamp()`, not `/1e9`. Template: `tools/long_tuning/extract_bench.py`.
- **Bench helper scripts on `/home/isc/`:** `bench_replay.sh` (READY→play bag→GO; TTY-interactive; operator does the physical arming), `foxglove_launch.sh`, `read_debug.py`, `read_status.py`, `steer_focus.py`, `drive_win.py`.

---

## 2. VERIFIED this session (measured / data-grounded — trust these)

- **Heartbeat blocker was real and is fixed.** The recurring `AS→EMERGENCY / emerg:dv_lost_hb` was micro-ROS DVPC↔uDV link congestion collapse (single FreeRTOS thread, blocking single-in-flight USB TX starving the reliable `/dv/status` ACKs). Fixed by uDV **#1 best-effort `/dv/status` reader + #3 decouple RX from blocking TX + #5 RX-ring wraparound one-liner** (issue `IFS08-DV-uDV#166`). Result: `dv_age` stays 0 ms (no cliff), car drove **~50 s** (autocross) and **~12 s** (hairpin) in DRIVING. This is solid.
- **Freewheel over-read is real (stands artifact).** On stands the drivetrain is unloaded, so `odometry_filter`'s rpm→speed makes `/odom.vx` over-read **~2.7×** (hairpin peak **8.7 m/s** vs GT 3.25; autocross hit 26.5). Data-verified across bags with identical sensor input. It is the root of the longitudinal misbehavior on stands (see §4).
- **Deadband/throttle collapse.** `deadband_v(0.2) == throttle_max(0.2)` made throttle **binary {0, 0.2}** — no proportional region → chatter + glitch spikes (exactly the "spikes/intermittent throttle" symptom). Found with the offline PI harness (`tools/long_tuning/long_harness.py`). Fix: `deadband_v → 0.05`.
- **Stanley de-tune holds.** `k=2.0, k_yaw_rate=0.5` → **0** command sign-flips over a 50 s run; the old ±1 bang-bang limit cycle is gone.
- **The bench replay never completes** (see §4 for the honest, mcap-only cause analysis).

## 2b. Car-side live edits (offline-patched, built, `dv restart`ed; survive power cycles)

| Param / const | Value | File |
|---|---|---|
| `throttle_max` | **0.2** | control_node.py |
| `deadband_v` | **0.05** | control_node.py |
| `max_steer_deg` | **18.2** ⚠(assumed, see §3) | control_node.py |
| `kRpmToMs` | **0.00727** (wheel r=0.202) | odometry_filter.hpp |
| `RPM_TO_MS` | **0.00727** | cone_graph_slam_node.py |
| Stanley `k / k_yaw_rate` | **2.0 / 0.5** | control_node.py |
| regen relay | `linear.x = throttle − brake` | mission_control_node.py |

---

## 3. ASSUMED / UNVALIDATED (the honest caveats — do NOT treat as fact)

- **The entire steering-ratio chain is from a spreadsheet, never physically measured.** `STEERING_RATIO=5.0` (360° column → 72.1° wheel), `MOTOR_TO_COLUMN=1.1`, rack `87.9 mm/rev`, column limit `±100°`, road-wheel ceiling `18.2°`. **No one has put a gauge on a wheel.** uDV `#172` and pipeline `#61/#63` (`max_steer_deg 28→18.2`) all rest on these numbers. They may be right, wrong, or half-right — the **`1.1`-inside-`REDUCTORA` question alone flips the ceiling between 18.2° and 20.0°**, and we shipped 18.2.
- **The LWS-vs-stepper "divergence" was never actually diagnosed.** On the hairpin the LWS `actual` (column) ran to the rail while the stepper `motor` tracked the small `target`. A ratio explanation was layered on, but the adversarial-verify pass showed SLAM tracked the hairpin fine and several of the "divergence" sub-claims (res/status timing, "perception died at 11.8 s") were **wrong**. We do not know if that divergence was a ratio issue, mechanical slip, an LWS fault, or trace mis-reading.
- **`max_steer_deg=18.2` is live on the car but unproven.** It only makes `norm=1` mean a real angle *if* the uDV `#172` numbers are correct. It is currently paired with a flashed `#172`.

---

## 4. Why the bench replay never completes (mcap-only, TS-hypothesis excluded)

Multi-agent analysis of the hairpin sim bag (completed 38 s clean) vs the bench
output (died 16.8 s), restrained to the data:

- **Freewheel over-read → longitudinal limit cycle.** `/odom.vx` over-reads (peak 8.7) → the longitudinal loop closes on a phantom overspeed → commands **full regen** (`linx=−1.0` on 203/490 ticks; the loaded sim brakes **0/1397 ever**) → unloaded wheels **stop↔spin** the whole drive, never cruising. *This is a stands artifact — it will not happen on the loaded floor.*
- **Pipeline is late + starved.** Nodes activate 4.6 s, first `/odom` 7.7 s, first `/Path` 8.2 s (sim: 1.4 / 2.4 s). `cone_detection` runs **3.7 Hz** on the car vs **10 Hz** in sim → sparse/late path. Would block a clean lap on its own. *Suspect the `feat/9` height-crop + `max_input_points=40k` cap is not active in this build.*
- **SLAM is fine.** The prime "over-read corrupts SLAM" hypothesis was **refuted** — `/slam/pose` tracks the hairpin (yaw 0→−75°), every `/Path` carries 24 poses. SLAM is accurate-but-sparse, not divergent.
- **The 16.8 s emergency itself is not mcap-derivable.** It coincides with `TS:off` during a spin-up, but a *larger* earlier spin-up (8.7 m/s) was survived, and all pipeline topics were still publishing at the trip. The trigger is firmware/ECU-side and out of scope for the bags.

**So: to get one full replay, fix the velocity source (loaded floor, or feed a
load-independent `v_meas`), and restore perception throughput. Do NOT retune
control gains (correct given its `v_meas`).**

---

## 5. PRs / issues opened this session

**Pipeline (`isc-fs/IFS08-DV-PIPELINE`):** ⚠ **`main` is the integration branch here** (default; `dev` is stale, `dev ⊆ main`). I mis-targeted these at `dev` (wrong) — they reached `main` anyway because `dev` merges up, but **future pipeline PRs go to `--base main`**. The bench branch `prerun/rosbag-onboard` is stale (26 behind main); **the DVPC should pull `main`**, which already has all of the below **plus the team's `cone_detection` perf fixes #70–#74 (the 3.7 Hz starvation fix)**.
- `#50` (→dev→main) — `throttle_max 0.6→0.2`; prerun mirror `#52` still open (close — moot once car pulls main)
- `#54` — wheel radius 0.228→0.202, `kRpmToMs 0.00821→0.00727` (Option B); prerun mirror `#56`
- `#58` — pipeline-side steering ratio — **CLOSED** (moved to uDV)
- `#61` — `max_steer_deg 28→18.2` (closes `#59`) ⚠assumed; prerun mirror `#63` — **hold** (unvalidated 18.2)
- `deadband_v→0.05` is on `main` too. (earlier) `#43`/`#44` regen; `#41` Stanley de-tune.
- Open, need a human: `#39` (team's `/slam/finished` lap detector, base dev→retarget main), `#29` (brake-relay, base main). `#47` draft QoS on dev — decide now that #166 is resolved.

**uDV (`isc-fs/IFS08-DV-uDV @ prerun/rosbag-onboard`):**
- `#172` — steering kinematics `grados = norm × 18.2 × 5.0 × 1.1`, clamp ±100. **Flashed.** ⚠assumed numbers. The uDV team pushed the clamp+sign+ratio commits; **do not push to that branch** (I force-clobbered + restored it once — a caution).
- `#165` (`/debug` emergency-reason instrumentation), `#166` (heartbeat congestion issue).

---

## 6. Open problems / honest next steps (priority order)

1. **MEASURE the wheel angle before trusting any steering PR.** Put a digital angle gauge on one front wheel, sweep known road-wheel angles, read the true command→wheel curve. This one cheap measurement tells us whether 5.5 is right, whether the range is ~18° or ~20°, and whether the "divergence" is mechanical or a sensor lie. **This should have preceded #172/#61.** Toolkit is built: `tools/steering_check/` (`steer_staircase.py`, `steer_verify.py`, `README.md`).
2. **uDV needs a steer-test actuation mode** to validate `#172`: forward a normalized `/ctrl/cmd` steer through `steer_norm_to_grados` → `0x521` **outside DRIVING, torque-inhibited**. The existing inspection sweep bypasses `#172` (uses `send_steer_angle` directly) so it validates the *mechanism* (stages 2–4) but not the *ratio math* (stage 1).
3. **Longitudinal: test on the loaded floor**, not stands — the freewheel poisons `v_meas`. (User is doing this.)
4. **Perception throughput:** confirm the `cone_detection` point-cap (`feat/9`) is active on the car build; pre-warm the ~3 s SLAM IMU calibration during READY. Needed for a clean *complete* lap regardless of the emergency.
5. **Unresolved:** the `1.1`-in-`REDUCTORA` question (18.2 vs 20.0); the `fs_msgs` foxglove schema; the root of the 16.8 s AS emergency (firmware-side).

---

## 7. Tooling built (in-repo)

- `tools/long_tuning/extract_bench.py` — standalone mcap→CSV (no ROS), steering + longitudinal channels.
- `tools/long_tuning/long_harness.py` — offline PI tuning harness (imports the real `PIVelocity`, drives it against a plant from `settings.json`); found the deadband collapse.
- `tools/long_tuning/gen_steering_html.py` + `steering_divergence.html` — the steering-divergence diagnostic chart.
- `tools/steering_check/` — the lateral verification toolkit (§6.1): `steer_staircase.py` (command a known-angle staircase), `steer_verify.py` (per-stage log checks + command-vs-physical slope/offset, auto-detects holds), `README.md`, `gauge_template.csv`.

---

## 8. Reference

Full per-topic knowledge is in the auto-memory:
`project_bench_rosbag_replay`, `project_steering_ratio_kinematics`,
`project_wheel_radius_calibration`, `project_longitudinal_robustness`,
`project_regen_never_applied`, `project_diagnosis_vy_drift_root_cause`.
Repos: pipeline is a submodule (`pipeline/`, edits → `IFS08-DV-PIPELINE`, base `dev`);
uDV `IFS08-DV-uDV`; steering ECU `IFS08-DV-STEERING` (local clone `~/Documents/Github/IFS08-DV-STEERING`, `Ir_a_Grados(grados=column)`, `REDUCTORA=20`, clamp ±60 default / ±100 calibrated).
