# Sim timing: real time and slow motion

How the simulation's clock relates to the real one, and how to choose between testing the car's computer and comparing algorithms.

## Why it matters

Every frame advances the simulation by exactly 1/60 s (`bUseFixedFrameRate`, `FixedFrameRate=60` in `Config/DefaultEngine.ini`), and the physics takes one step per frame. That makes runs repeatable, but it ties the simulation's speed to the frame rate:
- **At 60 frames per second**, a simulated second takes a real second: the **real-time factor** is 1.
- **Below 60**, the simulation runs slower than real time. A PC that only draws 45 frames per second runs it at 0.75.

The pipeline runs on the simulation's clock (the bridge publishes `/clock`, every node has `use_sim_time`), but it computes in real time. So when the simulation is slow, the pipeline gets more real time per simulated second than it has on the car:

| Real-time factor | 50 ms of pipeline compute takes, in sim time |
|---|---|
| 1.0 | 50 ms |
| 0.75 | 37 ms |
| 0.45 | 23 ms |

Driving results (cones, path) hold either way. Timing results (lap times, latency, whether the car's computer keeps up) only hold near 1.

## The two modes

Chosen on the menu's Drive page (TIMING, shown with Autonomous), with `"Timing": {"SpeedFactor": ...}` in settings.json, over RPC with `setSpeedFactor <f>`, or by the mission benchmark's `--speed-factor`. The speed applies only while the pipeline has the car (API control); manual driving is always real time. Switching the menu's driver to Autonomous resets it to Real time, so slow motion is always a choice made on purpose.

| Mode | For | What the sim does |
|---|---|---|
| **Real time** (`1`, the default) | testing the car's computer, such as the LattePanda | Runs as fast as it can, up to 60 frames per second. The factor should read 1; below 0.95 the HUD line turns orange (red below 0.8), and a benchmark run records `race/real_time_ok: 0`. |
| **Slow motion** (`0.5`, `0.25`, `0.1`, or any value down to 0.05) | comparing algorithms without the PC's speed in the result | Spaces the frames out so the simulation runs at that fraction of real time: at 0.25, 15 frames per real second. The pipeline gets 4 times its real time per simulated second. |

In slow motion nothing but the pipeline's time budget changes:
- Every frame is still the same 1/60 s physics step.
- The sensors run on simulation time: the RPC sensor stream sends 400 frames per simulated second (it resends each tick's frame for the simulation time that tick advanced, so at real time it sends what it always did), the UDP broadcaster sends one per engine frame, and the LiDAR's 10 Hz is counted in game time. The bridge's two wall-timer topics (the LWS steering sensor at 100 Hz, `/tire_loads` at 20 Hz) are held to their rate in simulation time (`ros2/src/ifssim_bridge/include/sim_rate_gate.h`), so they don't repeat.
- The pipeline's 40 Hz control and its watchdog run on `/clock`, so they tick 40 times per simulated second, as on the car.

Checked with one autocross lap of `track_20260404_013721` driven by the pipeline: 173.3 s, no cones down, at real time, and 173.4 s, no cones down, at 0.25x (12 real minutes).

It isn't an exact lockstep. The pipeline gets more time, not unlimited time, and small timing differences between runs remain. If a node needs more than its slowed-down share, pick a slower speed.

## What shows it

- **The HUD:** "Real time 1.00x 60 fps", or "Slow motion 0.25x: 0.25x 15 fps", averaged over the last two seconds. The colour compares the factor with the target.
- **`getSimStatus`:** `real_time_factor`, `real_fps`, `speed_factor` (the setting) and `target_factor` (what the factor should read now).
- **The mission benchmark:** `race/real_time_factor`, `race/speed_factor` and `race/real_time_ok` in each run's `results.json` (`tools/sim_benchmark/README.md`).

## The menu's own frame rate

While the menu is up and nothing drives, the sim draws 30 frames a second (Settings, Performance: 60, 30 or 15), and 5 while its window is minimised, so it costs little in the background. That cap never applies to a run, or to an autonomous session with the menu up over it.

## Graphics: what the view costs

Settings, Graphics in the menu, kept between launches (`FFSDSGraphics`):
- **Quality:** Low (the project's own settings in `Config/DefaultEngine.ini`, every quality group low), Medium or High.
- **Resolution:** 25% (the default), 50%, 75% or 100% of the window, upscaled.
- **View while the pipeline drives:** Shown, or Hidden. Hidden stops drawing the world while the pipeline has the car (API control on and the EBS released) and the menu is closed; the HUD and the Esc box still show.
- **Find the best for this PC:** tries High at 100% first and works down (High 75%, Medium 75%, Medium 50%, Low 50%, Low 25%), a few seconds each, and keeps the first whose GPU time per frame (the 90th percentile of `RHIGetGPUFrameCycles`) stays under 12.5 ms, three quarters of a 60 fps frame. If nothing does, it picks Low at 25% with the view hidden.

**None of this changes what the LiDAR sees.** The LiDAR renders its own depth and colour images of the scene. With the car at rest on Parking lot A, its scans (points per scan, mean range, intensity) were the same, within the scatter between repeats, with the view off, at 100% resolution, and with the shadow, post-process, anti-aliasing, shading, texture and effects groups at high. The view distance, foliage and level-of-detail settings are not offered: they decide which geometry the LiDAR's own render holds.

What it costs, measured on a GTX 1660 SUPER (12-thread CPU) with the LiDAR at 1.74 M points/s, all at 60 fps and real time. The GPU reads about 13% with the sim closed; percentages don't add up exactly, since the card changes its clock with the load:

| Setup | GPU | CPU (whole PC) |
|---|---|---|
| Low, 25% (as before) | 51% | 15% |
| The view off | 38% | 6% |
| The LiDAR off | 39% | 19% |
| The LiDAR and the view off | 16% | 11% |
| Low, 100% | 72% | 15% |
| High, 100% | 85% | 14% |
| High, 50% | 66% | 15% |

So the view and the LiDAR each take about half of the sim's GPU work, and the view most of its CPU. Find the best picked High at 100% on that PC (9.6 ms of GPU per frame).

## Known limits

- The sim runs below real time for about 30 s after it starts, while it compiles shaders (#734). The HUD shows it; wait for 1.00x before a timing run.
- The pacing sleeps between frames, so the factor can dip slightly below the slow motion's value under load; the HUD and the benchmark judge it against 95% of the target.
