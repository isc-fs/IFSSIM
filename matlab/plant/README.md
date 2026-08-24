# IFSSIM plant model (Simulink)

The vehicle plant, as a Simulink model your engineers own. The simulator keeps
terrain, sensors, collision, the referee and the ROS bridge; **this** is the car.

## Why it is generated from a script

`build_plant_skeleton.m` writes every `.slx` here. That is deliberate:

* **`.slx` is binary — it does not diff and it does not merge.** The structure and
  the port contract therefore live in a text file that can be reviewed, while
  subsystem *contents* are owned and edited by engineers in Simulink.
* Regenerating replaces the skeleton, not your work: each subsystem is a
  **separate referenced model**, so three people can work at once without
  fighting over one file.

```matlab
addpath matlab/plant
build_plant_skeleton      % writes matlab/plant/generated/
verify_plant_skeleton     % compiles every model, reports pass/fail
```

## Who owns what

| Model | Owner | Takes | Gives |
|---|---|---|---|
| `IFSSIM_Steering` | dynamics | `Cmd` | road-wheel angles ×4 |
| `IFSSIM_Powertrain` | powertrain | `Cmd`, wheel ω | drive torque ×4, motor/battery state |
| `IFSSIM_Brakes` | braking | `Cmd`, wheel ω | brake torque ×4 |
| `IFSSIM_TireSuspension` | dynamics | road, pose, steer, torques | per-wheel state, tyre wrench |
| `IFSSIM_Aero` | aero | pose | aero force and moment |
| `IFSSIM_Chassis` | dynamics | tyre + aero wrench, env | pose, velocity, acceleration |

Numbers on the top-level blocks are the signal-flow order: commands become
torques and angles, torques and angles become tyre forces, tyre forces become
motion.

Every block starts as a **correctly-ported placeholder emitting zeros**, so the
whole model compiles and runs from day one. Nobody is blocked waiting for
someone else's subsystem.

## Parameters: settings.json is the source of truth

```matlab
P = ifssim_params();        % reads settings.json
ifssim_params_report(P)     % prints every value AND where it came from
```

**Do not type numbers into a block.** Read them from `IFSSIM_P`.

This is not style. This project has already paid for the alternative twice:

* Chaos ran **25 kN/m** per corner while the load-transfer model used
  **56.9 kN/m** — the same car, two stiffnesses, 2.3× apart, because each model
  held its own copy of the number.
* `IFS_Sim` (2024-25) carries mass **237 kg** and tyre radius **0.30 m** against
  `settings.json`'s **275 kg** and **0.202 m**. A 0.30 m tyre radius puts every
  speed and energy figure it ever produced ~48% out.

A retyped parameter is a divergence with a delay fuse. `ifssim_params_report`
prints provenance per field — `settings.json` or `default` — because "the model
uses 275 kg" and "the model fell back to a 290 kg default because the file was
silent" look identical in a block diagram and are completely different claims.

It also runs sanity checks, each of which exists because something here has
already failed it: wheel radius plausible for a 10″ wheel, ride frequency in the
3–5 Hz band, and wheel-rate ×4 equalling the declared `HeaveStiffness`.

## Rules of the contract

The port interface is what the simulator depends on. Inside your block, do what
you like.

* **Do not change port names, types or widths.** Buses are defined in
  `ifssim_plant_buses.m`; changing one is a platform-side change too.
* **Fixed step, 1/960 s, no continuous states.** This model becomes an FMU, and
  an FMU with a data-dependent substep count is not reproducible. 1/960 is the
  first 60-divisible rate at which the motor current-loop time constant is
  representable at all.
* **Body frame is ISO 8855 / REP-103** — x forward, y **left**, z up. World is ENU.
* **Wheel order is FL, FR, RL, RR**, everywhere. It is already load-bearing on
  the C++ side; changing it is a silent, symmetric, nearly undetectable bug.

## What is deliberately NOT here

`matlab/IFS_Sim/` is the 2024-25 drive-cycle model, kept for reference. It is
built on **Simscape Driveline**, which is **not licensed** on this account, so it
cannot simulate as-is. Its architecture and battery parameters are worth
harvesting; its Driveline blocks are not.

**Vehicle Dynamics Blockset** and **Powertrain Blockset** *are* licensed and are
the intended foundation for the tyre, suspension, steering and motor subsystems.
