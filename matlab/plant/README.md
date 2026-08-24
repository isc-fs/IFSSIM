# IFSSIM plant model (Simulink)

The vehicle plant, as a Simulink model your engineers own. The simulator keeps
terrain, sensors, collision, the referee and the ROS bridge; **this** is the car.

## Why it is generated from a script

`build_plant_skeleton.m` writes every `.slx` in `models/`. That is deliberate:

* **`.slx` is binary — it does not diff and it does not merge.** The structure and
  the port contract therefore live in a text file that can be reviewed, while
  subsystem *contents* are owned and edited by engineers in Simulink.
* Regenerating replaces the skeleton, not your work: each subsystem is a
  **separate referenced model**, so three people can work at once without
  fighting over one file.

```matlab
addpath matlab/plant
build_plant_skeleton      % writes matlab/plant/models/
verify_plant_skeleton     % compiles every model, reports pass/fail
```

The folder is `models/`, not `generated/`. The skeleton is generated; the models
are **not disposable**. Once an engineer fills in a subsystem, that file is the
work — regenerating rewrites the top-level wiring and any block still holding a
placeholder, and a folder called `generated` invites someone to delete it.

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

## Chassis is filled in

`IFSSIM_Chassis` is implemented as the worked example — 6-DOF Newton-Euler in the
body frame, forward Euler at 1/960 s. State lives in Unit Delay blocks so it is
visible on the canvas; the equations live in one MATLAB Function block, because
Newton-Euler as sixty primitives is the same six lines with worse typography and
forty chances to mis-wire a signal.

```matlab
build_chassis            % regenerate it
test_chassis_physics     % run it against closed-form answers
```

Two things it does deliberately, both of which are easy to get wrong and both of
which are live bugs in the current simulator:

* **`accel_proper` excludes gravity.** It is what an accelerometer reads. The
  simulator today finite-differences a world velocity and adds +g — which is
  where the EKF's missing Coriolis terms came from.
* **The Coriolis term `-omega x v` is present.** Body-frame rates are not
  inertial. Dropping it is exactly the bug that made lateral velocity drift
  during sustained cornering.

`test_chassis_physics` checks each against a closed-form answer, including the
one that catches sign and frame errors: **an accelerometer in free fall must
read zero.**

### Inertia is an assumption

`settings.json` has no inertia tensor, and UE only scales whatever its physics
asset computes — so there is nothing authoritative to read. `P.Assumed.Ixx/Iyy/Izz`
are typical FS values, **not measured for this car**.

`Izz` sets yaw response, which is precisely what the controller is tuned against.
A 20% error there will present as a controller gain problem. Measure it (bifilar
pendulum, or CAD mass properties) and move it into `settings.json`.

## TireSuspension is filled in

Per-wheel suspension and Pacejka tyre forces, summed into a body-frame wrench.

```matlab
build_tiresuspension
test_tiresuspension_physics
```

**Wheel speed is a real integrated state.** That is the headline. Chaos snaps
wheel speed to ground speed, which makes longitudinal slip structurally
unrepresentable and leaves `/motor_rpm` as chassis speed round-tripped through a
gear ratio. Here a wheel spins up from torque and can genuinely lock or slip —
the test confirms a free wheel dropped onto ground moving at 10 m/s accelerates
until its slip ratio reaches zero, settling at exactly `vx/Rw`.

Simplified deliberately, and stated so nobody assumes otherwise:

* **Quasi-static suspension** — spring and damper between the body corner and the
  road, no unsprung-mass DOF. Costs wheel-hop fidelity over kerbs, saves four
  stiff states.
* **Magic Formula with a friction ellipse** — no relaxation length, camber
  thrust, load-sensitive mu or thermal model. Transient lateral response is
  slightly quick.
* **Slip divides by `max(|vx|, 1 m/s)`.** Below that the tyre model is not to be
  trusted, so a launch-from-rest study must say so rather than quietly believing
  the number.

`Fz` is clamped at zero — a tyre cannot pull on the road — which is what lets an
inside wheel lift in a corner instead of inventing negative grip.

### A test that was wrong twice, worth reading before writing your own

The naive check "at a large slip angle, lateral force approaches mu·m·g" fails,
and both reasons are the model being right:

1. With `omega = 0` and the body at speed the wheels are **locked**, so the
   friction budget goes longitudinally, not laterally.
2. Left long enough they **spin up** and the slip ratio returns to zero.

And this coefficient set peaks near **5°** of slip angle, so at 45° the Magic
Formula is already down to about a quarter of peak. Assert invariants — the
friction ellipse holds, force opposes slip — not a guessed operating point.

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
