# IFSSIM plant model

The vehicle, as a Simulink model your engineers own. The simulator keeps terrain,
sensors, collision, the referee and the ROS bridge — **this is the car.**

---

## Start here

```matlab
cd matlab/plant
ifssim_setup          % paths, parameters, buses
ifssim_plant_check    % build everything, compile it, test it
```

`ifssim_plant_check` is the only command you need day to day. Run it after you
pull and before you commit. If it says `PLANT OK`, the model is healthy.

```
--- summary ---
  [ok  ] parameters load
  [ok  ] build
  [ok  ] all models compile
  [ok  ] chassis physics
  [ok  ] tyre/suspension physics
  [ok  ] steering physics
  [ok  ] powertrain physics

PLANT OK   (46 s)
```

Then open one of two models:

```matlab
open_system('IFSSIM_Plant')    % the car: six subsystems, wired
open_system('IFSSIM_Drive')    % press Run and watch it drive
```

`IFSSIM_Drive` is the one to keep open while tuning. It drives a scripted
manoeuvre — accelerate, turn, straighten, brake — with scopes on speed, rates,
position, wheel speed, tyre load, slip, and motor RPM and power. The manoeuvre
is four lines in the `Driver` block; edit `build_drive_harness.m` to change it.

> **Always run `ifssim_setup` before opening a model.** Without it the bus
> objects are missing, ports go red, and the errors suggest someone committed a
> broken model. They didn't — the workspace is just empty.

---

## Who owns what

| Model | Owner | Status |
|---|---|---|
| `IFSSIM_Chassis` | dynamics | implemented — 6-DOF rigid body |
| `IFSSIM_TireSuspension` | dynamics | implemented — Pacejka + suspension |
| `IFSSIM_Steering` | dynamics / controls | implemented — Ackermann |
| `IFSSIM_Powertrain` | powertrain | implemented — motor, drivetrain, battery |
| `IFSSIM_Brakes` | braking | implemented — EBS, all four corners |
| `IFSSIM_Aero` | aero | implemented — body-frame drag and downforce |

Each is a **separate referenced model**, so you can work on yours while somebody
else works on theirs. `.slx` is binary and does not merge — one shared model
would mean one person at a time.

---

## I want to change something

### …a number (mass, tyre mu, gear ratio, spring rate)

Edit **`settings.json` at the repo root**, not the model. Then:

```matlab
ifssim_params_report    % confirm it took, and see where every value came from
```

The simulator reads the same file, so this is what keeps the Simulink plant and
the running sim describing the same car.

**Never type a number into a block.** Read it from `IFSSIM_P`. This project has
already paid for the alternative twice — see *Why parameters work this way* below.

### …the physics inside a subsystem

1. Open the model: `open_system('IFSSIM_TireSuspension')`
2. Edit the MATLAB Function block, or replace it with whatever blocks you prefer
3. `ifssim_plant_check` — did anything break?
4. Add an assertion to the matching `test_*_physics.m` covering what you changed

**But note:** subsystems with a `build_*.m` script are *regenerated* by
`ifssim_plant_build`, which will overwrite hand edits. Put your change in the
builder script. That is deliberate — a `.slx` cannot be reviewed in a pull
request, and a model nobody can review is a model nobody can trust.

### …a placeholder into a real model (Brakes, Aero)

Copy the pattern from `build_steering.m` — it is the smallest complete example.
Then register it in two places:

* `ifssim_plant_build.m` → add your builder to the `builders` list
* `ifssim_plant_check.m` → add your test to the `tests` list

### …the port interface

Edit `ifssim_plant_buses.m`. Then tell whoever owns the C++ side, because the
platform depends on those names and widths. This is the one change that is not
local to you.

---

## Rules of the contract

Inside your block, do what you like. At the ports:

* **Do not change port names, types or widths** without changing the bus
  definition and telling the platform side.
* **Fixed step, 1/960 s, no continuous states.** This model becomes an FMU, and
  an FMU with a data-dependent substep count is not reproducible. 1/960 is the
  first 60-divisible rate at which the motor current-loop time constant is
  representable at all.
* **Body frame is ISO 8855 / REP-103** — x forward, y **left**, z up. World is ENU.
* **Wheel order is FL, FR, RL, RR**, everywhere. It is already load-bearing on
  the C++ side, so changing it is a silent, symmetric, nearly undetectable bug.

---

## Three compile errors you will hit

All three cost time the first time. None of them are your maths.

**1. "Simulink is unable to determine sizes and/or types… errors in the block body"**

Usually a workspace constant that is not *declared*. A MATLAB Function block does
not pick up base-workspace variables the way a script does. Declare it:

```matlab
d = Stateflow.Data(chart); d.Name = 'IFSSIM_Mass'; d.Scope = 'Parameter';
```

See any `build_*.m` — they all do this in a loop.

**2. The same error, but caused by feedback**

If a signal comes from a Unit Delay whose input is your own output, nothing can
infer a size — the circle has no starting point. Set port sizes explicitly
(`d.Props.Array.Size`).

**3. "Error due to multiple causes"**

Simulink's least useful message. `verify_plant_skeleton` walks the nested causes
and prints the real one. Use it rather than guessing.

**Also:** passing a bus straight into a MATLAB Function block works but is
fragile. Put a **Bus Selector** outside instead — it is more robust *and* it
documents on the canvas which fields the block actually consumes.

---

## Why parameters work this way

`ifssim_params.m` reads `settings.json` and records, per field, whether the value
came from the file or fell back to a C++ default. `ifssim_params_report` prints
it. That distinction matters: *"the model uses 275 kg"* and *"the model fell back
to a 290 kg default because the file was silent"* look identical in a block
diagram and are completely different claims.

Two things this project has already lived through:

* Chaos ran **25 kN/m** per corner while the load-transfer model used
  **56.9 kN/m** — same car, two stiffnesses, 2.3× apart, because each held its
  own copy of the number.
* `matlab/IFS_Sim` (2024-25) carries mass **237 kg** and tyre radius **0.30 m**
  against `settings.json`'s **275 kg** and **0.202 m**. A 0.30 m tyre radius puts
  every speed and energy figure it ever produced ~48% out.

A retyped parameter is a divergence with a delay fuse.

`ifssim_params_report` also runs sanity checks, each of which exists because
something here has already failed it — wheel radius plausible for a 10″ wheel,
ride frequency in the 3–5 Hz band, wheel-rate ×4 equalling the declared
`HeaveStiffness`.

---

## Assumptions, and what to measure

`P.Assumed` holds everything the plant needs that `settings.json` does not carry.
It is a separate struct on purpose — you can see at a glance what is measured and
what is guessed.

| assumption | why it matters | how to fix it |
|---|---|---|
| **inertia tensor** `Ixx 30, Iyy 110, Izz 125` | `Izz` sets yaw response, which is what the controller is tuned against. A 20% error reads as a gain problem. | bifilar pendulum, or CAD mass properties |
| **wheel inertia** 0.21 kg·m² | how fast a wheel spins up or locks | CAD, or a spin-down test |
| **Ackermann fraction** 1.0 | inner/outer steer split | measure the steering arms |
| **steer actuator** effectively instant | transient steering response | bench the DV steering motor |
| **battery pack** 95s, 8.5 Ah | energy and power limits | confirm it describes *this* car, then move to `settings.json` |
| **slip regularisation** 1 m/s | below this the tyre model is not trustworthy | inherent — a launch study must say so |

**Unmeasured is not the same as wrong** — but it must be visible. Every one of
these is flagged in the model annotation too, so you meet it while reading the
block rather than after trusting a result.

---

## Also worth knowing

Three sources disagree about maximum steering angle: `settings.json` says **28°**,
the pipeline uses **18.2°**, and the real 5:1 steering ratio with a ±60° column
clamp implies about **12°**. That is a calibration question rather than a
modelling one, but it should be settled before anyone trusts a lap time.

`matlab/IFS_Sim/` is the 2024-25 drive-cycle model, kept for reference. It is
built on **Simscape Driveline**, which is **not licensed** on this account, so it
cannot simulate as-is. Its architecture and battery parameters were worth
harvesting; its Driveline blocks are not. **Vehicle Dynamics Blockset** and
**Powertrain Blockset** *are* licensed if you want richer tyre or motor models.
