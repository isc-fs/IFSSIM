# Vehicle dynamics workbench

For the manual team. Open MATLAB, change a number, see what the car does.
No ROS, no game engine, no Docker — just MATLAB.

```matlab
cd matlab/design
addpath(genpath(fullfile('..','plant')))

vd_report        % the full characterisation, as tables
vd_plots         % the five standard figures, also saved to figures/
```

## Trying a change

```matlab
vd_study('TireMu', 1.65)
vd_study('CoGHeight', 0.3441, 'Assumed.Izz', 200)
vd_study('RollStiffnessFront', 32000, 'RollStiffnessRear', 17000)
```

Prints the car as built beside the car with your change, and writes the study's
figures to `figures/study/`.

**Nothing you do in `vd_study` changes the car.** `settings.json` is untouched,
so the simulator, the plant and everybody else's run are unaffected. It exists
so you can ask a question without committing to the answer.

Your override goes in *before* anything is derived, so the whole chain follows
it. Raise `HeaveStiffness` and the ride frequency, the damper coefficient and
the anti-roll bar rates all move with it — you cannot accidentally produce a
car whose numbers disagree with each other.

## Making a change real

1. Edit the number in `matlab/car/car_spec.m`. **Every parameter needs a
   source string** — the file refuses to build without one. Say where the
   number came from: a measurement, a drawing, a datasheet, or an assumption.
2. `build_car` — checks the car, writes `settings.json`, so the simulator and
   the plant now describe the same car you just studied.
3. `vd_report` again.

## What each command tells you

| | |
|---|---|
| `vd_report` | understeer gradient, skid pad lap time, limit grip, transient response, and what the anti-roll bar does |
| `vd_plots` | understeer curves, tyre curves, the balance sweep, step response, corner loads |
| `vd_study` | any of the above, before and after a change |
| `vd_constant_radius(R)` | one circle, in detail |
| `vd_skidpad` | the FS event as a lap time |
| `vd_step_steer(v, delta)` | one transient |
| `ifssim_params_report` | every parameter, and **where each number came from** |
| `validate_dualtrack` | how well this model agrees with the full Simulink plant |

## The numbers to distrust first

`ifssim_params_report` marks the provenance of everything. Four numbers this
page rests on have none worth the name:

- **`CoGHeight`** is DISPUTED — 0.300 here, 0.3441 in the VD department file,
  nobody has measured the IFS-08. Load transfer is *linear* in it.
- **`RollStiffnessFront/Rear`** have no source at all, and their RATIO is the
  balance the whole report turns on.
- **`Assumed.Izz`** is a typical value, not a measurement. Only the transient
  section depends on it.
- **`TireMu`** is 1.40 from an unvalidated curve fit; the tyres department says
  1.65 at 1000 N. Try `vd_study('TireMu', 1.65)` and see what it is worth.

## What this is

The **design model**: two states, algebraic load transfer, no suspension
dynamics, speed prescribed. It is the model the vehicle-dynamics literature
builds controllers on, and it agrees with the full Simulink plant to within
0.6–8.6% of yaw rate on a ramp steer (`validate_dualtrack`).

It has **never been checked against the real car**. The tyre is fitted to no
data at all. Treat everything here as what the car *as described* would do.
