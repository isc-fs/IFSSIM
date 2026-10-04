# Tyres

The tyre department's view of the car, built the same way as `vd/`, `pt/` and
`aero/`.

```matlab
cd matlab/tyres
tyre_parameters   % what each parameter moves, and which are inert on purpose
tyre_report       % the curve, the car on it, and the TireMu dispute
tyre_plots        % Fy against slip angle; peak mu and stiffness against load
tyre_study('Tyre.LoadSensitivity', -0.4)
```

`plant/tyre_report` is a different tool. It sweeps the plant's actual Simulink
tyre block and checks it against the file. This folder asks what the curve
does to the car.

**Nobody has put this tyre on a rig.** Every coefficient is a fit or an
assumption, so read the tables as "which guess matters most".

## What the guesses are worth

| question | lap | skid pad |
|---|---|---|
| `TireMu` 1.40 (car) or 1.65 (tyres department) | 66.15 → 61.24 s | 5.19 → 4.78 s |
| `Tyre.LoadSensitivity` −0.15 or −0.4 (likelier for a slick) | 66.15 → 69.32 s | 5.19 → 5.42 s |

Load sensitivity is the dangerous one. The mild value the car runs
under-charges every newton of load transfer, so it flatters the car.

## Two levels, and why both

- **The lap engine sees only the peak of the curve.** `TireMu`, load
  sensitivity and nominal load move the lap.
- **The shape (`Pacejka.Lat*`) reaches the car through the understeer
  gradient.** The design model computes it on the full curve, taken from two
  steady-state trims at 12 m/s. A shape change that moves K and not the lap
  is a handling change, not a speed change.
- **The longitudinal shape (`Pacejka.Lon*`) reaches the 75 m time,** through
  slip on the power limit (`pt_model`).
- **Camber reaches the lap through roll.** The lap engine charges the design
  model's camber penalty: the share of body roll the geometry does not recover
  tilts the tyres and costs peak grip. It was left out at first and did not
  matter while camber gain was assumed to be 0.80. With the IFS-08's real
  0.125 at the front it costs 0.37 s a lap, and without it the skid pad drifted
  1.3% from the full design model.

`Tyre.Pressure`, `Tyre.RimWidth` and `Tyre.RefVelocity` are inert on purpose:
the terms they feed are zeroed or switched off. `tyre_parameters` checks that
they still do nothing and warns if one starts to move something.

## The shortcuts, and their checks

`tyre_kpis` takes about 1.5 s because it replaces two 50-second design-model
tests. `test_tyre_kpis` holds each shortcut to the test it replaces:

| | fast | slow |
|---|---|---|
| understeer gradient, two trims against an R = 60 m sweep | −0.357 deg/g | within 0.02 deg/g |
| skid pad, lap engine against `vd_skidpad` | 5.190 s | 5.223 s |

The test also checks that cornering stiffness at nominal load is exactly the
declared value, since `PKY1` is solved to put it there.
