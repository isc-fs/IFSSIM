# MATLAB

Everything about the car that is not the game engine. Three live directories
and an archive.

| | what it is | who it is for |
|---|---|---|
| [`spec/`](spec/README.md) | **the car.** Every parameter, with the source it came from | everyone — this is the file you edit |
| [`vd/`](vd/README.md) | the **design model** and the vehicle-dynamics workbench — handling, balance, lap times, energy | the manual team, per department |
| [`plant/`](plant/README.md) | the **reference plant** — 6-DOF Simulink model, exported as an FMU and loaded by Unreal | simulator work, and validating `vd/` against |
| [`archive/`](archive/README.md) | superseded models kept for provenance | nobody, until you need to know where a number came from |

## The direction everything flows

```
        spec/car_spec.m          <-- the only file you edit
               |
    +----------+-----------+--------------------+
    |          |           |                    |
   vd/       plant/    settings.json      (docs, reports)
 design     Simulink    EXPORT for the
  model      + FMU      UE5 bridge and
                        the ROS tools
```

`settings.json` is **generated**, by `spec/build_car.m`. Nothing in MATLAB
reads it. It exists because the C++ bridge and the Python tools parse it, and
it is downstream of the spec exactly like the Simulink models are — so the
simulator and the plant cannot disagree about what car they are simulating.

## Start here

```matlab
cd matlab/vd
vd_report                    % what the car does: balance, grip, lap times
vd_study('TireMu', 1.65)     % what a change would do, without changing anything
```

```matlab
cd matlab/plant
ifssim_setup
ifssim_params_report         % every parameter, and where each number came from
ifssim_plant_check           % build everything and run every test
```

## Changing a number

1. Edit it in `spec/car_spec.m`. **Every parameter needs a source string** —
   the file refuses to build without one. Say where it came from: a
   measurement, a drawing, a datasheet, or an assumption.
2. Run `build_car`. It checks the car, writes `settings.json`, and can rebuild
   the plant and re-export the FMU.
3. Re-run whatever you were looking at.

Anything `ifssim_params_report` marks ASSUMED, DISPUTED or UNKNOWN is a number
nobody has measured on the IFS-08. There are more of them than is comfortable,
and the four that matter most are named at the bottom of
[`vd/README.md`](vd/README.md).
