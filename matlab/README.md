# MATLAB models

Two models live here, and only one of them is wired into the simulator.

| | what it is | status |
|---|---|---|
| [`plant/`](plant/README.md) | the **vehicle dynamics plant** — tyres, suspension, steering, powertrain, aero, brakes | **live.** Exported as an FMU and loaded by Unreal at runtime |
| [`IFS_Sim/`](IFS_Sim/README.md) | a 2024–25 longitudinal **drive-cycle / energy** model | superseded, kept for provenance and its drive cycles |

They are not two versions of the same thing. `plant/` answers *how does
the car respond to a command* — a 6-DOF handling model. `IFS_Sim/`
answers *how much energy does a lap take* — a longitudinal model with a
battery and a drive cycle, and no lateral dynamics at all.

## If you are looking for the car's numbers

`settings.json` at the repo root is the single source for every
parameter the simulator uses. `plant/ifssim_params.m` reads it and
reports where each value came from:

```matlab
cd matlab/plant
ifssim_setup
ifssim_params_report    % every parameter, and whether it is measured,
                        % defaulted, or an outright assumption
```

Anything it marks `ASSUMPTION` is a number nobody has measured on the
IFS-08. There are more of them than is comfortable — see the
"Assumptions, and what to measure" section of
[`plant/README.md`](plant/README.md).

## A model that is NOT here

`docs/MODEL_IFS_08/` in the working tree holds spreadsheets and a
Simscape example. **None of it is in version control**, and one of the
two subdirectories is a MathWorks tutorial download rather than an ISC
model. `ISC_IFS_08.xlsx` in there is cited as authoritative by the
steering TODO in `ros2/src/ifssim_bridge/include/ifssim_ros_wrapper.h`,
which means a contested parameter's source of truth is a file git has
never seen. Worth fixing; not fixed here.
