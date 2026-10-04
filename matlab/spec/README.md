# The car

**Edit your car's file in `cars/`. Run `build_car`. That is the whole workflow.**

You do not need to know how the simulator works to change the car. You need
to know which number you want to change.

```matlab
cd matlab/spec
build_car          % check the car, write settings.json
build_car('plant') % ... and rebuild the Simulink plant
build_car('all')   % ... and export the FMU the UE5 simulator loads
```

## Why it works this way

`settings.json` used to be edited by hand, and MATLAB read it. That meant a
number could be changed in one place and quietly disagree with the model
built from it. It is how the wheelbase came to be 57 mm longer than the
measured distance between the wheel centres, and stayed that way.

So the direction is now one-way:

```
   cars/ifs08.m ──►  check_car  ──►  settings.json   (UE5 reads this)
   (you edit)                  └──►  Simulink plant ──► FMU
```

`settings.json` is an **output**. Editing the physics values in it does
nothing lasting — the next build overwrites them, and until then the plant
and the file disagree, which is the exact failure this exists to prevent.
The sensor and mission settings further down that file are *not* generated
and are still edited there.

## Every value has to say where it came from

A car file will not let you add a parameter without a source. Use one of:

| | meaning |
|---|---|
| `MEASURED` | somebody put an instrument on this car. Say what and when. |
| `GEOMETRY` | read off the IFS-08 hardpoint table. Say which points. |
| `DERIVED` | computed from other entries here. Say the formula. |
| `ASSUMED` | a typical value for a car like this. Nobody measured it. |
| `DISPUTED` | sources disagree. Say who says what. |
| `UNKNOWN` | nobody knows. **This is a legitimate and useful answer.** |
| `DESIGN` | a target for a car not built yet. Say who set it and when. |
| `INHERITED` | written for you: carried over from the parent car, not confirmed for this one. |

`UNKNOWN` is not a failure state. Six values are `UNKNOWN` today and saying
so out loud is worth more than a confident number nobody can defend. Every
build prints the tally.

## What the checks do, and do not do

`check_car` compares values against *each other*. It cannot tell you a number
is right — only the car can do that — but it can tell you two numbers cannot
both be right. Every error found in the August 2026 audit was of that kind:

- a wheelbase that disagreed with the wheel centres
- a suspension stiffness implying a 5 Hz ride frequency when the only
  recorded figure was 2.9–3.0 Hz
- a tyre curve keeping 45% of its grip when sliding, where a slick keeps 70–85%

None were typos. Each was a value that made sense alone and failed the moment
it was compared with another. **A warning is not a build failure** — it is
something worth seeing every time until somebody resolves it.

## Things that reach outside this repo

`WheelRadius` is used by the autonomy pipeline to convert motor rpm into road
speed (`kRpmToMs`). Change it here without changing it there and `/odom` gains
a silent bias no test in this repo will catch. `check_car` reminds you on
every run.

## Where the numbers came from

`docs/IFS_08_measured_parameters.md` records what the team workbook says, with
cell references. Read it before changing anything geometric — and note that
the workbook's `MONO` design block is labelled **IFS-06/07**, so most of it
describes the *previous* car. Only `Susp_Geometry` is IFS-08.

## Running an event on the plant

```matlab
accel_run(75)      % FS acceleration event, on the plant
accel_compare(75)  % the same event, plant vs the old point-mass model
```

`accel_run` is the pattern for the manual team. The point is not the number
it prints — it is that the number comes from the **same plant** the
driverless simulator drives, parameterised from the **same `car_spec`**.
The longitudinal model in `DYNAMIC_MOD/GeneralCalculations` answers the same
question with its own copy of the mass, the tyre and the drag, and its own
copy of a number is its own copy of a mistake.

What the plant gives you that a point-mass model cannot: wheel spin as a
real state, load transfer through the suspension, the actual tyre at the
actual slip including past the peak, and aero, brakes and battery already
wired in.

`accel_compare` exists because the two models currently disagree by most of
a second and a half on a four-second event, and that gap is worth looking at
rather than averaging away. Most of it is launch wheelspin — the point-mass
model caps traction at μ·Fz, which is a car with perfect traction control,
while the plant spins to a slip ratio of 26. Neither is right.

## Prototypes

One file per car, in `cars/`:

| car | file | role |
|---|---|---|
| IFS-08 | `cars/ifs08.m` | **the simulator's car.** Its `settings.json`, plant models and FMU are committed and loaded by UE5. |
| IFS-09 | `cars/ifs09.m` | the next prototype. Inherits the IFS-08 and overrides only what differs. |

```matlab
ifssim_car('IFS-09')          % work on the IFS-09; every tool follows
build_car('plant')            % builds into matlab/build/cars/IFS-09/, never over the IFS-08
car_diff('IFS-08','IFS-09')   % what is different, and how much is still inherited
ifssim_car('IFS-08')          % back
```

- **Inherited values say so.** An IFS-09 value carried over from the IFS-08
  has the source `INHERITED from IFS-08 -- <the IFS-08's source>`. The
  IFS-08's `MEASURED` describes the IFS-08; nobody has weighed the IFS-09
  just because somebody weighed the IFS-08. `check_car` counts what's still
  inherited.
- **Only what differs goes in the new car's file.** A value that's the same on
  both cars stays inherited. Copying it would make two copies of one number,
  free to drift.
- **The active car is per session** (`ifssim_car`). A MATLAB restart is back
  on the IFS-08, on purpose: a choice that outlived the session would let
  somebody build "the car" a week later and get the wrong one. Batch jobs and
  CI choose with the environment variable `IFSSIM_CAR`.
- **Build outputs are separated.** The cars' Simulink models share names
  (`IFSSIM_Plant`, …), so only one car's models folder is ever on the path
  (`ifssim_models_dir`). `settings.json` is written only for the simulator's
  car.
- **Adding a prototype is adding a file.** `cars/ifs10.m` with
  `C = inherit(ifs09(), 'IFS-10');` is a car.

