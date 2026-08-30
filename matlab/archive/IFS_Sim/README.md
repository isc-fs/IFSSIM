# IFS_Sim — the 2024–25 drive-cycle model

**Superseded. Not wired into the simulator.** The live vehicle model is
[`../plant/`](../plant/README.md).

Kept rather than deleted for two reasons: it holds recorded drive cycles
that are not reproducible from anything else in the repo, and it is the
likely provenance of at least one number the current plant still carries
as an assumption.

## What it actually models

A **longitudinal energy** model — how much battery a lap costs. It has a
drive cycle, a motor, a driveline and a battery. It has no lateral
dynamics, no suspension and no tyre slip, so it cannot answer any
handling question and was never meant to.

| file | what |
|---|---|
| `IFS_Sim.slx` | the model |
| `IFS_Sim_Vehicle_Params.m` | its parameters, and the script that runs it |
| `Schedule_Autocross.mat` | autocross drive cycle |
| `Schedule_Endurance_22km.mat` | 22 km endurance drive cycle |

The two `Schedule_*.mat` files are the part most worth keeping. Note the
script converts their speed column by `1.60934`, so it is stored in mph
and used in km/h.

## What in here is superseded

Several parameters describe a different car from the one `settings.json`
now describes. Do not copy numbers out of this file without checking
them:

| | IFS_Sim | current (`settings.json`) |
|---|---|---|
| mass | 237 kg | 275 kg |
| tyre radius | 0.3 m | 0.202 m |
| max motor torque | 240 N·m | 230 N·m |

The tyre radius is the clearest signal that this predates the IFS-08 as
built — 0.3 m is not this car's wheel.

## What may still be live

`IFS_Sim_Vehicle_Params.m` gives a wheel-plus-tyre inertia of

```
Wheel_Inertia = 0.02179531
Tire_Inertia  = 0.193286      ->  0.21508 kg*m^2 combined
```

The plant carries `P.Assumed.WheelInertia = 0.21`, and its comment
already cites this file as one of two corroborating sources — the other
being a solid-disc estimate at 10 kg and r=0.202, which gives 0.204.
So the link is recorded, in that direction.

What is *not* recorded is where **this** file's 0.21508 came from. Two
components to five and six significant figures look measured or
CAD-derived rather than estimated, but nothing here says which. If they
were measured for the IFS-08's wheel and tyre, the plant can cite a
measurement and stop calling it an assumption. If they were themselves
an estimate, then the plant's "corroborated two ways" is one estimate
agreeing with another, which is weaker than it reads.

Worth five minutes with whoever built this model.

## Battery, for reference

```
19 series x 5 stacks x 4.2 V  =  399 V pack
8.5 Ah, initial SOC 0.9
```

Consistent with the "EMRAX 228 MV at 400 V bus" the current settings
describe. The plant reports `Powertrain.batt_soc` and
`Powertrain.batt_voltage` on its output bus, so if a real energy model
is ever wanted behind those, this is the starting point.
