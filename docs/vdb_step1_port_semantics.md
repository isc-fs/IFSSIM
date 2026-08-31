# Step 1 — what the Double Wishbone block's ports actually mean

Measured against the installed block, not read from documentation. Every
number below was produced by sweeping an input and reading an output.

## `WhlPz` — wheel vertical displacement, z-DOWN, zero at static

```
Fz = F0z - Kz * WhlPz
```

Exact to the last digit across the sweep, with `Cz = 0`:

| `WhlPz` [m] | `WhlF` [N] | `Fz - F0z` | `-Kz*WhlPz` |
|---|---|---|---|
| −0.050 | 1600.0 | 1009.2 | 1009.2 |
| −0.020 | 994.5 | 403.7 | 403.7 |
| 0.000 | 590.8 | 0.0 | 0.0 |
| +0.020 | 187.1 | −403.7 | −403.7 |
| +0.050 | −418.4 | −1009.2 | −1009.2 |

So `WhlPz` is displacement from the static reference in a **z-down** frame:
**positive is droop, negative is bump**, and `WhlPz = 0` is the static ride
position. `F0z` is the load carried there.

This is what the first attempt got wrong. Feeding `WhlPz = -StaticSuspLength`
(−0.098 m) told the block the suspension was compressed 98 mm from static, so
it returned 590.8 + 20184×0.098 = 2568.8 N. The block was right; the input was
a ride height where a displacement belonged.

## `Kz` and `F0z` take per-axle ROW vectors

`[front rear]` is accepted; `[front; rear]` fails in mask initialisation. So
our 590.8 / 758.1 static split and any front/rear spring split map directly:

```matlab
set_param(blk, 'Kz','[20184 20184]', 'F0z','[590.8 758.1]')
```

## `WhlAng` is [3 × 4] and row 1 is camber, in radians

With `IdealSuspEn = 'off'`, `Camber = -1.5 deg`, `CamberHslp = -0.80`:

| `WhlPz` [m] | camber, wheels 1..4 [deg] |
|---|---|
| −0.040 | −1.675, +1.675, −1.675, +1.675 |
| 0.000 | +0.158, −0.158, +0.158, −0.158 |
| +0.040 | +1.992, −1.992, +1.992, −1.992 |

Left and right are mirrored, which is correct. The slope is 3.667° over 0.08 m
= 45.8 deg/m = **0.80 rad/m**, i.e. `CamberHslp` is **radians per metre of
travel**, matching its magnitude exactly.

**`IdealSuspEn` defaults to `on` and zeroes all of this.** With it on, `WhlAng`
is identically zero however the wheel moves — an ideal suspension keeps the
tyre upright. Anyone wiring this block up and seeing no camber has hit that
switch, not a broken model.

**Open**: at `WhlPz = 0` camber reads +0.158°, not the −1.5° the `Camber` mask
parameter was set to. The rate is confirmed; the static offset is not yet
explained and needs one more calibration before step 2 can claim to reproduce
our camber curve.

## The block does NOT clamp `Fz` at zero — this changes step 2

At 0.20 m of droop it returns **−3446 N**. A negative normal force is the tyre
pulling the car down onto the road.

Our plant clamps at zero and `test_tiresuspension_physics` asserts it, in two
tests: a wheel over a hole must carry nothing, and the others must stay
loaded. That behaviour is not optional — without it the car falls through
kerb edges and generates grip from a lifted wheel.

**So step 2 must put an explicit `max(Fz, 0)` between `WhlF` and the tyre's
`Fext`**, and keep the existing tests pointed at it. This is not a defect in
the block; a suspension model with no ground contact model in it cannot know
the wheel has left the road. It is a wiring requirement, and it would have
been found by `plant_ab`'s wheel-lift manoeuvre after the fact — better to
know now.

## Not yet established

`VehP` and `VehV` are [3 × 4] and did not affect `WhlF` in any sweep here, all
of which held them at zero. They presumably feed the geometry and the
`VehF`/`VehM` wrench. Step 2 needs them pinned down before the body-force path
can be trusted.
