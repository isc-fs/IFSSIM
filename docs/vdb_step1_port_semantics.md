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

### The camber offset — resolved

`Camber` **is** in radians, and the block reports camber in the OPPOSITE sign
to the mask: a mask of −1.5° comes out as **+1.5°** on wheel 1, mirrored left
to right. With `CamberHslp = 0` the output at `WhlPz = 0` is exactly +1.5000°.

The residual offset comes from the gain being applied about an internal
reference, not about `WhlPz = 0`. It moves linearly with `CamberHslp`:

| `CamberHslp` | camber at `WhlPz = 0` [deg] |
|---|---|
| 0.00 | +1.5000 |
| −0.20 | +1.1646 |
| −0.40 | +0.8292 |
| −0.80 | +0.1583 |
| −1.00 | −0.1771 |

1.677° per unit of `CamberHslp`, dead linear, which puts the internal
reference **29.3 mm** from `WhlPz = 0`. So the full relationship is affine:

```
camber_out(WhlPz)  =  -Camber  +  CamberHslp * (WhlPz + 0.02927)
```

Affine means invertible: pick the static camber and the rate you want, solve
for the two mask values. No fitting, no iteration.

### Our camber gain is not the block's camber gain

They are different parameterisations and must be converted, not copied.
`Susp.CamberGainFront` is DIMENSIONLESS — the fraction of body roll the
geometry takes back out of the tyre. `CamberHslp` is **radians of camber per
metre of wheel travel**. For roll `phi`, the outer wheel travels `(t/2)*phi`,
so:

```
CamberHslp = 2 * CamberGain / track
```

Our front gain of 0.80 on a 1.200 m track is `CamberHslp = 1.333 rad/m`.
Copying 0.80 straight across would give a car with 60% of the camber recovery
it was asked for, and nothing would flag it.

**Also settled:** `VehP` z does not affect camber — held at 0, ±0.1 m, the
output never moved. So `VehP` is not the ride-height reference the gain is
applied about.

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
