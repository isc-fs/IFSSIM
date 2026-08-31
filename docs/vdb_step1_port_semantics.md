# Step 1 — what the Double Wishbone block's ports actually mean

Measured against the installed block, not read from documentation. Every
number below was produced by sweeping an input and reading an output.

## The port map — get this wrong and everything downstream is fiction

Read off the Inport/Outport blocks inside the mask, not guessed:

| # | Input | # | Output |
|---|---|---|---|
| 1 | `WhlPz` | 1 | `Info` (bus) |
| 2 | `WhlRe` | 2 | `VehF` |
| 3 | `WhlVz` | 3 | `VehM` |
| 4 | `WhlFx` | 4 | `WhlF` |
| 5 | `WhlFy` | 5 | `WhlV` |
| 6 | `WhlM` | 6 | `WhlAng` |
| 7 | `VehP` | | |
| 8 | `VehV` | | |
| 9 | `StrgAng` | | |

**Output 1 is a bus, not `WhlF`.** It carries `Camber`, `Caster`, `Toe`,
`Height`, `Power`, `Energy`, `dWhlX`, `dWhlY`, `VehF`, `VehM`, `WhlF`, `WhlV`,
`WhlAng` — everything, in one signal. Wiring it somewhere expecting a force
gives an "Unrecognized field name" at log time if you are lucky, and a bus
where a 3×4 belongs if you are not. `WhlF` on its own is **output 4**.

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
reference **29.3 mm** from `WhlPz = 0`.

> **Corrected in step 2, on two counts.** The formula first written here was
>
> ```
> camber_out(WhlPz)  =  -Camber  +  CamberHslp * (WhlPz + 0.02927)
> ```
>
> which contradicts the table directly above it: with `CamberHslp = -0.80` the
> table's slope is **+0.80 rad/m**, not −0.80. And 29.3 mm is not a constant of
> the block — it is `F0z/Kz`, this axle's static deflection, which is 0.0292 m
> at the front but **0.0376 m at the rear**. A single shared value is wrong at
> one end of the car by 0.5° of camber.
>
> The relationship, re-measured one wheel at a time and fitted across all four
> (`sigma = [+1 -1 +1 -1]`):
>
> ```
> gamma_i = sigma_i * [ -Camber + CamberHslp * (F0z/Kz - WhlPz_i) ]
> ```
>
> This reproduces every row of every table on this page, and the map is
> **diagonal** — moving one wheel changes only that wheel's camber. It looks
> coupled at first glance only because the other three sit at a non-zero datum.

Affine means invertible: pick the static camber and the rate you want, solve
for the two mask values. No fitting, no iteration. `vdb_camber_mask` does it.

### Our camber gain is not the block's camber gain

They are different parameterisations and must be converted, not copied.
`Susp.CamberGainFront` is DIMENSIONLESS — the fraction of body roll the
geometry takes back out of the tyre. `CamberHslp` is **radians of camber per
metre of wheel travel**. For roll `phi`, the outer wheel travels `(t/2)*phi`,
so:

```
CamberHslp = -2 * (1 - CamberGain) / track
```

> **Also corrected in step 2.** This was first written as
> `CamberHslp = 2 * CamberGain / track`, which is wrong in the sign and in the
> factor, and it produced 2.4° of camber at 3° of roll where 0.6° was wanted.
>
> Two things drive the corrected form. Body roll reaches the block **through
> the travel** — a roll `phi` puts `-(t/2)*phi` into `WhlPz` — so the block's
> output is already road-relative and no separate roll term is added. And what
> should be left standing after the geometry does its work is the part it does
> **not** recover, `(1 - gain) * phi`, not the part it does.

Our front gain of 0.80 on a 1.200 m track is `CamberHslp = -0.333 rad/m`.
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

## `VehF` is PER-WHEEL, not a body wrench — this shapes step 2

`VehF` and `VehM` are both **[3 × 4]**: a force and a moment per wheel, being
what that corner applies to the body. With `WhlFy = 1000 N` and `VehP = 0`:

```
x:       0       0       0       0
y:    1000    1000    1000    1000     <- WhlFy passes straight through
z:  -590.8  -590.8  -758.1  -758.1     <- the suspension load, z-down
```

Those z values are exactly our static corner loads, with the sign the z-down
convention requires.

**So the block does not hand back an assembled body wrench.** Somebody still
has to sum four per-wheel forces about their moment arms — which is precisely
what `tiresusp_post` already does, correctly, including the contact-patch arm
that took a whole commit to get right. Step 2 keeps it.

That is a simplification, not a disappointment: it means step 2 replaces the
suspension FORCE and KINEMATICS calculation and leaves the wrench assembly
alone, so the fix that gave the car load transfer at all is not re-litigated.

**`VehP` must be left at ZERO.** Its name suggests corner positions; feeding
it ours turned the correct −590.8/−758.1 into +5464/+5297. Whatever frame it
is in, it is not the body frame our geometry is written in, and zero produces
the right answer. `VehV` was held at zero throughout and never moved anything.

## Summary — everything step 2 needs

| input | what to feed it |
|---|---|
| `WhlPz` | `-delta`, our existing suspension deflection (compression positive) |
| `WhlVz` | `-ddelta` |
| `WhlFx`, `WhlFy` | tyre forces, one step delayed to break the algebraic loop |
| `WhlRe` | wheel radius |
| `WhlM`, `VehP`, `VehV` | zeros [3 × 4] |
| `StrgAng` | [front rear] road-wheel angle |

| mask | value |
|---|---|
| `IdealSuspEn` | `'off'` — or camber is identically zero |
| `Kz`, `F0z` | row vectors `[front rear]` |
| `Camber`, `CamberHslp` | by inversion of the affine law above; `CamberHslp = 2*gain/track` |

| output | use |
|---|---|
| `WhlF` | → `max(Fz, 0)` → tyre `Fext`. The clamp is not optional |
| `WhlAng(1,:)` | → tyre `Camber` |
| `VehF`, `VehM` | per-wheel; keep `tiresusp_post` for the wrench |

## The anti-roll bar — the block has one, and it is not our parameterisation

`AntiSwayEnByAxl`, `AntiSwayR` (arm radius, m), `AntiSwayTrsK` (torsional
stiffness, N·m/rad), `AntiSwayNtrlAng`. This matters for step 3: taking the
block's `WhlF` without enabling it would silently delete our roll stiffness
contribution and quietly rebalance the car.

Measured, sweeping `K`, `R` and travel:

```
dFz_1  =  -(K / R^2) * (WhlPz_1 - WhlPz_2) * k(dz/R)
```

which lines up with our own `Karb * (delta_1 - delta_2) / t^2` — same sign,
once you account for `WhlPz = -delta` — under

```
AntiSwayTrsK = Karb * R^2 / t^2      (for k -> 1)
```

`k` is a pure function of `dz/R`, and it is the arm geometry, not an error:

| `dz/R` | 0.005 | 0.01 | 0.1 | 0.2 | 0.4 | 0.8 |
|---|---|---|---|---|---|---|
| `k` | 1.0000 | 0.9999 | 0.9917 | 0.9678 | 0.8832 | 0.6586 |

So the block's bar **softens as it works**, where ours is dead linear. At a
realistic 0.2 m arm and the 31 mm of travel that 3° of body roll produces,
`dz/R = 0.16` and `k ≈ 0.98` — a 2% cut in the ARB term, about 0.9% of front
roll stiffness. Small, real, and a behaviour change that has to be reported
rather than absorbed silently, because roll stiffness sets the balance.

**`AntiSwayR` is therefore a real vehicle parameter, not a fitting constant.**
It belongs in `car_spec` with a source string like everything else, and it
cannot be set to whatever makes the numbers match.
