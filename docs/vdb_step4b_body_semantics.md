# Step 4b — what `Vehicle Body 6DOF` means, measured

Every number here came from driving the block and reading it back. Nothing is
read off documentation, for the same reason as step 1: the step 1 write-up
that *was* reasoned rather than measured turned out to contradict its own
table.

## The frame is z-DOWN and gravity is internal

Released with no applied force, the body falls to `Xe(3) = +4.905 m` in one
second and `Vb(3) = +4.905 m/s`. That is `½gt²` and `gt` at `g = 9.81`, so:

- **z points DOWN.** Positive `Xe(3)` is *below* the origin.
- **Gravity is applied inside the block**, at a fixed 9.81 m/s². There is no
  gravity mask parameter.

Pushing with `FExt` moves `Xe` the same way on every axis (+x gives +x, +y
gives +y, −z gives −z), so the applied-force frame and the position frame
agree. It is a conventional vehicle frame: x forward, y right, z down.

**`IFSSIM_PoseBus.position` is World ENU — z UP.** So the handover needs a real
frame conversion, not a copy, and `Env.gravity_z` (which our chassis takes as
an input and the block has no port for) has to be reconciled: either accept
9.81 and delete the parameter, or inject the difference through `FExt`. A sign
error anywhere in here is silent — the car simply drives underground.

## `Acc.ax/ay/az` is NOT proper acceleration, and it is not in m/s²

`PoseBus.accel_proper` is documented as *"Proper acceleration at the BODY
ORIGIN"* — what an accelerometer actually reads. The obvious mapping is
`BdyFrm.Cg.Acc.ax/ay/az`. That mapping is **wrong twice over**:

| condition | `ax` | `az` | `xddot` | `zddot` |
|---|---|---|---|---|
| free fall | 0.000 | **1.000** | 0.000 | 9.810 |
| held against gravity | 0.000 | 0.000 | 0.000 | 0.000 |
| held, pushed 10 m/s² | **1.020** | 0.000 | 10.000 | 0.000 |

An accelerometer in free fall reads **zero**. `az` reads **1.000**. And
`ax = 1.020` where `xddot = 10.000`, i.e. `10/9.81`.

So `ax/ay/az` is **kinematic acceleration expressed in g**, and
`xddot/yddot/zddot` is the same quantity in m/s². *Neither is proper
acceleration.* It has to be computed:

```
accel_proper = a_kinematic_body - R' * g_world
```

which is zero in free fall and −9.81 z when standing still, as it should be.

This matters more than a unit slip normally would: `accel_proper` is what the
pipeline's `/imu` is built from. Wiring `ax` straight through would publish an
IMU reading that is 9.81× too small **and** carrying gravity where it should
not be — and the plant already has one historical IMU scaling bug of exactly
this family.

## The mapping table

| `IFSSIM_PoseBus` | source | conversion |
|---|---|---|
| `position` | `Xe` | ENU ← z-down |
| `quat` | `Euler` / `DCM` | Euler → `[w x y z]`, body→world |
| `vel_world` | `Ve` | ENU ← z-down |
| `vel_body` | `Vb` | frame only |
| `omega_body` | `pqr` | frame only |
| `alpha_body` | `Info.BdyFrm.Cg.AngAcc.pdot/qdot/rdot` | frame only |
| `accel_proper` | `Info.BdyFrm.Cg.Acc.xddot/yddot/zddot` | **minus gravity**, then frame |
| `attitude` | `Euler(1:2)` + `Xe(3)` | heave sign flips |

## Two things that would double-count

**The block has its own aerodynamics** — `Cd = 0.3`, `Af = 2` by default. We
have `IFSSIM_Aero`. Left alone, the car gets drag twice. Set `Cd = 0` and
`Af = 0` and keep aero where it is tested.

**The block's defaults are a passenger car**: `m = 2000`, `Iveh =
diag(430, 1900, 2100)`, `a = 1.4`, `b = 1.6`, `h = 0.35`, `w = [1.9 1.9]`.
Every one must come from `car_spec`. This is the same failure the tyre
paramset had, where 174 of 238 coefficients were a passenger car's, and it
will not announce itself — a 2000 kg car simply understeers.

## `FSusp`/`MSusp` are `[3 × 4]`, per wheel

Which is what the double-wishbone block's `VehF`/`VehM` already produce. So
the per-wheel forces go straight in and the body does the moment arithmetic
that `tiresusp_post` does today. That is the summation and moment-arm code
this step gets to delete — and the one whose moment-arm bug cost a day
earlier in this work.
