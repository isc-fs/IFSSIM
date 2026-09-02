# Simscape Multibody suspension port

A double-wishbone corner as a real linkage, so suspension geometry stops being
three numbers somebody chose and starts being coordinates somebody can measure.

## Why

The plant does not contain suspension geometry. It contains its consequences,
as assumed constants in `car_spec`:

| constant | value | what it really is |
|---|---|---|
| `Susp.MotionRatioFront` | 0.70 | a consequence of the rocker geometry |
| `Susp.CamberGainFront` | 0.80 | a consequence of the wishbone lengths and heights |
| `Susp.ArbArmRadiusFront` | 0.20 | a consequence of the bar and drop-link layout |

A multibody corner computes all three. Those assumptions collapse into one
hardpoint set — which the suspension department can measure, argue about, and
replace from CAD.

## Files

- `sm_hardpoints.m` — the pickup points, one corner. **Placeholders**, built
  from the car's real track and wheelbase split with a plausible FS layout.
  Replace wholesale from CAD; a hardpoint set is a geometry, not seven
  independent knobs.
- `build_sm_corner.m` — the linkage: two wishbones on revolute chassis pivots,
  three ball joints, a track rod, an upright.
- `sm_corner_sweep.m` — sweeps the lower arm and reports camber, toe, track
  change against wheel travel.

## First result, with the placeholder geometry

Swept ±53 mm about static:

```
camber slope        +27.97 deg/m
implied camber gain  0.293   (car_spec assumes 0.800)
bump steer          +7.11 deg/m   (car_spec targets 0)
scrub               18.0 mm across the sweep
```

The bump-steer figure is the interesting one. `car_spec` sets
`BumpSteerFront = 0` as a design TARGET and the plant applies exactly zero,
because it has no hardpoint from which toe change could emerge. Here it falls
out of the geometry, and the tie-rod inboard position is the hardpoint that
moves it.

**The gain is reported as a magnitude.** Whether it signs + or - depends on a
mirroring convention this port has not pinned against the plant's, and
asserting an unchecked sign is the mistake the camber work already made once.

## Three things that cost time, so they are written down

1. **Wishbone axes are not coordinate axes.** A Revolute Joint turns about its
   own +Z; a wishbone turns about the line joining its two chassis pickups.
   Each arm carries a rotation matrix mapping +Z onto that line, with the arm
   vector re-expressed in the rotated frame. Getting it wrong does not fail to
   build — it silently swings the arm through the wrong plane.
2. **Rigid Transform base and follower are not interchangeable.** Wiring both
   connections to the follower leaves the base unconnected; Simscape says so
   and carries on, with every offset inverted.
3. **`PointMass` has no rotational inertia.** Every body here rotates, and a
   rotating body with zero moment of inertia is a singular mass matrix.
   Simscape reports it as *"error evaluating equations at time 0.0 ... there
   may be a singularity"*, which reads like a solver-tolerance problem and
   sends you to the wrong place entirely. Use `Custom` with real moments.

## What is NOT answered

The corner is a closed kinematic loop, which makes it a DAE wanting a stiff
variable-step solver (`ode23t`). That is right for a design study and is **not**
the fixed-step plant. **Whether any of this exports as a fixed-step FMU is
open**, and given how much the sample-time negotiation cost on far simpler
models, it should be prototyped in isolation before anything depends on it.
