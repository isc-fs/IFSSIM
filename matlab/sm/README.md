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
camber slope        -27.97 deg/m   (SAE: bump drives camber negative, top in)
camber gain         +0.293         (car_spec assumes +0.800)
bump steer          +7.11 deg/m    (car_spec targets 0)
scrub               18.0 mm across the sweep
```

So this geometry recovers **29%** of body roll where the plant assumes **80%**.

## Motion ratio, from the pushrod and rocker

The corner now carries a pushrod (ball joint at each end) driving a rocker on a
chassis revolute, with the damper's length read as the distance between the
rocker's damper end and its chassis mount. Motion ratio is that length's change
per unit wheel travel:

| travel mm | -53 | -27 | **0** | +13 | +26 | +40 | +53 |
|---|---|---|---|---|---|---|---|
| motion ratio | 0.684 | 0.694 | **0.678** | 0.641 | 0.557 | 0.340 | 0.181 |

**0.678 at static** against the 0.700 `car_spec` assumes, giving a wheel rate of
18 940 N/m from the same spring where the plant uses 20 184 — about 6% softer.
It was not aimed for: it fell out of laying the rocker to the standard rule.

The curve is strongly **regressive**. That is this placeholder — 60 mm rocker
arms against ±53 mm of travel, which is wider than a typical FS car's ±25–30 mm,
so the rocker rotates a long way. Within ±26 mm it is a milder 0.69 → 0.56.
Replace the hardpoints from CAD rather than tuning these to look like a good
design.

### The first rocker toggled, and the sweep now refuses that geometry

The first placeholder put the rocker's damper arm at 60° to the damper instead
of 90°. As the rocker turned, that arm lined up with the damper and the linkage
went through **toggle**: the motion ratio ran from +0.70 in droop to **−2.54** in
bump — the damper lengthening as the wheel rose. A reasonable-looking static
value (0.416) came out of a broken mechanism.

It also leaked. Camber gain drifted 0.293 → 0.291 and scrub 18.0 → 17.3 mm,
though a pushrod should not touch the wishbone kinematics at all: near a toggle
the rocker loop stiffens and the assembly solver compromises on the arm angle.
Fixing the rocker restored both to the decimal.

The sweep now flags any geometry whose motion ratio changes sign inside the
travel — *"LINKAGE TOGGLES within the travel — geometry is invalid"* — and stops
rather than report a number. That matters as much for real CAD geometry as for
placeholders: a design that toggles is a design error, not a characteristic.
Mutation-tested by restoring the original rocker.

The bump-steer figure is the interesting one. `car_spec` sets
`BumpSteerFront = 0` as a design TARGET and the plant applies exactly zero,
because it has no hardpoint from which toe change could emerge. Here it falls
out of the geometry, and the tie-rod inboard position is the hardpoint that
moves it.

## The camber sign is pinned, two ways

Camber is reported **SAE: positive = top of the wheel outward.** For the left
wheel modelled here, a positive rotation about +x tips the top toward -y, i.e.
inward, so SAE camber is the negative of the x angle.

That derivation is not taken on trust, because the camber work in this project
already got a sign wrong once. The sweep checks it two ways:

- **algebra** — the wheel's tilt read off the full rotation matrix. Built from
  the same sensed angles, so it only confirms the sign derivation has no slip.
- **independent** — the upper ball joint's world **position**, which never goes
  through an angle decomposition. Top-in means that point moves toward the car
  centre. Across the sweep it goes from -75.0 mm at static to -80.0 mm at full
  bump, and tracks the angle-derived camber at r = +1.0000.

Mutation-tested: flipping the camber sign drives the independent check to
r = -1.0000 and fails it, and the algebra check with it.

The gain is now **signed in car_spec's own convention** — the fraction of body
roll the geometry recovers — so the two numbers compare directly.

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
