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

- `import_susp_geometry.m` — reads the team workbook's `Susp_Geometry` sheet and
  writes `spec/cars/ifs08_hardpoints.csv`, keeping the sheet's coordinates and
  the row of every point. The CSV is committed; the workbook is only needed to
  regenerate it.
- `sm_hardpoints.m` — the active car's hardpoints, converted to the car's frame
  (x forward, y left, z up, origin at the CoG ground projection). Each car's
  spec says which file and which datum (`C.Hardpoints`).
- `susp_geometry.m` — static geometry with the workbook's own definitions:
  kingpin, caster, scrub, trail, instant and roll centres, swing arms,
  anti-dive, anti-lift and anti-squat.
- `build_sm_corner.m` — the linkage: two wishbones on revolute chassis pivots,
  three ball joints, a track rod, an upright, pushrod and rocker.
- `sm_corner_sweep.m` — sweeps the lower arm; reports camber, toe, track change
  and motion ratio against travel, and checks the model assembles exactly on
  the hardpoints.
- `sm_fixed_step_probe.m` — whether a closed-loop corner fits the real-time
  plant: solver accuracy, the travel limit, and FMU export.
- `test_susp_geometry.m`, `test_kinematics_feed.m` — the checks below.

## The IFS-08's own geometry

The hardpoints are the team workbook's. Two things about the sheet's frame
were resolved from its own data:

- **X grows rearward**, whatever the header says. The front wheel centre is at
  x = 160 and the rear at 1730, and the sheet's +6.2° caster puts the upper
  ball joint at the larger x.
- **Z = 0 is not the ground.** Wheel centres sit at z = 310 with the sheet's
  203.2 mm tyre radius, so the ground is at z = 106.8. With that ground the
  roll centres are 15.1 mm front and 31.1 mm rear. The workbook's MONO sheet
  independently gives 12.8 and 31 mm. The `Susp_Geometry` sheet's own 28.2 and
  58.0 mm are measured from its Z = 0. **Unconfirmed by the suspension team.**

`test_susp_geometry` reproduces all 20 figures the sheet computes, with the
sheet's own datum, to its last printed digit. That proves the import, the frame
conversion and the definitions against a calculation somebody else did.

The multibody sweep on the same hardpoints, fitted over ±30 mm of travel:

| | front | rear | assumed until 2026-10 |
|---|---|---|---|
| camber gain (share of body roll recovered) | 0.125 | 0.253 | 0.80 / 0.60 |
| bump steer (toe-in per metre of bump) | +2.86 deg/m | +0.02 deg/m | 0 / 0 |
| motion ratio at static | 0.829 | 0.833 | 0.70 / 0.70 |
| static camber (from the spindle points) | −2.56° | −2.18° | −1.5° / −1.0° |

The model assembles within 3 nm of the hardpoints at static. The camber slope
agrees with 1/FVSA to within 2%, two independent methods. These values are now
`DERIVED` in `spec/cars/ifs08.m`, and `test_kinematics_feed` re-derives them so
the spec cannot drift from its own geometry. Static camber is sensitive: 1 mm
of spindle height is 1.5°.

The near-horizontal lower arms give a long front swing arm (4.9 m) and very
little camber recovery. That is the single largest change from the assumptions.

## History: the placeholder rocker

What follows was found on the placeholder geometry, before the real hardpoints
were imported. The toggle check it led to now guards the real geometry too.

### Motion ratio, from the pushrod and rocker

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

## Multibody in the real-time plant

`sm_fixed_step_probe` puts one closed-loop corner (the real front hardpoints,
the car's spring and damper on the rocker) through the plant's pipeline:

- At 600 N on the contact patch (28.7 mm of travel, what the car uses) it runs
  at the plant's 1/960 s step **with the plant's own solver settings** (explicit
  ode1 plus a backward-Euler Simscape local solver), 0.09 mm from a tight
  variable-step reference.
- Exported as an FMI 3.0 FMU and stepped the way the simulator steps it
  (`tools/fmu/step_timing`), it runs at about **80× real time**, close to what
  the whole current plant costs (66×). Four corners fit a frame comfortably.
- The plant's solver holds the linkage up to **58.5 mm of bump**. Beyond that
  the linkage itself snaps through: even the reference jumps from 58.5 to
  92 mm for 200 N more. A bump stop before that is a requirement, in the model
  and on the car.

The first run of this probe used 1500 N, drove the wheel 90 mm into bump and
concluded the opposite. The load was unrepresentative; the note stays so
nobody repeats it.
