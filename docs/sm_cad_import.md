# Getting the IFS-08 assembly into Simscape Multibody

Route A: export the SolidWorks assembly with Simscape Multibody Link and let
`smimport` build the model. This is written for someone rebuilding the assembly
anyway — most of what follows is cheaper to get right in CAD than to repair in
MATLAB afterwards.

## 0. The add-in, once

On the Windows machine, in MATLAB (**run as administrator**, it writes into the
SolidWorks install):

```matlab
smlink_linksw          % registers the add-in with SolidWorks
```

Then in SolidWorks: **Tools > Add-Ins** and tick *Simscape Multibody Link*.
The MATLAB and add-in versions must match — R2025b here.

Export is **Tools > Simscape Multibody Link > Export > Simscape Multibody**,
which writes `<assembly>.xml` plus a folder of geometry files. Both come back
to this repo; `smimport('<assembly>.xml')` builds the model.

## 1. Build a MECHANISM, not a picture

The importer turns **parts into bodies** and **mates into joints**. Anything in
the assembly that is not part of the linkage becomes a body it has to constrain,
and every extra mate becomes a joint or a redundant constraint.

Include, per corner:

- upright / hub carrier
- upper wishbone, lower wishbone
- track rod (tie rod)
- pushrod and rocker  ← **please include these**
- damper body and shaft as two parts
- wheel centre as a part or a clearly-named reference geometry

Leave out, or suppress before exporting: bodywork, aero, fasteners, wiring,
cosmetic hardware, anything bolted-on that does not move relative to what it is
bolted to. A bracket that never moves relative to the chassis should be *part
of* the chassis body, not its own part.

**The pushrod and rocker matter specifically.** They are what turn
`Susp.MotionRatioFront` from an assumed 0.70 into a computed number. Without
them the corner still gives camber, toe and scrub, but the motion ratio stays
a guess — and it is the number that sets wheel rate from spring rate.

## 2. One part per rigid body

If two parts never move relative to each other, make them one body — either a
single part, or a subassembly set to **Rigid**. Every avoidable body is an
avoidable set of constraints.

## 3. Mates are the joints, so mate deliberately

This is where imports go wrong. SolidWorks lets you over-define an assembly and
still show it as fully mated; Simscape then sees **redundant constraints** and
either warns or fails to solve.

Aim for the minimum mates that produce the intended degrees of freedom:

| joint wanted | mate it cleanly as |
|---|---|
| wishbone chassis pivot (revolute) | concentric on the pivot axis + one coincident to locate along it |
| ball joint (spherical) | point-to-point coincident, nothing else |
| damper / pushrod sliding (prismatic) | concentric + one anti-rotation |

Avoid: belt/gear mates, width mates, limit mates, and any mate added only to
stop a part spinning visually. **A free spin about a rod axis is harmless in the
model and cheaper than a redundant constraint.**

A double wishbone is a **closed loop** — chassis, lower arm, upright, upper arm,
back to chassis. Closed loops are exactly where redundancy bites, so if you
over-mate anywhere, over-mate somewhere else.

## 4. Give everything mass properties

Assign materials, or set mass properties explicitly, on every part that moves.
Missing or zero inertia is not a warning you will notice — it produces
*"error evaluating equations at time 0.0 ... there may be a singularity"*,
which reads like a solver tolerance problem and sends you to the wrong place.
This bit us already, on a body defined as a point mass with no rotational
inertia.

## 5. Fix the chassis, and put the origin somewhere meaningful

- The chassis must be **fixed** (or the assembly's ground) so the world frame
  attaches to it.
- The assembly origin becomes the **World frame origin**. Please put it, or a
  clearly named reference coordinate system, at a datum we can state — front
  axle centreline at ground level is ideal, CoG ground projection is what the
  plant uses.
- Tell us the **axis convention** (which way is +X, +Y, +Z). SolidWorks
  assemblies are usually Y-up; the plant is Z-up, x forward, y left. That
  conversion is a rotation we apply once, deliberately, and it is exactly the
  kind of thing that otherwise becomes a silent sign error.

## 6. What we need alongside the export

- the axis convention and origin datum (see above)
- static ride height, or the assembly posed at static
- whether the geometry is left or right corner, and whether the other side is
  mirrored

## 7. What happens on this side

`smimport` builds the model; then it needs a pass to:

- replace redundant mate-derived joints with the minimal set,
- re-point the sweep in `matlab/sm/sm_corner_sweep.m` at the imported model,
- and produce the real camber, bump-steer, scrub and motion-ratio curves.

The placeholder result to beat, from `matlab/sm/README.md`, is camber gain
0.293 and bump steer 7.11 deg/m against a `car_spec` that assumes 0.800 and 0.
Those are properties of invented hardpoints and mean nothing about the car;
the CAD numbers are the first ones that will.

## 8. Still open, and worth knowing before investing

The imported corner will be a DAE on a stiff variable-step solver. That is
right for a design study and is **not** the fixed-step plant. **Whether any of
it exports as a fixed-step FMU is unanswered**, and on this project the
sample-time negotiation has already been expensive on far simpler models. Treat
the multibody model as a design tool first; the plant question is separate and
should be prototyped on its own before anything depends on it.
