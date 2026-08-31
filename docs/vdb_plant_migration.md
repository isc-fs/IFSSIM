# Adopting the VDB suspension: a migration plan

**Status: plan only. No code has moved.**

The decision is to adopt Vehicle Dynamics Blockset's suspension and body blocks
in the plant, accepting a refactor. This is what that costs, in what order, and
where it can go wrong.

---

## 1. What was established, not assumed

All of this was read off the installed blocks, not from documentation.

| block | inputs | outputs |
|---|---|---|
| `Independent Suspension - Double Wishbone` | `WhlPz WhlRe WhlVz WhlFx WhlFy WhlM VehP VehV StrgAng` | `Info VehF VehM WhlF WhlV WhlAng` |
| `Vehicle Body 6DOF` | `FSusp MSusp FExt MExt WindXYZ` | `Info Vb pqr DCM Euler Xe Ve` |
| `Combined Slip Wheel 2DOF` (already in use) | `BrkPrs AxlTrq Vx Vy Camber YawRate Prs Gnd Fext ScaleFctrs` | `Info Omega Fx Fy Fz Mx My Mz` |

**The critical fact, and it corrects an earlier assessment of mine.**
`Vehicle Body 6DOF` takes **`FSusp`/`MSusp`** — suspension force and moment —
and does **not** own a ground model. It is a rigid-body integrator with
external force inputs. So adopting VDB does **not** mean giving up the
per-wheel road planes the engine raycasts for us. I previously said it did,
on the strength of a standalone harness that returned 2568 N/wheel against our
591/758; that was my harness feeding wrong `WhlPz` semantics, not an
architectural incompatibility.

Licences confirmed by checkout, not by `license('test')` alone: Vehicle
Dynamics Blockset, Simscape, **and Simscape Multibody**. An older note in this
repo saying Multibody is unlicensed is wrong.

## 2. Target architecture

```
                    Road (per-wheel plane, from the engine's raycasts)
                      │
   Pose ──┬───────────┼─────────────────────────┐
          │           ▼                         │
          │   wheel z  →  DOUBLE WISHBONE  ──────┼──→ WhlF (Fz)   ──┐
          │   StrgAng →   SUSPENSION      ──────┼──→ WhlAng(camber)─┤
          │                  ▲    │              │                   ▼
          │      WhlFx/WhlFy │    └──→ VehF/VehM │            TYRE BLOCK
          │                  │                   │        (Fext←Fz, Camber←WhlAng)
          │                  └───────────────────┼──────────── Fx, Fy
          │                                      │
          └────────────────────────────→ VEHICLE BODY 6DOF ──→ Pose
                          aero, Env, Sync ──────→ (FExt/MExt)
```

Two subsystems change. `IFSSIM_Steering`, `IFSSIM_Powertrain`, `IFSSIM_Brakes`
and `IFSSIM_Aero` are untouched.

- **`IFSSIM_TireSuspension`** — hand-written pre/post code replaced by the DW
  block plus the existing tyre block.
- **`IFSSIM_Chassis`** — hand-written 6-DOF integrator replaced by
  `Vehicle Body 6DOF`.

## 3. What must not change

The plant's contract, because the engine and the FMU depend on it:

- Top-level ports and buses: `Cmd`, `Road`, `Env`, `Sync` in; `Pose`,
  `Wheels`, `Powertrain` out. Bus **element names and order** are part of the
  FMI signature.
- `ode1` fixed-step at 1/960, dividing the 1/60 communication step.
- **State reset through `Sync`.** The FMU is reset to an arbitrary pose by the
  platform, and that path is proven working. `Vehicle Body 6DOF` carries the
  integrator states now, so the reset must reach *its* states. This is the
  single highest-risk item in the migration.

## 4. Order of work, with a gate on each step

Each step ends with `ifssim_plant_check` green, all ten stages. No step
proceeds on a red suite.

**Step 0 — an A/B harness.** Before anything changes: a script that runs the
current plant and a candidate plant through the same manoeuvres and reports
per-signal divergence. Without this there is no way to tell a modelling
difference from a mistake, and every later step is guesswork. Reuse
`validate_dualtrack`'s pattern; the manoeuvres already exist.

**Step 1 — `WhlPz` semantics, in isolation.** Drive the DW block alone until
static loads come out at 591/758 N. This is where the earlier attempt failed
and it is cheap to resolve in isolation. Deliverable: a one-page note stating
exactly what `WhlPz`, `VehP` and `VehV` are measured from and in which frame.

**Step 2 — DW suspension inside `IFSSIM_TireSuspension`**, keeping our chassis.
Feed `VehF`/`VehM` into the existing `tyre_force`/`tyre_torque` outputs. The
suspension changes; nothing downstream knows. Gate: the ten stages, plus A/B
divergence in static loads, roll, and load transfer within a stated tolerance.

**Step 3 — close the tyre loop.** `WhlF`→tyre `Fext`, `WhlAng`→tyre `Camber`,
tyre `Fx`/`Fy`→`WhlFx`/`WhlFy`. This creates an algebraic loop; break it the
way the plant already breaks the wheel-speed loop, with one step of delay, and
say so in the code. Gate: the ten stages, and the camber curve compared
against `matlab/vd`'s.

**Step 4 — `Vehicle Body 6DOF` replacing `IFSSIM_Chassis`.** Repack
`Vb/pqr/DCM/Euler/Xe/Ve` into `IFSSIM_PoseBus`. Route aero and `Env` into
`FExt`/`MExt`. Gate: the ten stages **and a state-reset test** — the FMU's
`Sync` path must still restore an arbitrary pose exactly. Do not merge this
step without that test.

**Step 5 — FMU export and the engine.** Re-export, re-run the importer gates,
and A/B the shadow FMU against Chaos with `tools/fmu/ab_plant.py`.

## 5. Where this can go wrong

- **State reset (step 4).** Highest risk. If `Vehicle Body 6DOF`'s states
  cannot be set externally, the FMU reset contract breaks and the migration
  stops at step 3. **Retire this risk first** — before step 1 — by testing
  whether the block's states can be written. It is a day's work to find out
  and it decides whether steps 4 and 5 exist at all.
- **Algebraic loop (step 3).** Known shape, known fix, but the delay changes
  the transient. Quantify against the current plant rather than assuming.
- **Wheel lift.** Our suspension clamps `Fz` at zero so a wheel can leave the
  ground, and a test asserts it. Whether DW does the same is unknown; check
  early, because a plant that cannot lift a wheel is worse than the one we have.
- **Solver stiffness.** VDB blocks carry continuous states. They compile under
  `ode1` at 1/960, but "compiles" is not "is stable at the limit". Watch the
  0–75 m launch and the skid-pad limit, which are where this plant has broken
  before.
- **Parameter re-homing.** Our springs, bars, camber and static loads become DW
  mask parameters. Every one must still come from `car_spec` through
  `ifssim_params`, or the single source of truth is lost — which is a bigger
  regression than anything the block buys.

## 6. What is actually bought

- Roll-centre migration, caster, scrub, anti-dive and anti-squat — none of
  which exist today in either model.
- Camber that varies with travel rather than a constant rate, i.e. a real
  camber curve rather than a straight line.
- One suspension model instead of two divergent ones, since `matlab/vd` could
  then be validated against a plant that has the same kinematics rather than a
  plant that has none.

What it does **not** buy: any of it becomes real grip only when the tyre has
camber coefficients, and nobody has put this tyre on a rig. The kinematics
would be exact and their consequence still assumed.
