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

## 3b. PROBE RESULT — the highest-risk item, resolved

Run before anything else, as the plan said to. **The answer changes the plan:
step 4 should not be done at all.**

**`Vehicle Body 6DOF` cannot accept an arbitrary state injection.** It contains
five integrators — `phi/theta/psi`, `p,q,r`, `ub,vb,wb`, `xe,ye,ze`, and one
more — and every one is `InitialConditionSource = internal`,
`ExternalReset = none`. There is no port and no parameter that writes state at
run time; `Xe_o`, `eul_o`, `xbdot_o`, `p_o` are compile-time initial
conditions, not a runtime reset.

Two mechanisms exist today and they are not the same thing:

| | what it does | who uses it |
|---|---|---|
| FMU `canGetAndSetFMUStateOverride` | snapshot and restore an OPAQUE state the FMU itself captured | determinism and replay |
| **`Sync` bus** | write an ARBITRARY pose the platform chose | putting the car on the start gate |

Only the second is at risk, and it is the one that matters operationally. It
works today because our chassis owns its integrator inside a MATLAB Function
block and the code overwrites its own state when `sync.enable` is set.

**It is achievable by forking the block.** It is not P-coded — 868 blocks are
visible under the mask — and all five integrators accept
`InitialConditionSource='external'` and `ExternalReset='rising'`. But that
means owning a modified copy of a MathWorks block, re-applying the
modification at every MATLAB upgrade, and routing reset and IC signals down
through several levels of nested subsystem.

### FORK PROBE — the chassis is takeable after all

Decision taken to adopt the body block too, so the fork was tested rather than
argued about. It works, and it is cheaper than the paragraph above implies:

```
  broke 1 nested library link
  phi/theta/psi    now has 3 input ports (was 1)
  p,q,r            now has 3 input ports (was 1)
  ub,vb,wb         now has 3 input ports (was 1)
  xe,ye,ze         now has 3 input ports (was 1)
  Integrator       now has 3 input ports (was 1)
  5/5 integrators now take an external IC and reset
  COMPILES with the modified integrators
```

One link break, five integrators, and the block still compiles. The remaining
work is routing the reset trigger and the four IC values from the block
boundary down to depth 3-7 — bounded engineering, now proven possible.

**And the maintenance objection mostly dissolves, because this plant is
GENERATED.** Every `.slx` here is written by a `build_*.m` script, so the fork
is not a modified block checked into the repo — it is a scripted
transformation applied at build time, exactly as the probe applied it. It
re-applies itself on every build. If a MATLAB upgrade moves the internals, the
build FAILS LOUDLY at the `find_system` that no longer finds five integrators,
rather than silently drifting. That is a better failure mode than a
hand-maintained fork, and it is the argument that makes taking the chassis
reasonable.

What it still costs: the transformation is coupled to MathWorks' internal
block structure, which is not a supported interface and can change without
notice. Budget for it breaking at some upgrade, and keep the guard sharp
enough that it breaks the build rather than the car.

### Superseded: the earlier recommendation to keep our chassis

The value of this migration — roll-centre migration, caster, scrub, anti-dive
and anti-squat, a camber curve that is a curve — is **entirely in the
suspension block**. `Vehicle Body 6DOF` is a 6-DOF rigid-body integrator, and
ours already is one, working, with proven state injection. It buys nothing we
do not have and it is the only step that threatens the reset contract.

And the interfaces already line up: the DW block outputs `VehF` and `VehM`,
3-vectors of force and moment on the body, which is exactly what our chassis
takes today as `tyre_force` and `tyre_torque`. Steps 1-3 drop in against an
unchanged chassis.

~~**So: do steps 1, 2 and 3. Do not do step 4 or 5.**~~ Superseded by the fork
probe above: all five steps are in. Step 4 gains a preliminary — build and
verify the state-injection fork before wiring the body block into the plant.

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

### Step 2 — DONE, and the gate as written would have passed a severed wire

Built behind `build_tiresuspension(outdir, overrides, useVDB)`, default off.
The block takes `WhlPz = -delta` and `WhlVz = -ddelta` from our existing
suspension pass; `WhlAng` row 1 replaces the closed-form camber into the
tyre's `Camber` port. Its own `WhlF` is computed and terminated — taking it
is step 3, and it still needs the `max(Fz, 0)` clamp step 1 found.

**No algebraic loop**, and that was measured rather than assumed: a 1000 N
`WhlFy` moved only `VehF`'s y row and reached neither `WhlF` nor `WhlAng`. No
delay is needed here. Step 3 will need one; step 2 does not.

Three things this step actually turned up, none of them anticipated:

1. **The gate above is not a gate.** "A/B divergence in static loads, roll and
   load transfer" cannot see camber at all — the Magic Formula camber terms are
   zeroed for want of rig data, so camber changes no force by construction. The
   first version of `vdb_step2_check` compared the wrench, reported exact
   agreement on all six manoeuvres, and **passed a mutant** with the static
   camber shifted 0.5° and the slope shifted 0.5 rad/m. The check now compares
   the camber signal at the tyre input, logged by name at build time, and the
   mutants are caught. A wrench comparison would never have caught any of them.

2. **Both formulations had the mirroring inverted**, in opposite ways. In the
   ISO tyre axis system y points left on *both* wheels, so static camber —
   symmetric in space, tops leaning toward each other — reads with **opposite**
   signs left to right, while body roll — the same tilt direction in space —
   reads with the **same** sign and must stay outside the mirror. The closed
   form did exactly the reverse of both. It changed no force, for the same
   reason as above, and it would have changed every one of them the day rig
   data arrives.

3. **The step 1 camber formula was wrong in the sign and in the datum**, and
   its own table said so. Corrected in `docs/vdb_step1_port_semantics.md`.

The two now agree to **5.5e-4 deg** through roll, which is the small-angle
residual and nothing else: the closed form is linear in roll, the block works
off travel proportional to `sin(roll)`. Under heave they diverge on purpose —
the block produces camber where the closed form has no term at all. That is
the capability being bought, so `vdb_step2_check` reports its size rather than
asserting on it.

**Still unpinned:** the overall sign of gamma. Nothing in this plant can settle
it while the camber coefficients are zero — there is no observable to check it
against. It wants rig data, not an argument.

**Step 3 — close the tyre loop.** `WhlF`→tyre `Fext`, `WhlAng`→tyre `Camber`,
tyre `Fx`/`Fy`→`WhlFx`/`WhlFy`. This creates an algebraic loop; break it the
way the plant already breaks the wheel-speed loop, with one step of delay, and
say so in the code. Gate: the ten stages, and the camber curve compared
against `matlab/vd`'s.

**Step 4a — the state-injection fork, standalone.** A `fork_vehicle_body.m`
that takes the library block, breaks the link, gives the five integrators an
external IC and reset, and routes those to the subsystem boundary. Gate: a
test that runs the forked body, injects an arbitrary pose mid-run, and asserts
the state jumps to it exactly — the same contract `IFSSIM_Chassis` honours
today. Do not proceed to 4b until that test is green.

**Step 4b — `Vehicle Body 6DOF` replacing `IFSSIM_Chassis`.** Repack
`Vb/pqr/DCM/Euler/Xe/Ve` into `IFSSIM_PoseBus`. Route aero and `Env` into
`FExt`/`MExt`. Gate: the ten stages **and a state-reset test** — the FMU's
`Sync` path must still restore an arbitrary pose exactly. Do not merge this
step without that test.

**Step 5 — FMU export and the engine** (after step 3, not step 4). Re-export, re-run the importer gates,
and A/B the shadow FMU against Chaos with `tools/fmu/ab_plant.py`.

## 5. Where this can go wrong

- ~~**State reset (step 4).**~~ **RESOLVED, see §3b.** The block's states
  cannot be written at run time. The migration stops at step 3 by design, and
  loses nothing by doing so.
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
