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

### Step 4a — DONE, the fork honours the contract

`fork_vehicle_body.m` drops the block in, breaks the outer and nested library
links, and converts all five integrators to an external IC with a rising-edge
reset. `test_vehicle_body_fork` drives the body with a real force, injects a
pose unrelated to its trajectory at t = 1 s, and gets **exact** landings:

| state | error | when |
|---|---|---|
| Euler angles | 0.000e+00 | t = 1.0000 |
| body rates | 0.000e+00 | t = 1.0000 |
| body velocity | 0.000e+00 | t = 1.0000 |
| earth position | 0.000e+00 | t = 1.0000 |

and it keeps integrating afterwards (4.04 m in the next 0.5 s), because a
reset that never releases would pass every assertion above and still be
broken.

**Routing turned out much cheaper than "depth 3-7 of nested subsystem".**
Global `Goto`/`From` carries the trigger and the four ICs: the `From` sits
beside the integrator, the `Goto` at the model's top level, and **nothing in
between is touched**. No ports are added to MathWorks' subsystems, which is
both less work now and less to redo at an upgrade.

**The first run of the gate reported four failures that were not there.** It
sampled one step past the trigger, by which time the body had integrated away
from the injected value -- 7 m/s times 1 ms is the 0.007 m it was "failing"
by. The state had jumped perfectly and the probe was late. The assertion now
states the contract instead of trusting an index: within one step of the
trigger the state must equal the injected value exactly, and the search
reports which step it landed on, so a late or gradual reset stays visible.

Mutation-tested: wiring one integrator's reset to a constant zero is caught
(6.4 m on earth position) while the other three still pass, so the gate is
per-state rather than a blanket.

**Step 4b — `Vehicle Body 6DOF` replacing `IFSSIM_Chassis`.** Repack
`Vb/pqr/DCM/Euler/Xe/Ve` into `IFSSIM_PoseBus`. Route aero and `Env` into
`FExt`/`MExt`. Gate: the ten stages **and a state-reset test** — the FMU's
`Sync` path must still restore an arbitrary pose exactly. Do not merge this
step without that test.

### Step 4b — DONE, behind `build_chassis(outdir, useVDB)`, off by default

Same six inputs, same `IFSSIM_PoseBus` out; what changes is who integrates.
`vdb_step4b_check` drives both chassis identically:

| case | \|dpos\| | \|dvel\| | \|dquat\| |
|---|---|---|---|
| free roll | 0 | 0 | 0 |
| drive force +x | 0 | 0 | 0 |
| lateral force +y | 0 | 0 | 0 |
| yaw moment +z | 0 | 0 | 4.6e-08 |
| combined + roll | 7.8e-04 | 3.7e-03 | 2.7e-04 |
| teleport mid-run | 0 | 0 | 0 |

Both teleport to exactly `[12, -7, 0.3]`. The ten stages pass on the default
path (`PLANT OK`, 624 s).

**Step 4a was forked against the wrong contract, and its own gate could not
see it.** `IFSSIM_Chassis` overwrites its state on EVERY step while
`sync_en > 0.5` -- the platform holds enable high while positioning the car.
The fork used `ExternalReset='rising'`, which injects the pose once and
immediately lets go. Step 4a's test asserted "jumps, then keeps integrating",
which is exactly what edge semantics do, so it passed. **The A/B against the
incumbent is what caught it**: the teleported car fell 4.6 m while the real
chassis held station. The fork is now `'level'` and the 4a gate uses a pulse,
so it tests both halves -- holds station to 0.000e+00, then releases.

That is the argument for A/B-ing against the thing being replaced rather than
against a specification: the specification is what was misread.

**The one non-zero row was chased, not tolerated.** `alpha_body` agrees
between the two to **2.2e-16** on every step, so the physics is identical,
gyroscopic term included. What differs is how orientation is INTEGRATED: ours
advances a quaternion by forward Euler and renormalises, the block advances
Euler angles. Both first order, different truncation, and identical whenever
the rotation is planar -- which is why yaw-only is exact and roll-plus-yaw is
not. The tolerance is that truncation; the machine-precision `alpha`
agreement is what guards the physics.

**What was disabled deliberately:** the block's own aerodynamics, `Cd = 0.3`
and `Af = 2`, which would have silently doubled our drag. Aero stays in
`IFSSIM_Aero`, where it is parameterised from the car and tested. Its other
passenger-car defaults -- 2000 kg, `Iveh` diag(430,1900,2100), 1.9 m track --
are all overwritten from `car_spec`.

### The ten stages with `useVDB` — RUN, and they FAIL

`useVDB` is now threaded through `ifssim_plant_build` and
`ifssim_plant_check(verbose, useVDB)`. Running the gate the plan actually
asks for:

```
variant     VDB (Double Wishbone suspension, Vehicle Body 6DOF)
  [ok  ] parameters load        [ok  ] steering physics
  [ok  ] build                  [ok  ] powertrain physics
  [FAIL] all models compile     [ok  ] aero physics
  [FAIL] chassis physics        [FAIL] brake physics
  [ok  ] tyre/suspension        [FAIL] whole-car drive
PLANT HAS FAILURES   (548 s)
```

**So step 4b is NOT complete.** Its own A/B passes and the body is right; the
plant around it does not yet run. Four failures, one root cause, and it is a
trap this repo already documented in `build_tiresuspension.m`:

```
Fixed-step size of parent model 'IFSSIM_Plant' and model
'IFSSIM_TireSuspension' must match because the referenced model contains a
hybrid of discrete and continuous components. The parent uses
0.0010416666666666667, the referenced model 0.0010416666666666671.
```

Those differ in the last bit of the double. The existing comment on the
`Memory` block explains exactly this: once a referenced model is a hybrid of
discrete and continuous components, Simulink stops being lenient and demands
the two step sizes agree to the last bit — and they never do, because both
are *negotiated* fundamental sample times rather than anything parsed from a
literal.

Making the chassis continuous (`ode1`, which the body block's five
integrators require) is what tipped the plant into that regime. The third
failure is a variant of the same thing: `test_chassis_physics` hardcodes
`FixedStepDiscrete` in its harness, which cannot simulate a chassis that now
has continuous states.

**This is why the flag is off by default, and the default path stays green**
(`PLANT OK`, 624 s, verified separately).

What it needs, and none of it is speculative:

1. `test_chassis_physics` must pick its solver from what the chassis actually
   is, rather than hardcoding a discrete one.
2. The parent and referenced models must be made to agree on step size to the
   bit. Either every model takes one identical literal, or
   `IFSSIM_TireSuspension` is kept strictly non-hybrid so the question never
   arises — which is what the `Memory`-not-`Unit-Delay` choice was already
   protecting, and what the continuous chassis has now undone from the other
   side.

### Where it got to: 8 of 10, and the last two are bookkeeping

After fixing the causes above, the VDB variant reaches:

```
  [ok  ] parameters load        [ok  ] powertrain physics
  [ok  ] build                  [ok  ] aero physics
  [ok  ] all models compile     [FAIL] brake physics
  [ok  ] chassis physics        [FAIL] whole-car drive
  [ok  ] tyre/suspension physics
  [ok  ] steering physics
```

**Every physics stage passes.** The two failures are one cause, and it is not
vehicle behaviour: the test harness and `IFSSIM_Plant` negotiate fixed steps
2 ulp apart -- `...671` against `...667` -- and Simulink requires them to match
to the bit because the referenced model is a hybrid of discrete and continuous
components.

What was fixed on the way, all of it real and kept:

- `test_chassis_physics` hardcoded `FixedStepDiscrete`, which cannot simulate
  continuous states. A bug independent of this migration.
- The step literal is variant-dependent and now lives in `ifssim_step()`
  instead of being hand-copied. `build_plant_skeleton` never received the
  flag, so the plant DECLARED one value while NEGOTIATING another.
- The closed-form camber output dangled in the VDB branch -- the mirror of the
  `WhlPz`/`WhlVz` dangle on the default branch. Caught by the same
  connectivity check.

**What did not work, so the next attempt does not repeat it:**

1. Making every model declare the same literal. They already do -- all three
   declare `'1/960'` -- and the plant still negotiates `...667` while a
   harness declaring that same string negotiates `...671`. **Declared and
   negotiated are different numbers**, and only the negotiated one is
   compared.
2. Having the harness read the plant's declared `FixedStep`. It faithfully
   inherits the mismatch rather than avoiding it.
3. `get_param(mdl,'CompiledSampleTime')`. That is a BLOCK parameter and errors
   on a model. `Simulink.BlockDiagram.getSampleTimes` is the model-level API,
   and reading the negotiated period from it is where this was left.

The honest summary is that this is ULP roulette against Simulink's rate
negotiation, and it was stopped deliberately rather than solved. The physics
is done; the bookkeeping is not.

**The default path is unaffected and green** (`PLANT OK`, 702 s) with all of
the above in place.

### Aero — DEFERRED, and not for scheduling reasons

The obvious next candidate after the body is aero, since `Vehicle Body 6DOF`
already carries `Cd` and `Af`. It was looked at and deliberately not taken.

The block's aero is **drag only** -- one coefficient and a frontal area. Our
`IFSSIM_Aero` models:

- drag along the velocity vector, `CdA`;
- **downforce** `ClA`, split front to rear by an aero balance `abal`;
- the pitch couple, from application points at `aF` and `-bR`;
- the drag moment about the CoP height above the CoG;
- downforce applied **normal to the floor**, not along world -Z, so a rolled
  car does not get its downforce pointing 5 degrees wrong exactly when it is
  cornering hard and downforce matters most.

None of that survives the swap. For a Formula Student car with wings,
downforce and aero balance are the aero department's primary levers, and the
whole point of this plant is to let each department study its own parameters.
Taking the block here would trade a model built for this car against a
generic drag term.

So the block's aero stays **disabled** (`Cd = 0`, `Af = 0`) and ours stays.
Revisiting is worthwhile only if the Vehicle Dynamics Blockset ships a
DEDICATED aero block carrying downforce and balance -- that library has not
been enumerated yet, and this note should not be read as saying it does not.
The test is simple: if a candidate block cannot express `ClA` and a front/rear
balance, it is not a replacement for what is here.

**Step 5 — FMU export and the engine** (after step 3, not step 4). Re-export, re-run the importer gates,
and A/B the shadow FMU against Chaos with `tools/fmu/ab_plant.py`.

### Step 5a — re-exported, all required gates pass

The checked-in FMU was from before steps 2, 3, 4a and 4b. Re-exported from the
default plant and put through `tools/fmu/inspect_fmu.py`:

```
FMI 3.0, Simulink R2025b, binaries ['aarch64-darwin'], sourceCode present
internal step 0.001041666666666667, event mode true
variables: 22 input, 22 output, 57 parameter
  [PASS] Co-Simulation present
  [PASS] canGetAndSetFMUState
  [PASS] internal step divides 1/60   -- 16.000000 substeps
  [PASS] binary for this host (Darwin/arm64)
  [WARN] multi-instance declared
```

The WARN is pre-existing and honest: Simulink declares the FMU
single-instance-per-process because its generated code is non-reentrant. The
inspector's own note is the right one -- that is fine for IFSSIM **only if**
sequential reload works, and that has to be PROVEN with
instantiate/free/instantiate rather than assumed or waved away by overriding
the flag.

**Now proven.** `tools/fmu/reload_probe/run.sh` dlopens the FMU exactly as
`FSDSFmi3.cpp` does and runs three full cycles -- instantiate, initialise,
60 `fmi3DoStep` calls, free -- in a single process:

```
  [ok  ] cycle 1: instantiate, init, 60 steps, free
  [ok  ] cycle 2: instantiate, init, 60 steps, free
  [ok  ] cycle 3: instantiate, init, 60 steps, free
  Sequential reload WORKS: 3 full cycles in one process.
```

Three cycles and not one, deliberately: a single instantiate/free proves
nothing about state left behind, and the failure being looked for shows up on
the SECOND instantiate, after the first instance's statics have been touched.
Stepping matters for the same reason -- instantiating is the cheap half; it is
`fmi3DoStep` that touches the generated code's statics.

The probe was checked against a negative control (a wrong instantiation
token), which it correctly fails. That first attempt also exposed a flaw in
the probe's own reporting: a cycle-1 failure was being announced as a reload
defect, when it means the FMU never came up at all and says nothing about
reentrancy. It now distinguishes the two, because a tool that misdiagnoses its
own negative control will misdiagnose a real one.

So the constraint is real but not binding: **the FMU cannot host two cars at
once, and does not need to. It can be dropped and reloaded in one session,
which is what the platform actually does.**

### Step 5b — BLOCKED on a simulator run, and deliberately not started

`ab_plant.py` computes nothing itself. It parses the `FSDS Plant shadow:`
lines the UE pawn emits once a second into

```
~/Library/Logs/Unreal Engine/IFSSIMEditor/IFSSIM.log
```

That file does not exist, and there is no fixture standing in for it. The A/B
therefore needs a Play session in UE with the shadow FMU loaded, which is the
operator's to start, not this work's.

Worth keeping the tool's own framing when it does run: divergence there is
**not a defect count**. The FMU reproduces the `settings.json` car; Chaos has
three arcade assists, a snap-to-ground wheel model and a hidden aero model.
The Chaos trace is a reference trajectory, not a target. The question it
answers is whether the two are in the same regime and where they part company.

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
