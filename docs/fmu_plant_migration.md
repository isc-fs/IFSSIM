# Moving the vehicle plant out of Chaos and into a team-authored FMU

**Status:** design proposal, not yet approved
**Date:** 2026-08-24
**Basis:** four read-only codebase audits, external FMI research, three independent
boundary designs, three adversarial critiques, and a fact-check pass — plus direct
verification of the load-bearing engine claims.

## Evidence convention

Used throughout. Do not remove it: this document exists to steer months of student
effort, and the difference between "verified" and "sounds right" is the whole value.

| Tag | Meaning |
|---|---|
| `[V]` | Opened the file and confirmed it |
| `[A]` | Asserted by audit with a `file:line` citation, not re-opened |
| `[R]` | External research, fact-checked |
| `[?]` | **Unverified.** Must not be stated as fact anywhere downstream |

---

## 1. The decision

**Cut the boundary at the whole chassis.** The FMU integrates the 6-DoF body,
suspension, tires, wheel rotational dynamics, driveline, powertrain, brakes, steering and
aerodynamics. The platform keeps terrain, the road probe, collision bodies, the referee,
all sensors, `FSDSRandom`, the bridge, missions, the fixed-timestep clock, and rendering.

**Get there interface-first.** The first deliverable is not an FMU. It is an `IFSDSPlant`
seam with today's Chaos code behind it as the first driver, plus a trajectory-determinism
harness built *before* anything moves.

**Named fallback:** cut at the wheel hub instead (FMU emits hub torques, Chaos keeps body
and tire). The decision point is Phase 4's exit criterion. This is a fallback, not a
waypoint.

### Why not the hub-torque cut

It is the lower-risk boundary and it deserves a fair hearing, but its ceiling is lower
than it looks. Two verified facts close it:

**Chaos does not integrate wheel speed from torque while the wheel is in contact.**
`[V]` `WheelSystem.cpp:217-224`:

```cpp
if (bInContact)
{
    float GroundOmega = GroundVelocityVector.X / FMath::Max(Re, KINDA_SMALL_NUMBER);
    Omega += ((GroundOmega - Omega + SlipOmega));
}
```

A full snap to contact-patch speed, with no `DeltaTime` on the correction. Under braking,
`WheelLocked` gives `Omega *= 0.9f` `[V]` — an arcade decay, not lockup. **Longitudinal
slip ratio is structurally not modelled**, and an FMU sitting upstream of this can never
fix it. It would carry rotor and driveline inertia that acts on nothing.

**Chaos post-processes the FMU's torques with rules the FMU cannot see.** `[V]`
`bool Braking = BrakeTorque > FMath::Abs(DriveTorque)` (`:92`), and the braking branch
uses the brake force *alone*. An FMU emitting simultaneous regen and EBS has one of the
two **silently discarded, never summed** — and the IFS-08 blends exactly those. Also
`[V]` `ETorqueCombineMethod::Override` sets drive *and* brake in one branch (`:62-67`),
so there is no way to override drive while leaving brake alone.

And the tire is a 1-D lateral slip curve with a scalar friction circle `[V]`
(`WheelSystem.cpp:84`) — no combined slip, no relaxation length, no camber thrust, no
load-sensitive μ. For a Formula Student team the tire is usually the subsystem they most
want to own, and this boundary hands them everything except it.

### Why interface-first rather than a direct spike

The platform **cannot currently measure trajectory-level determinism at all** `[A]`. Migrate
first and there is no way to tell an FMU regression from a migration bug. The harness is
also the single most valuable artifact here — it ships value even if the FMU never arrives.

---

## 2. Findings that change the picture (verified this session)

### 2.1 BLOCKER — Chaos squares and rate-limits the steering command

`[V]` `ChaosVehicleMovementComponent.cpp:632-634`:

```cpp
SteeringInputRate.RiseRate = 2.5f;
SteeringInputRate.FallRate = 5.0f;
SteeringInputRate.InputCurveFunction = EInputFunctionType::SquaredFunction;
```

The chain is `SetSteeringInput` → `RawSteeringInput` → rate-limited via
`InterpInputValue` (`:1218`) → `CalcControlFunction` (`:1765`) → solver. `[V]`
`SquaredFunction` returns `sign(x)·x²` (`ChaosVehicleMovementComponent.h:435`). `[V]` A
repo-wide grep over `Plugins/FSDSPlugin/Source` for `SteeringInputRate|ThrottleInputRate|
BrakeInputRate|InputCurveFunction|RiseRate|FallRate` returns **nothing** — engine defaults
are live. `[V]` The pawn drives it through `SetSteeringInput` at `FSDSVehiclePawn.cpp:793`.

So the actual road-wheel angle is `MaxSteerAngle · sign(s)·s²`, where `s` is rate-limited
such that 0 → full lock takes ~0.4 s.

**A 0.5 command produces 0.25 of full lock.**

Consequences:

- Every parity baseline captured before this is fixed is measured through a squaring
  nonlinearity plus a 0.4 s lag.
- It dwarfs the reverse-Ackermann defect fixed in `96e0e4e`, which was ~15%.
- **It is a strong candidate mechanism for the Stanley limit cycle** in this repo's
  history: small corrections are squared toward nothing, the controller winds up, and it
  saturates at ±1 — where squaring is the identity, so the loop gain jumps discontinuously.
  This is a live autonomy hypothesis, independent of any FMU work, and it should be tested
  first.

`[V]` `ThrottleInputRate` and `BrakeInputRate` are `LinearFunction` but carry
`RiseRate=6 / FallRate=10`, so they rate-limit. They matter the moment brakes go through
`SetBrakeInput`.

### 2.2 Twelve wheel parameters run at engine defaults nobody chose

`[V]` Neither wheel class sets: `SpringPreload` (50), `RollbarScaling` (0.15),
`CorneringStiffness` (1000), `MaxWheelspinRotation` (30), `SlipThreshold` (20),
`SkidThreshold` (20), `SideSlipModifier` (1.0), `bAffectedByBrake` (true),
`MaxHandBrakeTorque` (**3000 N·m/wheel**), `SuspensionForceOffset` (0), `SuspensionAxis`,
`SuspensionSmoothing` (0).

`RollbarScaling = 0.15` is a live anti-roll bar nobody chose. `SpringPreload = 50` shifts
static ride height. And the EBS deceleration the entire safety case rests on is an engine
default.

### 2.3 `FWheelStatus` lacks wheel omega and steer angle — `PhysicsVehicleOutput()` has them

`[V]` `FWheelStatus` has no `AngularVelocity` and no `SteeringAngle`
(`ChaosWheeledVehicleMovementComponent.h:78-140`). `FWheelsOutput` does
(`ChaosVehicleManagerAsyncCallback.h:224-280`). The trace harness must read the latter.
`SteeringAngle` there is in **degrees**; any contract annotating it `[rad]` is wrong.
`SlipAngle`'s unit at the fill site is `[?]` — check before use.

### 2.4 Correction to an agent claim

One critique warned that Chaos-side ABS/TC might be clipping brake torque. `[V]`
`bABSEnabled` and `bTractionControlEnabled` are not assigned in `UChaosVehicleWheel`'s
constructor, so they zero-initialise **false**, and the FSDS wheel classes never set them.
They are inactive. Log them, but this is not a live hazard.

---

## 3. Signal contract

Conventions, declared once and asserted at the seam: plant world frame right-handed,
Z-up, **metres**, ENU. Body frame ISO 8855 / REP-103 — x forward, y left, z up. Angles
rad, force N, torque N·m. Every UE-cm and left-handed conversion lives in **one** adapter.
Wheel order **FL=0, FR=1, RL=2, RR=3**, unchanged `[A]`.

### 3.1 Platform → Plant

| Signal | Units | Rate | Notes |
|---|---|---|---|
| `cmd_throttle` | – [0,1] | 60 | regen fold moves *into* the plant |
| `cmd_regen` | – [0,1] | 60 | separate: capped by cell current, not motor envelope. Wire stays one signed value |
| `cmd_steer_norm` | – [-1,1] | 60 | **stays normalised at the seam** — see §3.4 |
| `cmd_ebs_latch` | bool | 60 | latch is platform; the torque it commands is plant |
| `cmd_handbrake` | bool | 60 | distinct from EBS, so the 3000 N·m default dies |
| `road[4].valid` | bool | 60 | distinguishes airborne from "road at z=0" |
| `road[4].plane_point` | m, ENU | 60 | **patch, not point** — §4.2 |
| `road[4].plane_normal` | unit | 60 | |
| `road[4].plane_residual` | m | 60 | plane-fit quality; plant must detect a bad fit |
| `road[4].mu` | – | 60 | 0.7 uniformly today; the `1/0.7` compensation must die in the same commit |
| `road[4].surface_id` | uint8 | 60 | tarmac / kerb / grass / gravel |
| `external_wrench.{force,torque,point}` | N, N·m, m | 60 | cone/barrier reaction, from **cone-side** impulse |
| `chassis_grounded` | bool | 60 | nose-down / rollover — **missing from all three designs** |
| `gravity` | m/s² vec | on change | must be told, not assumed |
| `dt` | s | per step | literal `1.0/60.0`, never `FApp::GetDeltaTime()` |
| `sim_time` | s | per step | |
| `reset_request` | pose + flags | event | one entry point; all RPC handlers route through it |
| `scenario_seed` | int32 | event | only if the plant has a stochastic term |

### 3.2 Plant → Platform

| Signal | Units | Rate | Notes |
|---|---|---|---|
| `pose_world.{position,quat}` | m ENU, quat | 60 | most-read signal in the codebase |
| `vel_world` | m/s | 60 | **the signal that silently becomes zero with physics off** |
| `vel_body` | m/s | 60 | publish both; don't make the platform rotate |
| `omega_body` | rad/s | 60 | two consumers apply *different* sign flips today; contract must pick one |
| `alpha_body` | rad/s² | 60 | **new, required** for lever-arm translation of offset sensors |
| `accel_proper_body` | m/s² | 60 | **at the body origin**, not a sensor mount — §3.4 |
| `wheel[4].omega` | rad/s | 60 | `/motor_rpm` is currently faked from chassis v_x |
| `wheel[4].steer_actual` | rad | 60 | `/steering_angle` is currently a command echo |
| `wheel[4].fz` | N | 60 | `/tire_loads` |
| `wheel[4].{fx,fy,slip_ratio,slip_angle}` | N, – | 60 | **no Chaos analogue at all** |
| `wheel[4].susp_travel` | m | 60 | closes the loop on the parametric heave/pitch terms |
| `wheel[4].in_contact` | bool | 60 | |
| `wheel_visual[4]` | m + quat | 60 | Chaos animates bones for free today; **nothing will afterwards** |
| `chassis_attitude` (φ, θ, z) | rad, m | 60 | currently caller-supplied off the wire; becomes real state |
| `motor_rpm` / `motor_shaft_torque` / `motor_power` | RPM, N·m, W | 60 | signed; negative torque = regen |
| `gear`, `max_rpm` | int, RPM | low | **on the `getCarState` RPC contract; all three designs omitted them** |
| `plant_ok`, `plant_status` | bool, enum | 60 | replaces `IsSimulatingPhysics()`; failure branch **logs**, never publishes zeros |
| `step_cost_us` | µs | 60 | diagnostic — **must be excluded from any determinism hash** |
| `mass_properties_readback` | kg, m, kg·m² | once | successor to `VerifyAllWheelConfigsApplied` |

### 3.3 Coverage against the audit

The audit found exactly four genuinely Chaos-vehicle-owned reads `[A]`:
`GetWheelState().SpringForce` → `wheel[4].fz`; `GetCurrentGear()` → `gear`;
`GetEngineMaxRotationSpeed()` → `max_rpm`; `GetEngineRotationSpeed()` → dead fallback,
delete rather than port. Rigid-body reads map to `pose_world` / `vel_*` / `omega_body`.
All covered.

### 3.4 Four contract decisions that reverse a design's position

1. **`cmd_steer_norm` stays normalised.** A norm→rad→norm round-trip is not the identity
   and would break the Phase-3 bit-identity gate on the first run. Add `cmd_steer_rad` as
   a parallel field for non-Chaos drivers; land unit normalisation as its own commit after
   the gate passes once. Note three constants disagree today — wheel CDO 28°,
   `settings.json` 28, bridge default `max_steering_angle_rad = 0.5` `[A]`.
2. **`accel_proper_body` is at the body origin, plus `alpha_body`.** Defining it at a
   sensor mount either violates the platform-owns-sensors boundary or is unimplementable
   without α×r. `[A]` The IMU has no mount offset today, so body-origin is correct now and
   stays correct when one is added.
3. **One owner per shared constant.** `MaxSteerAngle`, `GearRatio`,
   `DrivetrainEfficiency` currently appear on both sides. `[A]` The runtime already ignores
   `settings.json`'s `DrivetrainEfficiency` in favour of a hardcoded `constexpr 0.92` — do
   not carry that fork across.
4. **`thermal_derate`/`overload_j` are the wrong state-restore witness.** `[A]`
   `EmraxMotor.cpp:162-166` hardcodes them every step — they are constants that match by
   accident every time. Use chassis position after 20 s of scripted input, plus wheel omega
   and suspension deflection.

### 3.5 Deliberately unresolved

- `SlipAngle` units at the Chaos fill site `[?]`.
- Whether the plant must output predicted trace endpoints or the probe can derive them.
- **Barrier contact is out of contract.** A 275 kg car at 10 m/s into a rigid fence
  resolves in well under one communication step. Declare it a run-terminating referee
  event, in writing.
- Tire thermal, tire wear, HV battery: out of scope for v1, stated explicitly.

---

## 4. Step scheduling and determinism

### 4.1 Rates

```
UE game/render tick ....... 60 Hz   (bUseFixedFrameRate=True)   UNCHANGED
FMU communication step .... 60 Hz   (h = 1/60 s), 1 doStep per game tick
FMU internal fixed step ... 960 Hz  (16 substeps)
```

**60 Hz communication, not 240.** `[A]` The sim clock advances once per game tick, so four
sub-frames would carry identical timestamps; sensors tick once per frame; and `[A]` the only
terrain that exists is a single flat plane, so patch extrapolation error is exactly zero.
Spend the saved complexity on the contact channel.

**960 Hz internal.** `[R]` FMI 3.0 guidance is that the communication step be an integer
multiple of the internal step, so 1000 Hz (16.667 substeps) is out. `[A]` `EmraxMotor.cpp:147`
computes `Alpha = min(Dt/max(τ,1e-4), 1)`; at 480 Hz with τ = 1.5 ms, `Dt/τ = 1.389` — still
clamped, so the current loop still does not exist. **960 Hz is the first 60-divisible rate at
which the motor lag is modelled at all.** Declare the fastest represented time constant as a
contract property and assert `Dt < τ` for every first-order lag.

### 4.2 Where it runs

`TG_PrePhysics`: latch controls → road probe → `Plant->PreStep()`. Engine physics
integrates between. `TG_PostPhysics`: `Plant->PostStep()` → publish snapshot → impose pose
(kinematic drivers only). Sensors keep ticking in `TG_DuringPhysics`.
`[A]` `FFSDSUdpBroadcaster` keeps running as an engine-ticked `FTickableGameObject` —
**it is not dead code and needs no explicit caller.**

**`PreStep`/`PostStep`, not `Step`.** This exists so the Chaos driver is a real
implementation rather than a wrapper around a lie, which is what makes the Phase-3
bit-identity gate achievable. It is a leaky abstraction — under Chaos the engine owns
integration, between the two calls — and this document says so rather than hiding it.

**Loop breaking:** the probe uses wheel poses from the end of the previous step. Explicit,
documented, one-step extrapolation. `[R]` FMI's only sanctioned alternative is
`GetFMUState` + repeated `doStep`, which does not fit a frame budget.

**The probe must be ≥3 traces per wheel with a least-squares plane fit and a published
residual.** A single line trace returns one point and one *face* normal, which is piecewise
constant and jumps at triangle edges — precisely the ringing the patch exists to avoid. Do
not ship "patch" as a rebranded point query.

### 4.3 Determinism: preserved, strengthened, and newly at risk

**Preserved.** `[A]` Today: fixed 60 Hz, no substepping, no async physics — exactly one
16.667 ms Chaos step per game tick on the game thread. New invariant: **N game ticks == N
`doStep` calls of exactly 1/60 s**, enforced by passing the literal constant and asserting
`FApp::GetFixedDeltaTime() == 1.0/60.0` at BeginPlay. The plant's step size stops being
inherited from the frame rate — strictly better than today.

**`doStep` runs inline on the game thread.** The standard FMI advice to move it to a worker
is wrong *for this project*: it makes the number of frames between input-set and output-read
depend on machine load, which is exactly the nondeterminism this platform just spent a
season removing. If 16 substeps do not fit, drop to 480 or 240 Hz before reaching for a
thread.

**Hard FMU gates**, checked in the real `modelDescription.xml` *and* empirically:

- `canGetAndSetFMUstate` = true. `[R]` **Spelling trap:** FMI 2.0 spells it
  `canGetAndSetFMUstate` (lowercase *s*), FMI 3.0 `canGetAndSetFMUState` (capital *S*). A
  parser matching one reads false on the other and silently disables the entire
  deterministic-reset path. `[R]` This is *not* an FMI 3.0 differentiator — FMI 2.0 CS has
  the full family.
- Fixed internal step, with `(1/60)/step` an integer within 1e-9. Reject variable-step
  FMUs — a data-dependent substep count destroys the step-count invariant
  `simContinueForTime` relies on `[A]`.
- `canBeInstantiatedOnlyOncePerProcess` = false. `[R]` `exportToFMU`'s
  `supportMultiInstance` defaults **off**; without it, editor reload without a process
  restart is impossible.

**Newly at risk.**

- **Cross-machine bit-reproducibility gets worse.** One binary becomes two with a
  floating-point ABI between them. Mitigation: the built `.fmu` is the versioned artifact —
  hash it and the GUID into the run manifest, pin compiler flags, forbid `-ffast-math` and
  FMA contraction, refuse parity claims across different `.fmu` hashes.
- **Plant determinism ≠ observable determinism.** `[A]` The TCP sensor path is paced by a
  wall-clock `Sleep(0.0025f)` re-emitting a cache that changes at 60 Hz, so duplicate-frame
  count is machine-dependent. **Hash sim-side, never from recorded topics.**
- **Roughly two-thirds of the reset gaps live outside the plant** `[A]`: `GpuScanCounter`
  never restarting, cone-yaw keyed on seed value rather than generation, knocked-over cone
  bodies never restored, `Referee::ResetState` wiping cone registration without
  re-registering, `UWorld::GetTimeSeconds` not rewindable, the `PreviousVelocity`
  acceleration spike, and the bridge's latched stamps. **Fix these first and independently** —
  perfect FMU state serialization still yields non-repeatable runs while they stand.

---

## 5. Phases

Each ships alone, is independently valuable, and has an exit criterion that can fail.

### Phase 0 — Baseline hygiene (no interface, no FMU)

Every item changes the number the baseline captures, so it lands first. All are bug fixes
on their own merits.

- **Set the six input-rate/curve fields** to linear and effectively instant (§2.1). Re-run
  the autocross fixture and **report whether steering feel changes** — this may be a live
  autonomy bug.
- **Zero Chaos's hidden aero.** `[A]` `ApplyAerodynamics` runs unconditionally from
  untouched defaults (Cd = Cl = 0.3, DragArea 2.52 m²) *on top of* the pawn's own aero →
  ~1.84× drag, ~1.42× downforce, in disagreeing frames. Make "the platform applies zero
  aerodynamic force" a tested invariant.
- **Settle 290 vs 275 kg.** `[A]` `Mass = P.Mass` is assigned in BeginPlay *after*
  `UpdateMassProperties` already ran. Log `GetBodyMass()` at first Tick. A 5% mass error
  masquerades as a tire/powertrain modelling error for a whole campaign.
- **Enumerate and dispose of the twelve inherited wheel defaults** (§2.2) plus the three
  arcade-assist sims `[A]`. If any is non-neutral the FMU should **not** reproduce it — but
  the team must know it was there.
- **Re-apply and re-verify wheel config after every `ResetVehicleState()`** `[A]`, and add
  `ResetVehicleState()` to `resetScenario`, which currently omits it. `run_scenario.py`
  calls `load_track` before every run, so **today's benchmarks may come from a
  CDO-configured car.**
- **Referee reset gaps.** `[A]` `ResetState()` empties cone registration with no
  re-register → DOO, OC and lap detection are **permanently dead after the first reset**.
  Any A/B campaign built on `resetScenario` records 0/0/0 for every run after the first.
- **Sensor-side reset gaps:** key cone-yaw on `GetGeneration()`, reset `GpuScanCounter`,
  add a bridge-side stamp reset.
- Fix stale comments that would otherwise be ported as fiction `[A]` — a non-existent
  `ApplyPhysicsSettings()` referenced three times, a "MaxTorque was zeroed" comment (it is
  643), and a Tier-1 carve-out citing a "never set" `SpringRate` that `4c5ec82` now sets.

**Exit:** all merged; a written record of the actual runtime values of every inherited
default; a `resetScenario` that leaves the referee functional, proven by a two-run script
scoring DOO on both runs.

### Phase 1 — The instrument (still pure Chaos)

Scripted deterministic `(t, throttle, steer, regen)` driver replacing RPC control when
armed `[A]`. Per-step sim-side trace over an explicitly enumerated field list, read from
`PhysicsVehicleOutput()` (§2.3), hashed FNV-1a over **raw bits**, with `step_cost_us` and
every diagnostic excluded by construction.

**Two hashes, not one:** raw-bit for identity gates, and a separately named *quantized*
hash for cross-driver A/B. Never let the quantized one satisfy an identity gate — that is a
smuggled tolerance.

**Exit:** two fresh sessions, same seed and script, produce identical raw-bit hashes; a
deliberately perturbed run (290 → 275 kg) produces a different one. **This is the first
trajectory-level determinism test the platform has ever had**, and it ships value even if
the FMU never arrives.

### Phase 2 — Throwaway second driver, provisional interface

~200 lines of dynamic bicycle plus the existing EMRAX model, driven through a draft
`IFSDSPlant`, pawn kinematic. Purpose: discover what an imposed-pose plant actually breaks
— cone knock-over, referee, every sensor, `/tire_loads`, wheel bone animation, Gear/MaxRPM,
the silent-zero guards. **Expect to throw the code away and keep the findings.**

**Exit:** a written list of every platform site that broke, which becomes the Phase-3
contract. Zero FMI dependency, zero MATLAB question, zero waiting on anyone.

### Phase 3 — The seam, behaviour-neutral

`IFSDSPlant` + `FFSDSChaosPlant` wrapping existing code verbatim. Repoint every read site,
including the five omitted from the original design list — LiDAR (CPU and GPU paths),
distance sensor, barometer, magnetometer, and the referee's OC test `[A]` — each tagged
**geometry-coupled** (must use the pose the mesh occupies) or **state-coupled** (must use
the snapshot). Ban `GetVelocity()` and `GetPhysicsAngularVelocityInRadians()` from sensor
and bridge code. Invert all five `IsSimulatingPhysics()` guards `[A]` to an explicit
`plant_ok` that logs on failure. Merge the two duplicated frame packers and **deliberately
resolve** the GssVelY sign divergence (UDP negates, TCP does not) `[A]`.

**Exit:** Phase-3 raw-bit hash **==** Phase-1 hash. Bit-identical or it does not ship.
*If bit-identity cannot be recovered within ~2 days, stop and escalate — do not accept a
tolerance. The harness stops being a decision procedure the moment you do.*

### Phase 4 — Shadow instrumentation, zero behaviour change

Road probe and cone-side contact accumulator both run and both record into the trace;
**neither feeds the plant.** Built and validated against a **deliberately non-flat test
level (ramp + crowned surface)** — a stub returning z=0, normal=+Z, μ=0.7 passes every test
that exists today.

**Exit — this is the boundary decision point:** trace hash unchanged; probe-vs-Chaos
contact agreement quantified on ramp and crown; a written go/no-go on whether the patch is
stable enough to feed a 61 kN/m spring loop. **No-go ⇒ fall back to hub-torque**, which
costs an interface change and nothing else.

### Phase 5 — FMI importer, no vehicle model

`[R]` Use **Modelon FMI Library** (BSD-3, FMI 2.0 + 3.0, macOS in CI, 3.0.4 as of
2025-06-25). **Not fmi4c** — CI is Windows and Ubuntu only, no macOS job, effectively
single-maintainer. **Not FMI4cpp** — archived, FMI 2.0 only. **Not the ORNL UE plugin** —
`[R]` no LICENSE on any UE5 branch, though the UE_4.27 branch carried an explicit Apache-2.0
grant that the UE5 branches silently dropped. Describe it as *inconsistently licensed, ask
the maintainers*, **not** as all-rights-reserved. Read it for architecture; do not copy.

`[R]` Extract `resources/` to a real filesystem path — the spec requires it available in
extracted form for the instance lifetime; it **cannot** be mounted from a `.pak`. Handle
both binary-directory naming schemes (`darwin64` vs `aarch64-darwin`). Validate against
`modelica/Reference-FMUs`. Use **FMPy** as an offline oracle: same scenario through FMPy
and through the UE5 importer, diff trajectories. Ship a **no-FMU degraded mode** — `[A]` the
existing `UFloatingPawnMovement` fallback is the template.

**Exit:** Reference FMUs load, step, and `GetFMUState`/`SetFMUState` round-trip
bit-identically on all three host platforms the team uses.

> **STATUS `[V]` 2026-08-24 — the core of this phase is DONE on macOS/arm64.** An FMU
> exported by Simulink R2025b loads, instantiates, steps and round-trips its state inside
> IFSSIM, driven over the `fmuSelfTest` RPC:
>
> ```
> fmiVersionReported  3.0
> library             fsds_fmu_spike.dylib   (aarch64-darwin)
> stepSize            1/60 s
> yAfterFirstStep     0.015624999999999998   = 15/960, forward Euler, exact
> yRunA / yRunB       0.18229166666666632    = 175/960, bitwise identical
> stateRoundTrip      true
> ```
>
> Two things are established rather than merely observed. First, the arithmetic is
> **right**, not just stable: with a unit input into a 1/960 s forward-Euler integrator
> the reported output is `(N-1)/960` after `N` substeps, and 16 substeps per
> communication step is exactly what `fixedInternalStepSize` declares — so the FMU is
> genuinely stepping at its declared internal rate. Second, `GetFMUState`/`SetFMUState`
> reproduced **bitwise**, compared as raw `uint64` with no tolerance, because a tolerance
> would accept an FMU that restores approximately, which is not restoring.
>
> **This is the primitive `resetScenario` never had.** The platform's own reset restores
> four hand-enumerated fields and does not touch Chaos solver state at all; an FMU plant
> replaces that with an opaque blob that provably round-trips.
>
> Deliberately NOT proven yet: Linux and Windows hosts, an FMI 2.0 FMU (this binding is
> 3.0 only, by design), and sequential instantiate/free/instantiate — the reclassified
> multi-instance question from §7, which still needs its own probe.

### Phase 6 — Parity FMU v0

A hand-written C FMU built by the project's own CMake, reproducing **the `settings.json`
car, not Chaos**. Reproducing Chaos would mean reverse-engineering three arcade-assist
sims, a snap-to-ground wheel model and a hidden aero model — a research project, not a
migration step. **The Chaos trace is a reference trajectory, not a target.**

Kinematic pawn swap lands here: `SetSimulatePhysics(false)`, mesh moved as a **kinematic
target** (`ETeleportType::None`, **no sweep**) via `BodyInstance::SetBodyTransform`;
contact wrench recovered from **cone-side** `OnComponentHit` `NormalImpulse` `[V]` (cones
*are* simulating, so it is populated). Per-wheel cylinder overlaps for cone knock-down;
road probe excludes the PhysicsBody channel — and this document states plainly that doing
so forfeits exact comparability with Chaos's channel trace.

**Exit:** `ab_plant.py` reports max/RMS divergence in position, yaw, v_x and per-wheel Fz
against the Chaos baseline on the 2026-07-12 gold-standard autocross fixture, with **both
controls green** — a positive control (recorded-trace replay against itself must hash-match;
if not the verdict is INCONCLUSIVE, not FAIL) and a negative control (290 vs 275 kg must
diverge). A scripted cone strike scores the same DOO count before and after.

### Phase 7 — Team model replaces v0

Simulink export, fixed-step solver, state flags on, per-host binaries merged into one
`.fmu` (`[R]` spec-legal). Retune `odometry_filter` and `cone_slam` against honest wheel
odometry as a **separate, explicitly-scoped change** against the same fixture.

### Phase 8 — Retire Chaos-as-plant

Delete the CDO tire path, the movement-component subclass, both wheel classes, the aero
and EMRAX runtime paths; demote `ComputeTireLoadsParametric` to an out-of-band oracle.
**Keep `FFSDSChaosPlant` as long as it is the only validated reference — a second
implementation is not dead code, it is the control group.**

`[V]` **Reparenting note:** `AWheeledVehiclePawn` is `CHAOSVEHICLES_API`, so `ChaosVehicles`
cannot leave `Build.cs` while the base class is retained, and retaining it keeps a movement
component that still runs `CreateVehicle` from deleted wheel classes. The base contributes
exactly one useful line — `SetGenerateOverlapEvents(true)` on the mesh `[A]`, which
`USkeletalMeshComponent` defaults **off** and which the finish-line trigger depends on.
Reparent to `APawn`, create the mesh yourself, call that line explicitly, and regression-test
the finish-line overlap.

---

## 6. Risk register (blockers)

| # | Risk | Mitigation |
|---|---|---|
| B1 | **Chaos squares and rate-limits steering** `[V]` §2.1. Every baseline is measured through `sign(s)·s²` and a 0.4 s lag. Possibly a live autonomy bug. | Phase 0. Set all six fields, add to readback set, re-measure, re-run the autocross fixture and report. |
| B2 | **Silent zeros, not errors.** `[A]` With physics off, `GetVelocity()` returns an unwritten value and three guards fall to zero branches. GSS, GPS velocity, both odom twists and the IMU gyro publish well-formed, correctly-covarianced **zeros** at full rate with nothing in the log. | Phase 3. Invert every guard to `plant_ok` with a logging failure branch — **with Chaos still running**, so the refactor is provably neutral. |
| B3 | **Cone contact mechanism.** `[V]` Swept moves truncate at the first blocking hit and `NormalImpulse` is zero for swept blocking collisions, so a naive wrench channel is identically zero forever and the poses fork permanently. | Kinematic target + cone-side impulse. Prototype and assert on a scripted cone strike **before** anything else in Phase 6. If it cannot work, the kinematic-proxy premise fails and the fallback boundary is mandatory. |
| B4 | **No determinism harness ⇒ nothing is attributable.** Only RNG seeding is proven. `[A]` The one reset test used `/imu`, later shown unusable, and reported FAIL where the honest verdict was INCONCLUSIVE. | Phases 1 and 6, including a positive control. |
| B5 | **`resetScenario` leaves the referee permanently blind** `[A]` — 0/0/0 on every run after the first. | Phase 0. |
| B6 | **`ResetVehicleState()` reverts wheel config to CDO with no re-push or verify** `[A]`. Every benchmark to date may be from a CDO-configured car. | Phase 0. |
| B7 | **Aero double-counted**, ~1.84× drag / ~1.42× downforce, in disagreeing frames `[A]`. | Phase 0. |
| B8 | **`settings.json` Mass probably never reaches the body** (290 vs 275 kg) `[A]`. | Phase 0. |
| B9 | **Road contact has no interface and flat terrain makes any stub pass** `[A]`. | Phase 4, against a ramp/crown level. This is the go/no-go. |
| B10 | **`canGetAndSetFMUstate` is optional and spelled differently in FMI 2.0 vs 3.0** `[R]`. A parser matching one silently disables state restore, turning a known-incomplete reset into an unknown-incomplete black box. | Phase 5 hard gate; handle both spellings; verify empirically with a real integrator witness. |
| B11 | **Platform binary coverage.** `[R]` A Windows-exported FMU contains `win64` only, and there is no macOS cross-compile option. Discovering this at Phase 7 wastes Phases 5–7. | **Run the export spike before Phase 5** — export a trivial FMU on every OS the team uses, unzip, list `binaries/`, load each from UE. If darwin fails, the hand-written-C-plus-CMake path (already assumed by Phase 6) becomes the plan of record. |

Major risks (M1–M14) are held in the analysis appendix: two-pose fork for geometry-coupled
sensors, honest wheel omega as a *downstream behaviour change*, cross-machine
reproducibility, the twelve inherited defaults, observable-vs-plant determinism, duplicated
frame packers, unsynchronised cross-thread reads, the two suspension calibrations, the dead
friction-brake channel and unchosen 3000 N·m EBS torque, cone-under-wheel, the `1/0.7` μ
double-application, FMI discontinuity handling, `.fmu` packaging and linkage, and student
turnover.

---

## 7. Open questions this document does not paper over

**Toolchain and licensing — not settleable from public sources.**

1. ~~**Which MATLAB release, and does the campus bundle include the FMU export product?**~~
   **ANSWERED 2026-08-24 by measurement on this machine — and the answer reverses the
   risk.** `[V]` MATLAB R2025b (maca64, Apple Silicon), licence 1088581:

   ```
   Simulink Compiler   licensed, checkout OK
   MATLAB Compiler     licensed, checkout OK
   Simulink Coder      licensed, checkout OK
   Embedded Coder      licensed
   MATLAB Coder        licensed
   ```

   **Nothing needs to be bought.** Every product FMU export requires is already
   entitled and a licence seat checks out successfully.

   The blocker is that they are **not installed**. `matlab.addons.installedAddons`
   lists only MATLAB, Simulink, Aerospace, Navigation, PDE, Robotics and UAV.
   `Simulink.FMUExporter` constructs and accepts every option, and the FMI 2 and FMI 3
   code-generation targets (`RTWCG_FMU2_target.c`, `RTWCG_FMU3_target.c`) ship in
   `matlabroot/rtw/c` — but the implementation function `exportToFMU_fcn` is absent,
   so `export()` fails with `MATLAB:UndefinedFunction`.

   **Fix: re-run the MathWorks installer with the existing licence and add the
   Compiler/Coder products. Free.** Re-check with
   `tools/fmu/matlab/check_fmu_export.m`, which separates *not licensed* (a purchase)
   from *not installed* (an installer run) from *not present* (wrong release) — three
   states with completely different costs that all present as "it does not work".

   This was written up as the single biggest cost unknown of the migration. It was a
   free installer run, and it was answerable in ten minutes on the machine the
   simulator already runs on.

   `[V]` **The exporter's option surface also answers several other questions**, since
   every platform gate turns out to be an explicit export option:
   `FMIVersion`, `FMUType`, `canGetAndSetFMUStateOverride`,
   `canBeInstantiatedOnlyOncePerProcessOverride`, **`SaveSourceCodeToFMU`** (so
   source-code FMUs — the cross-platform escape hatch — are supported), and
   `GenerateLinuxBinaryWithWSL` / `GenerateWindowsBinaryWithDocker` for producing
   other hosts' binaries.

2. ~~**Which platform tuple does a Mac-hosted export write?**~~ **ANSWERED `[V]`:
   `aarch64-darwin`.** A real R2025b export on this Apple Silicon Mac produced
   `binaries/aarch64-darwin/<model>.dylib`. The loader's tuple table already accepts it
   alongside the FMI 2.0 `darwin64` spelling, so this is confirmed rather than assumed.

3. ~~**Do Simulink's fixed-step CS FMUs set `canHandleVariableCommunicationStepSize`?**~~
   **ANSWERED `[V]`: no — `false`.** Which is what this platform wants: a fixed 1/60 s
   communication step is the invariant, not a limitation to work around.

4. **Does the intended plant qualify for FMI 3.0 Event Mode?** `[V]` The exported FMU
   advertises `hasEventMode="true"`, so the release supports it. Whether a plant full of
   integrators qualifies for the one-step-delay fix is still unproven — the "direct
   feedthrough blocks only" restriction was reported for Simulink-generated FMUs and this
   spike model is too trivial to test it. Do not let Event Mode carry the FMI 3.0
   decision until a realistic model is tried.

5. ~~**Sourcing caveat: MathWorks bot-blocks the relevant pages.**~~ Superseded — the
   questions those pages would have answered were answered by measurement instead.

6. **Multi-instance: the gate was wrong, and the real question is still open.** `[V]`
   A default Simulink export sets **`canBeInstantiatedOnlyOncePerProcess="true"`**.
   Investigated properly, and the finding is not what the original gate assumed:

   - **The attribute is honest.** Simulink sets it because its default
     `CodeInterfacePackaging` is `'Nonreusable function'` — the generated C really does
     hold global state, so two *simultaneous* instances would corrupt each other. This is
     a true statement about the code, not a missing export option.
   - **`canBeInstantiatedOnlyOncePerProcessOverride` does not fix it, and should not.**
     Set to `off` the attribute stays true; set to `on`, the exporter raises a **modal
     confirmation dialog** — it is asking you to promise something unsafe. Either way the
     flag is only a *declaration*; flipping it does not make the code reentrant.
   - **Genuine reentrancy needs `CodeInterfacePackaging = 'Reusable function'`.**
     `set_param` accepts it, but `FMUExporter.export` then raises a modal dialog of its
     own, so **the reentrant path cannot be exported headlessly** — it breaks
     `matlab -batch`, and therefore CI. No `settings()` key suppresses it. `[V]` Embedded
     Coder is licensed but NOT installed, which may or may not be related; untested.

   **The gate has been reclassified from REQUIRED to a WARNING in both inspectors, with
   the reasoning recorded in the code.** This is a reclassification with a stated reason,
   not a check silenced to get a green run — the distinction matters, and the next person
   should be able to audit the decision.

   Justification: IFSSIM never wants two plants at once. It wants **sequential** reuse —
   instantiate, run a session, `freeInstance`, instantiate again in the same editor
   process. Whether the attribute forbids that is not decidable by reading XML.

   **Therefore this becomes an empirical test, not a manifest check**, and it belongs in
   Phase 5's exit criteria: *instantiate → step → freeInstance → instantiate again in one
   process, and confirm the second instance behaves identically.* If it fails, the cost is
   an editor restart per plant reload — annoying, not fatal — and the escape hatch is a
   GUI export where the dialog can be answered once.

   Do **not** close this by overriding the flag. That converts a known limitation into an
   unknown one.

**Engineering questions the plan cannot decide by fiat.**

6. **Can the team actually author a tire model?** This determines the boundary and must be
   answered by measurement, not a meeting. Run the intended tire and suspension models
   offline against a measured Fz/Fy trace from the Phase-1 baseline and check the fit across
   the roll/heave envelope of an autocross lap. If the answer is "not really", the
   hub-torque fallback is the right destination and this document says so without
   embarrassment.
7. **Is bit-identity achievable at the Phase-3 seam?** The critiques disagree. Plausible
   float-reordering sources: force-accumulator ordering, `PreviousVelocity` sampling
   relative to the solve, and a physics-thread flush boundary introduced by harvesting wheel
   state in `TG_PostPhysics`. **Pre-commit to a stop rule (~2 days)** rather than discovering
   the temptation to accept a tolerance under deadline.
8. **Chassis-vs-terrain contact** (grounded nose, kerb strike, rollover) is absent from
   every proposed contract. Either add `chassis_grounded` plus a forwarded wrench, or
   **declare it out of scope in writing** — not by omission.

---

## 8. What this fixes for free

Only claims the audit or direct verification supports.

- **The missing-Coriolis / vy-drift class of EKF bug.** `[A]` IMU linear acceleration is a
  finite difference of `GetVelocity()` over the render delta with `+980 cm/s²` bolted on,
  computed **twice independently**. A plant emitting true body-frame proper acceleration
  removes the whole class. **This is the highest-value side effect** and it is what the
  odometry-filter redesign is currently blocked on.
- **The `/imu` acceleration spike at every repeat.** `[A]` `resetScenario` zeroes physics
  velocity but not `PreviousVelocity`, so the next tick computes `(0 − v_prev)/dt`.
- **`/motor_rpm` circularity.** `[A]` Motor RPM is chassis v_x round-tripped through the
  wheel radius, which `odometry_filter` multiplies by `kRpmToMs` to recover the v_x it came
  from. Wheel slip is structurally invisible — and `[V]` **Chaos could never have fixed
  this**, because it snaps `Omega` to ground speed. Only a full-chassis plant makes it real.
- **`/steering_angle` stops being a command echo** and becomes a measurement.
- **The CDO-mutation trap, permanently.** `[A]` Native class-default mutation at
  construction, two disjoint config routes, verification covering 4 of ~12 fields — the root
  cause of a season of "configuration that never reaches the solver" bugs. FMI parameters
  are set explicitly and read back.
- **Two suspension calibrations collapse to one** — 2.3× apart until `4c5ec82`, with
  nothing detecting the divergence.
- **The unchosen safety numbers become chosen** — starting with a 3000 N·m/wheel EBS torque
  that is currently an engine default.
- **Aero moves to body frame with a real CoP.** `[A]` The moment arm was a 10× bug as
  recently as `96e0e4e`; this deserves a regression test on the aero pitch couple.

**Fixed by phases here, but not by the FMU** — say so explicitly so nobody claims credit
twice: the aero double-count, the mass question, referee reset blindness, the wheel-config
revert, the GssVelY divergence, cross-thread reads, the cone-yaw and `GpuScanCounter`
seeding gaps, and the steering squared curve. Every one stands on its own merits whether or
not an FMU ever arrives.

**Explicitly NOT fixed:** sensor tick / render tick coupling (a 960 Hz plant is still
sampled at 60 Hz by sensors that tick once per frame); observable byte-stream determinism,
unless the bridge's sensor source moves onto the game-thread transport;
`UWorld::GetTimeSeconds` not being rewindable; all pipeline-side state (EKF covariance,
SLAM map, planner); and the referee's off-course rule, which `[A]` is a crude nearest-cone
heuristic that silently disables below 11 cones and remains the weakest link in scoring
regardless of plant fidelity.
