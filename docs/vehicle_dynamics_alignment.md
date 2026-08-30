# Vehicle dynamics: what the literature does, and what we do

Written 2026-08-30 after reading the four documents in `docs/Vehicle Dynamics/`:

| Document | What it is |
|---|---|
| `TFG_Aniol_Escofet` | BCN eMotorsport, model-based torque vectoring, CAT15x |
| `Design of a torque vectoring system for a FSAE Electric vehicle` | Politecnico di Torino / UPM, UPM 03E |
| `05_Torque_Vectoring_ETH_Zurich__FSG_Academy` | AMZ Racing lecture, FSG Academy 2020/21 |
| `ME 4013 Tutorial` | Model-Based System Design course, Simulink |

They agree on a **method**. The equations are the easy part and we already had
most of them; the method is where we diverged.

## 1. There is a model ladder, and you pick a rung by job

AMZ states it outright for the suspension (slides 20-22):

| Rung | What it has |
|---|---|
| Steady State | no spring/damper element; weight transfer around the CG; aero |
| Quarter Car | spring/damper, transient effects — *"adequate description for most mechanic suspension layouts"* |
| Full Car | full kinematic layout |

and for yaw (18-19): a **bicycle model** for the yaw reference and velocity
estimation, a **full car model** only where you need per-wheel slip ratio /
slip angle and torque distribution.

## 2. Two models with different jobs — never one model doing both

Both theses build a *design* model and validate it against a *reference plant*:

| | design model | reference plant |
|---|---|---|
| Escofet | Dual-Track NTV, **algebraic** load transfer, MF tyre, no suspension states | VI-CarRealTime, 14 DOF |
| Fricano | 2-DOF linear bicycle | CarSim |

Fricano says it plainly: for the design model, *"the lateral load transfer is
considered to be influenced only by the lateral acceleration of the vehicle."*

## 3. Validation means a number, measured against a vehicle

Escofet eq. (12):

```
eps_rel = 100 * sum|psidot_real - psidot_model| / sum|psidot_real|
```

on **yaw rate**, over three manoeuvres of increasing difficulty:

| Manoeuvre | Fit |
|---|---|
| Ramp Steer | 96.74% |
| Skid Pad | 98.77% |
| AutoX | 82.52% |

## 4. Build crude, close the loop, then refine components against data

ME 4013 is a methodology course. Lecture 2 creates the motor subsystem with a
deliberately *"preposterous"* model and says **observe and assess the results**.
Lectures 4-9 are *Improved MGA, Improved GenSet, Improved Engine, Improved
Battery, Improved MG*. Experimental data enters late, to refine components.

---

## Where we align

Model hierarchy with referenced models and bus interfaces; one parameter file
(`car_spec.m` -> `settings.json` -> `ifssim_params.m`, stronger than the
tutorial's init-file pattern); Magic Formula with an enforced friction circle;
aero as `0.5*rho*A*C*v^2`; the same slip definitions; longitudinal and lateral
load transfer.

## Where we do not

**We built one model and gave it both jobs.** The Simulink/FMU plant is being
pushed toward the VI-CarRealTime rung while simultaneously being what the
pipeline is developed against. No document does this. It is why every fidelity
gap becomes a controller blocker.

**Our suspension was on a rung that is not on the ladder.** Spring/damper
transients but no kinematic layout: no roll axis, no motion ratio, no anti-roll
bar, no anti-squat/anti-dive. That is the cost of transient states without the
load transfer the free algebraic model gets. *Partly addressed* — see below.

**We have never produced a fit number.** Every check in the plant suite is
internal consistency or closed form. Not one compares against a vehicle.

**The tyre is fitted to nothing.** AMZ slide 23: manufacturer data ->
independent tests -> *fit the model to the data*. `car_spec` itself records
"the tyres dept says 1.65 at 1000 N for the Hoosier 16.0x7.5-10" while we run
mu = 1.40.

**The pipeline's vehicle model is kinematic** where all four documents use a
dynamic bicycle built on cornering stiffness. That connects directly to two
open problems: vy drift and the EKF's missing Coriolis terms.

---

## The data problem, measured

Escofet's method needs steering, speed and yaw rate from a vehicle. We have the
signals — `/steering_angle`, `/motor_rpm`, `/imu` — and
`tools/vd_validation/bag_to_vd_csv.py` extracts them from either bag format.

What we do **not** have is a run fast enough for the answer to mean anything:

| Bag | Mean speed | Max speed |
|---|---|---|
| `car_parity_autocross_*` (5 bags) | 0.92 m/s | 2.80 m/s |
| `trackA_manual_001602` | 2.54 m/s | 4.52 m/s |

Every bag in the repo is a crawl, and all are simulator recordings (they carry
`/lidar/Lidar1`, the pre-rename sim topic; the manual one also has
`/testing_only/odom`). Load transfer, tyre load sensitivity and combined slip
do essentially nothing below ~5 m/s. A fit against these bags would look
excellent and would prove nothing about the model where it matters.

**What is needed: one fast run.** A skidpad or ramp-steer, driven manually — no
autonomy, so none of the AS-state blockers apply — with `/steering_angle`,
`/motor_rpm` and `/imu` recorded. Skidpad is the highest-value single run: it is
steady state, it is a fixed 15.25 m radius, and it saturates the tyres.

## Status of the fix list

- [x] Load transfer exists at all (commit `ab456bb`)
- [x] Roll stiffness distribution — the car can now be balanced (this commit)
- [x] Anti-roll bars, derived from `RollStiffnessFront/Rear`
- [x] `HeaveStiffness` sourced from the recorded ride frequency
- [ ] Roll centres — `RollCenterFront/Rear` are still read by nothing. The
      geometric term needs the tyre lateral force back in the suspension pass,
      which is a feedback path the model does not have yet. ~17% of transfer.
- [x] A dual-track design model, separate from the reference plant
      (`matlab/design/`), validated against the plant on a ramp steer:
      99.4% / 94.4% / 91.4% at 6 / 9 / 12 m/s, mean 95.1%. For scale, the
      thesis calls 96.7% on a ramp steer satisfactory — against a real car.
- [ ] One fit number, against a vehicle, at speed
- [ ] A tyre fitted to real data (TTC, or the tyres dept's 1.65 @ 1000 N)
