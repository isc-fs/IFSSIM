![ISC Logo](http://iscracingteam.com/wp-content/uploads/2022/03/Picture5.jpg)

# IFSSIM

A Formula Student Driverless simulator. Unreal Engine 5.7 provides
the world — terrain, a Hesai-class LiDAR, cones, a full
FS-Rules-2026 referee — and the vehicle dynamics are a separate,
swappable plant. A ROS 2 bridge and a Mission Control UI sit on top.
Developed by ISC Racing Team as the test bench for the autonomy stack
that runs on their IFS-08 race car.

The simulator is split along one deliberate seam:

| | owns | lives in | worked on with |
|---|---|---|---|
| **Platform** | terrain, sensors, cones, referee, rendering | `Plugins/FSDSPlugin/` | Unreal Engine, C++ |
| **Plant** | vehicle dynamics — tyres, suspension, powertrain, aero | `matlab/plant/` | Simulink, exported as an FMU |
| **Autonomy** | perception, SLAM, planning, control | `pipeline/` (submodule) | ROS 2, Python |

The seam between platform and plant is the `IFSDSPlant` interface: SI
units, ISO 8855 body frame, ENU world. Two implementations exist
today — Chaos Vehicles (the default, and the validated reference) and
the Simulink FMU (running alongside it as a shadow). See
[`docs/fmu_plant_migration.md`](docs/fmu_plant_migration.md) for why,
and what still has to be true before the FMU takes over.

[![CI](https://github.com/isc-fs/IFSSIM/actions/workflows/ci.yml/badge.svg?branch=dev)](https://github.com/isc-fs/IFSSIM/actions/workflows/ci.yml)
[![release](https://img.shields.io/github/v/release/isc-fs/IFSSIM?display_name=tag&sort=semver)](https://github.com/isc-fs/IFSSIM/releases/latest)

---

## I just want to run it

→ **[`docs/SETUP.md`](docs/SETUP.md)** — 20–45 min, end-to-end first
time setup. Windows and macOS in parallel. Covers prereqs, clone,
sim build (or pre-built download), Docker stack, first session in
Mission Control, troubleshooting.

Once you've finished SETUP, **[`docs/OPERATING.md`](docs/OPERATING.md)**
is the daily-ops reference: recording MCAP bags, refreshing the bridge
after a source edit, switching tracks, common-failure fixes.

## I want to wire my autonomy code to it

→ **[`docs/AUTONOMY.md`](docs/AUTONOMY.md)** — end-to-end architecture
(sim → bridge → SLAM / planning / control → Mission Control), the
integration contract, the topics and frames the bridge guarantees.
The same code that runs on the real IFS-08 runs against this sim.

## I want to change how the car drives

→ **[`matlab/plant/README.md`](matlab/plant/README.md)** — the vehicle
dynamics model, in Simulink. Tyres, suspension, steering, powertrain,
aero, brakes, each its own block with its own owner. Built from
scripts so a regenerated model is reviewable in a diff, and
parameterised from `settings.json` so there is one source for every
number. This is the entry point if you work on dynamics, powertrain
or braking rather than on the simulator itself.

## I want to know what every knob does

→ **[`docs/REFERENCE.md`](docs/REFERENCE.md)** — exhaustive technical
reference: every sensor and its noise model, the RPC API, the wire
format, the ROS 2 topics with types and rates, vehicle physics
parameters, configuration schema, coordinate conventions.

## I want to see what changed between releases

→ **[`CHANGELOG.md`](CHANGELOG.md)** — release-level human-readable
delta. Known limitations live there too — read it before reporting
"the IFS-08 CoG looks off".

## I want to contribute

→ **[`docs/CONTRIBUTING.md`](docs/CONTRIBUTING.md)** — branch flow,
CI gate, release process, conventions.

---

*ISC Racing Team — IFSSIM*
