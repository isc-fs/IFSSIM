![ISC Logo](http://iscracingteam.com/wp-content/uploads/2022/03/Picture5.jpg)

# IFSSIM

A Formula Student Driverless simulator built on Unreal Engine 5.7,
with a Hesai-class LiDAR, a full FS-Rules-2026 referee, a ROS 2
bridge, and a Mission Control orchestration UI. Developed by ISC
Racing Team as the test bench for the autonomy stack that runs on
their IFS-08 race car.

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
