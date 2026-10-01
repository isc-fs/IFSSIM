# Collision matrix for the trackside environment

Which queries and bodies meet which kinds of geometry, and why. This implements rules 1 and 2 of [ENVIRONMENT_ROADMAP.md](ENVIRONMENT_ROADMAP.md):
- what the LiDAR sees must match what the car collides with;
- props never count as ground, and wheels never drive on them.

| Query or body | Mechanism | Ground | `FSDSProp` (solid) | `FSDSPropLidarOnly` |
|---|---|---|---|---|
| GPU LiDAR (default) | scene-depth render of visible primitives | Hit | Hit | Hit |
| CPU LiDAR | `ECC_Visibility` trace, simple collision, default responses | Block | Block | Block |
| Cone ground snap | object-type `WorldStatic` query | Hit | no hit | no hit |
| Road probe | object-type `WorldStatic` query | Hit | no hit | no hit |
| Chaos wheel trace | `ECC_WorldDynamic` trace filtered by the car's `WheelTraceCollisionResponses` | Block | **Ignore** | **Ignore** |
| Chassis (`Vehicle`) and cones (`PhysicsActor`) | rigid-body contact | Block | Block | Ignore |

## The pieces

**Object channels** (`Config/DefaultEngine.ini`, `[/Script/Engine.CollisionProfile]`):
- `FSDSProp` is `ECC_GameTraceChannel1`, and `FSDSPropLidarOnly` is `ECC_GameTraceChannel2`.
- Both default to **Block**. Every existing profile that doesn't override them meets props, including the engine's `Vehicle` profile (the car) and `PhysicsActor` (the cones), and so does every existing query, including the CPU LiDAR's Visibility trace.
- The engine assigns slots in declaration order, and `Public/FSDSCollision.h` names them. `validateCollisionMatrix` checks that the two agree.

**Profiles:**

| Profile | Object type | Collision | Responses | For |
|---|---|---|---|---|
| `FSDSProp` | `FSDSProp` | query and physics | blocks everything | bins, bollards, timekeeping tripods, tyre walls |
| `FSDSPropLidarOnly` | `FSDSPropLidarOnly` | query only, no physics | blocks only Visibility | low kerbs and strips a wheel could reach |
| `FSDSTerrain` | `WorldStatic` | query and physics | blocks everything | the floor, a ground pad, terrain |

**The one exception, made in code:** `AFSDSVehiclePawn`'s constructor sets the wheel trace to Ignore both prop types. Without it, a solid prop's Block default would let a wheel climb a bollard as if it were ground.

## Rules for new geometry

- **Ground** uses `FSDSTerrain`, and its simple collision must equal its render surface: a Landscape with `CollisionMipLevel == SimpleCollisionMipLevel == 0`, or a mesh using complex as simple. The road probe and the cone snap only find `WorldStatic`.
- **A prop a wheel could reach** from the cone corridor must be `FSDSPropLidarOnly`, or be drivable ground (WorldStatic, steps of 3 cm or less). That covers any kerb or strip under 0.30 m. A LiDAR-only kerb is a phantom to the car, which is acceptable, because reaching it means the car has already left the corridor.
- **A prop's simple collision follows its visual silhouette.** The default GPU LiDAR sees the render mesh and the CPU LiDAR sees simple collision, so a mismatch makes the two paths disagree. It also lets the car hit, or miss, something the LiDAR showed differently.
- **Nothing overhangs the track corridor.**

## How to check it

`validateCollisionMatrix [ahead_m]` (an RPC; `tools/smoke/test_collision_matrix.py` runs it):
- **What it spawns:** a 1.0 × 1.0 × 0.6 m `FSDSProp` box 2.5 m to the left of the car's path at `ahead_m + 2`, and a 0.4 × 3.0 × 0.15 m `FSDSPropLidarOnly` strip across the path at `ahead_m`.
- **When it checks:** a few frames later, because Chaos adds new bodies to its scene queries on a later tick.
- **What it checks:** every row above except the GPU LiDAR, using the queries the sim itself makes, plus the channel slots. A control query must hit the solid prop, so a body that isn't in the scene yet can't pass as "ignored".
- **Afterwards:** the props stay, so the car can drive over the strip and a LiDAR consumer can look for them. `validateCollisionMatrix clear` removes them.

**The GPU LiDAR row** needs the LiDAR running. `tools/smoke/lidar_sees_test_props.py` (run inside `dv_pipeline_stack`) counts `/lidar_points` returns inside each prop's box, with the props spawned and then cleared:
- **The car is held still:** it engages the EBS first. With the regen-only service brake, a stopped car can creep.
- **Old scans are skipped:** it counts only scans stamped at least 0.5 s of sim time after each change. Scans stamped up to about 0.12 s after a change can still show the old scene, at least with the UE window in the background.

Measured on the Windows dev PC on 2026-10-01:

| Check | Result |
|---|---|
| `validateCollisionMatrix` | all 15 checks pass, on flat ground and on the 8° test ramp |
| GPU LiDAR at 1.74M pts/s, solid prop (5 runs) | 1,102–1,213 points per scan; 0 once cleared |
| GPU LiDAR at 1.74M pts/s, strip (5 runs) | 1,001–1,078 points per scan; 0 once cleared |
| Driving straight over the strip at 10 m/s, 2 runs | height within 0.09 cm, pitch within 0.15°, IMU vertical acceleration 9.65–9.92 m/s² |
| Same drive with no props | height 0.08 cm, pitch 0.14°, vertical acceleration 9.47–10.05 m/s² |
