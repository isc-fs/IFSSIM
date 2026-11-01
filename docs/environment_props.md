# Trackside props: meshes and LiDAR materials

The props a track's environment sidecar lists (`docs/environment_sidecar.md`)
are built by `AFSDSEnvironmentActor`
(`Plugins/FSDSPlugin/Source/FSDSPlugin/Private/Environment/FSDSEnvironmentBuilder.cpp`).
Each class uses a **real mesh** when one is installed, and otherwise falls back
to **primitives**: engine basic shapes, sized to the class's footprint. A fresh
clone has no licensed meshes and still builds, runs and passes CI.

## Real meshes

| Class | Mesh the sim looks for | Fitted to (cm, W × D × H) | Without it |
|---|---|---|---|
| `bollard` | `/Game/ThirdParty/Props/SM_Bollard` | 20 × 20 × 75 | post with a reflective band |
| `bin` | `/Game/ThirdParty/Props/SM_Bin` | 50 × 50 × 65 | cylinder |
| `tripod` | `/Game/ThirdParty/Props/SM_Tripod` | 100 × 100 × 115 | three legs and a head |
| `tyre_wall` | `/Game/ThirdParty/Props/SM_TyreStack`, else the repo's `RaceCourse/Model/Environment/tire_stacked` | 60 × 60, four side by side | four cylinders |
| `cone_stack` | the sim's small cone mesh (always) | — | — |

A mesh is scaled uniformly until it fits those bounds, centred on the prop and
stood on the ground by the bottom of its bounds, so its pivot and import scale
don't matter. Its own collision is used, with the `FSDSProp` profile.

`getEnvironment` reports what each class was built from:
`"meshes": {"bollard": "/Game/ThirdParty/Props/SM_Bollard.SM_Bollard", "bin": "primitives", ...}`.

### Installing them

`Content/ThirdParty/` is git-ignored. Marketplace assets may ship inside a
packaged build, but this repo is public, so they can't be committed as source.
On each machine that builds the sim:

1. Get the assets from Fab (free scanned assets, under Fab's licence) and add
   them to the project from the editor.
2. Move each static mesh to `Content/ThirdParty/Props/` and name it as in the
   table, materials and textures alongside.
3. Rebuild or recook. `Config/DefaultGame.ini` always cooks
   `/Game/ThirdParty/Props`, so a packaged build carries them.

Keep them light: the sim's frame rate is the LiDAR's rate. Prefer meshes
under ~20k triangles with 2K textures, or enable Nanite on denser ones and
check the frame rate before and after.

## What the LiDAR sees

Shape comes from the mesh. Intensity comes from the reflectance LUT in
`FSDSLidarSensor.cpp`, indexed by each part's stencil ID (`FSDSPropStencil`
in `FSDSReferee.h`), at 905 nm:

| Material | Stencil | Reflectance | Used by |
|---|---|---|---|
| Rubber | 5 | 0.06 | tyre wall (real mesh too) |
| Dark plastic | 6 | 0.12 | bin, bollard body, tripod head |
| Aluminium | 7 | 0.45 | tripod legs |
| Retroreflective | 8 | 1.00 | the bollard's band |

A real mesh other than the tyre stack is left untagged, so the LiDAR reads
its reflectance from the mesh's own colour: a white or reflective band on a
scanned bollard comes out bright without any tagging. The small cones in a
cone stack use the yellow cone's entry.

The LUT is only read with `fsds.LidarGPU.StencilEncoder 1`. It is off by
default (#488), and then the LiDAR's colour capture never renders, so
intensity carries no material information for props or cones.
