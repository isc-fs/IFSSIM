# The car's art (IFS-08 mesh)

The car is drawn with the IFS-08 itself, generated from the team's Blender livery scene. The scene is a SolidWorks export of the KIN CAD with the livery and procedural materials. Nothing is hand-edited in the editor. A CAD or livery change means running the two scripts again.

```
IFS08_livery.blend (≈10 M tris, ~270 procedural Cycles materials)
  └─► tools/assetgen/blender/build_ifs08_vehicle.py   (Blender, background)
        cull + decimate + bake → FBX + PNG + manifest.json
          └─► tools/assetgen/ue_import_vehicle.py       (UnrealEditor-Cmd, pythonscript)
                └─► /Game/Vehicle/IFS08  (Content/Vehicle/IFS08, Git LFS)
```

`settings.json` `Vehicles.FSCar.Mesh` selects it. With no `Mesh` block, or a `Body` that fails to load, the pawn draws the inherited Technion car (`/Game/Vehicle/TechnionCar`), as before.

## What the sim gets

| Asset | What it is |
|---|---|
| `SK_IFS08` | the body: skeletal mesh, bones `root` + `WheelFL/FR/RL/RR` at the hub centres, no geometry on the wheel bones |
| `SK_IFS08_PhysicsAsset` | one box on `root`, the chassis between the wheels: the car's collision body (`Vehicle`, [collision_matrix.md](collision_matrix.md)) |
| `SM_IFS08_Wheel_<c>` | what spins: tyre (lettering baked in), rim, disc, hub, centre nut; pivot at the hub |
| `SM_IFS08_Corner_<c>` | upright and caliper: they steer and travel but do not spin; pivot at the hub |
| `MI_IFS08_{Livery,Mech,Wheels}` | `M_IFS08_Baked` with the baked BaseColor, Normal and ORM (AO, roughness, metallic) |
| `MI_FX_*` | LED dies, the TSAL and the brake light: `M_IFS08_FX`, `Color` and `Glow` (0 = off) |
| `MI_FX_DISPLAY_SCREEN` | the dash display: `M_IFS08_Screen`, a `Screen` texture (a render target later) |

**Frame.** The mesh origin sits on the ground under the centre of mass: Wheelbase × (1 − WeightDistFront) behind the front axle, on the centreline. The pawn puts the centre of mass at the body origin and places the physics wheels around it from `settings.json` (`PlaceWheelsFromSettings`), so the art agrees with the physics when `settings.json` holds the CAD's numbers (#944). The export reads `WeightDistFront` from `settings.json` so the two agree; `--weight-front` overrides it. If they differ, the drawn wheels (posed at the physics wheels) sit off the uprights by the difference.

**Wheels.** The body's skeleton has wheel bones only because Chaos looks wheels up by bone. The wheels and corners are separate meshes, posed every tick from the wheel state of whichever plant drives the car:
- Chaos: `AFSDSVehiclePawn::PoseWheelsFromChaos`, the same maths as the engine's wheel-controller anim node (spin and steer on the rest pose, suspension offset along the suspension axis).
- FMU: `PoseWheelsFromPlant`, which also drew the Technion car's cylinders.

The rest position is the physics wheel's (bone + placement offset), so a wheel is drawn where the physics has it.

## Regenerate

Run these with UnrealEditor closed: the bake uses the GPU.

```
"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b <IFS08_KIN>\cad\blender\IFS08_livery.blend ^
    --python tools\assetgen\blender\build_ifs08_vehicle.py -- --out <IFS08_KIN>\cad\blender\export_ue\ifs08 --preview
set IFSSIM_VEHICLE_DIR=<IFS08_KIN>\cad\blender\export_ue\ifs08
set IFSSIM_VEHICLE_DASH=<IFS08_KIN>\cad\blender\textures\wheel_dash_default.png
UnrealEditor-Cmd IFSSIM.uproject -run=pythonscript -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities ^
    -script="%CD%\tools\assetgen\ue_import_vehicle.py" -unattended -nullrhi
```

The Blender script never saves the .blend. `--no-bake` gives geometry only, in a few minutes. `--preview` renders stills of the result next to the FBX. The import checks that each wheel bone landed on its hub (in UE cm). If the FBX axes or units went wrong, it fails rather than producing a car that drives sideways.

## How the game-ready version is made

1. **Scene to meshes.** Drop the ground, Jarama, cameras and lights. Apply modifiers, and turn curves and text into meshes.
2. **Split.** Parts within a corner whose CAD name says tyre, rim, rotor, hub, hub nut or safety clip spin with the wheel. Uprights and calipers go to the corner. Everything else is body. Suspension arms and pushrods stay on the body: they do not follow travel or steer yet.
3. **Cull.** Rays from ~220 viewpoints around the car, none lower than 4 cm (a cone's eye), plus a panorama from the cockpit. A part no ray reaches (bearings, diff internals, fasteners inside parts) is dropped.
4. **Decimate.** The triangle budget (`--budget`, 450 k) is shared by how much of each part the rays saw. Tyre and rim lettering are not kept as geometry: they are baked into the tyre.
5. **Bake** (Cycles, GPU), from the full-resolution parts onto the decimated ones, into three 4K atlases:
   - **Livery**: bodywork panels, wings, carbon.
   - **Mech**: the rest of the body, plus the corners.
   - **Wheels**.

   The passes are base colour, roughness, tangent normal, metallic (each material's Principled Metallic, or its name), and AO on the decimated car. Cycles' diffuse-colour pass drops the metallic lobe, so the base-colour bake runs with Metallic forced to 0.
6. **Export** FBX in the export frame (x forward, y left, z up, metres). The FBX axis conversion gives UE x forward, y right, cm.
