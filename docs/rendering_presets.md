# Rendering presets

The sim has two rendering presets.

| Preset | What it sets | Use it for |
|---|---|---|
| `sim` (default) | `Config/DefaultEngine.ini`: 25 % screen resolution, minimum scalability groups, no Lumen. | Every test, benchmark and pipeline run. |
| `showcase` | Full resolution, every `sg.*` group at 3 (epic), Lumen global illumination and reflections. | Demos, screenshots and renders. |

`showcase` costs a lot of frame time. Below real time the car drives differently (the headless runner waits for RTF ≥ 0.98 for this reason), so never benchmark on it. `getSimStatus` reports the active preset as `"quality": "sim" | "showcase"`.

Neither preset changes what the sensors measure: the LiDAR depth capture turns lighting, GI and post-processing off whatever the preset.

## Switching

- Start with it: add `-showcase` to the sim's command line.
- At runtime, in the console: `FSDS.Quality showcase` or `FSDS.Quality sim`. Going back to `sim` restores the ini values.

Code: `Plugins/FSDSPlugin/Source/FSDSPlugin/Private/Rendering/FSDSQualityPreset.cpp`.

## Daylight

`showcase` also swaps the level's legacy lighting for physically based daylight (`AFSDSDaylight`):

- **Sun** where it stands during the event: about 20 August, 14:00 CEST, at Montmeló (60.7° up, bearing 182.8°). The rig reads the loaded world's `frame.x_bearing_deg` to aim it, and re-aims when a track with another world loads. Earlier in the day the main grandstand's shadow covers the straight.
- **Sky** from a sky atmosphere, and a sky light recaptured from the scene in real time. The level's own sky light was captured once from the old flat map, so faces in shade went near black.
- **Haze** from the sky atmosphere alone. A height fog drew a dark band along the horizon, where long, low sight lines cross the most fog.
- **Exposure** fixed. The level's auto-exposure was tuned for its old lighting and blew the scene out by two to three stops. The default, −1 stop, puts sunlit asphalt near sRGB 120, as in photos of the circuit; adjust it with `fsds.Daylight.Exposure <stops>`.

Going back to `sim` restores the level's lighting exactly.

Code: `Plugins/FSDSPlugin/Source/FSDSPlugin/Private/Environment/FSDSDaylight.cpp`.

## A render without a window

On the Mac, this renders a 4K frame of the Montmeló start with the start lights lit, then the sim can be stopped:

```bash
UnrealEditor IFSSIM.uproject -game -showcase -RenderOffscreen -ResX=3840 -ResY=2160 \
  -ExecCmds="DisableAllScreenMessages,FSDS.StartLights 1,FSDS.Shot 8 3840x2160"
```

`FSDS.Shot [seconds] [WxH]` waits (wall clock, so it works while the sim waits paused for a client), then takes a high-resolution screenshot. Lumen gathers its bounce light over a few seconds; a shot on the first frames comes out darker and noisier. The image lands in `Saved/Screenshots/`.

`FSDS.View x y z pitch yaw [fov]` puts a free camera at (x, y, z) in metres in the track frame (+y left), looking `pitch` degrees up and `yaw` degrees anticlockwise from +x, with a 70° field of view unless given; `FSDS.View` alone goes back to the car. For an aerial of the Montmeló straight:

```bash
UnrealEditor IFSSIM.uproject -game -showcase -RenderOffscreen -ResX=3840 -ResY=2160 \
  -ExecCmds="DisableAllScreenMessages,FSDS.View -140 -95 55 -17 28 75,FSDS.Shot 8 3840x2160"
```

The first run after a change compiles the high-quality shaders; run once and wait for `ShaderCompileWorker` to go idle before taking the frame you keep.
