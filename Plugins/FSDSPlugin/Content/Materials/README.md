# FSDSPlugin Plugin-Owned Materials

Plugin-internal `.uasset` materials live under this directory. Asset
references use the `/FSDSPlugin/Materials/` virtual path — gated on
`"CanContainContent": true` in `FSDSPlugin.uplugin`.

## Expected contents

- **`M_LiDARStencilEncoder.uasset`** — post-process material that
  encodes `SceneTexture:CustomStencil` into the alpha channel of
  the LiDAR's `FinalColorLDR` capture (#321 D-Phase-2). Without this,
  `FSDSLidarSensor::InitializeGPUPath` logs a warning and falls back
  to Phase-1 luminance behaviour.

  Author it via:
  - `Plugins/FSDSPlugin/Tools/create_lidar_stencil_encoder.py` (run
    from the UE5 editor's Python console — one-click), or
  - the manual recipe in issue #358 (~5 minutes in the material editor).

  After saving the asset, cook + relaunch — the next LiDAR scan picks
  up the per-cone-material reflectance LUT automatically.
