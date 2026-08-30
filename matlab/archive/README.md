# Archive

Models that are superseded but kept, because they are where several of the
car's numbers came from and deleting them would leave those numbers with no
provenance at all.

**Nothing here is maintained. Nothing here is on the path by default.**
If you are looking for the current models:

| I want to | Go to |
|---|---|
| change a parameter | `matlab/spec/car_spec.m` |
| study handling, balance, lap times | `matlab/vd/` |
| the full Simulink plant | `matlab/plant/` |

## `IFS_Sim/` (2024–25)

A longitudinal drive-cycle and energy model. Superseded by `matlab/vd`
(handling) and `matlab/plant` (the 6-DOF plant), kept for its drive cycles
and because two current parameters cite it: it carries mass 237 kg, tyre
radius 0.30 m and wheel inertia 0.215 kg·m², all of which disagree with the
IFS-08 numbers now in `car_spec`. Those disagreements are recorded in the
source strings there, and this is what they point at.

## `MODEL_IFS_08/`

The vehicle dynamics department's own model — a Simscape longitudinal /
powertrain / cooling set plus a MATLAB dynamics study. Still referenced by
`matlab/vd/accel_compare.m`, which compares our acceleration run against it,
so this is a live dependency rather than pure history.

Its `SIMSCAPE/fsDCycle.mat` drive cycle is a **MathWorks demo trace**, not a
Formula Student lap — it contains sustained speeds and accelerations no FS
track allows. `matlab/vd/fs_track_cycle.m` exists because of that.
