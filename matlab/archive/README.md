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

Its `SIMSCAPE/` subtree is **gone**, deliberately. It was MathWorks'
"Simscape Essentials for Automotive Student Teams" download — third party, not
ISC's work, and freely re-downloadable. The only thing anything used from it
was `fsDCycle.mat`, and that cycle is a MathWorks demo trace rather than a
Formula Student lap: sustained speeds and accelerations no FS track allows.
`matlab/vd/fs_track_cycle.m` exists because of that, and is what replaced it.

What remains is ISC's own: `DYNAMIC_MOD` (a live dependency of
`vd/accel_compare`), `ISC_IFS_08.xlsx` and `Performance.xlsx`.
