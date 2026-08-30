# Bags

Empty on purpose. Rosbags are large, regenerable by re-running the sim, and do
not belong in a working tree — 15 GB of them lived here.

The **vehicle-dynamics signals were extracted before deletion** and kept, in
`tools/vd_validation/data/`: 15 MB of CSV in place of 15 GB of bag, holding
steering angle, motor rpm, yaw rate and body accelerations at ~300 Hz. That is
everything a yaw-rate validation needs (see `docs/vehicle_dynamics_alignment.md`),
so the runs are still usable as replay fixtures.

Worth knowing about what was in them, so nobody goes looking for it again:

- All were **simulator** recordings, not car data.
- All were **crawls** — 2.80 m/s peak on the parity runs, 4.52 m/s on the
  manual one. Below about 5 m/s, load transfer, tyre load sensitivity and
  combined slip do essentially nothing, so none of them can validate the
  vehicle dynamics.
- Three of the five parity runs never steered at all (±0.002 rad).
- `trackA_manual_001602` had no `/steering_angle` channel, so its CSV has an
  empty steering column and cannot be used for yaw-rate validation either.

## Recording a new one

```bash
ros2 bag record -s mcap -o <name> /imu /steering_angle /motor_rpm
```

`-s mcap`, not the sqlite default: it opens natively in Lichtblick and Foxglove.
Then extract the signals and keep the CSV, not the bag:

```bash
python3 tools/vd_validation/bag_to_vd_csv.py <bag-dir> -o tools/vd_validation/data/<name>.csv
```

What the vehicle-dynamics work actually needs is **one fast manual run**,
ideally a skid pad — steady state, fixed 9.125 m path radius, tyres saturated.
Manual means no autonomy, so none of the AS-state blockers apply.
