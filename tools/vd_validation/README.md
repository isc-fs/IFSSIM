# Vehicle-dynamics validation data

The signals a yaw-rate validation needs, extracted from rosbags and kept as
CSV. `data/` is 6 MB standing in for the 15 GB of bags that used to sit in
`tools/bags/`, which were deleted.

`bag_to_vd_csv.py` produces these from either bag format without ROS on the
host — mcap through the mcap-ros2 reader, sqlite by parsing CDR directly,
because the older bags predate the switch to mcap and were the only ones
carrying steering.

```bash
python3 tools/vd_validation/bag_to_vd_csv.py <bag-dir> -o data/<name>.csv
```

Columns: `t_s, steer_rad, motor_rpm, yaw_rate_rps, ax_mps2, ay_mps2` at ~300 Hz.

## What is here, and what it is worth

| file | max speed | steer span | usable for |
|---|---|---|---|
| `car_parity_autocross_20260711_001410.csv` | 2.82 m/s | 0.50 rad | low-speed steering kinematics only |
| `car_parity_autocross_20260711_005328.csv` | 2.80 m/s | 0.50 rad | low-speed steering kinematics only |
| `trackA_manual_001602.csv` | 4.52 m/s | none | nothing on its own — the bag had no `/steering_angle` |

**None of these can validate the vehicle dynamics.** All are simulator
recordings, and all are crawls. Below about 5 m/s, load transfer, tyre load
sensitivity and combined slip do essentially nothing, so a model fitted to
these would score well and prove nothing. They are kept because they are the
only traces that exist and they exercise the steering and yaw plumbing.

Four further runs were extracted and then discarded: three `car_parity_*` and
`parity_test` recorded a **stationary** car — max speed 0.00 m/s, steering
span 0.0002 rad. Over 8 GB of bag holding a vehicle that never moved.

## What is actually needed

One fast **manual** run, ideally a skid pad: steady state, fixed 9.125 m path
radius, tyres saturated. Manual means no autonomy, so none of the AS-state
blockers apply. Record `/imu`, `/steering_angle` and `/motor_rpm`, extract the
CSV, and score the model with `matlab/vd/yaw_rate_fit.m` — the relative
yaw-rate error the literature validates against. See
`docs/vehicle_dynamics_alignment.md`.
