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

One fast **manual** run **on the real car**, ideally a skid pad: steady state,
fixed 9.125 m path radius, tyres saturated. Manual means no autonomy, so none
of the AS-state blockers apply.

Then score it:

```matlab
D = vd_car_data('tools/vd_validation/data/<run>.csv');
R = validate_vs_car(D);      % RMSE ax / ay / yaw rate + Escofet fit
```

`validate_vs_car` reports the same three RMSEs as the reference thesis
(Diwakar, TU Delft 2018), so the numbers sit directly against its table —
skid pad 0.65 / 1.26 m/s² / 0.162 rad/s, full lap 1.43 / 3.19 / 0.158.
`test_validate_vs_car` proves the harness against a synthetic car with a known
answer, since there is no real one yet.

### Record these, or the run cannot be scored

| topic | why | if missing |
|---|---|---|
| `/imu` | ax, ay, yaw rate — the things being compared | nothing to score |
| `/steering_angle` | the model is driven by it | **run is unusable** — this is what killed `trackA_manual` |
| `/motor_rpm` | forward speed | nothing to drive the model with |
| **motor or wheel torque** | lets ax be a *prediction* | ax is reported but is only the speed trace differentiated |

**Torque is the one most worth adding.** Without it the model is driven by the
speed the car had, so its ax is an input restated rather than a prediction —
which is the one place this departs from the thesis, whose model was driven by
measured wheel torque. The lateral numbers are genuine predictions either way.

**The steering scale is fitted, not assumed.** `/steering_angle` reaches
±0.50 rad against a measured 18.2° lock, so it is not simply the road-wheel
angle. At low speed a car is nearly kinematic, so `vd_car_data` fits the scale
from measured yaw rate and speed. Include a few seconds of **slow driving with
the wheel held** at the start of the run — that is what the fit uses. On the
simulator recordings here it comes out at −0.85, but that is a property of the
Chaos steering path (which squares its input), not of the car.

### Reading the result

Do not read the percentage fit on its own. In the self-test a model with
**double the correct yaw inertia still scores 93.8%** — comfortably above the
82.5% the literature calls satisfactory for an autocross lap — while its
yaw-rate RMSE is 17× the correct model's. The percentage is forgiving; the
RMSE is not. Report both.
