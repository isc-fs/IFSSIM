# Powertrain

The powertrain department's view of the car: motor, drivetrain, the pack as a
power source, and regen. It is built the same way as `vd/`, the suspension and
dynamics view.

```matlab
cd matlab/pt
pt_parameters    % what you can change, what binds, what does not
pt_report        % what the powertrain does, as numbers
pt_plots         % the same, as figures in pt/figures/
```

## What binds on this car

Run `pt_parameters` first. Its last column is computed by moving each
parameter 10% and watching the numbers this department is judged on:
75 m time, top speed, launch and regen. On the car as specified:

- **Launch is traction-limited** up to about 18 m/s. The motor could give
  more, but the rear tyres can't take it.
- **Everything above that is pack-limited.** At 200 A the pack sags to about
  321 V, which gives 59 kW at the shaft against an 80 kW motor.
  `MotorMaxPower` does not bind, and a bigger motor buys nothing.
  `Pack.CurrentLimit` and the cell arrangement do bind.
- **Regen is power-limited** almost from rest: `MaxRegenPower` binds and
  `MaxRegenTorque` does not.
- **`GearRatio` does not bind**, but only because the plant has **no motor
  speed limit**. The ratio's real trade, launch torque against the rev
  limiter, can't show up until one is added. The model reaches about
  6200 rpm at top speed.

These are facts about this car, not about the parameters. Change the pack and
the motor's numbers may start to bind; `pt_study` reports when the binding
limit changes.

## Trying a change

```matlab
pt_study('Pack.CurrentLimit', 250)
pt_study('DrivetrainEfficiency', 0.95, 'MaxRegenPower', 12000)
pt_study('plant', 'MaxRegenPower', 12000)    % also on the full plant, ~2 min
```

An override that doesn't bind is **refused**, and the refusal names what is
binding instead. A side-by-side table of zeroes reads as "it doesn't matter".
The true answer is "it doesn't matter *because* something else limits first",
and the something else is the useful part.

`'plant'` refuses accumulator parameters. `build_battery_pack` reads
`car_spec` directly, so no study override reaches the plant's pack. They do
reach `pt_model`, because `pack_from_cells` now accepts the override-able
parameter set.

## The model, and how far to trust it

`pt_model` is `build_powertrain`'s own envelope, evaluated quasi-steadily, on
the plant's own longitudinal Magic Formula. `test_pt_model` holds it to the
plant on the acceleration event:

| check | model | plant |
|---|---|---|
| shaft power on the pack limit | 59.14 kW | 58.86 kW |
| ax at 25 m/s | 0.579 g | 0.569 g |
| driven-tyre slip at 25 m/s | 0.041 | 0.036 |

The tolerances were set **after** the first comparison. The launch is not
compared, only bounded. `pt_model` holds the tyre at its peak (perfect
traction control), and the plant at full throttle has none. The difference in
75 m time, about 0.4 s, is the most a traction controller could be worth.

Two things are known and open:

- **About 25–40 N of unexplained body-side loss in the plant.** The wheel side
  balances exactly, so the loss is on the body side. It is about 1% of the
  car's weight, and it is why the agreement gets worse above 30 m/s.
- **Drivetrain efficiency is charged twice** in `build_powertrain`: once from
  pack to shaft, and once from shaft to wheel. Road power is η² ≈ 0.85 of pack
  power. The block's comment says the double charge was removed, but it wasn't.
  That's defensible only if η stands for motor *and* gears, and car_spec's
  source calls it "drivetrain". `pt_model` mirrors the plant until this is
  decided.
