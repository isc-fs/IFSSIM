# Aero

The aero department's view of the car, built the same way as `vd/` and `pt/`.

```matlab
cd matlab/aero
aero_parameters   % what each parameter moves, and where it reaches
aero_report       % the numbers, and the drag-for-downforce trade
aero_plots        % lap, cornering limit and balance, and the trade map
aero_study('ClA', 2.4, 'CdA', 1.1)
```

**No aero number on this car is measured.** `CdA` and `ClA` are UNKNOWN,
because no CFD run or tunnel test is recorded, and the balance is ASSUMED.
So this view is about *sensitivity*: which unknown is worth measuring first,
and which direction a concept should go in.

## The trade

`aero_report` differences the lap time in ClA and in CdA. On `lap_track`:

| change | lap | energy |
|---|---|---|
| +0.1 m² ClA | −0.090 s | — |
| +0.1 m² CdA | +0.030 s | +9.6 kJ |

A package change is faster when its *incremental* L/D is above about 0.33.
The straights on an FS endurance track are short, so drag costs little lap
time. It still costs energy, and endurance is scored on energy too. The
contours of `aero_trade.png` run nearly vertical, which is the same finding
drawn as a map.

## Balance

Aero balance is a handling number, not just a grip number. The downforce split
is 45% front against 43.8% front weight, so downforce loads the front
relatively more. The rear axle therefore saturates first, and earlier as speed
rises: the front/rear capacity ratio is 1.004 at 8 m/s and 1.023 at 22 m/s.
That's limit oversteer growing with speed, and it agrees with `vd_report`'s
negative understeer gradient. `aero_study('AeroBalanceFront', 0.40)` takes
22 m/s just past neutral (0.995, front-limited) and gains 0.11 s a lap.

(These figures include the camber loss the real geometry brings: with the
IFS-08's measured camber gain the lap costs 0.37 s more than it did on the
assumed one.)

## The engine, in `../lap/`

- `lap_ggv` builds the g-g-V envelope. Grip comes from the design model:
  load transfer and the Magic Formula peak. Drive comes from `pt_model`.
  Braking is four tyres with ideal balance, because this is the manual car
  and a driver has a brake pedal. Each speed's envelope is capped by
  `lap_balance`, the axle that saturates first in a steady corner. Without
  that cap, aero balance could not affect a lap time at all.
- `lap_sim` is the standard quasi-steady lap: corner speeds, a forward pass
  and a backward pass. It takes about 0.4 s.
- `lap_track` holds the track. `fs_track_cycle` uses the same table.
- `test_lap_sim` checks the engine against known answers:
  - on a car with no load sensitivity and no aero, a circle is driven at
    √(μgR), within 0.06%;
  - that same car is exactly yaw-balanced;
  - a straight from rest reproduces `pt_model`'s 75 m time within 0.6%;
  - more downforce at the same drag is never slower, and more drag at the
    same downforce is never faster.

What it isn't: a driver. There are no transients, no line optimisation and no
tyre temperature. Use it to rank designs. The absolute lap time is a bound,
not a prediction.

The aero maps are constant: no ride-height, pitch or yaw sensitivity. A real
package moves its balance as the car pitches under braking, which is exactly
when balance matters.
