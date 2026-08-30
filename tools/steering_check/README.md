# Steering (lateral) verification — on stands

Prove that a **commanded road-wheel angle produces the real physical wheel angle**,
after adding the steering-ratio chain (uDV #172 + pipeline `max_steer_deg=18.2`).

Steering needs **no road load**, so this is fully valid on stands.

## Why a physical gauge is unavoidable

Every on-board sensor (the LWS) reads the **column**, *through* the very ratios
you're validating (`motor→column` ×1.1, `REDUCTORA` 20, `column→wheel` ×5.0).
Trusting the readback is circular. Exactly **one** independent measurement breaks
the loop: the physical wheel angle.

## The chain and what checks each stage

```
cmd δ ─[uDV ×E=5.5]→ grados(target) ─[stepper]→ motor ─[mech]→ PHYSICAL wheel
                                                     LWS reads the column ↑
```

| Stage | Check | Source |
|---|---|---|
| 1 uDV ratio | `feedback[target]/δ` == E (5.5) | **logs** |
| 2 stepper exec | `feedback[motor]` ≈ `[target]` | **logs** |
| 3 LWS vs stepper | `feedback[actual]` ≈ `[motor]` (the divergence we saw) | **logs** |
| 4 column→wheel | physical δ vs commanded δ → slope 1.0 | **gauge** |

Stages 1–3 need no instrument (from the bag). Stage 4 is the one that proves the
angle is *real*.

## Procedure

1. **Mount the gauge.** Digital inclinometer (magnet) on a front wheel rim/upright,
   measuring rotation about the steer axis. Centre the wheels, zero the gauge.
   (Camera-over-wheel + fiducial works too — say the word and I'll add the
   auto-measure variant so no manual reads are needed.)
2. **Record + command the staircase** (on the car):
   ```bash
   ros2 bag record -s mcap -o steer_cal /ctrl/cmd /steering/feedback /steering_angle &
   python3 steer_staircase.py --angles 0,5,10,15,18,0,-5,-10,-15,-18,0 --hold 5 --max-steer 18.2
   ```
   ⚠ **Actuation path:** the uDV forwards `/ctrl/cmd` steering to DV-STEERING only
   when the drive path is active — confirm your rig actuates it (and that the
   pipeline's own controller isn't also publishing `/ctrl/cmd`). If DV-STEERING
   exposes a direct pit/inspection angle command, drive that instead; the analysis
   below works on whatever bag results.
3. **Record the gauge** at each printed `HOLD i` into `gauge.csv`:
   ```
   hold_index,physical_deg
   0,0.0
   1,5.2
   2,10.1
   ...
   ```
4. **Analyse** (locally, after scp'ing the bag, or on the car with a rosbag2 build):
   ```bash
   python3 steer_verify.py steer_cal gauge.csv
   ```

## Reading the output

- **Stage 1** `E_obs` should be ~5.5 → the uDV applies the ratio.
- **Stage 2** `motor≈target` → the stepper executes the command.
- **Stage 3** `actual≈motor` → the physical column tracks the stepper. A big gap =
  the **divergence** (slip / LWS fault / un-modelled 1.1). `actual/motor≈0.91`
  points at the un-modelled `motor→column` 1.1.
- **Stage 4** `physical δ = slope·commanded + offset`:
  - slope **1.0** → the whole chain is honest; commanded == real. ✅
  - slope ≠ 1 → net ratio wrong (rescale by that factor).
  - offset → centring/calibration.
  - `LWS/physical` gives the **true** column:wheel ratio — compare to the configured 5.0.
    If physical matches the command but this ≠ 5.0, the ratio constant (not the
    mechanism) is the error.

Do this **once** to confirm/tune the ratio + offset; afterward the LWS-derived
angle *is* the wheel angle, and Stages 1–3 self-verify every run from the logs.

## Files
- `steer_staircase.py` — commands the known-angle staircase (rclpy, on the car).
- `steer_verify.py` — bag (+ optional `gauge.csv`) → per-stage checks + command-vs-physical slope/offset. Auto-detects holds from `/ctrl/cmd`.
