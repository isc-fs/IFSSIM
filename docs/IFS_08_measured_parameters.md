# IFS-08 measured parameters — what the workbook says

`docs/MODEL_IFS_08/ISC_IFS_08.xlsx` is cited as authoritative by the
steering TODO in `ros2/src/ifssim_bridge/include/ifssim_ros_wrapper.h`,
and **it is not in version control**. This file records what is in it, so
the numbers the simulator rests on are reviewable without the workbook.

> **This is a survey, not a full extraction.** Cells were sampled from
> three of the six sheets. Anything below is quoted with its cell
> reference so it can be checked; anything not below has not been looked
> at. The workbook remains the source — this is an index to it.

The workbook is 30 MB, of which ~20 MB is a single embedded PNG and most
of the rest is other screenshots. The data itself is a few kilobytes.
That matters for the "should it be in git" question: committing it as-is
means versioning 30 MB of images to track a few hundred numbers.

## Sheets

| sheet | what it is |
|---|---|
| `MONO` | test-day setup and log sheet — drivers, conditions, car config |
| `Susp_Geometry` | **IFS-08 suspension hardpoints**, SAE J670e, full linkage table in mm |
| `Brakes` | brake sizing |
| `Scrutineering` | competition scrutineering checklist |
| `Pre-Test Check` | pre-run checklist |
| `Hoja1` | unexamined |

`Susp_Geometry` is the one with the most unused value: it is headed
"IFS-08 — SUSPENSION GEOMETRY ANALYSIS", carries a master linkage table
of wishbone pivots and ball joints in SAE J670e coordinates, and cites
Milliken & Milliken RCVD. The plant currently models suspension as a
spring/damper per corner with no linkage at all, and lists roll centre
heights as assumptions — this sheet has the geometry those would come
from.

## Cross-check against `settings.json`

| quantity | workbook | `settings.json` | |
|---|---|---|---|
| wheel radius | 20.32 cm `BI34` | 0.202 m | agree to 0.6% |
| spring rate | 300 lbs/in `R47` | rear 525.4 → 52 540 N/m | **exact** (300 lbs/in = 52 538 N/m) |
| static Ackermann | 100% `AG48` | `P.Assumed.AckermannFraction = 1.0` | agree — but the plant calls it an ASSUMPTION |
| tyre | Hoosier 16.0x7.5 R20 `R43` | same, in the wheel class | agree |
| track | 1200 mm `AG24` | front 1220 / rear 1190 | close; the sheet gives one number, settings gives two |
| **wheelbase** | **1570 mm** `AG23` | **1627 mm** | **disagree by 3.6% — see below** |
| peak motor torque | 220 N·m `BJ23` | 230 N·m | disagree |
| continuous torque | 130 N·m `BJ20` | not modelled | — |
| power limit | 80 kW rules / 75 kW cont. `BJ21`,`BJ22` | 80 000 W | agree |
| HV pack | 370 V `R8` | "400 V bus" in comments | close |
| tyre pressure | 1.2 bar `R44` | not modelled | — |
| scrub radius | 38 mm `AG47` | not modelled | — |
| kingpin inclination | 7° `AG44` | not modelled | — |
| front caster | 6.2° `AG42` | not modelled | — |
| steering stiffness | 2320 N·m/° `AM23` | not modelled | — |

## The wheelbase disagreement, and why it matters

The `MONO` sheet's design-parameter block is headed **`IFS-06/07`**
(`AE20`) and **`CHASSIS IFS-07`** (`AK22`). So its 1570 mm wheelbase
plausibly describes the *previous* chassis, not the IFS-08 — despite the
workbook being named `ISC_IFS_08`.

This is not academic. The steering TODO reasons:

> authoritative ISC_IFS_08.xlsx MONO sheet gives turning radius 4.5 m +
> wheelbase 1.570 m → max δ ≈ 0.336 rad

If 1570 mm is the IFS-07 and the IFS-08 is 1627 mm, that derivation
mixes two cars, and 0.336 rad (19.25°) is not this car's number. It is
one of **four** disagreeing figures for maximum steering lock:

| source | value |
|---|---|
| `FSDSWheelFront.cpp`, historically | 28° — invented, comment said "FS typical geometry" |
| `characterise_plant.m` | 22.4° — the tyre's peak-grip angle, now the clamp |
| uDV `MAX_STEER_ROADWHEEL_DEG` | 18.2° — column limit / effective ratio |
| this workbook, via the TODO | 19.25° — but possibly from an IFS-07 wheelbase |

Resolving that needs someone who knows which chassis the sheet
describes. It is tracked at #462.

## Two assumptions that may not need to be

The plant labels these ASSUMPTION in `matlab/plant/ifssim_params.m`:

- **`AckermannFraction = 1.0`** — the workbook records static Ackermann
  as 100% (`AG48`), with an adjacent note "adjustable??" (`AG49`). If
  that 100% is measured, the plant can cite it.
- **Roll centre heights** — `Susp_Geometry` has the hardpoints these are
  computed from.

And one that turns out to be corroborated rather than assumed: the rear
spring rate matches the workbook's 300 lbs/in exactly, so `SpringRate`
is measured even though nothing in the code says where it came from.

## What to do with the workbook

Three options, none taken here because it is a decision about what
belongs in this repo:

1. **Commit as-is** — 30 MB, mostly screenshots, versioned forever.
2. **Commit a stripped copy** — the same workbook with embedded images
   removed is a few hundred kB and keeps every number and formula.
3. **Leave it out and keep this file current** — cheapest, but the
   authority stays outside version control, which is the problem that
   prompted this.

Option 2 looks best and is not much work; it needs someone to confirm
the images are not themselves data (some may be scope traces or test
photos).
