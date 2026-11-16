# Emulated boards (IFS-vHIL) in a sim run

A run spec with `boards: {mode: vhil, system: ecu-ams-udv}` replaces the sim's fake uDV
(`sim_supervisor`) with the car's own ECU, AMS and uDV firmware, running on IFS-vHIL's
emulated MainLites in Docker. The sim, the pipeline and Mission Control are as today.

```
python3 tools/sim_benchmark/run_spec.py missions-basic \
  --set boards.mode=vhil --set boards.system=ecu-ams-udv \
  --set benchmarks.missions.settings.missions=acceleration,autocross
```

The Benchmarks page's vHIL box does the same on Local. It is grey, with the reason, when
Docker, the IFS-vHIL checkout or its image is missing (`launch/rig.py`, part `vhil`).

## Choosing each board's firmware

`boards.firmware: {ecu: <ref>, ams: <ref>, udv: <ref>}`, or the page's per-board box ("branch,
#PR or commit", with a Pick button), takes a branch, a tag, `#123` (a pull request's head), or a
commit id (the full 40 characters, or a short one that is a branch head). Without one, a board
is built at the default in `bench.yaml` (`dev`). Every ref is resolved to a full commit first
(`launch/boards.resolve_ref`, `git ls-remote`) and the firmware is built at that commit, so a
commit that is on no branch works too and the run records what it used: `boards.json` next to
the job's `spec.json` has each board's ref, commit and repository. A board named in the spec
whose repository is not in `bench.yaml` is refused, not built at the default.

The page offers the boxes only once "Emulated (vHIL)" can be chosen, which needs the rig check
(`run_spec.py --rig`, run by the sim from the checkout it was started from) to say vHIL is ready:
that checkout must have this code. Real boards (HIL) stay greyed until a bench is wired.

## What runs where

| | |
|---|---|
| **Firmware** | built by IFS-vHIL's recipe at the spec's refs (`boards.firmware: {udv: feat/x}`), cached in the Docker volume `vhil-data` |
| **Boards container** | `tools/vhil_boards/server.py` in the IFS-vHIL image, on the host network: it boots `systems/ecu-ams-udv.yaml`, steps it in 80 ms slices (its plants every 20 ms) and never lets it run ahead of the wall clock (1.0× on this PC, also beside a full benchmark), answering on `127.0.0.1:47800` |
| **Stack** | `sim_supervisor_node` reads `IFSSIM_VHIL_BOARDS` and, when set, publishes what the firmware says (`/assi/state`, `/ami/mission`) and hands the firmware what the pipeline sent (`/dv/status`, `/ctrl/cmd`) and the operator panel (`/sim/*`): `sim_supervisor/vhil_link.py` |
| **Start / stop** | `tools/sim_benchmark/launch/boards.py`, called by `launch/run.py` around each job; the same steps for the sim's Local target and any runner |

The uDV's micro-ROS link (USB CDC to the pipeline) is not modelled in vHIL. The node is
that link's other end: `/dv/status` and `/ctrl/cmd` are written into the firmware's own
globals (what `ros_set_dv_status` / `ros_set_ctrl_cmd_norm` do), and its AS state is read out.

## What is the firmware's, what is modelled

The firmware's: the uDV's AS state machine, EBS initialisation, mission gate and R2D; the
ECU's start FSM, DV R2D latch and torque (`0x507` request -> `0x362` Nm, caps included);
the AMS's precharge. With `IFSSIM_VHIL_TORQUE=ecu` (default) the car's throttle is the
torque the emulated ECU asks the inverter for (`-Nm / 240`); `cmd` uses `/ctrl/cmd`
directly (the emulated AS state still gates it).

Modelled around the boards (`server.py` `Panel`, vHIL's plants): the shutdown circuit and
tractive system (TSMS, RST_EXT and the precharge press that the operator does on the
car), the ASMS level, the EBS tanks, the brake line, the RES on the DV bus (GO, e-stop),
and the inverter, whose speed is the sim's `/motor_rpm`.

## Before and between missions

The car is made ready before the mission starts, as the team does: the server brings up the
tractive system, the ASMS and the EBS check at power-on, so the uDV waits in AS Off for a
mission (`/sim/mission`) and READY (`/sim/intent`). From READY to the first torque is about 7 s, all of it the firmware's own (measured with
`tools/vhil_boards/timeline.py`, virtual time, from the operator's GO):

| after GO | what |
|---|---|
| 0 s | the uDV was in AS Ready; its GO is refused until it has been there 5 s (`READY_DWELL_MS`, "FS-Rules AS-Ready dwell", IFS09-DV-uDV `app_task.cpp`) |
| +4.8 s | AS Driving; the uDV sends its R2D request (0x510); the ECU enters R2dDelay and confirms (0x511, +0.09 s) |
| +6.8 s | after the ECU's 2 s ready-to-drive sound it sends the inverter Ready (0x04) and TorqueEnable (0x06) words, and the first torque (0x362) 10 ms later |

The fake uDV has neither wait, so the car moves later with boards, and an acceleration referee
time (whose clock starts when the event is set) reads slower for it. The pipeline's own 3 s
calibration after AS Driving overlaps the ECU's 2 s sound.

## Comparing times with and without boards

The referee starts acceleration's clock when the event is set, not at a start line (autocross
starts it at the first line crossing), so an acceleration time includes everything before the car
moves. Compare `race/drive_time_s` (from the car's first motion to the finish), not
`race/total_time_s`. Five jobs each on this PC: referee time 12.10 s without boards, 16.44 s with
(the car first moves at 4.72 s and 8.87 s); drive time 7.48 s and 7.74 s (sd 0.10 s each).

## Fidelity: what is exact and what is approximated

Exact: the firmware images (the ones the car runs, bootloader included), the boards' CAN
buses (arbitration and load in virtual time), the AS state machine, the EBS initialisation,
the ECU's R2D and torque path, the AMS precharge, and the inverter's reports every 10 ms
(the emulation sends them itself, ±4 µs).

Approximated, to hold 1.0x beside the sim (each measured unpaced beside the live sim):

| approximation | what it costs the boards' view | what it buys |
|---|---|---|
| 1 ms sync quantum (vHIL's opt-in, not the gate's 0.5 ms) | a board sees another's frame up to 1 ms late | 1.17x instead of 1.05x |
| plants step every 20 ms (not 10 ms) | the inverter's state and speed in a report, the AIR contacts and the shutdown circuit react up to 20 ms late | 1.17x instead of 0.89x |
| the micro-ROS downlink written into the firmware's variables | the message callbacks do not run; the STOPPING debounce and the clamp of `/ctrl/cmd` are mirrored | USB is not modelled |
| the operator, the AMI box, the RES, the EBS tanks and the brake line are stand-ins | what the car's electrics do around the boards | needed to arm the car |

## Whether a run was real time

The boards must not fall behind the wall clock, or the firmware sees time slowed. The server
logs its pace every 2 s; a mission benchmark reads that log for the run's window and puts
`box/boards_rtf`, `box/boards_max_lag_s` and `race/boards_real_time_ok` in `results.json`
(real time = 0.97x or more and never more than 2 s behind). Compare only runs with
`race/boards_real_time_ok` 1: a PC that cannot hold it (the Windows PC does not) gives a run of
the car with slowed boards, not a run of the car.

## Limits

- Only `ecu-ams-udv`: the sim's uDV node needs the car's three boards.
- The emulated uDV's own IMU is a static BMI088 (its `vehicle_standstill` is always true);
  the pipeline still gets the sim's IMU. Steering and wheel-speed CAN are not fed.
- Not deterministic (wall-clock paced), unlike vHIL's lock-step runs: compare repeats.
- The boards held 1.0x beside the sim and a full pipeline on this PC (worst lag 0.5 s over a 9-minute
  job; their plants step every 20 ms, 10 ms costs about 20 % of the speed). If a PC cannot hold 1.0× the boards fall behind; `boards.log` in the job's folder
  records the real-time factor and the worst lag.
