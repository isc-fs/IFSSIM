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

## What runs where

| | |
|---|---|
| **Firmware** | built by IFS-vHIL's recipe at the spec's refs (`boards.firmware: {udv: feat/x}`), cached in the Docker volume `vhil-data` |
| **Boards container** | `tools/vhil_boards/server.py` in the IFS-vHIL image, on the host network: it boots `systems/ecu-ams-udv.yaml`, steps it in 40 ms slices and never lets it run ahead of the wall clock (about 1.0×, measured on this PC), answering on `127.0.0.1:47800` |
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

## Limits

- Only `ecu-ams-udv`: the sim's uDV node needs the car's three boards.
- The emulated uDV's own IMU is a static BMI088 (its `vehicle_standstill` is always true);
  the pipeline still gets the sim's IMU. Steering and wheel-speed CAN are not fed.
- Not deterministic (wall-clock paced), unlike vHIL's lock-step runs: compare repeats.
- If the PC cannot hold 1.0× the boards fall behind; `boards.log` in the job's folder
  records the real-time factor and the worst lag.
