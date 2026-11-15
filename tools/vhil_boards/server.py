#!/usr/bin/env python3
"""Emulated ECU, AMS and uDV (IFS-vHIL) paced to the wall clock, for a sim run.

Runs *inside* the IFS-vHIL image (``launch/boards.py`` starts it), with the vHIL
checkout on ``PYTHONPATH``. It boots a vHIL system (default ``ecu-ams-udv``) from
the same firmware images the car runs, steps it in short slices of virtual time
and never lets it run ahead of the wall clock, so the sim (a fixed 60 Hz,
real time) and the pipeline can talk to it as they would to the car.

Between the boards and the sim sit the pieces the car has around them, as
``vhil.plants`` models them: the shutdown circuit, the tractive system (AMS
precharge), the inverter and its motor. What the *driver, the AMI box, the RES
and the EBS plumbing* do is this file's "operator" (``Panel``): the sim's
``/sim/intent`` and ``/sim/mission`` topics reach it through the TCP link, and it
turns them into what the boards see: the ASMS level, tank pressures, the RES
frame on the DV CAN, the brake sensor, the mission on the uDV.

The uDV talks micro-ROS over USB to the pipeline; USB is not modelled. The two
things that came over that link, ``/dv/status`` and ``/ctrl/cmd``, are written into
the firmware's own globals (``ros_set_dv_status``/``ros_set_ctrl_cmd_norm``
do exactly that), and the AS state is read out of them. Everything else (AS state
machine, EBS init, the ECU's R2D and torque, the inverter) is the firmware's.

Wire protocol (newline-delimited JSON on 127.0.0.1:<port>)::

    server -> client   {"t": virtual s, "rtf": .., "lag_s": .., "as_raw": 0..4, "assi": 0..4,
                        "mission": idx|-1, "torque_nm": .., "ecu_state": .., "ams_state": ..,
                        "ts_active": .., "armed": .., "inv_rpm": ..}   every slice
    client -> server   {"cmd": "dv_status", "v": 0..7} | {"cmd": "ctrl", "throttle": f, "steer": f}
                       {"cmd": "mission", "idx": n} | {"cmd": "intent", "v": 0|1|2}
                       {"cmd": "estop", "v": bool} | {"cmd": "motor_rpm", "v": mech rpm}
                       {"cmd": "quit"}
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

from vhil import elf
from vhil.cosim import Port
from vhil.plant_ts import make_plants
from vhil.plants import COUNTS_TO_V, Inverter, _e2e
from vhil.sim import Sim
from vhil.system import REPO

# Firmware ASState (as_state.h) -> the ASSI byte on /assi/state (dv_interface.h).
ASSI = {0: 0, 1: 2, 2: 3, 3: 1, 4: 4}
AS_OFF, AS_READY, AS_DRIVING, AS_EMERGENCY, AS_FINISHED = 0, 1, 2, 3, 4
# ECU: motor erpm / MotorPolePairs on 0x506 (catalog/firmware/ecu.yaml, test_ecu_dv.py).
POLE_PAIRS = 10
BRAKE_HELD_COUNTS = 3100      # above BrakeDvHardRaw (2500): the ECU's 0x505 "brake over limit"
BRAKE_FREE_COUNTS = 700       # released, below the R2D arm level (750)
TANK_BAR = 8.0                # both EBS tanks, above the firmware's storage threshold
GPIOF = "sysbus.gpioPortF"
DASH_CHG = 10                 # AMS PF10, as tests/sim/test_plant_ts.py presses it
DV_STOPPING = 7             # dv_interface.h DV_STATUS_STOPPING
STALE_S = 0.5                 # how long a command from the sim stays fresh


class Silent:
    """The tractive-system plant reports the DC link itself (0x466); the
    inverter must not send a second one."""

    def report(self):
        return None


class DrivenInverter(Inverter):
    """vhil's inverter state machine and frames, with the motor's speed given by the sim (the
    car's real wheels) instead of the placeholder model.

    The inverter reports every 10 ms (0x461 state, 0x463 speed). The plants are stepped every
    20 ms to keep the emulation in real time, so the reports are not sent from step() (that would
    halve their rate): each step hands the emulation the next ticks' frames (Port.schedule) and
    the emulation sends them every 10 ms itself, as the car's inverter does. What changes between
    steps is the 4-bit E2E counter, which is in the schedule; the state and the speed are the
    step's, so they lag the commands by up to one step instead of one tick."""

    LOOKAHEAD = 6          # ticks scheduled ahead: more than a step, replaced by the next one
    PERIOD_US = 10_000

    def __init__(self):
        super().__init__(dc_link=Silent(), vdc_signal="v_link", gain_rpm_per_nm_s=0.0, drag_per_s=0.0)
        self.target_rpm = 0.0
        self.scheduled = False

    def step(self, port, t_us):
        for f in port.received("inv_cmd"):
            if len(f.data) >= 3:
                self._command(f.data[2] & 0x7F, t_us)
        for f in port.received("inv_torque"):
            if len(f.data) >= 4:
                self.torque_nm = int.from_bytes(f.data[2:4], "little", signed=True)
        self.rpm = self.target_rpm
        if port.step_ms <= self.period_ms:       # stepped at the report rate: the plain way
            return super().step(port, t_us)
        ticks = port.ticks("inv_state")
        first = ticks[0] if ticks else t_us + self.PERIOD_US
        k0 = max(0, -(-(t_us + 1 - first) // self.PERIOD_US))
        raw = int(round(self.rpm)) & 0xFFFFF
        speed = bytes([0, 0, 0, 0, 0, (raw & 0x0F) << 4, (raw >> 4) & 0xFF, (raw >> 12) & 0xFF])
        state, rpm = [], []
        for k in range(k0, k0 + self.LOOKAHEAD):
            at = first + k * self.PERIOD_US
            state.append((at, _e2e(7, k & 0x0F, lambda b: b.__setitem__(4, self.state & 0x7F))))
            rpm.append((at, speed))
        port.schedule("inv_state", state, self.PERIOD_US / 1000)
        port.schedule("inv_rpm", rpm, self.PERIOD_US / 1000)


class Boards:
    def __init__(self, system: str, firmware: dict[str, str], quantum_us: int | None):
        self.sim = Sim(REPO / "systems" / f"{system}.yaml", firmware, quantum_us=quantum_us).start()
        self.sim.wait_for_app()
        self.port = Port(self.sim)
        self.plants_by_name = make_plants(self.sim.system, self.sim)
        self.inv = DrivenInverter()
        self.plants = list(self.plants_by_name.values()) + [self.inv]
        self.sdc = self.plants_by_name.get("sdc")
        self.udv_elf = self.sim.firmware["udv"]
        self.udv, self.ams, self.ecu = self.sim.io("udv"), self.sim.io("ams"), self.sim.io("ecu")
        self._addr: dict[str, int] = {}
        self.has_ts = self.sdc is not None

    # -- the uDV's globals (the firmware's own atomics) ---------------------
    def addr(self, sym: str) -> int:
        if sym not in self._addr:
            self._addr[sym] = int(elf.symbol(self.udv_elf, sym)[0])
        return self._addr[sym]

    def w8(self, sym, v):
        self.sim.monitor(f"sysbus WriteByte {self.addr(sym):#x} {int(v) & 0xFF}", board="udv")

    def w32(self, sym, v):
        self.sim.monitor(f"sysbus WriteDoubleWord {self.addr(sym):#x} {int(v) & 0xFFFFFFFF}", board="udv")

    def wf(self, sym, v):
        self.w32(sym, struct.unpack("<I", struct.pack("<f", float(v)))[0])

    def r(self, sym, n=1):
        return self.sim.read_symbol("udv", sym, n)

    def run(self, ms):
        self.port.run(self.plants, ms)


class Panel:
    """What the operator, the AMI box, the RES and the EBS plumbing do."""

    def __init__(self, b: Boards):
        self.b = b
        self.intent, self.estop, self.mission = 0, False, -1
        self.dv_status, self.dv_at = 0, 0.0
        self.throttle, self.steer, self.ctrl_at = 0.0, 0.0, 0.0
        self.phase = "idle"
        self.phase_t = 0
        self.armed = False
        self.res_byte = None
        self.asms = False           # the ASMS lever, as the operator last left it
        self.restart = False        # the operator went OFF after a run: power the car down and up
        self.used = False           # a mission has been asked for since power-on
        self.streak = 0             # consecutive STOPPING messages, as the firmware counts them
        self.fed_at = -1e9          # virtual us of the last refresh of the pipeline's side
        self.sent: dict[str, int] = {}
        self.brake = None
        # The RES box joins the DV bus when the operator asks for READY (step), so that a car made
        # ready before the mission does not go to AS Ready before one is selected.

    def _res(self, byte0):
        if byte0 == self.res_byte:
            return
        if self.res_byte is None:
            self.b.sim.can("can_dv").send_periodic("res", 0x191, bytes([byte0, 0, 0, 0, 0, 0, 100, 0]), 20)
        else:
            self.b.sim.can("can_dv").update_periodic("res", bytes([byte0, 0, 0, 0, 0, 0, 100, 0]))
        self.res_byte = byte0

    def _now(self):
        return self.b.sim.now_us()

    def step(self, wall: float):
        b = self.b
        raw = b.r("g_telemetry_as_state")
        # RES: bit0 = not e-stop, bit2 = GO
        if self.intent >= 1:
            self._res((0x00 if self.estop else 0x01) | (0x04 if self.intent >= 2 and not self.estop else 0))
        # The operator's OFF powers the whole car down: the next autonomous mission starts from AS Off,
        # the ECU out of its DV latch, the AIRs open and the tractive system to be precharged again.
        # The emulation is restarted from power-on (about a second), which is that, exactly.
        if self.intent >= 1:
            self.used = True
        if self.intent == 0 and self.used:
            self.restart = True
            return AS_OFF
        # The car is made ready before the mission starts, as the team does: tractive system on, ASMS
        # on, the EBS checked. The uDV then sits in AS Off until a mission is selected (the AMI box)
        # and the operator asks for READY, which is what /sim/mission and /sim/intent do.
        if self.phase == "idle":
            b.udv.set_voltage("PF9", 3.0); self.asms = True                         # ASMS
            b.udv.set_voltage("PC0", TANK_BAR / 11); b.udv.set_voltage("PF10", TANK_BAR / 11)   # tanks
            b.ecu.set_voltage("PF7", BRAKE_HELD_COUNTS * COUNTS_TO_V)               # brake line held
            self.phase, self.phase_t = ("ams_ok" if b.has_ts else "ts"), self._now()
        if self.phase == "ams_ok" and b.port.level("ams_ok"):
            b.sdc.apply({"reset": True}, self._now()); self.phase, self.phase_t = "reset", self._now()
        elif self.phase == "reset" and self._now() - self.phase_t > 300_000:
            b.sdc.apply({"reset": False}, self._now()); b.sdc.apply({"tsms": True}, self._now())
            self.phase, self.phase_t = "tsms", self._now()
        elif self.phase == "tsms" and self._now() - self.phase_t > 700_000:
            b.ams.set_input(GPIOF, DASH_CHG, True); self.phase, self.phase_t = "dash", self._now()
        elif self.phase == "dash" and self._now() - self.phase_t > 300_000:
            b.ams.set_input(GPIOF, DASH_CHG, False); self.phase, self.phase_t = "ts", self._now()
        elif self.phase == "ts" and self._now() - self.phase_t > 200_000 and b.r("g_telemetry_ts_active"):
            self.phase = "armed"; self.armed = True
        if self.armed and self.asms and self.mission >= 0 and self.sent.get("mission") != self.mission:
            b.w32("g_can_mission_id", self.mission); b.w8("g_set_mission_ready", 1)
            self.sent["mission"] = self.mission
        # The brake: held until the car drives, and then until the ECU has confirmed ready-to-drive
        # (0x511, which the uDV mirrors in g_telemetry_r2d). The ECU takes the uDV's request (0x510) only
        # while the brake is over its limit, so releasing it as soon as the uDV is in AS Driving raced
        # that request and now and then left the ECU in WaitStartBrake with no torque for ever.
        # Back on when the run is over.
        brake = self.brake
        if raw == AS_DRIVING and self.brake != BRAKE_FREE_COUNTS and b.r("g_telemetry_r2d"):
            brake = BRAKE_FREE_COUNTS
        elif raw in (AS_EMERGENCY, AS_FINISHED) and self.phase == "armed":
            brake = BRAKE_HELD_COUNTS
        if brake is not None and brake != self.brake:
            b.ecu.set_voltage("PF7", brake * COUNTS_TO_V)
        self.brake = brake if brake is not None else self.brake
        # the pipeline's side of the (unmodelled) micro-ROS link: the firmware's own setters.
        # The stamps are refreshed every 100 ms of virtual time, the values only when they change.
        if self._now() - self.fed_at >= 100_000:
            self.fed_at = self._now()
            tick = b.r("xTickCount", 4)
            if wall - self.dv_at < STALE_S:
                # ros_set_dv_status (ros_globals.cpp): one call per /dv/status message, and the uDV's
                # debounce counts consecutive STOPPING messages (issue #176); this refresh is one
                # message per 100 ms, the pipeline's 10 Hz.
                streak = min(255, self.streak + 1) if self.dv_status == DV_STOPPING else 0
                if streak != self.streak:
                    b.w8("g_dv_stopping_streak", streak); self.streak = streak
                if self.sent.get("dv") != self.dv_status:
                    b.w8("g_dv_status", self.dv_status); self.sent["dv"] = self.dv_status
                b.w32("g_dv_status_stamp_ms", tick)
            if wall - self.ctrl_at < STALE_S:
                # ros_set_ctrl_cmd_norm: never trust the wire, clamp to the normalised range
                thr, steer = (max(-1.0, min(1.0, v)) for v in (self.throttle, self.steer))
                key = (round(thr, 3), round(steer, 3))
                if self.sent.get("ctrl") != key:
                    b.wf("g_accel_cmd", thr); b.wf("g_steer_cmd", steer); self.sent["ctrl"] = key
                b.w32("g_ctrl_cmd_stamp_ms", tick)
        return raw


class Link(threading.Thread):
    """One client at a time: the sim's bridge node."""

    def __init__(self, port: int):
        super().__init__(daemon=True)
        self.srv = socket.create_server(("127.0.0.1", port), reuse_port=False)
        self.conn = None
        self.inbox: list[dict] = []
        self.lock = threading.Lock()
        self.start()

    def run(self):
        while True:
            conn, _ = self.srv.accept()
            with self.lock:
                self.conn = conn
            for line in conn.makefile("r"):
                try:
                    with self.lock:
                        self.inbox.append(json.loads(line))
                except ValueError:
                    pass
            with self.lock:
                self.conn = None

    def take(self) -> list[dict]:
        with self.lock:
            out, self.inbox = self.inbox, []
        return out

    def send(self, msg: dict):
        with self.lock:
            conn = self.conn
        if conn is not None:
            try:
                conn.sendall((json.dumps(msg) + "\n").encode())
            except OSError:
                pass


def apply(panel: Panel, boards: Boards, msg: dict, wall: float) -> bool:
    c = msg.get("cmd")
    if c == "dv_status":
        panel.dv_status, panel.dv_at = int(msg["v"]), wall
    elif c == "ctrl":
        panel.throttle, panel.steer, panel.ctrl_at = float(msg["throttle"]), float(msg["steer"]), wall
    elif c == "mission":
        panel.mission = int(msg["idx"])
    elif c == "intent":
        panel.intent = int(msg["v"])
    elif c == "estop":
        panel.estop = bool(msg["v"])
    elif c == "motor_rpm":
        boards.inv.target_rpm = float(msg["v"]) * POLE_PAIRS
    elif c == "quit":
        return False
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--system", default="ecu-ams-udv")
    ap.add_argument("--port", type=int, default=47800)
    ap.add_argument("--slice-ms", type=float, default=80.0)
    ap.add_argument("--no-pace", action="store_true", help="run flat out (to measure the emulation's speed)")
    ap.add_argument("--plant-step-ms", type=float, default=20.0, help="the plants' step (the system file's port: step_ms, 10): 20 gives the emulation about 20%% more speed")
    ap.add_argument("--quantum-us", type=int, default=1000, help="500 (the gate's) or 1000 (faster)")
    ap.add_argument("--fw", action="append", default=[], metavar="BOARD=ELF", help="a firmware image (repeat)")
    a = ap.parse_args(argv)
    fw = dict(x.split("=", 1) for x in a.fw)
    boards = Boards(a.system, fw, a.quantum_us)
    if a.plant_step_ms:
        boards.port.step_ms = a.plant_step_ms
        boards.inv.period_ms = a.plant_step_ms
    panel, link = Panel(boards), Link(a.port)
    print(f"vhil-boards ready on 127.0.0.1:{a.port} ({a.system}, quantum {a.quantum_us} us)", flush=True)
    v0, w0 = boards.sim.now_us(), time.monotonic()
    last_log, running, extra_at, extra = 0.0, True, -1e9, {}
    while running:
        wall = time.monotonic()
        for msg in link.take():
            running = apply(panel, boards, msg, wall) and running
        raw = panel.step(wall)
        if panel.restart:
            old = panel
            boards.sim.stop()
            boards = Boards(a.system, fw, a.quantum_us)
            if a.plant_step_ms:
                boards.port.step_ms = a.plant_step_ms
                boards.inv.period_ms = a.plant_step_ms
            panel = Panel(boards)
            panel.mission, panel.estop, panel.intent = old.mission, old.estop, old.intent
            panel.dv_status, panel.dv_at, panel.throttle, panel.steer, panel.ctrl_at = old.dv_status, old.dv_at, old.throttle, old.steer, old.ctrl_at
            v0, w0 = boards.sim.now_us(), time.monotonic()
            print("vhil-boards: restarted from power-on (operator OFF)", flush=True)
            continue
        boards.run(a.slice_ms)
        virt = (boards.sim.now_us() - v0) / 1e6
        ahead = virt - (time.monotonic() - w0)         # > 0: the emulation is ahead of the wall clock
        if ahead > 0 and not a.no_pace:
            time.sleep(ahead)
        lag = max(0.0, -ahead)
        if boards.sim.now_us() - extra_at >= 250_000:
            extra_at = boards.sim.now_us()
            extra = {"mission": boards.r("g_can_mission_id", 4), "ecu_state": boards.sim.read_symbol("ecu", "g_last_ctrl_state"),
                     "ams_state": boards.sim.read_symbol("ams", "g_state_telemetry"), "ts_active": boards.r("g_telemetry_ts_active"),
                     "udv": {"ebs_init": boards.r("g_telemetry_ebs_init_state"), "asms": boards.r("g_telemetry_asms_on"),
                             "abs_ok": boards.r("g_telemetry_abs_checks_ok"), "brakes": boards.r("g_telemetry_brakes_engaged"),
                             "r2d": boards.r("g_telemetry_r2d"), "sdc_res_open": boards.r("g_telemetry_sdc_res_open")}}
            extra["mission"] -= (1 << 32) if extra["mission"] >= 1 << 31 else 0
        state = {
            "t": round(boards.sim.now_us() / 1e6, 3), "rtf": round(virt / max(time.monotonic() - w0, 1e-6), 3),
            "lag_s": round(lag, 3), "as_raw": raw, "assi": ASSI.get(raw, 0),
            **extra, "torque_nm": boards.inv.torque_nm, "armed": panel.armed, "phase": panel.phase, "inv_rpm": round(boards.inv.rpm),
            "inv_state": boards.inv.state,
        }
        link.send(state)
        if wall - last_log > 5:
            print(json.dumps(state), flush=True)
            last_log = wall
    boards.sim.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
