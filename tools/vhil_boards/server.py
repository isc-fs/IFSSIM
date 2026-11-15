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
from vhil.plants import COUNTS_TO_V, Inverter
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
STALE_S = 0.5                 # how long a command from the sim stays fresh


class Silent:
    """The tractive-system plant reports the DC link itself (0x466); the
    inverter must not send a second one."""

    def report(self):
        return None


class DrivenInverter(Inverter):
    """vhil's inverter state machine and frames, with the motor's speed given
    by the sim (the car's real wheels) instead of the placeholder model."""

    def __init__(self):
        super().__init__(dc_link=Silent(), vdc_signal="v_link", gain_rpm_per_nm_s=0.0, drag_per_s=0.0)
        self.target_rpm = 0.0

    def step(self, port, t_us):
        self.rpm = self.target_rpm
        super().step(port, t_us)


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
        self.fed_at = -1e9          # virtual us of the last refresh of the pipeline's side
        self.sent: dict[str, int] = {}
        self.brake = None
        # the RES box is on the DV bus from power-on, not-in-e-stop, no GO
        self._res(0x01)

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
        self._res((0x00 if self.estop else 0x01) | (0x04 if self.intent >= 2 and not self.estop else 0))
        if self.intent >= 1 and self.phase == "idle":
            b.udv.set_voltage("PF9", 3.0)                                           # ASMS
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
        if self.armed and self.mission >= 0 and self.sent.get("mission") != self.mission:
            b.w32("g_can_mission_id", self.mission); b.w8("g_set_mission_ready", 1)
            self.sent["mission"] = self.mission
        # the brake: held until the car drives, back on when it stops being driven
        brake = (BRAKE_FREE_COUNTS if raw == AS_DRIVING else
                 BRAKE_HELD_COUNTS if (raw in (AS_EMERGENCY, AS_FINISHED) and self.phase == "armed") else self.brake)
        if brake is not None and brake != self.brake:
            b.ecu.set_voltage("PF7", brake * COUNTS_TO_V)
        self.brake = brake if brake is not None else self.brake
        # the pipeline's side of the (unmodelled) micro-ROS link: the firmware's own setters.
        # The stamps are refreshed every 100 ms of virtual time, the values only when they change.
        if self._now() - self.fed_at >= 100_000:
            self.fed_at = self._now()
            tick = b.r("xTickCount", 4)
            if wall - self.dv_at < STALE_S:
                if self.sent.get("dv") != self.dv_status:
                    b.w8("g_dv_status", self.dv_status); self.sent["dv"] = self.dv_status
                b.w32("g_dv_status_stamp_ms", tick)
            if wall - self.ctrl_at < STALE_S:
                key = (round(self.throttle, 3), round(self.steer, 3))
                if self.sent.get("ctrl") != key:
                    b.wf("g_accel_cmd", self.throttle); b.wf("g_steer_cmd", self.steer); self.sent["ctrl"] = key
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
    ap.add_argument("--slice-ms", type=float, default=40.0)
    ap.add_argument("--quantum-us", type=int, default=1000, help="500 (the gate's) or 1000 (faster)")
    ap.add_argument("--fw", action="append", default=[], metavar="BOARD=ELF", help="a firmware image (repeat)")
    a = ap.parse_args(argv)
    fw = dict(x.split("=", 1) for x in a.fw)
    boards = Boards(a.system, fw, a.quantum_us)
    panel, link = Panel(boards), Link(a.port)
    print(f"vhil-boards ready on 127.0.0.1:{a.port} ({a.system}, quantum {a.quantum_us} us)", flush=True)
    v0, w0 = boards.sim.now_us(), time.monotonic()
    last_log, running, extra_at, extra = 0.0, True, -1e9, {}
    while running:
        wall = time.monotonic()
        for msg in link.take():
            running = apply(panel, boards, msg, wall) and running
        raw = panel.step(wall)
        boards.run(a.slice_ms)
        virt = (boards.sim.now_us() - v0) / 1e6
        ahead = virt - (time.monotonic() - w0)         # > 0: the emulation is ahead of the wall clock
        if ahead > 0:
            time.sleep(ahead)
        lag = max(0.0, -ahead)
        if boards.sim.now_us() - extra_at >= 250_000:
            extra_at = boards.sim.now_us()
            extra = {"mission": boards.r("g_can_mission_id", 4), "ecu_state": boards.sim.read_symbol("ecu", "g_last_ctrl_state"),
                     "ams_state": boards.sim.read_symbol("ams", "g_state_telemetry"), "ts_active": boards.r("g_telemetry_ts_active")}
            extra["mission"] -= (1 << 32) if extra["mission"] >= 1 << 31 else 0
        state = {
            "t": round(boards.sim.now_us() / 1e6, 3), "rtf": round(virt / max(time.monotonic() - w0, 1e-6), 3),
            "lag_s": round(lag, 3), "as_raw": raw, "assi": ASSI.get(raw, 0),
            **extra, "torque_nm": boards.inv.torque_nm, "armed": panel.armed, "inv_rpm": round(boards.inv.rpm),
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
