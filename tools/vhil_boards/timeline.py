"""Timestamps from the operator's READY to the first torque, on the emulated boards (virtual time).

    docker run --rm -v <IFS-vHIL>:/work:ro -v $PWD/tools/vhil_boards:/boards:ro -v vhil-data:/vhil \
        -e PYTHONPATH=/work -w /work ifs-vhil python /boards/timeline.py

Drives server.Boards and server.Panel as the sim's node would (mission, READY, GO, /dv/status
RUNNING, a 0.4 throttle) and prints, from GO, when the uDV reached AS Driving, the ECU's R2D
request and confirm, its inverter words and its first torque request.
"""
import sys, time
sys.path.insert(0, "/boards")
import server
fw = dict(l.strip().split("=", 1) for l in open("/vhil/fw/built.txt") if "=" in l)
b = server.Boards("ecu-ams-udv", fw, 1000)
b.port.step_ms = 20.0; b.inv.period_ms = 20.0
p = server.Panel(b)
now = lambda: b.sim.now_us() / 1e6
ev = {}
def mark(k, t=None):
    ev.setdefault(k, now() if t is None else t)
def loop(ms=20):
    w = time.monotonic()
    p.dv_at = p.ctrl_at = w
    p.step(w); b.run(ms)
    raw = b.r("g_telemetry_as_state"); ecu = b.sim.read_symbol("ecu", "g_last_ctrl_state")
    if raw == 1: mark("AS READY (uDV)")
    if raw == 2: mark("AS DRIVING (uDV)")
    mark(f"ecu state {ecu}") if ecu in (2, 3, 4, 5) else None
    return raw, ecu
# pre-armed: wait until the TS bring-up is done and the AS idles in OFF
while not p.armed: loop()
for _ in range(25): loop()
p.mission = 1; p.dv_status = 2
t_ask = now(); ev["operator asks READY (mission+intent 1)"] = t_ask; p.intent = 1
while True:
    raw, ecu = loop()
    if raw == 1: break
t_ready = now()
for _ in range(10): loop()       # the pipeline prepares meanwhile; GO when the backend sends it
t_go = now(); ev["operator GO (intent 2)"] = t_go; p.intent = 2
p.dv_status = 2
t_end = t_go + 15
while now() < t_end:
    raw, ecu = loop()
    if raw == 2: p.dv_status = 3; p.throttle = 0.4; p.steer = 0.0
t0 = t_ask * 1e6
def first(bus, ids, pred=lambda f: True):
    fr = [f for f in b.sim.can(bus).frames(ids, since_us=int(t_go * 1e6)) if pred(f)]
    return fr[0].t_us / 1e6 if fr else None
ev["first 0x510 uDV R2D request (acu)"] = first("can_acu", [0x510], lambda f: f.data[0] != 0)
ev["first 0x511 ECU R2D confirm (acu)"] = first("can_acu", [0x511], lambda f: f.data[0] != 0)
ev["first 0x507 uDV torque request >0"] = first("can_acu", [0x507], lambda f: int.from_bytes(f.data[:4], "little", signed=True) > 0)
ev["first 0x360 ECU->inv word Ready (0x04)"] = first("can_inv", [0x360], lambda f: (f.data[2] & 0x7F) == 0x04)
ev["first 0x360 ECU->inv word TorqueEn (0x06)"] = first("can_inv", [0x360], lambda f: (f.data[2] & 0x7F) == 0x06)
ev["first 0x362 torque != 0 (ECU->inv)"] = first("can_inv", [0x362], lambda f: int.from_bytes(f.data[2:4], "little", signed=True) != 0)
print("t from GO (s)  event      [GO at %.2f s virtual]" % t_go)
for k, v in sorted(((k, v) for k, v in ev.items() if v is not None and v >= t_go - 0.0001), key=lambda kv: kv[1]):
    print("  %+6.3f  %s" % (v - t_go, k))
print("before GO: operator asks READY at %.2f s, AS READY at %.2f s (+%.2f s)" % (t_ask, t_ready, t_ready - t_ask))
b.sim.stop()
