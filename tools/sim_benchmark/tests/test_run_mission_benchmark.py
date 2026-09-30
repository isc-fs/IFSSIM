"""The mission benchmark against a fake Mission Control and a fake sim (nothing real runs)."""

from __future__ import annotations

import json
import math
import socketserver
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run_mission_benchmark as rmb  # noqa: E402


class FakeWorld:
    """What the fake Mission Control and sim share: the referee, and what was asked of them."""

    def __init__(self, laps_per_poll=1, start_fails=False, never_finishes=False):
        self.calls: list[str] = []
        self.laps = 0
        self.required = 1
        self.running = False
        self.laps_per_poll = laps_per_poll
        self.start_fails = start_fails
        self.never_finishes = never_finishes

    def referee(self):
        if self.running and not self.never_finishes:
            self.laps = min(self.required, self.laps + self.laps_per_poll)
        done = self.running and self.laps >= self.required and not self.never_finishes
        return {
            "laps": self.laps,
            "required_laps": self.required,
            "finished": done,
            "lap_times": [30.0 + i for i in range(self.laps)],
            "doo_counter": 1,
            "oc_counter": 0,
            "cones": 120,
            "event": "trackdrive",
        }


def serve_mc(world: FakeWorld) -> str:
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body):
            raw = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            world.calls.append(f"GET {self.path}")
            if self.path == "/api/sim/status":
                return self._send(200, {"connected": True})
            if self.path == "/api/event/state":
                return self._send(200, world.referee())
            if self.path == "/api/vehicle/state":
                return self._send(
                    200,
                    {
                        "x": 1.0,
                        "y": 2.0,
                        "speed": 3.5,
                        "controls": {"throttle": 0.2, "steering": 0.0},
                    },
                )
            return self._send(404, {})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}") if n else {}
            world.calls.append(
                f"POST {self.path} {json.dumps(body, sort_keys=True) if body else ''}".strip()
            )
            if self.path == "/api/event/start":
                if world.start_fails:
                    return self._send(
                        502, {"ok": False, "error": "did not reach DV_READY"}
                    )
                world.running, world.laps, world.required = True, 0, body["num_laps"]
                return self._send(200, {"ok": True})
            if self.path == "/api/pipeline/stop":
                world.running = False
            return self._send(200, {"ok": True})

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return f"http://127.0.0.1:{srv.server_address[1]}"


def serve_sim(world: FakeWorld) -> tuple[str, int]:
    class H(socketserver.StreamRequestHandler):
        def handle(self):
            line = self.rfile.readline().decode().strip()
            world.calls.append(f"SIM {line}")
            if line == "getStartGatePose":
                out = {
                    "x": 10.0,
                    "y": 0.0,
                    "z": 0.1,
                    "qw": 1.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                }
            else:
                out = {"ok": True}
            self.wfile.write((json.dumps(out) + "\n").encode())

    srv = socketserver.ThreadingTCPServer(("127.0.0.1", 0), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return "127.0.0.1", srv.server_address[1]


def _args(tmp_path, **kw):
    a = dict(
        missions="trackdrive",
        tracks="t1.csv",
        repeats=None,
        laps=2,
        timeout_s=5.0,
        start_noise=False,
        mission_overrides=None,
        pipeline_on="bench_pc",
        panda_start=None,
        panda_stop=None,
        panda_sha=None,
        sim="x:1",
        mc_url="",
        mc_key=None,
        results_root=str(tmp_path / "results"),
        poll_s=0.01,
    )
    a.update(kw)
    return type("A", (), a)()


@pytest.fixture
def world():
    return FakeWorld()


def _run(tmp_path, world, **kw):
    host, port = serve_sim(world)
    url = serve_mc(world)
    logs = []
    rc = rmb.run(
        _args(tmp_path, **kw),
        rmb.Sim(host, port),
        rmb.MissionControl(url),
        log=logs.append,
    )
    return rc, logs


def test_a_run_on_the_bench_pc_writes_what_the_sim_pages_read(
    tmp_path, world, monkeypatch
):
    monkeypatch.setenv("BENCH_PROGRESS_FILE", str(tmp_path / "progress.json"))
    rc, _ = _run(tmp_path, world)
    assert rc == 0
    # the order the operator panel uses: stop, load, reset with the seed, start, follow, stop
    order = [
        c.split(" ", 2)[1] if c.startswith(("GET", "POST")) else c for c in world.calls
    ]
    assert (
        order.index("/api/track/t1.csv/load")
        < order.index("SIM resetScenario 1")
        < order.index("/api/event/start")
    )
    assert (
        'POST /api/event/start {"mission": "trackdrive", "num_laps": 2}' in world.calls
    )
    [d] = (tmp_path / "results" / "mission").glob("*/*/seed1")
    man = json.loads((d / "manifest.json").read_text())
    res = json.loads((d / "results.json").read_text())
    assert man["job_type"] == "sim_e2e" and man["mock"] is False
    assert man["referee"]["bFinished"] and man["referee"]["Laps"] == [30.0, 31.0]
    assert man["code"]["pipeline"]["ran_on"] == "bench_pc"
    assert (
        res["race/finished"] == 1
        and res["race/n_laps"] == 2
        and res["race/penalty_s"] == 2.0
    )
    assert (d / "telemetry.csv").read_text().startswith("t,x,y,speed")
    p = json.loads((tmp_path / "progress.json").read_text())
    assert "trackdrive · t1" in p["detail"] and 0 < p["fraction"] <= 1


def test_a_mission_that_doesnt_start_or_finish_is_recorded_as_such(tmp_path):
    w = FakeWorld(start_fails=True)
    rc, logs = _run(tmp_path, w)
    assert rc == 1 and any("error (mission didn't start: 502" in x for x in logs)
    w2 = FakeWorld(never_finishes=True)
    rc, logs = _run(tmp_path / "b", w2, timeout_s=0.1)
    assert rc == 1 and any(": timeout" in x for x in logs)
    [d] = (tmp_path / "b" / "results" / "mission").glob("*/*/seed1")
    assert json.loads((d / "results.json").read_text())["race/finished"] == 0


def test_repeats_use_their_seed_and_start_noise_moves_the_car(tmp_path, world):
    _run(tmp_path, world, repeats=2, start_noise=True)
    assert [c for c in world.calls if c.startswith("SIM resetScenario")] == [
        "SIM resetScenario 1",
        "SIM resetScenario 2",
    ]
    poses = [c for c in world.calls if c.startswith("SIM simSetVehiclePose")]
    assert len(poses) == 2 and poses[0] != poses[1]
    new, off = rmb.offset_pose(
        {"x": 10, "y": 0, "z": 0, "qw": 1, "qx": 0, "qy": 0, "qz": 0},
        {"lateral_m": 0.3, "longitudinal_m": 0.3, "yaw_deg": 5},
        seed=1,
    )
    assert abs(off["lateral_m"]) <= 0.3 and abs(off["yaw_deg"]) <= 5
    assert math.isclose(
        math.degrees(2 * math.atan2(new["qz"], new["qw"])), off["yaw_deg"], abs_tol=1e-6
    )
    assert (
        rmb.offset_pose(
            {"x": 0, "y": 0, "qw": 1},
            {"lateral_m": 0.3, "longitudinal_m": 0.3, "yaw_deg": 5},
            1,
        )[1]
        == off
    )


def test_the_latte_panda_needs_its_commands_and_runs_them(tmp_path, world):
    with pytest.raises(rmb.BenchmarkError, match="--panda-start and --panda-stop"):
        _run(tmp_path, world, pipeline_on="latte_panda")
    marker = tmp_path / "panda.log"
    rc, _ = _run(
        tmp_path,
        world,
        pipeline_on="latte_panda",
        panda_start=f"echo started >> {marker}",
        panda_stop=f"echo stopped >> {marker}",
        panda_sha="echo abc1234def",
    )
    assert rc == 0 and marker.read_text().split() == ["started", "stopped"]
    [d] = (tmp_path / "results" / "mission").glob("*/*/seed1")
    code = json.loads((d / "manifest.json").read_text())["code"]["pipeline"]
    assert (
        code["ran_on"] == "latte_panda"
        and code["sha"] == "abc1234"
        and "latte panda" in code["sha_source"]
    )


def test_settings_merge_and_refuse_typos():
    m = rmb.mission_settings(
        ["acceleration", "trackdrive"],
        repeats=3,
        overrides={"trackdrive": {"laps": 3, "start_pose": {"noise": True}}},
    )
    assert m["acceleration"]["repeats"] == 3 and m["trackdrive"]["laps"] == 3
    assert (
        m["trackdrive"]["start_pose"]["noise"]
        and m["trackdrive"]["start_pose"]["yaw_deg"] == 5
    )
    assert [(r.mission, r.seed) for r in rmb.plan(m)][:4] == [
        ("acceleration", 1),
        ("acceleration", 2),
        ("acceleration", 3),
        ("trackdrive", 1),
    ]
    with pytest.raises(rmb.BenchmarkError, match="unknown mission"):
        rmb.mission_settings(["drag"])
    with pytest.raises(rmb.BenchmarkError, match="unknown setting 'lapz'"):
        rmb.mission_settings(["skidpad"], overrides={"skidpad": {"lapz": 2}})


def test_the_defaults_file_covers_every_mission():
    d = rmb.load_missions()
    assert set(d) == set(rmb.MISSIONS)
    for m in d.values():
        assert set(m) == set(rmb.MISSION_KEYS) and set(m["start_pose"]) == set(
            rmb.POSE_KEYS
        )
        assert (rmb.REPO / "Content" / "tracks").is_dir() is False or all(
            (rmb.REPO / "Content" / "tracks" / t).is_file() for t in m["tracks"]
        )
