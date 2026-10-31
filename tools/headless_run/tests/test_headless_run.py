"""headless_run around fake processes: a fake sim, a fake backend, a fake benchmark."""

from __future__ import annotations

import json
import socket
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent))

from headless_run.orchestrate import RunConfig, exit_code, run_loop, run_once  # noqa: E402
from headless_run.simproc import SimCommand, SimError, port_open, resolve_command  # noqa: E402


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def backend():
    """A Mission Control backend that says the sim is connected."""
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps({"connected": True}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    class Server(HTTPServer):
        def server_bind(self):          # skip HTTPServer's slow reverse-DNS lookup
            socketserver.TCPServer.server_bind(self)
            self.server_name, self.server_port = "localhost", self.server_address[1]

    srv = Server(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def cfg(tmp_path, port, mc_url, sim_args=(), bench_args=(), **kw):
    sim = SimCommand(argv=[sys.executable, str(HERE / "fake_sim.py"), str(port), *map(str, sim_args)])
    return RunConfig(sim_command=sim, results_root=tmp_path, port=port, mc_url=mc_url,
                     benchmark_cmd=[sys.executable, str(HERE / "fake_bench.py")],
                     benchmark_args=list(bench_args), sim_ready_timeout_s=10, mc_timeout_s=3,
                     poll_s=0.2, **kw)


def quiet(_msg):
    pass


def test_resolve_command(tmp_path):
    app = resolve_command("/b/IFSSIM.app")
    assert app.argv[0] == "/b/IFSSIM.app/Contents/MacOS/IFSSIM"
    assert "-RenderOffscreen" in app.argv and "-nullrhi" not in app.argv
    ed = resolve_command("/UE/UnrealEditor", project=str(tmp_path / "IFSSIM.uproject"), extra=["-x"])
    assert ed.argv[1].endswith("IFSSIM.uproject") and ed.argv[2] == "-game" and ed.argv[-1] == "-x"
    with pytest.raises(SimError):
        resolve_command("/UE/UnrealEditor")
    ed_app = resolve_command("/UE/UnrealEditor.app", project="/p/IFSSIM.uproject")
    assert ed_app.argv[0] == "/UE/UnrealEditor.app/Contents/MacOS/UnrealEditor" and ed_app.argv[2] == "-game"
    assert resolve_command("./IFSSIM.sh", settings_dir="/s").cwd == "/s"


def test_a_finished_run_is_recorded_and_the_sim_stopped(tmp_path, backend):
    port = free_port()
    rec = run_once(cfg(tmp_path, port, backend), log=quiet)
    assert rec["outcome"] == "finished" and exit_code(rec["outcome"]) == 0
    assert rec["sim_status"]["map"] == "fake"
    assert len(rec["mission_runs"]) == 1
    assert json.loads((Path(rec["run_dir"]) / "run.json").read_text())["outcome"] == "finished"
    assert (tmp_path / "headless" / "index.jsonl").read_text().count("\n") == 1
    assert not port_open("127.0.0.1", port)


def test_missions_not_finished_pass_through(tmp_path, backend):
    rec = run_once(cfg(tmp_path, free_port(), backend, bench_args=["--exit", "1"]), log=quiet)
    assert rec["outcome"] == "not_finished" and exit_code("not_finished") == 1


def test_a_benchmark_that_crashes_is_not_a_pipeline_result(tmp_path, backend):
    rec = run_once(cfg(tmp_path, free_port(), backend, bench_args=["--crash"]), log=quiet)
    assert rec["outcome"] == "benchmark_error" and exit_code(rec["outcome"]) == 3


def test_a_benchmark_that_cannot_start_fails_before_the_sim_boots(tmp_path, backend):
    c = cfg(tmp_path, free_port(), backend)
    c.benchmark_cmd = [sys.executable, "-c", "import no_such_module_xyz"]
    rec = run_once(c, log=quiet)
    assert rec["outcome"] == "benchmark_error" and "no_such_module_xyz" in rec["error"]
    assert "sim_ready_s" not in rec


def test_a_sim_crash_stops_the_benchmark(tmp_path, backend):
    t = time.monotonic()
    rec = run_once(cfg(tmp_path, free_port(), backend, sim_args=[1.5], bench_args=["--sleep", "60"]),
                   log=quiet)
    assert rec["outcome"] == "sim_crashed"
    assert time.monotonic() - t < 20


def test_a_sim_that_never_comes_up(tmp_path, backend):
    rec = run_once(cfg(tmp_path, free_port(), backend, sim_args=[-1]), log=quiet)
    assert rec["outcome"] == "sim_failed_to_start" and "exited during startup" in rec["error"]


def test_a_taken_port_is_refused(tmp_path, backend):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        s.listen(1)
        rec = run_once(cfg(tmp_path, s.getsockname()[1], backend), log=quiet)
    assert rec["outcome"] == "sim_failed_to_start" and "already taken" in rec["error"]


def test_no_backend(tmp_path):
    rec = run_once(cfg(tmp_path, free_port(), f"http://127.0.0.1:{free_port()}"), log=quiet)
    assert rec["outcome"] == "backend_unreachable"


def test_the_loop_gives_up_on_a_broken_machine(tmp_path, backend):
    recs = run_loop(cfg(tmp_path, free_port(), backend, sim_args=[-1]), cycles=10, pause_s=0,
                    max_infra_failures=2, log=quiet)
    assert [r["outcome"] for r in recs] == ["sim_failed_to_start"] * 2


def test_the_loop_keeps_going_while_runs_merely_fail(tmp_path, backend):
    recs = run_loop(cfg(tmp_path, free_port(), backend, bench_args=["--exit", "1"]), cycles=3,
                    pause_s=0, max_infra_failures=2, log=quiet)
    assert [r["outcome"] for r in recs] == ["not_finished"] * 3


def test_each_run_keeps_its_own_results(tmp_path, backend):
    """Repeats write the same mission/track/commit/seed folder; one per run, none lost."""
    recs = run_loop(cfg(tmp_path, free_port(), backend), cycles=2, pause_s=0, log=quiet)
    runs = [r["mission_runs"] for r in recs]
    assert all(len(r) == 1 for r in runs), runs
    assert runs[0] != runs[1]
    assert all(Path(r[0], "manifest.json").exists() for r in runs)
    assert all(Path(r[0]).is_relative_to(Path(rec["run_dir"])) for r, rec in zip(runs, recs))
