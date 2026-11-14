"""headless_run around fake processes: a fake sim, a fake backend, a fake benchmark."""

from __future__ import annotations

import json
import os
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

from headless_run.orchestrate import RunConfig, exit_code, run_command, run_loop, run_once  # noqa: E402
from headless_run.simproc import SimCommand, SimError, SimProcess, port_open, resolve_command  # noqa: E402


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
                     poll_s=0.2, **{"settle_s": 0.0, **kw})


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


def test_relative_paths_are_made_absolute(tmp_path, monkeypatch):
    # The sim starts in --settings-dir, so a relative --sim would point nowhere.
    monkeypatch.chdir(tmp_path)
    c = resolve_command("build/IFSSIM.sh", settings_dir="cfg")
    assert c.argv[0] == str(tmp_path / "build" / "IFSSIM.sh")
    assert c.cwd == str(tmp_path / "cfg")
    assert resolve_command("IFSSIM.sh").argv[0] == "IFSSIM.sh"     # bare name: PATH lookup


def test_a_sim_that_cannot_be_started_is_a_failed_run(tmp_path, backend):
    c = cfg(tmp_path, free_port(), backend)
    c.sim_command.cwd = str(tmp_path / "no_such_settings_dir")
    rec = run_once(c, log=quiet)           # must not raise
    assert rec["outcome"] == "sim_failed_to_start" and "couldn't start the sim" in rec["error"]
    assert (Path(rec["run_dir"]) / "run.json").exists()


@pytest.mark.skipif(sys.platform == "win32", reason="process groups")
def test_stop_waits_for_the_engine_not_just_the_launcher(tmp_path):
    # Like IFSSIM.sh: the launcher exits on SIGTERM at once, while the engine
    # it started ignores SIGTERM and keeps the port. stop() must kill it.
    port = free_port()
    launcher = tmp_path / "launcher.py"
    launcher.write_text(
        "import signal, subprocess, sys\n"
        f"subprocess.Popen([sys.executable, {str(HERE / 'fake_sim.py')!r}, {str(port)!r}],\n"
        "                 preexec_fn=lambda: signal.signal(signal.SIGTERM, signal.SIG_IGN))\n"
        "signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))\n"
        "signal.pause()\n")
    sim = SimProcess(SimCommand(argv=[sys.executable, str(launcher)]), port=port)
    sim.start()
    sim.wait_ready(10)
    t = time.monotonic()
    sim.stop(grace_s=1.0)
    assert time.monotonic() - t < 8
    assert not port_open("127.0.0.1", port)
    with pytest.raises(ProcessLookupError):
        os.killpg(sim.proc.pid, 0)


def test_a_stop_request_does_not_wait_for_the_sim_to_boot(tmp_path, backend):
    flag = {"stop": False}
    import threading as _t
    _t.Timer(0.5, lambda: flag.__setitem__("stop", True)).start()
    t = time.monotonic()
    rec = run_once(cfg(tmp_path, free_port(), backend, sim_args=[0, 30]), log=quiet,
                   should_stop=lambda: flag["stop"])
    assert rec["outcome"] == "interrupted"
    assert time.monotonic() - t < 10


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


def test_no_mission_starts_before_the_sim_keeps_real_time(tmp_path, backend):
    # the fake sim's clock runs at half speed for its first 2 s
    rec = run_once(cfg(tmp_path, free_port(), backend, sim_args=(0, 0.2, 2.0),
                       settle_s=20.0, settle_window_s=1.0), log=quiet)
    assert rec["outcome"] == "finished"
    assert rec["settle_rtf"] >= 0.98 and 2.0 <= rec["settle_s"] < 10 and "settle_timeout" not in rec


def test_a_sim_that_never_keeps_real_time_is_waited_for_at_most_settle_s(tmp_path, backend):
    rec = run_once(cfg(tmp_path, free_port(), backend, sim_args=(0, 0.2, 60.0),
                       settle_s=2.0, settle_window_s=1.0), log=quiet)
    assert rec["outcome"] == "finished"
    assert rec["settle_timeout"] and 0.4 < rec["settle_rtf"] < 0.6


def test_the_car_characterisation_runs_before_the_missions(tmp_path, backend):
    c = cfg(tmp_path, free_port(), backend,
            characterise_cmd=[sys.executable, str(HERE / "fake_characterise.py"), "--status", "fail"])
    t0 = time.time()
    rec = run_once(c, log=quiet)
    run_dir = Path(rec["run_dir"])
    res = json.loads((run_dir / "characterisation" / "result.json").read_text())
    manifest = next((run_dir / "mission").rglob("manifest.json"))
    assert t0 <= res["t"] <= manifest.stat().st_mtime          # before the benchmark
    assert res["args"]["sim"] == f"127.0.0.1:{c.port}"
    assert res["args"]["sim_log"] == str(run_dir / "sim.log")
    # Its result is recorded, but the run's outcome is the missions'.
    assert rec["characterisation"]["status"] == "fail" and rec["characterisation"]["exit"] == 1
    assert rec["outcome"] == "finished"
    assert json.loads((run_dir / "run.json").read_text())["characterisation"]["status"] == "fail"


def test_a_characterisation_that_hangs_is_cut_off(tmp_path, backend):
    c = cfg(tmp_path, free_port(), backend, characterise_timeout_s=0.5,
            characterise_cmd=[sys.executable, str(HERE / "fake_characterise.py"), "--sleep", "30"])
    rec = run_once(c, log=quiet)
    assert rec["characterisation"]["status"] == "timeout"
    assert rec["outcome"] == "finished"


def test_no_characterisation_unless_asked(tmp_path, backend):
    rec = run_once(cfg(tmp_path, free_port(), backend), log=quiet)
    assert "characterisation" not in rec
    assert not (Path(rec["run_dir"]) / "characterisation").exists()


def test_the_sims_cpu_weight_is_raised_only_when_asked():
    from headless_run.orchestrate import raise_sim_cpu_weight

    calls = []

    def run(argv, **kw):
        calls.append(argv)

        class R:
            returncode, stdout, stderr = 0, "ifssim-sim\n", ""
        return R()

    assert raise_sim_cpu_weight({}, run) is None and calls == []
    assert raise_sim_cpu_weight({"IFSSIM_SIM_CPU_SHARES": "8192"}, run) == "ifssim-sim cpu-shares 8192"
    assert calls[-1] == ["docker", "update", "--cpu-shares", "8192", "ifssim-sim"]
    assert raise_sim_cpu_weight({"IFSSIM_SIM_CPU_SHARES": "8192", "IFSSIM_SIM_CONTAINER": "other"},
                                run).startswith("other")
    assert raise_sim_cpu_weight({"IFSSIM_SIM_CPU_SHARES": "x"}, run).startswith("not applied")

    def fails(argv, **kw):
        class R:
            returncode, stdout, stderr = 1, "", "Error: No such container: ifssim-sim"
        return R()

    # a failed update never fails the run: it is recorded
    assert "No such container" in raise_sim_cpu_weight({"IFSSIM_SIM_CPU_SHARES": "8192"}, fails)


# --- exec: any command around a fresh sim (tools/ci/bench_run.sh runs a spec this way)
def _cmd(code: str) -> list:
    return [sys.executable, "-c", code]


def test_exec_runs_the_command_with_the_sim_up_and_passes_its_exit_code(tmp_path, backend):
    port = free_port()
    out = tmp_path / "seen.txt"
    code = (
        "import os, socket, sys; host, port = os.environ['IFSSIM_SIM_ADDR'].split(':');"
        "socket.create_connection((host, int(port)), 2).close();"
        f"open({str(out)!r}, 'w').write(os.environ['IFSSIM_MC_URL']); sys.exit(5)"
    )
    rc = run_command(cfg(tmp_path, port, backend), _cmd(code), log=quiet)
    assert rc == 5
    assert out.read_text() == backend
    rec = json.loads(next((tmp_path / "headless").glob("*_exec/run.json")).read_text())
    assert rec["outcome"] == "command_failed" and rec["command_exit"] == 5
    assert not port_open("127.0.0.1", port)


def test_exec_a_finished_command_is_0(tmp_path, backend):
    assert run_command(cfg(tmp_path, free_port(), backend), _cmd("pass"), log=quiet) == 0


def test_exec_a_sim_that_never_comes_up_is_a_machine_failure(tmp_path, backend):
    out = tmp_path / "ran"
    rc = run_command(cfg(tmp_path, free_port(), backend, sim_args=[-1]),
                     _cmd(f"open({str(out)!r}, 'w')"), log=quiet)
    assert rc == 3 and not out.exists()


def test_exec_a_sim_crash_stops_the_command(tmp_path, backend):
    t = time.monotonic()
    rc = run_command(cfg(tmp_path, free_port(), backend, sim_args=[1.5]),
                     _cmd("import time; time.sleep(60)"), log=quiet)
    assert rc == 3 and time.monotonic() - t < 20
    rec = json.loads(next((tmp_path / "headless").glob("*_exec/run.json")).read_text())
    assert rec["outcome"] == "sim_crashed"


def test_exec_without_a_backend_is_a_machine_failure(tmp_path):
    assert run_command(cfg(tmp_path, free_port(), "http://127.0.0.1:9"), _cmd("pass"), log=quiet) == 3


def test_exec_needs_a_command():
    from headless_run.__main__ import main
    assert main(["exec", "--sim", "attach"]) == 2
