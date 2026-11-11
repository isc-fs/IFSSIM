"""The local queue (launch/localq.py), its worker (run_spec.py --work) and the rig
checks (launch/rig.py), against a small manifest and a fake benchmark."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch import localq as lq  # noqa: E402
from launch import manifest as mf  # noqa: E402
from launch import rig as rg  # noqa: E402
from launch import run  # noqa: E402

# the "benchmark": records what it was given; FAKE_SLEEP keeps it busy, FAKE_RC fails it
FAKE = """
import json, os, sys, time
from pathlib import Path
out = Path(sys.argv[sys.argv.index("--results-root") + 1]) / "ran.jsonl"
out.parent.mkdir(parents=True, exist_ok=True)
with out.open("a") as f:
    f.write(json.dumps({"argv": sys.argv[1:], "spec": os.environ.get("BENCH_SPEC"),
                        "progress": os.environ.get("BENCH_PROGRESS_FILE")}) + "\\n")
p = os.environ.get("BENCH_PROGRESS_FILE")
if p:
    Path(p).write_text(json.dumps({"fraction": 0.5, "detail": "half way"}))
time.sleep(float(os.environ.get("FAKE_SLEEP", "0")))
sys.exit(int(os.environ.get("FAKE_RC", "0")))
"""

READY_RIG = {
    "id": "test",
    "host": "h",
    "parts": {
        k: {"status": "ready", "reason": ""}
        for k in ("sim", "docker", "pipeline_bench_pc", "latte_panda")
    },
}


@pytest.fixture
def m(tmp_path: Path) -> mf.Manifest:
    (tmp_path / "fake.py").write_text(FAKE)
    (tmp_path / "bench.yaml").write_text(
        yaml.safe_dump(
            {
                "schema": 1,
                "results": "results",
                "repos": {"ifssim": "."},
                "benchmarks": {
                    "missions": {
                        "command": [sys.executable, "fake.py", "--results-root", "{results}"],
                        "settings": {
                            "missions": {"type": "string", "flag": "--missions"},
                            "pipeline_on": {"type": "string", "flag": "--pipeline-on"},
                        },
                    }
                },
            }
        )
    )
    return mf.load(tmp_path)


def spec(**settings) -> dict:
    return {
        "name": "a test",
        "benchmarks": {"missions": {"repeats": 1, "settings": {"missions": "acceleration", **settings}}},
    }


def ran(m: mf.Manifest) -> list[dict]:
    f = m.results / "ran.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines()] if f.exists() else []


# ------------------------------------------------------------------ the queue
def test_queued_specs_are_listed_newest_first_and_a_waiting_one_cancels(m):
    a = lq.enqueue(m.results, spec(), requested_by="alice")
    b = lq.enqueue(m.results, spec(), requested_by="bob", trigger="mission_control")
    listed = lq.jobs(m.results)
    assert [j["id"] for j in listed] == sorted([a, b], reverse=True)
    assert {j["state"] for j in listed} == {"queued"}
    assert lq.next_queued(m.results)["id"] == min(a, b)
    assert lq.cancel(m.results, a) == "cancelled"
    assert lq.job(m.results, a)["state"] == "cancelled"
    assert lq.cancel(m.results, "nope") is None
    with pytest.raises(ValueError):
        lq.enqueue(m.results, ["not", "a", "spec"], requested_by="x")


def test_a_worker_runs_the_queue_oldest_first_and_records_who_asked(m):
    first = lq.enqueue(m.results, spec(), requested_by="alice", trigger="mission_control")
    second = lq.enqueue(m.results, spec(missions="skidpad"), requested_by="bob")
    assert run.work(m, once=True, check_rig=lambda: READY_RIG) == 0
    assert [lq.job(m.results, i)["state"] for i in (first, second)] == ["done", "done"]
    calls = ran(m)
    assert [c["argv"][c["argv"].index("--missions") + 1] for c in calls] == [
        "acceleration",
        "skidpad",
    ]
    rec = json.loads(Path(calls[0]["spec"]).read_text())
    assert rec["requested_by"] == "alice" and rec["trigger"] == "mission_control"
    assert rec["batch_id"] == first
    st = lq.job(m.results, first)["status"]
    assert st["total"] == 1 and st["done"] == 1 and st["progress"]["detail"] == "half way"
    assert (lq.queue_dir(m.results) / first / "worker.log").exists()
    # the worker left its heartbeat and the rig's checks for Mission Control
    assert lq.read_json(lq.bench_dir(m.results) / "rig.json")["id"] == "test"
    assert lq.worker(m.results)["alive"]


def test_a_failed_benchmark_fails_the_spec(m, monkeypatch):
    monkeypatch.setenv("FAKE_RC", "3")
    jid = lq.enqueue(m.results, spec(), requested_by="alice")
    run.work(m, once=True, check_rig=lambda: READY_RIG)
    st = lq.job(m.results, jid)["status"]
    assert st["state"] == "failed" and st["failed_jobs"] == 1


def test_a_spec_the_manifest_refuses_fails_with_the_reason(m):
    jid = lq.enqueue(m.results, {"benchmarks": {"drag_race": {}}}, requested_by="alice")
    run.work(m, once=True, check_rig=lambda: READY_RIG)
    j = lq.job(m.results, jid)
    assert j["state"] == "failed" and "drag_race" in j["status"]["error"]


def test_a_part_the_rig_lacks_fails_it_and_one_that_is_down_makes_it_wait(m):
    no_panda = json.loads(json.dumps(READY_RIG))
    no_panda["parts"]["latte_panda"] = {"status": "absent", "reason": "no LattePanda on this rig"}
    never = lq.enqueue(m.results, spec(pipeline_on="latte_panda"), requested_by="a")
    run.work(m, once=True, check_rig=lambda: no_panda)
    j = lq.job(m.results, never)
    assert j["state"] == "failed" and "no LattePanda" in j["status"]["error"]

    down = json.loads(json.dumps(READY_RIG))
    down["parts"]["latte_panda"] = {"status": "unavailable", "reason": "not answering"}
    waits = lq.enqueue(m.results, spec(pipeline_on="latte_panda"), requested_by="a")
    goes = lq.enqueue(m.results, spec(), requested_by="b")
    run.work(m, once=True, check_rig=lambda: down)
    assert lq.job(m.results, waits)["state"] == "queued"
    assert "not answering" in lq.job(m.results, waits)["status"]["waiting"]
    assert lq.job(m.results, goes)["state"] == "done"  # a job behind it doesn't wait


def test_a_running_spec_stops_when_cancelled(m, monkeypatch):
    monkeypatch.setenv("FAKE_SLEEP", "30")
    jid = lq.enqueue(m.results, spec(), requested_by="alice")
    (lq.queue_dir(m.results) / jid / "cancel").touch()  # asked before its first check
    lq.set_state(m.results, jid, state="queued")
    run.work(m, once=True, check_rig=lambda: READY_RIG)
    assert lq.job(m.results, jid)["state"] == "cancelled"


def test_one_worker_per_results_folder(m):
    import fcntl

    lq.bench_dir(m.results).mkdir(parents=True)
    with (lq.bench_dir(m.results) / "worker.lock").open("w") as held:
        fcntl.flock(held, fcntl.LOCK_EX)
        assert run.work(m, once=True, check_rig=lambda: READY_RIG) == 3


def test_queue_status_and_cancel_from_the_command_line(m, capsys, monkeypatch):
    monkeypatch.setattr(rg, "check", lambda *a, **k: READY_RIG)
    sfile = m.root / "s.yaml"
    sfile.write_text(yaml.safe_dump(spec()))
    assert run.main([str(sfile), "--repo", str(m.root), "--queue"]) == 0
    jid = capsys.readouterr().out.split()[1]
    assert run.main(["--repo", str(m.root), "--status", "--json"]) == 0
    status = json.loads(capsys.readouterr().out)
    assert status["jobs"][0]["id"] == jid and not status["worker"]["alive"]
    assert run.main(["--repo", str(m.root), "--cancel", jid]) == 0
    assert lq.job(m.results, jid)["state"] == "cancelled"
    assert ran(m) == []  # queueing runs nothing


# ------------------------------------------------------------------ the rig
def fake_runner(answers: dict[str, tuple[int, str, str]]):
    def run_(cmd, timeout):
        rc, out, err = answers.get(cmd[0], (127, "", f"{cmd[0]}: not installed"))
        return subprocess.CompletedProcess(cmd, rc, out, err)

    return run_


def test_the_rig_reports_each_part_ready_down_or_absent():
    status = rg.check(
        {"id": "desk", "latte_panda": {"host": "10.0.0.2", "user": "isc"}, "hil": {"bench": "bench-01"}},
        run=fake_runner(
            {
                "nvidia-smi": (0, "GPU 0: NVIDIA GeForce RTX 4070 Ti (UUID: x)\n", ""),
                "docker": (1, "", "permission denied while trying to connect\n"),
                "ssh": (255, "", "ssh: connect to host 10.0.0.2 port 22: No route to host\n"),
            }
        ),
    )
    p = status["parts"]
    assert p["sim"]["status"] == "ready" and "4070 Ti" in p["sim"]["gpu"]
    assert p["docker"] == p["docker"] | {"status": "unavailable"}
    assert "permission denied" in p["docker"]["reason"]
    assert p["pipeline_bench_pc"]["status"] == "unavailable"  # it runs in Docker
    assert p["latte_panda"]["status"] == "unavailable" and "No route" in p["latte_panda"]["reason"]
    assert p["hil"]["status"] == "absent"  # car-in-the-loop isn't wired yet

    bare = rg.check({"id": "laptop"}, run=fake_runner({}))
    assert bare["parts"]["latte_panda"]["status"] == "absent"
    assert bare["parts"]["sim"]["status"] == "unavailable"


def test_what_a_spec_needs_from_the_rig():
    assert rg.needs(spec()) == ["sim", "docker", "pipeline_bench_pc"]
    assert rg.needs(spec(pipeline_on="latte_panda")) == ["sim", "docker", "latte_panda"]
    assert rg.problems(spec(), READY_RIG) == []


def test_the_rig_descriptor_comes_from_the_environment(tmp_path, monkeypatch):
    f = tmp_path / "rig.yaml"
    f.write_text("id: desk\nlatte_panda: {host: 10.0.0.2}\n")
    monkeypatch.setenv(rg.ENV, str(f))
    assert rg.load() == {"id": "desk", "latte_panda": {"host": "10.0.0.2"}}
