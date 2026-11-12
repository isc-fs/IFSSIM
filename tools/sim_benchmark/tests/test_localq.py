"""The local queue (launch/localq.py), its worker (run_spec.py --work) and the rig
checks (launch/rig.py), against a small manifest and a fake benchmark."""

from __future__ import annotations

import dataclasses
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


def test_the_worker_from_the_command_line(m, monkeypatch):
    monkeypatch.setattr(rg, "check", lambda *a, **k: READY_RIG)
    jid = lq.enqueue(m.results, spec(), requested_by="alice")
    assert run.main(["--repo", str(m.root), "--work", "--once"]) == 0
    assert lq.job(m.results, jid)["state"] == "done"


def test_what_root_creates_goes_to_the_results_folders_owner(m, monkeypatch):
    # Mission Control's backend runs as root in its container
    chowned = []
    monkeypatch.setattr(lq.os, "geteuid", lambda: 0)
    monkeypatch.setattr(lq.os, "chown", lambda p, u, g: chowned.append((Path(p).name, u)))
    m.results.mkdir()
    jid = lq.enqueue(m.results, spec(), requested_by="mission_control")
    lq.set_state(m.results, jid, state="running")
    lq.cancel(m.results, jid)
    me = m.results.stat().st_uid
    names = {n for n, u in chowned if u == me}
    assert {".bench", "queue", jid, "cancel"} <= names
    assert any(n.startswith(".job.json") for n in names)
    assert any(n.startswith(".state.json") for n in names)


def test_a_worker_asked_to_stop_leaves_the_queue_and_says_it_stopped(m):
    jid = lq.enqueue(m.results, spec(), requested_by="alice")

    def rig_then_stop():
        lq.request_stop(m.results)  # Mission Control's Stop worker, while it waits
        return READY_RIG

    assert lq.worker(m.results)["stopping"] is False
    # without once=True it would wait for more specs: the stop is what ends it
    assert run.work(m, check_rig=rig_then_stop) == 0
    assert lq.job(m.results, jid)["state"] == "queued"  # left for the next worker
    w = lq.worker(m.results)
    assert not w["alive"] and not w["stopping"]


def test_a_stop_asked_before_the_worker_started_is_forgotten(m):
    lq.request_stop(m.results)
    jid = lq.enqueue(m.results, spec(), requested_by="alice")
    assert run.work(m, once=True, check_rig=lambda: READY_RIG) == 0
    assert lq.job(m.results, jid)["state"] == "done"


def test_docker_that_cant_reach_its_daemon_is_not_ready():
    # `docker info --format` exits 0 without a server version when the socket refuses
    status = rg.check({"id": "desk"}, run=fake_runner({
        "nvidia-smi": (0, "GPU 0: x\n", ""),
        "docker": (0, "\n", "permission denied while trying to connect to the Docker daemon socket\n"),
    }))
    assert status["parts"]["docker"]["status"] == "unavailable"
    assert "permission denied" in status["parts"]["docker"]["reason"]
    assert status["parts"]["pipeline_bench_pc"]["status"] == "unavailable"


def test_the_rig_check_is_saved_for_mission_control_with_the_code(m, monkeypatch):
    monkeypatch.setattr(rg, "check", lambda: READY_RIG)
    monkeypatch.setattr(run, "code_versions", lambda _m: {"ifssim": {"commit": "abc1234", "branch": "dev", "dirty": False}})
    assert run.main(["--repo", str(m.root), "--rig", "--json"]) == 0
    saved = lq.read_json(lq.bench_dir(m.results) / "rig.json")
    assert saved["id"] == "test" and saved["code"]["ifssim"]["branch"] == "dev"


def test_code_versions_reads_each_repository(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    subprocess.run(["git", "init", "-q", "-b", "dev", str(repo)], check=True)
    (repo / "f").write_text("x")
    subprocess.run(["git", "-C", str(repo), "add", "f"], check=True)
    subprocess.run(["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "x"], check=True)
    fake = mf.Manifest(root=repo, results=repo / "r", presets=None, repos={"ifssim": repo, "pipeline": tmp_path / "none"},
                       bag_dirs={}, benchmarks={})
    got = run.code_versions(fake)
    assert set(got) == {"ifssim"} and got["ifssim"]["branch"] == "dev" and not got["ifssim"]["dirty"]
    (repo / "f").write_text("y")
    assert run.code_versions(fake)["ifssim"]["dirty"]


# ------------------------------------------------------------------ code.pipeline and boards
def _git(*args, cwd):
    subprocess.run(["git", "-c", "user.name=t", "-c", "user.email=t@t", *args], cwd=cwd, check=True,
                   capture_output=True)


@pytest.fixture
def piped(m, tmp_path):
    """The manifest with a pipeline clone whose origin has dev, a branch and PR #7."""
    origin = tmp_path / "origin"
    origin.mkdir()
    _git("init", "-q", "-b", "dev", cwd=origin)
    (origin / "params.yaml").write_text("gate: 1\n")
    _git("add", ".", cwd=origin)
    _git("commit", "-qm", "dev", cwd=origin)
    _git("checkout", "-qb", "feat/x", cwd=origin)
    (origin / "params.yaml").write_text("gate: 2\n")
    _git("commit", "-qam", "branch", cwd=origin)
    _git("checkout", "-q", "dev", cwd=origin)
    _git("checkout", "-qb", "pr7", cwd=origin)
    (origin / "params.yaml").write_text("gate: 7\n")
    _git("commit", "-qam", "pr", cwd=origin)
    _git("update-ref", "refs/pull/7/head", "pr7", cwd=origin)
    _git("checkout", "-q", "dev", cwd=origin)
    _git("clone", "-q", str(origin), str(tmp_path / "pipeline"), cwd=tmp_path)
    return dataclasses.replace(m, repos={**m.repos, "pipeline": tmp_path / "pipeline"})


def test_code_pipeline_checks_out_a_branch_a_pr_or_a_commit(piped, tmp_path):
    for ref, gate in (("feat/x", "2"), ("#7", "7"), ("pr/7", "7")):
        where = tmp_path / "wt" / ref.replace("/", "_").replace("#", "")
        sha = run.pipeline_worktree(piped, ref, where)
        assert (where / "params.yaml").read_text() == f"gate: {gate}\n" and len(sha) == 40
        run.remove_pipeline_worktree(piped, where)
        assert not where.exists()
    dev = subprocess.run(["git", "-C", str(tmp_path / "origin"), "rev-parse", "dev"], capture_output=True, text=True).stdout.strip()
    assert run.pipeline_worktree(piped, dev[:9], tmp_path / "wt" / "c") == dev
    with pytest.raises(run.sp.SpecError, match="couldn't fetch branch nope"):
        run.pipeline_worktree(piped, "nope", tmp_path / "wt" / "n")


def test_a_queued_spec_runs_on_its_code_pipeline_and_cleans_up(piped):
    jid = lq.enqueue(piped.results, spec() | {"code": {"pipeline": "feat/x"}}, requested_by="alice")
    run.work(piped, once=True, check_rig=lambda: READY_RIG)
    st = lq.job(piped.results, jid)["status"]
    assert st["state"] == "done" and st["pipeline"]["ref"] == "feat/x"
    assert not (lq.queue_dir(piped.results) / jid / "pipeline").exists()  # the worktree is gone
    bad = lq.enqueue(piped.results, spec() | {"code": {"pipeline": "nope"}}, requested_by="alice")
    run.work(piped, once=True, check_rig=lambda: READY_RIG)
    st = lq.job(piped.results, bad)["status"]
    assert st["state"] == "failed" and "nope" in st["error"]


def test_boards_are_checked_against_the_manifest_and_need_car_in_the_loop(m):
    boards = {"firmware": {"ecu": {"repo": "isc-fs/ecu", "default": "dev"}, "ams": {"repo": "isc-fs/ams", "default": "dev"}},
              "hil": ["ecu"], "vhil": {"ecu-ams": ["ecu", "ams"]}}
    mb = dataclasses.replace(m, boards=boards)
    ok = spec() | {"boards": {"mode": "vhil", "system": "ecu-ams", "firmware": {"ams": "feat/y"}}}
    assert run.sp.validate(ok, mb)["boards"]["system"] == "ecu-ams"
    for bad, want in (
        ({"mode": "can"}, "boards.mode"),
        ({"mode": "vhil", "system": "udv"}, "boards.system"),
        ({"mode": "hil", "firmware": {"ams": "dev"}}, "boards.firmware.ams"),
    ):
        with pytest.raises(run.sp.SpecError, match=want):
            run.sp.validate(spec() | {"boards": bad}, mb)
    assert rg.needs(ok)[-1] == "vhil"
    probs = rg.problems(ok, rg.check({"id": "desk"}, run=fake_runner({})))
    assert {"part": "vhil", "status": "absent", "reason": "car-in-the-loop not wired yet"} in probs


def test_vhil_follows_docker_once_car_in_the_loop_is_wired(monkeypatch):
    docker_up = {"nvidia-smi": (0, "GPU 0: x\n", ""), "docker": (0, "27.0\n", "")}
    assert rg.check({"id": "d"}, run=fake_runner(docker_up))["parts"]["vhil"]["status"] == "absent"
    monkeypatch.setattr(rg, "CAR_IN_THE_LOOP", True)
    assert rg.check({"id": "d"}, run=fake_runner(docker_up))["parts"]["vhil"]["status"] == "ready"
    down = rg.check({"id": "d"}, run=fake_runner({"docker": (1, "", "Cannot connect\n")}))
    assert down["parts"]["vhil"] == {"status": "unavailable", "reason": "needs Docker"}
