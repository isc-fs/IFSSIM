"""The Benchmarks tab's backend (bench_queue.py): queueing into the local queue the
host's worker runs, and reading the queue and the rig's checks back."""

from __future__ import annotations

import time

import pytest

import bench_queue as bq

SPEC = {"name": "accel", "benchmarks": {"missions": {"settings": {"missions": "acceleration"}}}}


@pytest.fixture
def results(tmp_path, monkeypatch):
    monkeypatch.setenv("IFSSIM_BENCHMARK_RESULTS_ROOT", str(tmp_path))
    return tmp_path


def test_local_launch_queues_the_spec_and_says_when_no_worker_runs_it(results):
    out = bq.launch(SPEC, "local", requested_by="Alberto-cd")
    assert not out["worker_running"] and "--work" in out["hint"]
    (j,) = bq.queue()["jobs"]
    assert j["id"] == out["id"] and j["state"] == "queued"
    assert j["requested_by"] == "Alberto-cd" and j["trigger"] == "mission_control"
    assert j["name"] == "accel"
    # the mission benchmark is pointed at this backend, not the web UI's :3000
    assert j["spec"]["benchmarks"]["missions"]["settings"] == {
        "missions": "acceleration",
        "mc_url": "http://127.0.0.1:8000",
    }
    assert "mc_url" not in SPEC["benchmarks"]["missions"]["settings"]  # not changed in place


def test_a_spec_that_names_mission_control_keeps_it(results):
    spec = {"benchmarks": {"missions": {"settings": {"mc_url": "http://10.0.0.5:8000"}}}}
    bq.launch(spec)
    got = bq.queue()["jobs"][0]["spec"]["benchmarks"]["missions"]["settings"]
    assert got["mc_url"] == "http://10.0.0.5:8000"


def test_with_a_worker_running_there_is_no_hint(results):
    bq.localq().heartbeat(results)
    out = bq.launch(SPEC)
    assert out["worker_running"] and out["hint"] is None


def test_bad_launches_are_refused(results):
    with pytest.raises(bq.LaunchError) as e:
        bq.launch(SPEC, "runner")
    assert e.value.status_code == 501
    for bad in ({}, [], {"benchmarks": {}}):
        with pytest.raises(bq.LaunchError):
            bq.launch(bad)
    with pytest.raises(bq.LaunchError):
        bq.launch(SPEC, "cloud")
    assert bq.queue()["jobs"] == []


def test_cancel(results):
    jid = bq.launch(SPEC)["id"]
    assert bq.cancel(jid) == "cancelled"
    assert bq.queue()["jobs"][0]["state"] == "cancelled"
    assert bq.cancel("nope") is None
    assert bq.cancel("../../etc") is None


def test_the_rig_is_unknown_until_a_worker_checks_it_and_stale_without_one(results):
    assert bq.rig() == bq.rig() | {"known": False, "stale": True}
    lq = bq.localq()
    lq.write_json(
        lq.bench_dir(results) / "rig.json",
        {"id": "bench-pc", "checked_at": time.time(), "parts": {"sim": {"status": "ready"}}},
    )
    assert bq.rig() | {} == bq.rig() and bq.rig()["stale"]  # no worker heartbeat
    lq.heartbeat(results)
    r = bq.rig()
    assert r["known"] and not r["stale"] and r["parts"]["sim"]["status"] == "ready"


def test_presets_are_the_spec_files_as_json_with_what_they_need():
    got = {p["name"]: p for p in bq.presets()["presets"]}
    names = {p.stem for p in bq.presets_dir().glob("*.yaml")}
    assert set(got) == names and "missions-basic" in names
    basic = got["missions-basic"]
    assert basic["title"] == "missions, basic"
    assert basic["spec"]["benchmarks"]["missions"]["settings"]["missions"] == "all"
    assert basic["needs"] == ["sim", "docker", "pipeline_bench_pc"]
    # the leading comment, without the command lines after it
    assert basic["about"].startswith("The basic complete benchmark")
    assert "run_spec.py" not in basic["about"]


def test_a_preset_that_isnt_yaml_says_so(tmp_path, monkeypatch):
    (tmp_path / "good.yaml").write_text("# Fine.\nname: good\nbenchmarks: {missions: {}}\n")
    (tmp_path / "bad.yaml").write_text("name: [\n")
    monkeypatch.setattr(bq, "presets_dir", lambda: tmp_path)
    bad, good = bq.presets()["presets"]
    assert bad["name"] == "bad" and "bad.yaml" in bad["error"]
    assert good["about"] == "Fine." and good["needs"][:2] == ["sim", "docker"]
