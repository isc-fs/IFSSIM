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


def test_the_rig_is_unknown_until_checked_and_stale_once_old(results):
    assert bq.rig() == bq.rig() | {"known": False, "stale": True}
    lq = bq.localq()
    path = lq.bench_dir(results) / "rig.json"
    lq.write_json(path, {"id": "bench-pc", "checked_at": time.time(), "parts": {"sim": {"status": "ready"}}})
    # fresh checks count with or without a worker (run_spec.py --rig writes them too)
    r = bq.rig()
    assert r["known"] and not r["stale"] and r["parts"]["sim"]["status"] == "ready"
    lq.write_json(path, {"id": "bench-pc", "checked_at": time.time() - 600, "parts": {}})
    assert bq.rig()["stale"] and "10 min old" in bq.rig()["reason"]


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


def test_the_manifest_says_what_a_custom_run_can_set(results):
    bag = results / "capture" / "trackdrive_x_20260101_000000"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {duration: {nanoseconds: 2000000000}}\n")
    m = bq.manifest()
    assert m["error"] is None
    benches = {b["name"]: b for b in m["benchmarks"]}
    missions = {s["name"]: s for s in benches["missions"]["settings"]}
    assert missions["repeats"]["type"] == "integer" and missions["laps"]["help"]
    assert "mc_url" not in missions  # launch points it at this backend
    assert benches["sim_bag"]["bags"] == "simulator" and "slam" in benches["sim_bag"]["parts"]
    assert "control_node" in benches["missions"]["components"]
    # the bags are where this process sees the results, not bench.yaml's host path
    assert m["bags"]["simulator"] == [
        {"name": bag.name, "duration_s": 2.0, "size_bytes": bag.joinpath("metadata.yaml").stat().st_size}
    ]
    assert m["missions"] == ["acceleration", "skidpad", "autocross", "trackdrive"]
    assert m["mission_tracks"]["acceleration"] == ["acceleration.csv"]
    assert len(m["mission_tracks"]["autocross"]) >= 1
    assert "characterisation.csv" in m["tracks"]


def test_without_bench_yaml_the_manifest_says_why(results, tmp_path, monkeypatch):
    monkeypatch.setenv("IFSSIM_BENCH_MANIFEST", str(tmp_path / "nope" / "bench.yaml"))
    m = bq.manifest()
    assert "no bench.yaml" in m["error"] and m["benchmarks"] == []
    # and a launch is still queued: the worker checks it when it takes it
    assert bq.launch({"benchmarks": {"drag_race": {}}})["id"]


def test_check_runs_the_workers_validation(results):
    ok = bq.check({"benchmarks": {"missions": {"repeats": 2, "settings": {"missions": "skidpad"}}}})
    assert ok["ok"] and ok["errors"] == [] and ok["jobs"] >= 1
    assert ok["needs"] == ["sim", "docker", "pipeline_bench_pc"]
    bad = bq.check({"benchmarks": {"sim_bag": {"bags": []}, "missions": {"settings": {"laps": "two"}}}})
    assert not bad["ok"] and bad["jobs"] == 0
    assert any("sim_bag.bags" in e for e in bad["errors"])
    assert any("laps" in e for e in bad["errors"])  # every problem at once


def test_a_launch_the_worker_would_refuse_is_refused_now(results):
    with pytest.raises(bq.LaunchError) as e:
        bq.launch({"benchmarks": {"missions": {"settings": {"repeats_typo": 1}}}})
    assert e.value.status_code == 400 and "repeats_typo" in str(e.value)
    assert bq.queue()["jobs"] == []


def test_override_files_come_with_the_presets(tmp_path, monkeypatch):
    od = tmp_path / "specs" / "overrides"
    od.mkdir(parents=True)
    (od / "looser.yaml").write_text("# A looser gate.\npipeline: {cone_detection: {residual_gate_mse: 0.05}}\n")
    (od / "empty.yaml").write_text("name: nothing to override\n")
    monkeypatch.setattr(bq, "presets_dir", lambda: tmp_path / "specs")
    got = {o["name"]: o for o in bq.presets()["overrides"]}
    assert got["looser"] == {
        "name": "looser",
        "about": "A looser gate.",
        "pipeline": {"cone_detection": {"residual_gate_mse": 0.05}},
    }
    assert "no pipeline" in got["empty"]["error"]


def test_stop_worker_asks_the_worker_to_stop(results):
    bq.localq().heartbeat(results)
    out = bq.stop_worker()
    assert out["stopping"] and out["worker"]["stopping"]


def test_check_counts_the_runs_a_person_would(results):
    # skidpad and autocross from missions.yaml, 3 runs per track, the whole thing twice
    defaults = bq._mission_defaults()
    got = bq.check({"benchmarks": {"missions": {"repeats": 2, "settings": {"missions": "skidpad,autocross", "repeats": 3}}}})
    want = sum(len(defaults[m]["tracks"]) * 3 * 2 for m in ("skidpad", "autocross"))
    assert got["ok"] and got["runs"] == want
    assert got["lines"][0] == f"skidpad: {len(defaults['skidpad']['tracks'])} track{'' if len(defaults['skidpad']['tracks']) == 1 else 's'} x 3 x 2"
    # chosen tracks replace each mission's; no missions chosen is all of them
    got = bq.check({"benchmarks": {"missions": {"settings": {"tracks": "a.csv,b.csv"}}}})
    assert got["runs"] == 2 * sum(int(d.get("repeats") or 1) for d in defaults.values())


def test_the_manifest_offers_the_pipeline_repo_and_the_boards(results):
    m = bq.manifest()
    assert m["pipeline_repo"] == "isc-fs/IFS09-DV-PIPELINE"
    assert m["boards"]["firmware"]["ecu"]["repo"] == "isc-fs/IFS08-CE-ECU"
    assert m["boards"]["vhil"]["ecu-ams-udv"] == ["ecu", "ams", "udv"] and "ams" in m["boards"]["hil"]


def test_refs_lists_branches_and_prs_of_known_repos_only(results, monkeypatch):
    calls = []

    def fake(path):
        calls.append(path)
        if path.endswith("/branches?per_page=100"):
            return [{"name": "main"}, {"name": "feat/a"}]
        if "/pulls" in path:
            return [{"number": 88, "title": "Loop closure", "head": {"ref": "feat/a"}, "user": {"login": "x"}}]
        return {"default_branch": "main"}

    monkeypatch.setattr(bq, "_github_get", fake)
    monkeypatch.setattr(bq, "_refs_cache", {})
    got = bq.refs("isc-fs/IFS09-DV-PIPELINE")
    assert got["branches"] == ["feat/a", "main"] and got["default"] == "main"
    assert got["pulls"] == [{"number": 88, "title": "Loop closure", "branch": "feat/a", "author": "x"}]
    n = len(calls)
    bq.refs("isc-fs/IFS09-DV-PIPELINE")
    assert len(calls) == n  # cached
    assert "isn't a repository" in bq.refs("someone/else")["error"] and len(calls) == n


def test_refs_says_so_when_github_doesnt_answer(results, monkeypatch):
    def down(path):
        raise OSError("no network")

    monkeypatch.setattr(bq, "_github_get", down)
    monkeypatch.setattr(bq, "_refs_cache", {})
    got = bq.refs("isc-fs/IFS08-CE-AMS")
    assert "type a branch" in got["error"] and got["branches"] == []
