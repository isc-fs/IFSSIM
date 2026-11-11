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
    assert j["name"] == "accel" and j["spec"] == SPEC


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
