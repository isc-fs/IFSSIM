"""Uploading from several machines: a run goes to the tracker once, and half-finished runs wait."""

from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path

import pytest

from bench_tracking import cli, provenance
from bench_tracking.adapters import sim_bag
from bench_tracking.backends.base import Backend, UploadResult

from test_sim_bag import _perception, _prov, _session


class FakeServer(Backend):
    """Remembers run keys like the MLflow backend does with its bench.run_key tag."""

    name = "fake"
    can_find = True

    def __init__(self) -> None:
        self.runs: dict[str, str] = {}  # run id -> key
        self.uploads = 0

    def upload(self, b, *, figures, parent=None) -> UploadResult:
        self.uploads += 1
        rid = f"r{self.uploads}"
        self.runs[rid] = b.run_key
        return UploadResult(self.name, rid, None, b.name)

    def find(self, run_key):
        rid = next((r for r, k in self.runs.items() if k == run_key), None)
        return UploadResult(self.name, rid, None, "") if rid else None

    def claim(self, run_id, run_key):
        if run_id not in self.runs:
            return False
        self.runs[run_id] = run_key
        return True


@pytest.fixture
def server(monkeypatch, tmp_path):
    s = FakeServer()
    monkeypatch.setattr(cli, "get_backend", lambda name: s)
    monkeypatch.setattr(cli, "STATE_DIR", tmp_path / "state")
    return s


def _machine(root: Path, name: str) -> Path:
    """The same results, as another machine has them: another absolute path."""
    dst = root / name / "results"
    shutil.copytree(root / "results", dst)
    return dst


def test_same_run_on_two_machines_is_uploaded_once(tmp_path, server, monkeypatch):
    _session(
        tmp_path / "results",
        "20260925_100000",
        dict(_prov("1111", False), capture_id="c1"),
    )
    a, b = _machine(tmp_path, "alice"), _machine(tmp_path, "bob")
    (ba,) = sim_bag.load_all(a, full_bag_hash=False)
    (bb,) = sim_bag.load_all(b, full_bag_hash=False)
    assert ba.run_key == bb.run_key and ba.legacy_run_key != bb.legacy_run_key

    cli.Uploader("fake").up(ba)
    # bob has his own, empty, state file
    monkeypatch.setattr(cli, "STATE_DIR", tmp_path / "bob-state")
    u = cli.Uploader("fake")
    u.up(bb)
    assert server.uploads == 1 and u.counts["already there"] == 1


def test_runs_from_an_old_state_file_are_claimed_not_reuploaded(tmp_path, server):
    _session(tmp_path / "results", "20260925_100000", None)
    (b,) = sim_bag.load_all(tmp_path / "results", full_bag_hash=False)
    server.runs["old"] = "?"  # uploaded before keys were stored on the server
    cli.save_state("fake", {b.legacy_run_key: {"run_id": "old", "job_type": "sim_bag"}})
    u = cli.Uploader("fake")
    u.up(b)
    assert server.uploads == 0 and server.runs["old"] == b.run_key
    st = cli.load_state("fake")
    assert list(st) == [b.run_key] and st[b.run_key]["uploaded_here"]


def test_a_run_deleted_on_the_server_is_uploaded_again(tmp_path, server):
    _session(tmp_path / "results", "20260925_100000", None)
    (b,) = sim_bag.load_all(tmp_path / "results", full_bag_hash=False)
    cli.Uploader("fake").up(b)
    server.runs.clear()  # an admin deleted it (e.g. to re-import after an importer fix)
    cli.Uploader("fake").up(b)
    assert server.uploads == 2


def test_found_runs_are_not_this_machines_to_purge(tmp_path, server, monkeypatch):
    _session(tmp_path / "results", "20260925_100000", None)
    (b,) = sim_bag.load_all(tmp_path / "results", full_bag_hash=False)
    server.runs["theirs"] = b.run_key
    cli.Uploader("fake").up(b)
    assert cli.load_state("fake")[b.run_key]["uploaded_here"] is False


def test_unfinished_runs_wait(tmp_path):
    rd = _perception(tmp_path, "20260925_100000")
    assert not provenance.in_progress(rd)  # has results.json
    (rd / "results.json").unlink()
    (rd / "provenance.json").write_text(json.dumps(_prov("1", False)))
    assert provenance.in_progress(rd)
    old = time.time() - provenance.STALE_AFTER_S - 60
    for p in rd.iterdir():
        os.utime(p, (old, old))
    assert not provenance.in_progress(rd)  # crashed long ago: upload what there is


def test_running_session_is_skipped(tmp_path):
    d = _session(tmp_path, "20260925_100000", _prov("1", False))
    doc = json.loads((d / "session.json").read_text())
    (d / "session.json").write_text(json.dumps(dict(doc, status="running")))
    assert sim_bag.find_sessions(tmp_path) == []
    (d / "session.json").write_text(
        json.dumps(dict(doc, status="finished", finished_at="x"))
    )
    assert len(sim_bag.find_sessions(tmp_path)) == 1


def test_runs_that_recorded_their_code_are_not_paired(tmp_path):
    from test_sim_bag import _slam

    p = _perception(tmp_path, "20260925_100000")
    s = _slam(tmp_path, "20260925_100002")
    for rd in (p, s):
        (rd / "provenance.json").write_text(json.dumps(_prov("1", False)))
    ss = sim_bag.legacy_sessions(tmp_path)
    assert [list(x.parts) for x in ss] == [["perception"], ["slam"]]
    b = sim_bag.load_session(ss[0], full_bag_hash=False)
    assert "paired" not in b.tags
