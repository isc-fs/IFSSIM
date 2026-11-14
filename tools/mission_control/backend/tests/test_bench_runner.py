"""The Benchmarks tab's Runner target (bench_runner.py) against a fake GitHub."""

from __future__ import annotations

import io
import json
import os
import stat
import sys
import zipfile

import pytest

import bench_queue as bq
import bench_runner as br

SPEC = {"name": "accel", "benchmarks": {"missions": {"settings": {"missions": "acceleration"}}}}
WF = ".github/workflows/{}"


class FakeGitHub:
    def __init__(self):
        self.calls: list[tuple[str, str, dict]] = []
        self.routes: dict[tuple[str, str], object] = {}

    def on(self, method, url, answer):
        self.routes[(method, url)] = answer

    def __call__(self, method, url, **kw):
        self.calls.append((method, url, kw))
        a = self.routes.get((method, url), self.routes.get((method, url.split("?")[0])))
        if a is None and (method, url) not in self.routes:
            raise br.RunnerError(f"GitHub said 404 for {url}", 404)
        if isinstance(a, Exception):
            raise a
        return a(kw) if callable(a) else a


@pytest.fixture
def gh(monkeypatch, tmp_path):
    fake = FakeGitHub()
    monkeypatch.setattr(br, "http", fake)
    for env in ("IFSSIM_GITHUB_TOKEN", "GH_TOKEN", "IFSSIM_GITHUB_CLIENT_ID"):
        monkeypatch.delenv(env, raising=False)
    monkeypatch.setenv("IFSSIM_BENCHMARK_RESULTS_ROOT", str(tmp_path))
    br._cache.clear()
    br._device.clear()
    return fake


API = "https://api.github.com/"


def bench(tmp_path):
    return tmp_path / ".bench"


# ------------------------------------------------------------------ signing in
def test_a_token_in_the_environment_wins(gh, tmp_path, monkeypatch):
    br._save(bench(tmp_path), {"token": "stored", "login": "x"})
    assert br.token(bench(tmp_path)) == ("stored", "sign-in")
    monkeypatch.setenv("GH_TOKEN", "env-token")
    assert br.token(bench(tmp_path)) == ("env-token", "GH_TOKEN")


def test_without_a_client_id_sign_in_says_how_to_set_it_up(gh):
    with pytest.raises(br.RunnerError) as e:
        br.sign_in_start()
    assert e.value.status_code == 501 and "IFSSIM_GITHUB_CLIENT_ID" in str(e.value)


def test_the_device_flow_stores_the_token_for_this_user_only(gh, tmp_path, monkeypatch):
    monkeypatch.setenv("IFSSIM_GITHUB_CLIENT_ID", "Iv1.abc")
    gh.on("POST", "https://github.com/login/device/code",
          {"device_code": "dc", "user_code": "ABCD-1234", "verification_uri": "https://github.com/login/device",
           "expires_in": 900, "interval": 0})
    answers = iter([{"error": "authorization_pending"}, {"access_token": "gho_x", "token_type": "bearer"}])
    gh.on("POST", "https://github.com/login/oauth/access_token", lambda kw: next(answers))
    gh.on("GET", API + "user", {"login": "Alberto-cd"})
    start = br.sign_in_start()
    assert start["user_code"] == "ABCD-1234"
    assert br.sign_in_poll(bench(tmp_path))["state"] == "pending"
    br._device["next"] = 0
    got = br.sign_in_poll(bench(tmp_path))
    assert got == {"state": "signed_in", "login": "Alberto-cd"}
    assert br.token(bench(tmp_path)) == ("gho_x", "sign-in")
    if sys.platform != "win32":
        mode = stat.S_IMODE(os.stat(bench(tmp_path) / br.TOKEN_FILE).st_mode)
        assert mode == 0o600
    assert br.sign_in_poll(bench(tmp_path)) == {"state": "none"}
    br.sign_out(bench(tmp_path))
    assert br.token(bench(tmp_path)) == (None, "")


def test_a_refused_sign_in_fails(gh, tmp_path, monkeypatch):
    monkeypatch.setenv("IFSSIM_GITHUB_CLIENT_ID", "Iv1.abc")
    gh.on("POST", "https://github.com/login/device/code", {"device_code": "dc", "user_code": "X", "interval": 0})
    gh.on("POST", "https://github.com/login/oauth/access_token", {"error": "access_denied"})
    br.sign_in_start()
    assert br.sign_in_poll(bench(tmp_path))["state"] == "failed"


def test_account_says_whether_you_may_launch(gh, tmp_path, monkeypatch):
    assert br.account(bench(tmp_path))["signed_in"] is False
    monkeypatch.setenv("GH_TOKEN", "t")
    gh.on("GET", API + "user", {"login": "someone"})
    gh.on("GET", API + f"repos/{br.REPO}", {"permissions": {"pull": True, "push": False}})
    acc = br.account(bench(tmp_path))
    assert acc["login"] == "someone" and acc["can_launch"] is False


# ------------------------------------------------------------------ launching
def test_runner_launch_dispatches_bench_yml_with_the_spec(gh, tmp_path, monkeypatch):
    monkeypatch.setenv("GH_TOKEN", "t")
    gh.on("POST", API + f"repos/{br.REPO}/actions/workflows/bench.yml/dispatches", None)
    out = bq.launch(SPEC, "runner", name="accel quick")
    assert out["target"] == "runner" and len(out["id"]) == 8
    (method, url, kw), = gh.calls
    assert kw["token"] == "t" and kw["data"]["ref"] == br.DISPATCH_REF
    inputs = kw["data"]["inputs"]
    assert json.loads(inputs["spec"]) == SPEC  # as given: the runner points it at its own backend
    assert inputs["name"] == "accel quick" and inputs["id"] == out["id"]
    assert bq.queue(runner=False)["jobs"] == []  # nothing in the local queue


def test_runner_launch_needs_a_sign_in(gh):
    with pytest.raises(bq.LaunchError) as e:
        bq.launch(SPEC, "runner")
    assert e.value.status_code == 401 and not gh.calls


def test_runner_launch_without_write_access_is_403(gh, monkeypatch):
    monkeypatch.setenv("GH_TOKEN", "t")
    gh.on("POST", API + f"repos/{br.REPO}/actions/workflows/bench.yml/dispatches",
          br.RunnerError("GitHub said 403: Must have admin rights", 403))
    with pytest.raises(bq.LaunchError) as e:
        bq.launch(SPEC, "runner")
    assert e.value.status_code == 403


# ------------------------------------------------------------------ the queue
def run(rid, wf, status, conclusion=None, created="2026-11-14T10:00:00Z", started=None, updated=None,
        title="t", actor="Alberto-cd"):
    return {"id": rid, "path": WF.format(wf), "status": status, "conclusion": conclusion,
            "created_at": created, "run_started_at": started or created, "updated_at": updated or created,
            "display_title": title, "triggering_actor": {"login": actor}, "event": "workflow_dispatch",
            "head_branch": "dev", "head_sha": "abcdef123", "html_url": f"https://github.com/x/{rid}"}


def ts(s):
    return br._ts(s)


def test_the_queue_shows_every_gpu_workflow_and_finds_its_own_runs(gh, tmp_path, monkeypatch):
    monkeypatch.setenv("GH_TOKEN", "t")
    runs = [
        run(1, "bench.yml", "completed", "success", created="2026-11-14T08:00:00Z",
            started="2026-11-14T08:00:00Z", updated="2026-11-14T08:10:00Z"),
        run(2, "sim-pr.yml", "in_progress", created="2026-11-14T10:00:00Z", started="2026-11-14T10:00:00Z"),
        run(3, "bench.yml", "queued", created="2026-11-14T10:01:00Z", title="Bench: accel [abc12345]"),
        run(4, "ci.yml", "in_progress", created="2026-11-14T10:02:00Z"),  # not on the GPU runner
        run(5, "sim-pr.yml", "completed", "success", created="2026-11-14T07:00:00Z",
            started="2026-11-14T07:00:00Z", updated="2026-11-14T07:15:00Z"),
    ]
    gh.on("GET", API + f"repos/{br.REPO}/actions/runs?per_page=100", {"workflow_runs": runs})
    gh.on("GET", API + f"repos/{br.REPO}/actions/runs/2/jobs",
          {"jobs": [{"name": "sim", "status": "in_progress", "steps": [
              {"name": "Checkout", "status": "completed"}, {"name": "Drive the sim", "status": "in_progress"}]}]})
    q = br.queue(bench(tmp_path), now=ts("2026-11-14T10:05:00Z"))
    assert [r["run_id"] for r in q["runs"]] == [3, 2, 1, 5]
    queued, running = q["runs"][0], q["runs"][1]
    assert queued["id"] == "abc12345" and queued["state"] == "queued" and queued["ahead"] == 1
    # the sim check has ~10 min left of its 15, then this bench run's typical 10 min
    assert queued["eta_s"] == 600 + 600
    assert running["current"] == "sim: Drive the sim" and running["eta_s"] == 600
    assert q["runs"][2]["state"] == "done" and q["runner_offline"] is False


def test_a_push_runs_title_is_its_commit_messages_first_line(gh, tmp_path):
    gh.on("GET", API + f"repos/{br.REPO}/actions/runs?per_page=100",
          {"workflow_runs": [run(8, "sim-pr.yml", "completed", "success", title="fix: a thing (#1)\n\nlong body")]})
    assert br.queue(bench(tmp_path))["runs"][0]["name"] == "fix: a thing (#1)"


def test_a_run_queued_for_long_with_nothing_running_flags_the_runner(gh, tmp_path):
    gh.on("GET", API + f"repos/{br.REPO}/actions/runs?per_page=100",
          {"workflow_runs": [run(7, "bench.yml", "queued", created="2026-11-14T10:00:00Z")]})
    q = br.queue(bench(tmp_path), now=ts("2026-11-14T10:10:00Z"))
    assert q["runner_offline"] is True
    assert q["runs"][0]["current"] is None


def test_without_a_sign_in_the_queue_is_read_rarely(gh, tmp_path):
    gh.on("GET", API + f"repos/{br.REPO}/actions/runs?per_page=100", {"workflow_runs": []})
    for _ in range(5):
        br.queue(bench(tmp_path))
    assert len(gh.calls) == 1 and gh.calls[0][2]["token"] is None


def test_github_down_is_an_error_in_the_queue_not_a_failure(gh, tmp_path):
    q = bq.queue()
    assert q["jobs"] == [] and q["runner"]["error"] and q["runner"]["runs"] == []


def test_cancel(gh, tmp_path, monkeypatch):
    monkeypatch.setenv("GH_TOKEN", "t")
    gh.on("POST", API + f"repos/{br.REPO}/actions/runs/42/cancel", None)
    assert br.cancel(bench(tmp_path), 42)["state"] == "cancelling"


# ------------------------------------------------------------------ fetching results
def _zip(files: dict[str, str]) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for name, text in files.items():
            z.writestr(name, text)
    return buf.getvalue()


def test_fetched_results_land_where_local_runs_do(gh, tmp_path, monkeypatch):
    monkeypatch.setenv("GH_TOKEN", "t")
    results = tmp_path / "results"
    gh.on("GET", API + f"repos/{br.REPO}/actions/runs/9/artifacts",
          {"artifacts": [{"name": "bench-abc", "archive_download_url": "https://dl/9", "expired": False}]})
    gh.on("GET", "https://dl/9", _zip({
        "mission/acceleration_acceleration/abc/seed1/results.json": "{}",
        ".bench/20261114_x/1/spec.json": "{}",
        "headless/20261114_exec/run.json": "{}",
        "../escape.txt": "no",
    }))
    out = br.fetch_results(bench(tmp_path), results, 9)
    assert out["files"] == 3
    assert (results / "mission/acceleration_acceleration/abc/seed1/results.json").is_file()
    assert (results / "runner/9/.bench/20261114_x/1/spec.json").is_file()
    assert (results / "runner/9/headless/20261114_exec/run.json").is_file()
    assert not (results / ".bench").exists() and not (tmp_path / "escape.txt").exists()


def test_a_run_without_results(gh, tmp_path, monkeypatch):
    monkeypatch.setenv("GH_TOKEN", "t")
    gh.on("GET", API + f"repos/{br.REPO}/actions/runs/9/artifacts", {"artifacts": [{"name": "sim-pr-results"}]})
    with pytest.raises(br.RunnerError) as e:
        br.fetch_results(bench(tmp_path), tmp_path, 9)
    assert e.value.status_code == 404
