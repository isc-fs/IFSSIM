"""The job queue, the worker and the code checkout, with a fake benchmark (nothing real runs)."""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
import yaml

from bench_tracking.launch import checkout as co
from bench_tracking.launch import manifest as mf
from bench_tracking.launch import queue as qu
from bench_tracking.launch import submit as su
from bench_tracking.launch import worker as wk

FAKE = """
import json, os, sys, time
from pathlib import Path
time.sleep(float(os.environ.get("FAKE_SLEEP", "0")))
out = Path(sys.argv[sys.argv.index("--results-root") + 1]) / "ran.jsonl"
out.parent.mkdir(parents=True, exist_ok=True)
with out.open("a") as f:
    f.write(json.dumps({"argv": sys.argv[1:], "spec": json.load(open(os.environ["BENCH_SPEC"])),
                        "image": os.environ.get("BENCH_IMAGE")}) + "\\n")
print("benchmark output")
sys.exit(int(os.environ.get("FAKE_RC", "0")))
"""

MANIFEST = {
    "schema": 1,
    "results": "results",
    "repos": {"ifssim": {"path": ".", "default": "dev"}},
    "bags": {"simulator": "bags"},
    "benchmarks": {
        "sim_bag": {
            "command": [
                sys.executable,
                "fake.py",
                "{bag}",
                "--results-root",
                "{results}",
            ],
            "bags": "simulator",
            "parts": {"perception": "cone_detection"},
            "overrides_flag": "--pipeline-overrides",
            "settings": {"max_frames": {"type": "integer", "flag": "--max-frames"}},
        }
    },
}


def make_repo(d: Path) -> Path:
    d.mkdir(parents=True, exist_ok=True)
    (d / "fake.py").write_text(FAKE)
    (d / "bench.yaml").write_text(yaml.safe_dump(MANIFEST))
    (d / "bags" / "bag_a").mkdir(parents=True)
    (d / "bags" / "bag_a" / "metadata.yaml").write_text("x: 1\n")
    return d


@pytest.fixture
def q(tmp_path: Path) -> qu.Queue:
    return qu.Queue(f"sqlite:///{tmp_path / 'q.db'}")


def queue_spec(q: qu.Queue, repo: Path, spec: dict, trigger="web") -> list[int]:
    m = mf.load(repo)
    p = su.plan(m, spec, pin_code=False)
    return su.submit(q, p, trigger=trigger, requested_by="alice@tailnet")[1]


SPEC = {"benchmarks": {"sim_bag": {"bags": ["bag_a"]}}}


# ------------------------------------------------------------------ queue
def test_claim_takes_the_most_urgent_then_the_oldest(
    q: qu.Queue, tmp_path: Path
) -> None:
    repo = make_repo(tmp_path / "repo")
    commit = queue_spec(q, repo, SPEC, trigger="commit")
    web = queue_spec(
        q, repo, {**SPEC, "benchmarks": {"sim_bag": {"bags": ["bag_a"], "repeats": 2}}}
    )
    assert [j.id for j in q.queued_in_order()] == web + commit
    assert q.claim("w").id == web[0]
    assert q.get(web[0]).state == qu.RUNNING


def test_cancel(q: qu.Queue, tmp_path: Path) -> None:
    repo = make_repo(tmp_path / "repo")
    a, b = queue_spec(
        q, repo, {"benchmarks": {"sim_bag": {"bags": ["bag_a"], "repeats": 2}}}
    )
    assert q.cancel(b) == qu.CANCELLED  # queued: at once
    assert q.claim("w").id == a
    assert q.cancel(a) == qu.RUNNING and q.cancel_requested(
        a
    )  # running: the worker stops it
    assert q.claim("w") is None


def test_a_dead_workers_jobs_fail(q: qu.Queue, tmp_path: Path) -> None:
    repo = make_repo(tmp_path / "repo")
    (i,) = queue_spec(q, repo, SPEC)
    q.claim("w1")
    assert q.recover("w2") == [] and q.recover("w1") == [i]
    assert q.get(i).state == qu.FAILED


# ------------------------------------------------------------------ worker
def worker(q, repo, uploads=None, **kw) -> wk.Worker:
    calls = uploads if uploads is not None else []

    def upload(results: Path, job_id: int) -> list[str]:
        calls.append(job_id)
        return [f"run{job_id}"]

    return wk.Worker(q, repo, name="w", upload=kw.pop("upload", upload), **kw)


def test_worker_runs_the_job_and_uploads_its_runs(q: qu.Queue, tmp_path: Path) -> None:
    repo = make_repo(tmp_path / "repo")
    (i,) = queue_spec(
        q, repo, {**SPEC, "pipeline": {"cone_detection": {"residual_gate_mse": 0.05}}}
    )
    uploads: list[int] = []
    results = tmp_path / "shared-results"
    worker(q, repo, uploads, results=results).serve(once=True)
    j = q.get(i)
    assert j.state == qu.DONE and j.exit_code == 0 and j.runs == [f"run{i}"]
    assert uploads == [i]
    (line,) = (results / "ran.jsonl").read_text().splitlines()
    ran = json.loads(line)
    # the job's identity reaches the run's spec.json, so its runs can be found again
    assert (
        ran["spec"]["job_id"] == str(i)
        and ran["spec"]["requested_by"] == "alice@tailnet"
    )
    ov = Path(ran["argv"][ran["argv"].index("--pipeline-overrides") + 1])
    assert json.loads(ov.read_text()) == {"cone_detection": {"residual_gate_mse": 0.05}}
    assert "benchmark output" in Path(j.log_path).read_text()


def test_bags_can_live_elsewhere(q: qu.Queue, tmp_path: Path) -> None:
    repo = make_repo(tmp_path / "repo")
    shared = tmp_path / "srv-bags"
    (shared / "bag_a").mkdir(parents=True)
    (shared / "bag_a" / "metadata.yaml").write_text("x: 1\n")
    queue_spec(q, repo, SPEC)
    worker(q, repo, bags={"simulator": shared}, results=tmp_path / "r").serve(once=True)
    ran = json.loads((tmp_path / "r" / "ran.jsonl").read_text())
    assert ran["argv"][0] == str(shared.resolve() / "bag_a")


def test_failed_benchmark_fails_the_job(q, tmp_path, monkeypatch) -> None:
    repo = make_repo(tmp_path / "repo")
    (i,) = queue_spec(q, repo, SPEC)
    monkeypatch.setenv("FAKE_RC", "3")
    worker(q, repo).serve(once=True)
    j = q.get(i)
    assert j.state == qu.FAILED and j.exit_code == 3 and j.error == "exit code 3"


def test_cancelling_a_running_job_stops_it(q, tmp_path, monkeypatch) -> None:
    repo = make_repo(tmp_path / "repo")
    (i,) = queue_spec(q, repo, SPEC)
    monkeypatch.setenv("FAKE_SLEEP", "60")
    threading.Timer(1.0, lambda: q.cancel(i)).start()
    t0 = time.monotonic()
    worker(q, repo).serve(once=True)
    assert q.get(i).state == qu.CANCELLED
    assert time.monotonic() - t0 < 30


def test_upload_waits_for_mlflow(q, tmp_path) -> None:
    repo = make_repo(tmp_path / "repo")
    (i,) = queue_spec(q, repo, SPEC)

    def down(results, job_id):
        raise ConnectionError("MLflow at http://x cannot be reached")

    worker(q, repo, upload=down).serve(once=True)
    assert q.get(i).state == qu.UPLOAD_PENDING
    worker(q, repo).retry_uploads(force=True)
    assert q.get(i).state == qu.DONE and q.get(i).runs == [f"run{i}"]


# ------------------------------------------------------------------ code
def git(d: Path, *a: str) -> str:
    return subprocess.run(
        ["git", "-C", str(d), *a], check=True, capture_output=True, text=True
    ).stdout.strip()


@pytest.fixture
def origin(tmp_path: Path) -> tuple[Path, Path, list[str]]:
    """An 'origin' with two commits on dev, and the worker's clone of it."""
    src = make_repo(tmp_path / "src")
    git(src, "init", "-q", "-b", "dev")
    git(src, "config", "user.email", "t@t")
    git(src, "config", "user.name", "t")
    git(src, "add", "-A")
    git(src, "commit", "-qm", "one")
    shas = [git(src, "rev-parse", "HEAD")]
    (src / "fake.py").write_text(FAKE + "\n# two\n")
    git(src, "commit", "-qam", "two")
    shas.append(git(src, "rev-parse", "HEAD"))
    clone = tmp_path / "clone"
    subprocess.run(["git", "clone", "-q", str(src), str(clone)], check=True)
    return src, clone, shas


def test_refs_are_pinned_at_launch(origin) -> None:
    src, clone, shas = origin
    m = mf.load(clone)
    assert co.resolve(m, {"ifssim": "dev"})["ifssim"]["sha"] == shas[1]
    assert co.resolve(m, {"ifssim": shas[0][:10]})["ifssim"]["sha"] == shas[0][:10]
    with pytest.raises(co.CodeError, match="not a branch, tag or PR"):
        co.resolve(m, {"ifssim": "no-such-branch"})
    p = su.plan(m, SPEC)  # no code in the spec: the manifest's default branch
    assert p.code["resolved"]["ifssim"]["sha"] == shas[1]


def test_worker_runs_the_pinned_commit_in_its_own_checkout(q, origin, tmp_path) -> None:
    src, clone, shas = origin
    m = mf.load(clone)
    p = su.plan(m, {**SPEC, "code": {"ifssim": shas[0]}})
    (i,) = su.submit(q, p, trigger="web", requested_by="a")[1]
    results = tmp_path / "results"
    worker(q, clone, results=results).serve(once=True)
    j = q.get(i)
    assert j.state == qu.DONE, Path(j.log_path).read_text()
    assert "checkout" not in {
        p.name for p in (results / ".bench" / "jobs" / str(i)).iterdir()
    }
    assert git(clone, "worktree", "list").count("\n") == 0  # removed afterwards
    assert (results / "ran.jsonl").is_file()  # results outside the removed checkout


def test_image_choice(tmp_path: Path, origin) -> None:
    src, clone, shas = origin
    img = mf.Image(
        env="BENCH_IMAGE", registry="ghcr.io/x/stack", build=("make", "{tag}")
    )
    assert (
        co.choose_image(img, clone, "sha-abc1234")[0] == "ghcr.io/x/stack:sha-abc1234"
    )
    assert co.choose_image(img, clone, "other/img:1")[0] == "other/img:1"
    built = []
    ref, why = co.choose_image(
        mf.Image(env="E", build=("make", "{tag}"), local_tag="t:{short}"),
        clone,
        "build",
        run=lambda cmd, cwd: built.append(cmd) or 0,
    )
    assert built == [["make", ref]] and ref == f"t:{shas[1][:7]}" and "built" in why
