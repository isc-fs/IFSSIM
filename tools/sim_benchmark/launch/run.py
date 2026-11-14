"""run_spec: run a spec's benchmarks on this machine, one job at a time.

    python3 tools/sim_benchmark/run_spec.py sim-bag          # a preset from tools/sim_benchmark/specs
    python3 tools/sim_benchmark/run_spec.py my.yaml --set pipeline.slam_node.motion_model=imu
    python3 tools/sim_benchmark/run_spec.py sim-bag extra.yaml --dry-run   # merged spec + commands
    python3 tools/sim_benchmark/run_spec.py --list           # benchmarks, settings, bags, presets

The same command runs a spec on a laptop, on the bench PC's runner and from
Mission Control. It runs the checkout it is started in, as it is: the
benchmarks record the code themselves (provenance.json), dirty or not. A
spec's ``code:`` is checked out by whoever launches it (the bench PC's
workflow); here it is only reported. Nothing is uploaded.

Each job gets ``<results>/.bench/<batch>/<n>/``: ``spec.json`` (the job and
the spec it came from; the benchmark copies it into its run folder through
``$BENCH_SPEC``) and ``overrides.json`` (the ``pipeline:`` section, handed to
the benchmark's overrides flag).
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import yaml

from . import manifest as mf
from . import spec as sp

ENV_SPEC = "BENCH_SPEC"
ENV_TRIGGER = "BENCH_TRIGGER"
ENV_REQUESTED_BY = "BENCH_REQUESTED_BY"
WORK = ".bench"


def build_spec(m: mf.Manifest, sources: list[str], sets: list[str]) -> dict[str, Any]:
    docs = []
    for src in sources:
        path = Path(src)
        if not path.is_file():
            path = m.preset(src) or path
        if not path.is_file():
            known = ", ".join(m.preset_names()) or "none"
            raise sp.SpecError(f"{src}: no such file or preset (presets: {known})")
        docs.append(sp.load_file(path))
    docs += [sp.assignment(s) for s in sets]
    return sp.validate(sp.merge(*docs), m)


def command(m: mf.Manifest, rec: dict[str, Any], workdir: Path) -> list[str]:
    """The benchmark's command line for one job (``rec`` = ``Job.record``)."""
    bench = m.benchmarks[rec["benchmark"]]
    bag = rec.get("bag")
    subst = {
        "bag": str(m.bag_path(bench.bags, bag)) if bench.bags and bag else "",
        "results": str(m.results),
        "root": str(m.root),
    }
    cmd = [c.format(**subst) for c in bench.command]
    for k, v in (rec.get("settings") or {}).items():
        cmd += bench.settings[k].args(v)
    if rec.get("only") and bench.parts_flag:
        cmd += [bench.parts_flag, *rec["only"]]
    if rec.get("pipeline"):
        if not bench.overrides_flag:
            raise sp.SpecError(f"{rec['benchmark']} takes no pipeline overrides")
        cmd += [bench.overrides_flag, str(workdir / "overrides.json")]
    return cmd


def timeout_of(m: mf.Manifest, rec: dict[str, Any]) -> float:
    per = ((rec.get("spec") or {}).get("benchmarks") or {}).get(rec["benchmark"]) or {}
    return float(per.get("timeout_s") or m.benchmarks[rec["benchmark"]].timeout_s)


def execute(
    m: mf.Manifest,
    rec: dict[str, Any],
    workdir: Path,
    *,
    log: Path | None = None,
    should_stop: Callable[[], bool] | None = None,
    env: dict[str, str] | None = None,
) -> int:
    """Run one job in ``m.root``: write its spec.json (and overrides.json), start the
    benchmark, and wait. Exit 124 = timed out, 130 = stopped (``should_stop``)."""
    workdir.mkdir(parents=True, exist_ok=True)
    (workdir / "spec.json").write_text(json.dumps(rec, indent=2, default=str))
    if rec.get("pipeline"):
        (workdir / "overrides.json").write_text(json.dumps(rec["pipeline"], indent=2))
    cmd = command(m, rec, workdir)
    full_env = {**os.environ, **(env or {}), ENV_SPEC: str(workdir / "spec.json")}
    out = log.open("a") if log else None
    try:
        if out:
            out.write(f"$ {shlex.join(cmd)}\n")
            out.flush()
        proc = subprocess.Popen(
            cmd,
            cwd=m.root,
            env=full_env,
            start_new_session=True,
            stdout=out,
            stderr=subprocess.STDOUT if out else None,
        )
        return _wait(proc, timeout_of(m, rec), should_stop)
    finally:
        if out:
            out.close()


def _stop(proc: subprocess.Popen) -> None:
    os.killpg(proc.pid, signal.SIGTERM)
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def _wait(
    proc: subprocess.Popen, timeout_s: float, should_stop: Callable[[], bool] | None
) -> int:
    deadline = time.monotonic() + timeout_s
    try:
        while True:
            try:
                return proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
            if time.monotonic() > deadline:
                print(
                    f"!!! timed out after {timeout_s:.0f} s; stopping it",
                    file=sys.stderr,
                )
                _stop(proc)
                return 124
            if should_stop is not None and should_stop():
                _stop(proc)
                return 130
    except KeyboardInterrupt:
        _stop(proc)
        raise


def _me() -> str:
    import getpass
    import socket

    return f"{getpass.getuser()}@{socket.gethostname()}"


def where_and_who() -> dict[str, Any]:
    """Who asked for the run and where it runs, for each job's spec.json.

    ``requested_by`` is the person: what the launcher sets in ``$BENCH_REQUESTED_BY``
    (the GitHub user who started the workflow, or who is signed in to Mission Control),
    else ``user@host`` here. ``host`` is always this machine. Inside GitHub Actions,
    ``github`` says which run and runner, with a link to its logs."""
    import socket

    out: dict[str, Any] = {
        "requested_by": os.environ.get(ENV_REQUESTED_BY) or _me(),
        "host": socket.gethostname(),
    }
    env = os.environ
    if env.get("GITHUB_ACTIONS") == "true":
        server = env.get("GITHUB_SERVER_URL", "https://github.com")
        repo, run_id = env.get("GITHUB_REPOSITORY"), env.get("GITHUB_RUN_ID")
        out["github"] = {
            "run_id": run_id,
            "run_attempt": env.get("GITHUB_RUN_ATTEMPT"),
            "workflow": env.get("GITHUB_WORKFLOW"),
            "actor": env.get("GITHUB_ACTOR"),
            "runner": env.get("RUNNER_NAME"),
            "url": f"{server}/{repo}/actions/runs/{run_id}" if repo and run_id else None,
        }
    return out


ENV_PROGRESS = "BENCH_PROGRESS_FILE"
# where the benchmarks take the pipeline from (run_provenance.ENV_PIPELINE): a worktree of
# the spec's code.pipeline, or this checkout's pipeline/ when it's unset
ENV_PIPELINE = "BENCH_PIPELINE_DIR"


def _pipeline_refspec(ref: str) -> tuple[str | None, str]:
    """(what to fetch, what was asked) for a code.pipeline value: a PR ("#88", "pr/88"),
    a commit (7-40 hex: fetched only if it isn't here), or a branch."""
    import re

    r = ref.strip()
    pr = re.fullmatch(r"(?:#|pr/|pull/)(\d+)", r)
    if pr:
        return f"pull/{pr.group(1)}/head", f"PR #{pr.group(1)}"
    if re.fullmatch(r"[0-9a-f]{7,40}", r):
        return None, f"commit {r}"
    return r, f"branch {r}"


def pipeline_worktree(m: mf.Manifest, ref: str, where: Path) -> str:
    """Check the spec's code.pipeline out at ``where`` (a git worktree of the pipeline
    submodule, so provenance reads it like any checkout); returns its commit. A branch,
    a PR (#88) or a commit; ``branch@commit`` pins a commit and checks it's on the branch.
    The person's own pipeline/ checkout (its branch, its changes) is never touched."""
    import re

    repo = m.repos.get("pipeline")
    if repo is None:
        raise sp.SpecError("code.pipeline: the manifest has no pipeline repository")

    def git(*args: str) -> subprocess.CompletedProcess:
        return subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True)

    def first(r: subprocess.CompletedProcess) -> str:
        lines = (r.stderr or r.stdout or "").strip().splitlines()
        return lines[-1] if lines else f"exit {r.returncode}"

    # "branch@commit" / "#88@commit": that commit, which must be on the branch or PR
    base, _, pinned = ref.strip().rpartition("@")
    if base and re.fullmatch(r"[0-9a-f]{7,40}", pinned):
        refspec, what = _pipeline_refspec(base)
        if refspec is None:
            raise sp.SpecError(f"code.pipeline: {ref}: before the @ goes a branch or #PR")
        r = git("fetch", "--quiet", "origin", refspec)
        if r.returncode != 0:
            raise sp.SpecError(f"code.pipeline: couldn't fetch {what}: {first(r)}")
        tip = git("rev-parse", "FETCH_HEAD").stdout.strip()
        r = git("rev-parse", "--verify", "--quiet", f"{pinned}^{{commit}}")
        if r.returncode != 0 or git("merge-base", "--is-ancestor", r.stdout.strip(), tip).returncode != 0:
            raise sp.SpecError(f"code.pipeline: commit {pinned} isn't on {what}")
        sha = r.stdout.strip()
        where.parent.mkdir(parents=True, exist_ok=True)
        r = git("worktree", "add", "--detach", "--force", str(where), sha)
        if r.returncode != 0:
            raise sp.SpecError(f"code.pipeline: couldn't check out {pinned}: {first(r)}")
        return sha

    refspec, what = _pipeline_refspec(ref)
    if refspec is None:
        r = git("rev-parse", "--verify", "--quiet", f"{ref.strip()}^{{commit}}")
        if r.returncode != 0:
            git("fetch", "--quiet", "origin")
            r = git("rev-parse", "--verify", "--quiet", f"{ref.strip()}^{{commit}}")
        if r.returncode != 0:
            raise sp.SpecError(f"code.pipeline: no {what} in the pipeline repository")
    else:
        r = git("fetch", "--quiet", "origin", refspec)
        if r.returncode != 0:
            raise sp.SpecError(f"code.pipeline: couldn't fetch {what}: {first(r)}")
        r = git("rev-parse", "FETCH_HEAD")
    sha = r.stdout.strip()
    where.parent.mkdir(parents=True, exist_ok=True)
    r = git("worktree", "add", "--detach", "--force", str(where), sha)
    if r.returncode != 0:
        raise sp.SpecError(f"code.pipeline: couldn't check out {what}: {first(r)}")
    return sha


def remove_pipeline_worktree(m: mf.Manifest, where: Path) -> None:
    repo = m.repos.get("pipeline")
    if repo is None:
        return
    subprocess.run(["git", "-C", str(repo), "worktree", "remove", "--force", str(where)], capture_output=True)
    subprocess.run(["git", "-C", str(repo), "worktree", "prune"], capture_output=True)


def run_queued(
    m: mf.Manifest, j: dict[str, Any], on_tick: Callable[[], None] = lambda: None
) -> str:
    """Run one queued spec (``localq``) here, job by job; returns its final state."""
    from . import localq as lq

    jid = j["id"]
    d = lq.queue_dir(m.results) / jid
    try:
        spec = sp.validate(j["spec"], m)
        jobs = sp.expand(spec, m)
    except sp.SpecError as e:
        lq.set_state(m.results, jid, state="failed", ended=lq.now(), error=str(e))
        return "failed"
    env: dict[str, str] = {}
    ref = (spec.get("code") or {}).get("pipeline")
    tree = d / "pipeline" if ref else None
    if tree is not None:
        lq.set_state(m.results, jid, current=f"getting pipeline {ref}")
        try:
            sha = pipeline_worktree(m, str(ref), tree)
        except sp.SpecError as e:
            lq.set_state(m.results, jid, state="failed", ended=lq.now(), error=str(e), current=None)
            return "failed"
        env[ENV_PIPELINE] = str(tree)
        lq.set_state(m.results, jid, pipeline={"ref": ref, "sha": sha})
    try:
        return _run_jobs(m, j, spec, jobs, d, env, on_tick)
    finally:
        if tree is not None:
            remove_pipeline_worktree(m, tree)


def _run_jobs(
    m: mf.Manifest, j: dict[str, Any], spec: dict[str, Any], jobs: list[Any], d: Path,
    env: dict[str, str], on_tick: Callable[[], None],
) -> str:
    from . import localq as lq

    jid = j["id"]
    lq.set_state(
        m.results, jid, state="running", started=lq.now(), total=len(jobs), done=0,
        waiting=None,
    )
    origin = where_and_who() | {"requested_by": j.get("requested_by") or _me()}
    trigger = j.get("trigger") or "local"
    failed = 0
    for i, job in enumerate(jobs, 1):
        rec = job.record(spec) | {
            "job_id": f"{trigger}:{jid}/{i}",
            "batch_id": jid,
            "trigger": trigger,
            **origin,
        }
        progress = d / f"progress_{i}.json"
        lq.set_state(m.results, jid, current=job.label(), index=i, progress=None)

        def stop(progress: Path = progress) -> bool:
            on_tick()
            p = lq.read_json(progress)
            if p is not None:
                lq.set_state(m.results, jid, progress=p)
            return lq.cancel_requested(m.results, jid)

        rc = execute(
            m, rec, d / str(i), log=d / "worker.log", should_stop=stop,
            env={**env, ENV_PROGRESS: str(progress)},
        )
        last = lq.read_json(progress)  # a job may end before its first check
        if rc == 130 and lq.cancel_requested(m.results, jid):
            lq.set_state(m.results, jid, state="cancelled", ended=lq.now(), exit=130)
            return "cancelled"
        failed += rc != 0
        lq.set_state(
            m.results, jid, done=i, failed_jobs=failed,
            **({"progress": last} if last is not None else {}),
        )
    final = "done" if failed == 0 else "failed"
    lq.set_state(m.results, jid, state=final, ended=lq.now(), current=None)
    return final


def work(
    m: mf.Manifest,
    *,
    once: bool = False,
    poll_s: float = 2.0,
    rig_every_s: float = 60.0,
    check_rig: Callable[[], dict[str, Any]] | None = None,
) -> int:
    """``--work``: run the local queue's specs one at a time, oldest first. One worker
    per results folder (a lock file); exit 3 if another holds it. ``once``: stop when
    nothing is left to run."""
    import fcntl

    from . import localq as lq
    from . import rig as rg

    check_rig = check_rig or rg.check
    bd = lq.bench_dir(m.results)
    bd.mkdir(parents=True, exist_ok=True)
    lock = (bd / "worker.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"another worker is running for {m.results}", file=sys.stderr)
        return 3
    status: dict[str, Any] = {}
    last_rig = -1e9

    def tick(current: str | None = None) -> None:
        nonlocal status, last_rig
        if time.monotonic() - last_rig > rig_every_s:
            status = rig_status(m, check_rig)
            lq.write_json(bd / "rig.json", status)
            last_rig = time.monotonic()
        lq.heartbeat(m.results, job=current)

    # A stop asked for before this worker started was for an earlier one.
    lq.clear_stop(m.results)
    print(f"worker for {m.results}: waiting for queued specs", flush=True)
    while True:
        tick()
        if lq.stop_requested(m.results):
            # between specs: a running one has finished (Mission Control's Stop worker)
            lq.clear_stop(m.results)
            (bd / "worker.json").unlink(missing_ok=True)  # not running: at once, not in 30 s
            print("asked to stop: stopping", flush=True)
            return 0
        picked = None
        for j in reversed([j for j in lq.jobs(m.results) if j["state"] == "queued"]):
            probs = rg.problems(j["spec"], status)
            if any(p["status"] == rg.ABSENT for p in probs):
                lq.set_state(
                    m.results, j["id"], state="failed", ended=lq.now(),
                    error=f"this rig can't run it: {rg.describe(probs)}",
                )
                continue
            if probs:  # not now: it waits, and others may go first
                if j["status"].get("waiting") != rg.describe(probs):
                    lq.set_state(m.results, j["id"], waiting=rg.describe(probs))
                continue
            picked = j
            break
        if picked is None:
            if once:
                return 0
            time.sleep(poll_s)
            continue
        print(f"=== {picked['id']} {picked.get('name') or ''}", flush=True)
        state = run_queued(m, picked, on_tick=lambda: tick(picked["id"]))
        print(f"    {state}", flush=True)


def _run_direct(
    m: mf.Manifest, jobs: list[Any], records: list[dict[str, Any]], batch_dir: Path,
    env: dict[str, str], stop_on_error: bool,
) -> list[tuple[Any, int, float]]:
    """Run the jobs one by one, here (run_spec.py <spec>): (job, exit, seconds) each."""
    results = []
    for i, (job, rec) in enumerate(zip(jobs, records, strict=True), 1):
        print(f"\n=== [{i}/{len(jobs)}] {job.label()}", flush=True)
        t0 = time.time()
        rc = execute(m, rec, batch_dir / str(i), env=env)
        results.append((job, rc, time.time() - t0))
        if rc != 0:
            print(f"!!! job {i} failed (exit {rc})", file=sys.stderr)
            if stop_on_error:
                break
    return results


def code_versions(m: mf.Manifest) -> dict[str, dict[str, Any]]:
    """What code a local run uses: each repository in the manifest as it is checked out
    (a local run runs the checkout as it is). Mission Control shows it on its boxes."""
    out: dict[str, dict[str, Any]] = {}
    for name, path in m.repos.items():
        def git(*args: str) -> str | None:
            try:
                r = subprocess.run(
                    ["git", "-C", str(path), *args], capture_output=True, text=True, timeout=10
                )
            except (OSError, subprocess.TimeoutExpired):
                return None
            return r.stdout.strip() if r.returncode == 0 else None

        commit = git("rev-parse", "--short", "HEAD")
        if not commit:
            continue
        branch = git("rev-parse", "--abbrev-ref", "HEAD")
        out[name] = {
            "commit": commit,
            "branch": None if branch in (None, "HEAD") else branch,  # HEAD: a pinned submodule
            "dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
        }
    return out


def rig_status(m: mf.Manifest, check_rig: Callable[[], dict[str, Any]]) -> dict[str, Any]:
    """The rig's checks and the code a run would use, as rig.json holds them."""
    return check_rig() | {"code": code_versions(m)}


def print_status(summary: dict[str, Any]) -> None:
    w = summary["worker"]
    print(
        f"worker: {'running' if w['alive'] else 'not running'}"
        + (f" on {w.get('host')} (pid {w.get('pid')})" if w.get("pid") else "")
    )
    for j in summary["jobs"]:
        st = j["status"]
        bits = [st.get("current") or "", st.get("waiting") and f"waiting: {st['waiting']}",
                st.get("error")]
        if st.get("total"):
            bits.insert(0, f"{st.get('done', 0)}/{st['total']} jobs")
        extra = "  ".join(b for b in bits if b)
        print(f"  {j['id']}  {j['state']:<9} {j.get('name') or '-':<30} {extra}")


def print_rig(status: dict[str, Any]) -> None:
    print(f"rig {status['id']} on {status['host']}")
    for name, p in status["parts"].items():
        print(f"  {name:<18} {p['status']:<12} {p.get('reason') or ''}")


def print_list(m: mf.Manifest) -> None:
    print(f"repository  {m.root}")
    print(f"results     {m.results}")
    for name, b in m.benchmarks.items():
        print(f"\n{name}: {b.title}")
        if b.description:
            print(f"  {b.description}")
        if b.parts:
            print(f"  parts (only:) {', '.join(b.parts)}")
        comps = sorted(b.components())
        if comps:
            print(f"  pipeline:     {', '.join(comps)}")
        for s in b.settings.values():
            extra = f" [{'|'.join(map(str, s.choices))}]" if s.choices else ""
            print(f"  settings.{s.name:<18} {s.type}{extra}  {s.help}")
        if b.bags:
            bags = m.bags(b.bags)
            print(f"  bags ({len(bags)} in {m.bag_dirs[b.bags]}):")
            for x in bags:
                print(f"    {x}")
    print(f"\npresets: {', '.join(m.preset_names()) or 'none'}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="run_spec.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "spec", nargs="*", help="spec files or preset names, merged in order"
    )
    ap.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="one value, e.g. benchmarks.sim_bag.repeats=3 (wins over the files)",
    )
    ap.add_argument(
        "--repo", help="repository with bench.yaml (default: this checkout)"
    )
    ap.add_argument(
        "--results",
        metavar="DIR",
        help="write the runs here instead of bench.yaml's results folder (the runner: "
        "one folder per workflow run)",
    )
    ap.add_argument(
        "--print-spec",
        action="store_true",
        help="print the merged, checked spec as JSON and run nothing",
    )
    ap.add_argument(
        "--dry-run", action="store_true", help="print the jobs, run nothing"
    )
    ap.add_argument("--list", action="store_true", help="what the manifest offers")
    ap.add_argument("--stop-on-error", action="store_true")
    local = ap.add_argument_group(
        "the local queue",
        "Mission Control's Local target queues specs; one worker runs them, one at a time",
    )
    local.add_argument(
        "--queue", action="store_true", help="queue the spec instead of running it now"
    )
    local.add_argument(
        "--work", action="store_true", help="run queued specs, one at a time, until stopped"
    )
    local.add_argument(
        "--once", action="store_true", help="with --work: stop when nothing is left to run"
    )
    local.add_argument("--status", action="store_true", help="the worker and the queue")
    local.add_argument("--cancel", metavar="ID", help="cancel a queued or running spec")
    local.add_argument(
        "--rig", action="store_true", help="what this machine can run now (launch/rig.py)"
    )
    local.add_argument("--json", action="store_true", help="--status / --rig as JSON")
    a = ap.parse_args(argv)
    # Who started it, recorded in each run's spec.json: "local" by hand, or what the
    # launcher sets (the bench PC's workflow: "runner"; Mission Control: "mission_control").
    trigger = os.environ.get(ENV_TRIGGER, "local")

    try:
        m = mf.load(Path(a.repo) if a.repo else mf.find_root())
    except mf.ManifestError as e:
        print(e, file=sys.stderr)
        return 2
    if a.results:
        m = m.with_paths(results=Path(a.results).resolve())
    if a.list:
        print_list(m)
        return 0
    from . import localq as lq
    from . import rig as rg

    if a.rig:
        # saved for Mission Control too: its page runs this to check the machine
        # when no worker is running
        status = rig_status(m, rg.check)
        lq.write_json(lq.bench_dir(m.results) / "rig.json", status)
        print(json.dumps(status, indent=2)) if a.json else print_rig(status)
        return 0
    if a.status:
        summary = lq.summary(m.results)
        print(json.dumps(summary, indent=2, default=str)) if a.json else print_status(summary)
        return 0
    if a.cancel:
        state = lq.cancel(m.results, a.cancel)
        if state is None:
            print(f"no queued spec {a.cancel}", file=sys.stderr)
            return 1
        print(f"{a.cancel}: {state}" + (" (cancel requested)" if state == "running" else ""))
        return 0
    if a.work:
        return work(m, once=a.once)
    if not a.spec and not a.set:
        ap.error("give a spec file or preset (--list shows them)")

    try:
        spec = build_spec(m, a.spec, a.set)
        jobs = sp.expand(spec, m)
    except sp.SpecError as e:
        print(f"The spec has problems:\n{e}", file=sys.stderr)
        return 2
    if a.print_spec:
        print(json.dumps(spec, indent=2, default=str))
        return 0

    if a.queue:
        probs = rg.problems(spec, rg.check())
        if probs:
            print(f"note: not runnable here right now: {rg.describe(probs)}", file=sys.stderr)
        jid = lq.enqueue(
            m.results, spec, requested_by=where_and_who()["requested_by"], trigger=trigger
        )
        print(f"queued {jid} ({len(jobs)} job(s)); run_spec.py --work runs it")
        return 0

    batch = f"{datetime.now(timezone.utc):%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}"
    batch_dir = m.results / WORK / batch
    print("--- spec (merged) ---")
    print(yaml.safe_dump(spec, sort_keys=False).rstrip())
    print(f"--- {len(jobs)} job(s) ---")
    others = {k: v for k, v in (spec.get("code") or {}).items() if k != "pipeline"}
    if others:
        print(
            f"note: code {others} is checked out by whoever launches the spec; "
            f"this runs the checkout in {m.root} as it is"
        )
    if (spec.get("code") or {}).get("pipeline"):
        print(f"pipeline: {spec['code']['pipeline']}, checked out for this run (not this checkout's pipeline/)")
    records = []
    origin = where_and_who()
    for i, job in enumerate(jobs, 1):
        rec = job.record(spec) | {
            "job_id": f"{trigger}:{batch}/{i}",
            "batch_id": batch,
            "trigger": trigger,
            **origin,
        }
        records.append(rec)
        cmd = command(m, rec, batch_dir / str(i))
        print(f"[{i}] {job.label()}  (spec {job.spec_id})\n    {shlex.join(cmd)}")
    if a.dry_run:
        return 0

    env: dict[str, str] = {}
    tree = None
    if (spec.get("code") or {}).get("pipeline"):
        tree = batch_dir / "pipeline"
        try:
            sha = pipeline_worktree(m, str(spec["code"]["pipeline"]), tree)
        except sp.SpecError as e:
            print(e, file=sys.stderr)
            return 2
        print(f"pipeline at {sha[:7]} in {tree}")
        env[ENV_PIPELINE] = str(tree)
    try:
        results = _run_direct(m, jobs, records, batch_dir, env, a.stop_on_error)
    finally:
        if tree is not None:
            remove_pipeline_worktree(m, tree)

    print("\nDone:")
    for job, rc, dt in results:
        print(f"  {'ok    ' if rc == 0 else 'FAILED'} {dt:8.1f} s  {job.label()}")
    for job in jobs[len(results) :]:
        print(f"  not run            {job.label()}")
    print(f"\nResults: {m.results}")
    return (
        0 if all(rc == 0 for _, rc, _ in results) and len(results) == len(jobs) else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
