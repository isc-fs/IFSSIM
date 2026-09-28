"""bench-run: run a spec's benchmarks on this machine, one job at a time.

    bench-run sim-bag                              # a preset from the manifest's presets folder
    bench-run my.yaml --set pipeline.slam_node.motion_model=imu
    bench-run sim-bag extra.yaml --dry-run         # merged spec + commands, nothing runs
    bench-run --list                               # benchmarks, settings, bags, presets

It runs the checkout it is started in, as it is: the benchmarks record the
code themselves (provenance.json), dirty or not. A spec's ``code:`` is for
the central machine, which checks those commits out first; here it is only
reported. Nothing is uploaded: ``bench-track sync`` does that when you choose.

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


def _queue(m: mf.Manifest, spec: dict[str, Any], a: argparse.Namespace) -> int:
    from . import checkout as co
    from . import queue as qu
    from . import submit as su

    try:
        p = su.plan(m, spec)
    except co.CodeError as e:
        print(f"Cannot pin the code: {e}", file=sys.stderr)
        return 2
    for name, r in p.code["resolved"].items():
        print(f"code {name}: {r['ref']} = {r['sha'][:12]}")
    q = qu.Queue()
    batch, ids = su.submit(q, p, trigger="cli", requested_by=_me())
    print(f"queued {len(ids)} job(s) as batch {batch} in {q.url}: {ids}")
    return 0


def _me() -> str:
    import getpass
    import socket

    return f"{getpass.getuser()}@{socket.gethostname()}"


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
        prog="bench-run",
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
        "--dry-run", action="store_true", help="print the jobs, run nothing"
    )
    ap.add_argument("--list", action="store_true", help="what the manifest offers")
    ap.add_argument("--stop-on-error", action="store_true")
    ap.add_argument(
        "--queue",
        action="store_true",
        help="queue the jobs for bench-worker instead of running them here "
        "(its queue: $BENCH_QUEUE_URL; the code is pinned to commits now)",
    )
    a = ap.parse_args(argv)

    try:
        m = mf.load(Path(a.repo) if a.repo else mf.find_root())
    except mf.ManifestError as e:
        print(e, file=sys.stderr)
        return 2
    if a.list:
        print_list(m)
        return 0
    if not a.spec and not a.set:
        ap.error("give a spec file or preset (bench-run --list shows them)")

    try:
        spec = build_spec(m, a.spec, a.set)
        jobs = sp.expand(spec, m)
    except sp.SpecError as e:
        print(f"The spec has problems:\n{e}", file=sys.stderr)
        return 2

    batch = f"{datetime.now(timezone.utc):%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}"
    work = m.results / WORK / batch
    print("--- spec (merged) ---")
    print(yaml.safe_dump(spec, sort_keys=False).rstrip())
    print(f"--- {len(jobs)} job(s) ---")
    if spec.get("code"):
        print(
            f"note: code {spec['code']} is for the central machine; "
            f"this runs the checkout in {m.root} as it is"
        )
    records = []
    for i, job in enumerate(jobs, 1):
        rec = job.record(spec) | {
            "job_id": f"local:{batch}/{i}",
            "batch_id": batch,
            "trigger": "local",
            "requested_by": _me(),
        }
        records.append(rec)
        cmd = command(m, rec, work / str(i))
        print(f"[{i}] {job.label()}  (spec {job.spec_id})\n    {shlex.join(cmd)}")
    if a.dry_run:
        return 0
    if a.queue:
        return _queue(m, spec, a)

    results = []
    for i, (job, rec) in enumerate(zip(jobs, records, strict=True), 1):
        print(f"\n=== [{i}/{len(jobs)}] {job.label()}", flush=True)
        t0 = time.time()
        rc = execute(m, rec, work / str(i))
        results.append((job, rc, time.time() - t0))
        if rc != 0:
            print(f"!!! job {i} failed (exit {rc})", file=sys.stderr)
            if a.stop_on_error:
                break

    print("\nDone:")
    for job, rc, dt in results:
        print(f"  {'ok    ' if rc == 0 else 'FAILED'} {dt:8.1f} s  {job.label()}")
    for job in jobs[len(results) :]:
        print(f"  not run            {job.label()}")
    print(
        "\nResults are on disk only. To upload them:\n"
        f"  bench-track sync --results-root {m.results}"
    )
    return (
        0 if all(rc == 0 for _, rc, _ in results) and len(results) == len(jobs) else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
