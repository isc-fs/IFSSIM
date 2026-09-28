"""bench-worker: runs queued jobs, one at a time, and uploads what they produce.

    bench-worker --repo /srv/bench/ifssim --results-root /srv/bench/results \\
                 --bags simulator=/srv/bench/bags            # the central machine (systemd)
    bench-worker                                             # a laptop: this checkout, as it is

For each job (queue.py):

1. **Code.** A job with commits (``code``) runs in a fresh worktree of ``--repo``
   at those commits. A job without them runs ``--repo`` as it is, which is what a
   laptop wants when the central machine is down.
2. **Image.** The manifest's ``image:`` section and ``code.image`` (checkout.py).
3. **Run.** The benchmark, through ``bench-run``'s ``execute``: output to the
   job's log, stopped when the job is cancelled or times out.
4. **Upload** the runs it made (their ``spec.json`` carries the job id) to
   MLflow. If MLflow is down the job waits as ``upload_pending`` and is retried.

Results and bags stay in ``--results-root`` / ``--bags``, outside the
checkouts, so removing a worktree never removes results.
"""

from __future__ import annotations

import argparse
import getpass
import socket
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from .. import config
from . import checkout as co
from . import manifest as mf
from . import queue as qu
from . import run as rn

POLL_S = 5.0
RETRY_UPLOAD_S = 300.0


def default_name() -> str:
    return f"{getpass.getuser()}@{socket.gethostname()}"


def job_dir(results: Path, job_id: int) -> Path:
    return results / rn.WORK / "jobs" / str(job_id)


def upload_job(results: Path, job_id: int) -> list[str]:
    """Upload the runs job ``job_id`` made; returns their MLflow run ids.

    Raises ``ConnectionError`` when MLflow can't be reached (the job waits)."""
    import warnings

    from ..cli import BACKEND, Uploader, _unreachable, reachable
    from ..backends.mlflow_backend import MlflowBackend
    from ..suite import load_suite

    uri = MlflowBackend.default_uri()
    if not reachable(uri):
        raise ConnectionError(f"MLflow at {uri} cannot be reached")
    warnings.simplefilter("ignore", RuntimeWarning)
    suite = load_suite(
        replay_root=results, sim_root=None, sim_bag_root=results, full_bag_hash=True
    )
    mine = [
        b
        for b in suite.replays + suite.sim_bags
        if str((b.config.get("launch") or {}).get("job_id")) == str(job_id)
    ]
    u = Uploader(BACKEND)
    ids = []
    try:
        for b in sorted(mine, key=lambda b: b.started_at):
            res = u.up(b)
            if res is not None:
                ids.append(res.run_id)
    except Exception as e:  # noqa: BLE001
        if _unreachable(e):
            raise ConnectionError(str(e)) from e
        raise
    return ids


class Worker:
    def __init__(
        self,
        q: qu.Queue,
        repo: Path,
        *,
        results: Path | None = None,
        bags: dict[str, Path] | None = None,
        name: str | None = None,
        upload: Callable[[Path, int], list[str]] = upload_job,
    ) -> None:
        self.q = q
        self.repo = Path(repo).resolve()
        self.base = mf.load(self.repo)
        self.results = Path(results).resolve() if results else self.base.results
        self.bags = {k: Path(v).resolve() for k, v in (bags or {}).items()}
        self.name = name or default_name()
        self.upload = upload
        self._last_retry = 0.0

    # ---------------------------------------------------------------- loop
    def serve(self, once: bool = False, poll_s: float = POLL_S) -> None:
        stuck = self.q.recover(self.name)
        if stuck:
            print(
                f"marked failed (left running by a previous worker): {stuck}",
                flush=True,
            )
        print(
            f"bench-worker {self.name}: repo {self.repo}, results {self.results}",
            flush=True,
        )
        while True:
            self.retry_uploads()
            job = self.q.claim(self.name)
            if job is not None:
                self.run(job)
                continue
            if once:
                return
            time.sleep(poll_s)

    def retry_uploads(self, force: bool = False) -> None:
        if not force and time.monotonic() - self._last_retry < RETRY_UPLOAD_S:
            return
        self._last_retry = time.monotonic()
        for j in self.q.list(states=(qu.UPLOAD_PENDING,), limit=100):
            try:
                ids = self.upload(self.results, j.id)
            except ConnectionError:
                return  # still down; next time
            self.q.finish(j.id, qu.DONE if j.exit_code == 0 else qu.FAILED, runs=ids)

    # ---------------------------------------------------------------- one job
    def run(self, job: qu.JobRow) -> str:
        d = job_dir(self.results, job.id)
        d.mkdir(parents=True, exist_ok=True)
        logf = d / "worker.log"
        self.q.update(job.id, log_path=str(logf))

        def log(msg: str) -> None:
            stamp = datetime.now(timezone.utc).strftime("%H:%M:%S")
            with logf.open("a") as f:
                f.write(f"[{stamp}] {msg}\n")
            print(f"[job {job.id}] {msg}", flush=True)

        log(f"{job.label} (requested by {job.requested_by}, {job.trigger})")
        wt: Path | None = None
        code: dict[str, Any] = dict(job.code or {})
        try:
            resolved = code.get("resolved") or {}
            if resolved:
                wt = co.prepare(self.base, resolved, d / "checkout", log)
                root = wt
            else:
                log(f"no commits asked for: running {self.repo} as it is")
                root = self.repo
            m = mf.load(root).with_paths(
                self.results, {**self.base.bag_dirs, **self.bags}
            )
            rec = dict(job.job) | {
                "job_id": str(job.id),
                "batch_id": job.batch_id,
                "trigger": job.trigger,
                "requested_by": job.requested_by,
            }
            if rec["benchmark"] not in m.benchmarks:
                raise co.CodeError(
                    f"this code's bench.yaml has no benchmark {rec['benchmark']!r}"
                )
            env = {}
            if m.image is not None and (
                resolved or (code.get("requested") or {}).get("image")
            ):
                ref, why = co.choose_image(
                    m.image,
                    root,
                    (code.get("requested") or {}).get("image"),
                    log,
                    run=lambda cmd, cwd: _logged(cmd, cwd, logf),
                )
                env[m.image.env] = ref
                code["image"] = {"ref": ref, "why": why}
                self.q.update(job.id, code=code)
                log(f"image {ref}: {why}")
            if self.q.cancel_requested(job.id):
                return self._end(
                    job, qu.CANCELLED, None, "cancelled before it started", log
                )
            log("running")
            rc = rn.execute(
                m,
                rec,
                d / "work",
                log=logf,
                should_stop=lambda: self.q.cancel_requested(job.id),
                env=env,
            )
            log(f"exit {rc}")
            if rc == 130:
                return self._end(job, qu.CANCELLED, rc, "cancelled while running", log)
            err = None if rc == 0 else ("timed out" if rc == 124 else f"exit code {rc}")
            try:
                ids = self.upload(self.results, job.id)
            except ConnectionError as e:
                log(f"upload later: {e}")
                self.q.finish(job.id, qu.UPLOAD_PENDING, exit_code=rc, error=err)
                return qu.UPLOAD_PENDING
            log(f"uploaded {len(ids)} run(s)")
            return self._end(job, qu.DONE if rc == 0 else qu.FAILED, rc, err, log, ids)
        except Exception as e:  # noqa: BLE001  (one bad job must not stop the worker)
            log("".join(traceback.format_exception_only(type(e), e)).strip())
            with logf.open("a") as f:
                f.write(traceback.format_exc())
            return self._end(job, qu.FAILED, None, f"{type(e).__name__}: {e}", log)
        finally:
            if wt is not None:
                co.remove(self.repo, wt)

    def _end(self, job, state, rc, err, log, runs=None) -> str:
        self.q.finish(job.id, state, exit_code=rc, error=err, runs=runs)
        log(state)
        return state


def _logged(cmd: list[str], cwd: Path, logf: Path) -> int:
    import subprocess

    with logf.open("a") as f:
        f.write(f"$ {' '.join(cmd)}\n")
        f.flush()
        return subprocess.call(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="bench-worker",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--repo", help="the clone to run from (default: this checkout)")
    ap.add_argument("--results-root", help="where runs go (default: the manifest's)")
    ap.add_argument(
        "--bags",
        action="append",
        default=[],
        metavar="KIND=DIR",
        help="a bag folder instead of the manifest's, e.g. simulator=/srv/bench/bags",
    )
    ap.add_argument("--queue-url", help=f"default ${qu.ENV_URL} or {qu.DEFAULT_URL}")
    ap.add_argument("--name", help="this worker's name (default user@host)")
    ap.add_argument("--once", action="store_true", help="run what is queued, then stop")
    ap.add_argument(
        "--poll", type=float, default=POLL_S, help="seconds between queue checks"
    )
    a = ap.parse_args(argv)
    config.load()
    bags = {}
    for b in a.bags:
        if "=" not in b:
            ap.error(f"--bags {b}: expected KIND=DIR")
        k, v = b.split("=", 1)
        bags[k] = Path(v)
    try:
        w = Worker(
            qu.Queue(a.queue_url),
            Path(a.repo) if a.repo else mf.find_root(),
            results=Path(a.results_root) if a.results_root else None,
            bags=bags,
            name=a.name,
        )
    except mf.ManifestError as e:
        print(e, file=sys.stderr)
        return 2
    try:
        w.serve(once=a.once, poll_s=a.poll)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
