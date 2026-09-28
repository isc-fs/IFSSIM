"""The job queue: what the Launch page, PR checks and the CLI ask for, and what the worker runs.

One table, ``bench_jobs``. Anything that wants benchmarks run inserts jobs
(``submit``). Only the worker runs them: it takes the next one (``claim``),
one at a time, and records how it went. The database is Postgres on the
central machine (the ``bench`` database next to MLflow's) and a SQLite file
anywhere else, so a laptop can run the same page and worker when the central
machine is down.

    BENCH_QUEUE_URL=postgresql+psycopg://bench:...@127.0.0.1:5432/bench
    BENCH_QUEUE_URL=sqlite:////home/me/.local/share/ifssim-bench/queue.db   (the default)

States::

    queued -> running -> done | failed | cancelled
                      -> upload_pending -> done    (MLflow was down; the worker retries)
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import sqlalchemy as sa

ENV_URL = "BENCH_QUEUE_URL"
DEFAULT_URL = "sqlite:///" + str(
    Path("~/.local/share/ifssim-bench/queue.db").expanduser()
)

QUEUED, RUNNING, DONE, FAILED, CANCELLED, UPLOAD_PENDING = (
    "queued",
    "running",
    "done",
    "failed",
    "cancelled",
    "upload_pending",
)
ACTIVE = (QUEUED, RUNNING)
FINISHED = (DONE, FAILED, CANCELLED, UPLOAD_PENDING)
PRIORITY = {"web": 10, "pr": 10, "cli": 10, "commit": 0}

meta = sa.MetaData()
jobs = sa.Table(
    "bench_jobs",
    meta,
    sa.Column("id", sa.Integer, primary_key=True, autoincrement=True),
    sa.Column("batch_id", sa.String(40), nullable=False, index=True),
    sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
    sa.Column("started_at", sa.DateTime(timezone=True)),
    sa.Column("finished_at", sa.DateTime(timezone=True)),
    sa.Column("trigger", sa.String(16), nullable=False),
    sa.Column("requested_by", sa.String(200), nullable=False),
    sa.Column("name", sa.String(200)),
    sa.Column("label", sa.String(400), nullable=False),
    sa.Column("benchmark", sa.String(100), nullable=False),
    sa.Column("state", sa.String(20), nullable=False, index=True),
    sa.Column("priority", sa.Integer, nullable=False, default=0),
    sa.Column("spec_id", sa.String(40)),
    sa.Column(
        "job", sa.Text, nullable=False
    ),  # Job.record(): what the benchmark gets as spec.json
    sa.Column("code", sa.Text),  # {"requested": {...}, "resolved": {...}, "image": ...}
    sa.Column("github", sa.Text),  # where to report back (PR and commit checks)
    sa.Column("exit_code", sa.Integer),
    sa.Column("error", sa.Text),
    sa.Column("log_path", sa.Text),
    sa.Column("runs", sa.Text),  # MLflow run ids it produced
    sa.Column("cancel_requested", sa.Boolean, nullable=False, default=False),
    sa.Column("worker", sa.String(200)),
)

JSON_COLS = ("job", "code", "github", "runs")


def now() -> datetime:
    return datetime.now(timezone.utc)


@dataclass
class JobRow:
    id: int
    batch_id: str
    created_at: datetime
    started_at: datetime | None
    finished_at: datetime | None
    trigger: str
    requested_by: str
    name: str | None
    label: str
    benchmark: str
    state: str
    priority: int
    spec_id: str | None
    job: dict[str, Any]
    code: dict[str, Any] | None
    github: dict[str, Any] | None
    exit_code: int | None
    error: str | None
    log_path: str | None
    runs: list[str] | None
    cancel_requested: bool
    worker: str | None

    @property
    def duration_s(self) -> float | None:
        if not self.started_at:
            return None
        end = self.finished_at or now()
        return (_aware(end) - _aware(self.started_at)).total_seconds()


def _aware(t: datetime) -> datetime:
    # SQLite gives naive datetimes back
    return t if t.tzinfo else t.replace(tzinfo=timezone.utc)


def _row(r: Any) -> JobRow:
    d = dict(r._mapping)
    for k in JSON_COLS:
        d[k] = json.loads(d[k]) if d[k] else None
    for k in ("created_at", "started_at", "finished_at"):
        if d[k] is not None:
            d[k] = _aware(d[k])
    return JobRow(**d)


class Queue:
    def __init__(self, url: str | None = None) -> None:
        self.url = url or os.environ.get(ENV_URL) or DEFAULT_URL
        if self.url.startswith("sqlite:///"):
            Path(self.url.removeprefix("sqlite:///")).parent.mkdir(
                parents=True, exist_ok=True
            )
        self.engine = sa.create_engine(self.url, future=True, pool_pre_ping=True)
        meta.create_all(self.engine)

    # ------------------------------------------------------------ asking
    def submit(
        self,
        records: list[dict[str, Any]],
        *,
        batch_id: str,
        trigger: str,
        requested_by: str,
        code: dict[str, Any] | None = None,
        github: dict[str, Any] | None = None,
        priority: int | None = None,
    ) -> list[int]:
        """Queue one job per record (``Job.record(spec)``); returns their ids."""
        prio = PRIORITY.get(trigger, 0) if priority is None else priority
        ids = []
        with self.engine.begin() as c:
            for rec in records:
                res = c.execute(
                    jobs.insert().values(
                        batch_id=batch_id,
                        created_at=now(),
                        trigger=trigger,
                        requested_by=requested_by,
                        name=rec.get("name"),
                        label=rec.get("label") or rec["benchmark"],
                        benchmark=rec["benchmark"],
                        state=QUEUED,
                        priority=prio,
                        spec_id=rec.get("spec_id"),
                        job=json.dumps(rec, default=str),
                        code=json.dumps(code) if code else None,
                        github=json.dumps(github) if github else None,
                        cancel_requested=False,
                    )
                )
                ids.append(int(res.inserted_primary_key[0]))
        return ids

    def cancel(self, job_id: int) -> str | None:
        """Queued: cancelled now. Running: flagged; the worker stops it. Returns the new state."""
        with self.engine.begin() as c:
            j = c.execute(sa.select(jobs.c.state).where(jobs.c.id == job_id)).first()
            if j is None:
                return None
            if j.state == QUEUED:
                c.execute(
                    jobs.update()
                    .where(jobs.c.id == job_id, jobs.c.state == QUEUED)
                    .values(
                        state=CANCELLED,
                        finished_at=now(),
                        error="cancelled before it started",
                    )
                )
                return CANCELLED
            if j.state == RUNNING:
                c.execute(
                    jobs.update()
                    .where(jobs.c.id == job_id)
                    .values(cancel_requested=True)
                )
                return RUNNING
            return j.state

    def cancel_superseded(self, github_key: str, keep_sha: str) -> list[int]:
        """Queued PR/commit jobs for an older head of the same PR: cancelled."""
        out = []
        for j in self.list(states=(QUEUED,), limit=1000):
            g = j.github or {}
            if g.get("key") == github_key and g.get("sha") != keep_sha:
                if self.cancel(j.id) == CANCELLED:
                    out.append(j.id)
        return out

    # ------------------------------------------------------------ reading
    def get(self, job_id: int) -> JobRow | None:
        with self.engine.connect() as c:
            r = c.execute(sa.select(jobs).where(jobs.c.id == job_id)).first()
        return _row(r) if r else None

    def list(
        self,
        *,
        states: tuple[str, ...] | None = None,
        batch_id: str | None = None,
        limit: int = 100,
    ) -> list[JobRow]:
        q = sa.select(jobs)
        if states:
            q = q.where(jobs.c.state.in_(states))
        if batch_id:
            q = q.where(jobs.c.batch_id == batch_id)
        q = q.order_by(jobs.c.id.desc()).limit(limit)
        with self.engine.connect() as c:
            return [_row(r) for r in c.execute(q)]

    def queued_in_order(self) -> list[JobRow]:
        """What the worker will run, next first."""
        q = (
            sa.select(jobs)
            .where(jobs.c.state == QUEUED)
            .order_by(jobs.c.priority.desc(), jobs.c.id)
        )
        with self.engine.connect() as c:
            return [_row(r) for r in c.execute(q)]

    # ------------------------------------------------------------ the worker
    def claim(self, worker: str) -> JobRow | None:
        """The next job, marked running; None when there is nothing to do."""
        with self.engine.begin() as c:
            nxt = c.execute(
                sa.select(jobs.c.id)
                .where(jobs.c.state == QUEUED)
                .order_by(jobs.c.priority.desc(), jobs.c.id)
                .limit(1)
            ).first()
            if nxt is None:
                return None
            got = c.execute(
                jobs.update()
                .where(jobs.c.id == nxt.id, jobs.c.state == QUEUED)
                .values(state=RUNNING, started_at=now(), worker=worker)
            )
            if got.rowcount != 1:  # someone else took it
                return None
        return self.get(nxt.id)

    def update(self, job_id: int, **values: Any) -> None:
        for k in JSON_COLS:
            if k in values and values[k] is not None:
                values[k] = json.dumps(values[k], default=str)
        with self.engine.begin() as c:
            c.execute(jobs.update().where(jobs.c.id == job_id).values(**values))

    def finish(self, job_id: int, state: str, **values: Any) -> None:
        self.update(job_id, state=state, finished_at=now(), **values)

    def cancel_requested(self, job_id: int) -> bool:
        with self.engine.connect() as c:
            r = c.execute(
                sa.select(jobs.c.cancel_requested).where(jobs.c.id == job_id)
            ).first()
        return bool(r and r.cancel_requested)

    def recover(self, worker: str) -> list[int]:
        """Jobs this worker left running (it died): failed. They are never resumed."""
        stuck = [
            j.id for j in self.list(states=(RUNNING,), limit=1000) if j.worker == worker
        ]
        for i in stuck:
            self.finish(
                i, FAILED, error="the worker restarted while this job was running"
            )
        return stuck
