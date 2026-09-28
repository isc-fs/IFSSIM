"""Queue a spec: validate it, pin its code to commits, and add one job per bag, repeat and
sweep point. The Launch page, ``bench-run --queue`` and the PR checks all come through here."""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from . import checkout as co
from . import manifest as mf
from . import queue as qu
from . import spec as sp


@dataclass
class Plan:
    spec: dict[str, Any]  # validated
    jobs: list[sp.Job]
    code: dict[str, Any]  # {"requested": {...}, "resolved": {...}}


def new_batch_id() -> str:
    return f"{datetime.now(timezone.utc):%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}"


def plan(m: mf.Manifest, spec: dict[str, Any], *, pin_code: bool = True) -> Plan:
    """What launching ``spec`` would queue. With ``pin_code``, the code is resolved to
    commits now (the manifest's default branch when the spec names none), so the jobs run
    exactly that code however long they wait. Without it, jobs run the worker's checkout
    as it is."""
    s = sp.validate(spec, m)
    jobs = sp.expand(s, m)
    requested = dict(s.get("code") or {})
    if pin_code:
        for name, ref in m.default_refs.items():
            requested.setdefault(name, ref)
    resolved = co.resolve(m, requested) if pin_code else {}
    return Plan(s, jobs, {"requested": requested, "resolved": resolved})


def submit(
    q: qu.Queue,
    p: Plan,
    *,
    trigger: str,
    requested_by: str,
    github: dict[str, Any] | None = None,
) -> tuple[str, list[int]]:
    batch = new_batch_id()
    ids = q.submit(
        [j.record(p.spec) for j in p.jobs],
        batch_id=batch,
        trigger=trigger,
        requested_by=requested_by,
        code=p.code,
        github=github,
    )
    return batch, ids
