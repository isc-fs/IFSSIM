"""The Launch section of bench-view: start benchmarks, watch the queue, read a job.

    /launch            New run: pick benchmarks, bags and code; add a YAML; see the jobs; launch
    /launch/queue      what is running, what waits (next first), what finished
    /launch/job/<id>   one job: its spec, code, image, log, and the runs it produced

The page only writes jobs to the queue (``launch/queue.py``); ``bench-worker``
runs them. Where things are comes from the environment:

    BENCH_REPO        the checkout with bench.yaml (default: this one)
    BENCH_BAGS        bag folders instead of the manifest's, "simulator=/srv/bench/bags;..."
    BENCH_QUEUE_URL   the queue (default: a SQLite file in ~/.local/share/ifssim-bench)
    BENCH_ADMINS      logins that may cancel anyone's jobs, comma separated

Who launched a job is the ``Tailscale-User-Login`` header that ``tailscale
serve`` adds, or ``<user>@<host>`` when the page is opened locally.

The form and the YAML are one spec (design: docs/history/2026-09-28_benchmark-launcher-design.md
§3): the YAML is merged first, the form's fields on top. The form wins, and the
preview says where it overrode the YAML.
"""

from __future__ import annotations

import getpass
import os
import socket
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml
from dash import ALL, Dash, Input, Output, State, ctx, dcc, html

from ..launch import manifest as mf
from ..launch import queue as qu

SECTIONS = (("new", "New run"), ("queue", "Queue"))
KIND_OF_JOB = {"sim_bag": "simbag", "onboard_replay": "bag", "live_replay": "live"}
STATE_CLASS = {
    qu.DONE: "good",
    qu.FAILED: "bad",
    qu.CANCELLED: "",
    qu.RUNNING: "run",
    qu.QUEUED: "",
    qu.UPLOAD_PENDING: "warn",
}
LOG_TAIL = 400  # lines


# ======================================================================== setup
@dataclass
class Launcher:
    manifest: mf.Manifest
    queue: qu.Queue
    admins: frozenset[str]


@lru_cache(maxsize=1)
def launcher() -> Launcher:
    root = (
        Path(os.environ["BENCH_REPO"])
        if os.environ.get("BENCH_REPO")
        else mf.find_root()
    )
    m = mf.load(root)
    bags = {}
    for item in (os.environ.get("BENCH_BAGS") or "").split(";"):
        if "=" in item:
            k, v = item.split("=", 1)
            bags[k.strip()] = Path(v.strip())
    admins = {
        a.strip()
        for a in (os.environ.get("BENCH_ADMINS") or "").split(",")
        if a.strip()
    }
    return Launcher(m.with_paths(bag_dirs=bags), qu.Queue(), frozenset(admins))


def setup_problem() -> str | None:
    try:
        launcher()
        return None
    except Exception as e:  # noqa: BLE001
        launcher.cache_clear()
        return f"{type(e).__name__}: {e}"


def current_user() -> str:
    try:
        from flask import request

        who = request.headers.get("Tailscale-User-Login")
        if who:
            return who
    except RuntimeError:  # outside a request
        pass
    return f"{getpass.getuser()}@{socket.gethostname()}"


def may_cancel(j: qu.JobRow, who: str) -> bool:
    return j.requested_by == who or who in launcher().admins


def is_launch(pathname: str | None) -> bool:
    return (pathname or "").rstrip("/").startswith("/launch")


def parse(pathname: str | None) -> tuple[str, int | None]:
    parts = [p for p in (pathname or "").split("/") if p][1:]
    if parts[:1] == ["job"] and len(parts) > 1 and parts[1].isdigit():
        return "job", int(parts[1])
    if parts[:1] == ["queue"]:
        return "queue", None
    return "new", None


# ======================================================================== nav + head
def nav(pathname: str | None) -> list:
    page, _ = parse(pathname)
    sec = "queue" if page == "job" else page
    return [
        dcc.Link(
            [html.Span("Launch"), html.Span(_queue_count(), className="count")],
            href="/launch",
            className="nav-kind active",
        ),
        html.Div(
            [
                dcc.Link(
                    lab,
                    href="/launch" if s == "new" else f"/launch/{s}",
                    className="nav-sec" + (" active" if s == sec else ""),
                )
                for s, lab in SECTIONS
            ],
            className="nav-secs",
        ),
    ]


def _queue_count() -> str:
    if setup_problem():
        return "–"
    return str(len(launcher().queue.list(states=qu.ACTIVE, limit=1000)))


def head(pathname: str | None) -> list:
    page, jid = parse(pathname)
    title, blurb = {
        "new": (
            "New run",
            "Pick the code to test, the benchmarks and their bags, and launch. "
            "The central machine runs one job at a time.",
        ),
        "queue": (
            "Queue",
            "What is running, what waits (next first), and what finished.",
        ),
        "job": (f"Job {jid}", "One benchmark run from the queue."),
    }[page]
    return [
        html.Div(
            [
                html.Span("Launch", className="crumb"),
                html.Span(" › ", className="dim"),
                html.Span(title),
            ],
            className="title",
        ),
        html.Div(blurb, className="blurb"),
    ]


def page(pathname: str | None, search: str | None = None) -> list:
    problem = setup_problem()
    if problem:
        return [
            html.Div(
                [
                    html.B("Launching isn't set up here. "),
                    problem,
                    html.Br(),
                    "It needs the repository's bench.yaml (BENCH_REPO) and a queue "
                    "(BENCH_QUEUE_URL; SQLite by default). See tracking/DEPLOY.md.",
                ],
                className="callout",
            )
        ]
    p, jid = parse(pathname)
    if p == "queue":
        return queue_page()
    if p == "job":
        return job_page(jid)
    from . import launch_new

    return launch_new.page(search)


# ======================================================================== shared form bits
def _field(label: str, control, hint: str | None = None) -> html.Div:
    return html.Div(
        [
            html.Label(label, className="lf-label"),
            html.Div(
                [control, html.Div(hint, className="dim small") if hint else None]
            ),
        ],
        className="lf-field",
    )


def overridden(yaml_doc: Any, form: Any, prefix: str = "") -> list[str]:
    """Keys the form set to something other than what the YAML had."""
    out = []
    if isinstance(yaml_doc, dict) and isinstance(form, dict):
        for k, v in form.items():
            if k in yaml_doc:
                out += overridden(yaml_doc[k], v, f"{prefix}{k}.")
        return out
    if yaml_doc != form:
        out.append(prefix.rstrip("."))
    return out


# ======================================================================== Queue
def queue_page() -> list:
    return [
        dcc.Interval(id="lq-tick", interval=3000),
        html.Div(id="lq-body", children=queue_body()),
    ]


def queue_body() -> list:
    q = launcher().queue
    who = current_user()
    running = q.list(states=(qu.RUNNING,), limit=20)
    waiting = q.queued_in_order()
    done = q.list(states=qu.FINISHED, limit=40)
    out = []
    out.append(html.H3(f"Running ({len(running)})"))
    out.append(
        _jobs_table(running, who, show_elapsed=True)
        if running
        else html.Div("Nothing is running.", className="dim")
    )
    out.append(html.H3(f"Waiting ({len(waiting)})"))
    out.append(
        _jobs_table(waiting, who, numbered=True)
        if waiting
        else html.Div("Nothing is waiting.", className="dim")
    )
    out.append(html.H3("Finished (latest 40)"))
    out.append(
        _jobs_table(done, who, finished=True)
        if done
        else html.Div("Nothing has run yet.", className="dim")
    )
    return out


def _jobs_table(
    rows: list[qu.JobRow],
    who: str,
    *,
    show_elapsed=False,
    numbered=False,
    finished=False,
) -> html.Table:
    head = ["#" if numbered else "job", "what", "by", "state", "time", ""]
    body = []
    for i, j in enumerate(rows, 1):
        cells = [
            html.Td(
                str(i) if numbered else dcc.Link(str(j.id), href=f"/launch/job/{j.id}")
            ),
            html.Td(
                [
                    dcc.Link(j.label, href=f"/launch/job/{j.id}"),
                    html.Div(
                        " · ".join(
                            x for x in (j.name, j.trigger, f"spec {j.spec_id}") if x
                        ),
                        className="dim small",
                    ),
                ]
            ),
            html.Td(j.requested_by, className="small"),
            html.Td(_state_badge(j)),
            html.Td(_when(j, finished), className="num small"),
        ]
        act: list = []
        if j.state in qu.ACTIVE and may_cancel(j, who) and not j.cancel_requested:
            act.append(
                html.Button(
                    "Cancel", id={"type": "lq-cancel", "j": j.id}, className="btn small"
                )
            )
        if finished and j.runs:
            act += [_run_link(j, r, k) for k, r in enumerate(j.runs, 1)]
        cells.append(html.Td(act, className="nowrap"))
        body.append(html.Tr(cells))
    return html.Table(
        [html.Thead(html.Tr([html.Th(h) for h in head])), html.Tbody(body)],
        className="mt compact lq",
    )


def _state_badge(j: qu.JobRow) -> html.Span:
    text = j.state.replace("_", " ")
    if j.state == qu.RUNNING and j.cancel_requested:
        text = "stopping"
    return html.Span(
        text, className=f"badge st-{STATE_CLASS.get(j.state, '')}", title=j.error or ""
    )


def _when(j: qu.JobRow, finished: bool) -> str:
    if j.state == qu.QUEUED:
        return f"since {j.created_at:%d %b %H:%M}"
    d = j.duration_s
    dur = "" if d is None else (f"{d:.0f} s" if d < 120 else f"{d / 60:.0f} min")
    if finished and j.finished_at:
        return f"{j.finished_at:%d %b %H:%M}" + (f" · {dur}" if dur else "")
    return dur


def _run_link(j: qu.JobRow, run_id: str, k: int) -> dcc.Link:
    kind = KIND_OF_JOB.get(j.benchmark, "simbag")
    return dcc.Link(
        f"results{'' if k == 1 else f' {k}'} →",
        href=f"/{kind}/summary?r={run_id}",
        className="btn small ghost",
    )


# ======================================================================== one job
def job_page(jid: int | None) -> list:
    return [
        dcc.Interval(id="lj-tick", interval=3000),
        dcc.Store(id="lj-id", data=jid),
        html.Div(id="lj-body", children=job_body(jid)),
    ]


def job_body(jid: int | None) -> list:
    j = launcher().queue.get(jid) if jid is not None else None
    if j is None:
        return [html.Div(f"No job {jid}.", className="callout")]
    who = current_user()
    rec = j.job or {}
    code = j.code or {}
    resolved = code.get("resolved") or {}
    facts = [
        ("What", j.label),
        (
            "State",
            [
                _state_badge(j),
                html.Span(f"  {j.error}" if j.error else "", className="dim"),
            ],
        ),
        (
            "Requested by",
            f"{j.requested_by} ({j.trigger}) · {j.created_at:%a %d %b %H:%M} UTC",
        ),
        ("Batch", j.batch_id),
        (
            "Code",
            ", ".join(f"{n} {r['ref']} = {r['sha'][:12]}" for n, r in resolved.items())
            or "the worker's checkout as it is",
        ),
        (
            "Image",
            f"{code['image']['ref']}  ({code['image']['why']})"
            if code.get("image")
            else "chosen when the job starts"
            if j.state == qu.QUEUED
            else "the benchmark's default (bench.yaml names no image, or no commits were pinned)",
        ),
        ("Spec id", rec.get("spec_id")),
        ("Time", _when(j, j.state in qu.FINISHED) or "–"),
    ]
    act: list = []
    if j.state in qu.ACTIVE and may_cancel(j, who) and not j.cancel_requested:
        act.append(
            html.Button(
                "Cancel", id={"type": "lq-cancel", "j": j.id}, className="btn small"
            )
        )
    act += [_run_link(j, r, k) for k, r in enumerate(j.runs or [], 1)]
    act.append(
        dcc.Link(
            "Rerun / edit…", href=f"/launch?from={j.id}", className="btn small ghost"
        )
    )
    return [
        html.Div(
            html.Table(
                [html.Tr([html.Td(k, className="k"), html.Td(v)]) for k, v in facts],
                className="lj-facts",
            ),
            className="card lf-card",
        ),
        html.Div(act, className="lf-go-row"),
        html.H3("Log"),
        html.Pre(_log_tail(j), className="lf-log"),
        html.H3("Spec"),
        html.Pre(
            yaml.safe_dump(rec.get("spec") or rec, sort_keys=False), className="lf-pre"
        ),
    ]


def _log_tail(j: qu.JobRow) -> str:
    if not j.log_path:
        return "(waiting for the worker)"
    try:
        lines = Path(j.log_path).read_text(errors="replace").splitlines()
    except OSError:
        return f"(no log at {j.log_path} on this machine)"
    cut = len(lines) > LOG_TAIL
    return ("…\n" if cut else "") + "\n".join(lines[-LOG_TAIL:])


# ======================================================================== callbacks
def register(app: Dash, catalog=None) -> None:
    """``catalog()``: the viewer's run catalog, re-read when a job's runs aren't in it yet."""

    from . import launch_new

    launch_new.register(app)

    @app.callback(
        Output("lq-body", "children"),
        Input("lq-tick", "n_intervals"),
        Input({"type": "lq-cancel", "j": ALL}, "n_clicks"),
    )
    def refresh_queue(_n, cancels):
        _cancel_clicked(cancels)
        return queue_body()

    @app.callback(
        Output("lj-body", "children"),
        Input("lj-tick", "n_intervals"),
        Input({"type": "lq-cancel", "j": ALL}, "n_clicks"),
        State("lj-id", "data"),
    )
    def refresh_job(_n, cancels, jid):
        _cancel_clicked(cancels)
        j = launcher().queue.get(jid) if jid is not None else None
        if catalog is not None and j is not None and j.runs:
            c = catalog()
            if any(r not in c.rows for r in j.runs):
                c.refresh()  # so the results links open these runs
        return job_body(jid)


def _cancel_clicked(clicks) -> None:
    t = ctx.triggered_id
    if not (isinstance(t, dict) and t.get("type") == "lq-cancel" and any(clicks or [])):
        return
    lz = launcher()
    j = lz.queue.get(int(t["j"]))
    if j is not None and may_cancel(j, current_user()):
        lz.queue.cancel(j.id)
