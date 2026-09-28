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

import base64
import getpass
import os
import socket
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml
from dash import ALL, Dash, Input, Output, State, ctx, dcc, html, no_update

from ..launch import checkout as co
from ..launch import manifest as mf
from ..launch import queue as qu
from ..launch import spec as sp
from ..launch import submit as su

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
            "Pick benchmarks, bags and code, add a YAML with settings and parameter overrides, "
            "and launch. The central machine runs one job at a time.",
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


def page(pathname: str | None) -> list:
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
    return new_page()


# ======================================================================== New run
def new_page() -> list:
    lz = launcher()
    m = lz.manifest
    bench_cards = []
    for name, b in m.benchmarks.items():
        bags = m.bags(b.bags) if b.bags else []
        comps = sorted(b.components())
        fields = [
            html.Div(
                dcc.Checklist(
                    id={"type": "lf-on", "b": name},
                    options=[{"label": html.B(f" {b.title}"), "value": "on"}],
                    value=[],
                ),
                className="lf-title",
            ),
            html.Div(b.description, className="dim small"),
        ]
        if b.bags:
            fields.append(
                _field(
                    "Bags",
                    dcc.Dropdown(
                        id={"type": "lf-bags", "b": name},
                        options=[{"label": "all bags", "value": sp.ALL_BAGS}]
                        + [{"label": x, "value": x} for x in bags],
                        multi=True,
                        placeholder=f"{len(bags)} in {m.bag_dirs[b.bags]}",
                    ),
                )
            )
        if b.parts:
            fields.append(
                _field(
                    "Parts",
                    dcc.Checklist(
                        id={"type": "lf-only", "b": name},
                        options=[{"label": f" {p}", "value": p} for p in b.parts],
                        value=[],
                        inline=True,
                        className="lf-inline",
                    ),
                    "none ticked = all",
                )
            )
        fields.append(
            _field(
                "Repeats",
                dcc.Input(
                    id={"type": "lf-rep", "b": name},
                    type="number",
                    min=1,
                    step=1,
                    placeholder="1",
                    className="lf-num",
                ),
            )
        )
        hints = [
            f"settings.{s.name}: {s.type}" + (f" — {s.help}" if s.help else "")
            for s in b.settings.values()
        ]
        if comps:
            hints.append(f"pipeline: {', '.join(comps)} (see bench.yaml)")
        fields.append(
            html.Details(
                [
                    html.Summary("What the YAML can set for it"),
                    html.Pre("\n".join(hints), className="lf-pre"),
                ],
                className="small",
            )
        )
        bench_cards.append(html.Div(fields, className="card lf-card"))

    code_fields = [
        _field(
            f"{name}",
            dcc.Input(
                id={"type": "lf-ref", "r": name},
                type="text",
                placeholder=m.default_refs.get(name)
                or (
                    f"as pinned by {next(iter(m.default_refs), 'the main repository')}"
                    if path != m.root
                    else "branch, #PR or commit"
                ),
                className="lf-text",
                debounce=True,
            ),
            "branch, #PR or commit",
        )
        for name, path in m.repos.items()
    ]
    image = _field(
        "Image",
        dcc.Input(
            id="lf-image",
            type="text",
            placeholder="auto",
            className="lf-text",
            debounce=True,
        ),
        "auto, build, or a tag",
    )
    if m.image is None:
        image.style = {"display": "none"}
    code_fields.append(image)

    presets = m.preset_names()
    return [
        html.Div(
            [
                html.Div(
                    [
                        html.H3("1 · Benchmarks"),
                        *bench_cards,
                        html.H3("2 · Code"),
                        html.Div(code_fields, className="card lf-card"),
                        html.H3("3 · Compare and name"),
                        html.Div(
                            [
                                _field(
                                    "Compare to",
                                    dcc.Dropdown(
                                        id="lf-compare",
                                        options=[
                                            {
                                                "label": "the pinned baseline",
                                                "value": "pinned",
                                            },
                                            {
                                                "label": "the branch point on the base branch",
                                                "value": "merge-base",
                                            },
                                        ],
                                        placeholder="pinned baseline",
                                    ),
                                ),
                                _field(
                                    "Name",
                                    dcc.Input(
                                        id="lf-name",
                                        type="text",
                                        className="lf-text",
                                        debounce=True,
                                    ),
                                ),
                                _field(
                                    "Notes",
                                    dcc.Textarea(id="lf-notes", className="lf-notes"),
                                ),
                            ],
                            className="card lf-card",
                        ),
                    ],
                    className="lf-form",
                ),
                html.Div(
                    [
                        html.H3("4 · YAML (settings, parameter overrides, sweeps)"),
                        html.Div(
                            [
                                html.Div(
                                    [
                                        dcc.Dropdown(
                                            id="lf-preset",
                                            options=[
                                                {"label": f"preset: {p}", "value": p}
                                                for p in presets
                                            ],
                                            placeholder="Start from a preset…",
                                            className="lf-preset",
                                        ),
                                        dcc.Upload(
                                            html.Button(
                                                "Upload YAML…", className="btn small"
                                            ),
                                            id="lf-upload",
                                            accept=".yaml,.yml",
                                        ),
                                    ],
                                    className="lf-yaml-bar",
                                ),
                                dcc.Textarea(
                                    id="lf-yaml",
                                    className="lf-yaml",
                                    placeholder=_YAML_HINT,
                                    spellCheck=False,
                                ),
                            ],
                            className="card lf-card",
                        ),
                        html.H3("5 · Check and launch"),
                        html.Div(id="lf-preview", className="card lf-card"),
                        html.Div(
                            [
                                html.Button(
                                    "Launch",
                                    id="lf-go",
                                    className="btn primary",
                                    disabled=True,
                                ),
                                html.Span(
                                    f"as {current_user()}", className="dim small"
                                ),
                            ],
                            className="lf-go-row",
                        ),
                        html.Div(id="lf-result"),
                    ],
                    className="lf-side",
                ),
            ],
            className="lf",
        )
    ]


_YAML_HINT = """# optional; the form's fields win over what is here
benchmarks:
  sim_bag:
    settings: {max_frames: 200}
pipeline:
  cone_detection: {residual_gate_mse: 0.05}
  slam_node: {motion_model: imu}
sweep:
  pipeline.cone_detection.residual_gate_mse: [0.02, 0.05]
"""


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


def form_doc(
    names: list[str],
    on: list[list[str]],
    bags: list[list[str] | None],
    only: list[list[str] | None],
    reps: list[int | None],
    ref_names: list[str],
    refs: list[str | None],
    image: str | None,
    compare: str | None,
    name: str | None,
    notes: str | None,
) -> dict[str, Any]:
    """The form as a spec: only what was filled in."""
    doc: dict[str, Any] = {}
    benches = {}
    for b, is_on, bg, parts, rep in zip(names, on, bags, only, reps, strict=True):
        if not is_on:
            continue
        entry: dict[str, Any] = {}
        if bg:
            entry["bags"] = sp.ALL_BAGS if sp.ALL_BAGS in bg else list(bg)
        if parts:
            entry["only"] = list(parts)
        if rep:
            entry["repeats"] = int(rep)
        benches[b] = entry
    if benches:
        doc["benchmarks"] = benches
    code = {
        r: v.strip() for r, v in zip(ref_names, refs, strict=True) if v and v.strip()
    }
    if image and image.strip():
        code["image"] = image.strip()
    if code:
        doc["code"] = code
    if compare:
        doc["compare_to"] = compare
    if name and name.strip():
        doc["name"] = name.strip()
    if notes and notes.strip():
        doc["notes"] = notes.strip()
    return doc


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


def preview(yaml_text: str | None, form: dict[str, Any]) -> tuple[list, dict | None]:
    """(what to show, the spec to launch or None when it can't be)."""
    m = launcher().manifest
    try:
        ydoc = yaml.safe_load(yaml_text or "") or {}
        if not isinstance(ydoc, dict):
            raise sp.SpecError("the YAML must be a mapping (key: value)")
    except yaml.YAMLError as e:
        return [
            html.Div(["YAML: ", html.Code(str(e))], className="callout bad-callout")
        ], None
    except sp.SpecError as e:
        return [html.Div(str(e), className="callout bad-callout")], None
    merged = sp.merge(ydoc, form)
    if not merged.get("benchmarks"):
        return [
            html.Div(
                "Tick a benchmark (or name one in the YAML) to see the jobs.",
                className="dim",
            )
        ], None
    try:
        s = sp.validate(merged, m)
        jobs = sp.expand(s, m)
    except sp.SpecError as e:
        return [
            html.B("The spec has problems:"),
            html.Ul(
                [html.Li(line) for line in str(e).splitlines()], className="lf-errors"
            ),
        ], None
    over = overridden(ydoc, form)
    rows = [
        html.Tr([html.Td(i), html.Td(j.label()), html.Td(html.Code(j.spec_id))])
        for i, j in enumerate(jobs[:200], 1)
    ]
    body = [
        html.Div(
            [
                html.B(f"{len(jobs)} job(s)"),
                html.Span(" · run one at a time, in this order", className="dim"),
            ]
        ),
        html.Table(
            [
                html.Thead(html.Tr([html.Th("#"), html.Th("job"), html.Th("spec id")])),
                html.Tbody(rows),
            ],
            className="mt compact lf-jobs",
        ),
    ]
    if len(jobs) > 200:
        body.append(html.Div(f"… and {len(jobs) - 200} more", className="dim small"))
    if over:
        body.append(
            html.Div(
                ["Overridden by the form: ", ", ".join(over)],
                className="callout info small",
            )
        )
    body.append(
        html.Details(
            [
                html.Summary("The merged spec"),
                html.Pre(yaml.safe_dump(s, sort_keys=False), className="lf-pre"),
            ],
            open=False,
        )
    )
    return body, s


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

    @app.callback(
        Output("lf-yaml", "value"),
        Input("lf-preset", "value"),
        Input("lf-upload", "contents"),
        Input("url", "search"),
        State("url", "pathname"),
        prevent_initial_call=False,
    )
    def load_yaml(preset, upload, search, pathname):
        trig = ctx.triggered_id
        if trig == "lf-upload" and upload:
            _, data = upload.split(",", 1)
            return base64.b64decode(data).decode("utf-8", errors="replace")
        if trig == "lf-preset" and preset:
            p = launcher().manifest.preset(preset)
            return p.read_text() if p else no_update
        # "Rerun / edit…" from a job page: its spec
        q = dict(
            x.split("=", 1) for x in (search or "").lstrip("?").split("&") if "=" in x
        )
        if q.get("from", "").isdigit():
            j = launcher().queue.get(int(q["from"]))
            if j is not None:
                return yaml.safe_dump((j.job or {}).get("spec") or {}, sort_keys=False)
        return no_update

    @app.callback(
        Output("lf-preview", "children"),
        Output("lf-go", "disabled"),
        Input({"type": "lf-on", "b": ALL}, "value"),
        Input({"type": "lf-bags", "b": ALL}, "value"),
        Input({"type": "lf-only", "b": ALL}, "value"),
        Input({"type": "lf-rep", "b": ALL}, "value"),
        Input({"type": "lf-ref", "r": ALL}, "value"),
        Input("lf-image", "value"),
        Input("lf-compare", "value"),
        Input("lf-name", "value"),
        Input("lf-notes", "value"),
        Input("lf-yaml", "value"),
        State({"type": "lf-on", "b": ALL}, "id"),
        State({"type": "lf-ref", "r": ALL}, "id"),
    )
    def update_preview(
        on, bags, only, reps, refs, image, compare, name, notes, text, on_ids, ref_ids
    ):
        form = form_doc(
            [i["b"] for i in on_ids],
            on,
            bags,
            only,
            reps,
            [i["r"] for i in ref_ids],
            refs,
            image,
            compare,
            name,
            notes,
        )
        body, spec = preview(text, form)
        return body, spec is None

    @app.callback(
        Output("lf-result", "children"),
        Input("lf-go", "n_clicks"),
        State({"type": "lf-on", "b": ALL}, "value"),
        State({"type": "lf-bags", "b": ALL}, "value"),
        State({"type": "lf-only", "b": ALL}, "value"),
        State({"type": "lf-rep", "b": ALL}, "value"),
        State({"type": "lf-ref", "r": ALL}, "value"),
        State("lf-image", "value"),
        State("lf-compare", "value"),
        State("lf-name", "value"),
        State("lf-notes", "value"),
        State("lf-yaml", "value"),
        State({"type": "lf-on", "b": ALL}, "id"),
        State({"type": "lf-ref", "r": ALL}, "id"),
        prevent_initial_call=True,
    )
    def go(
        n,
        on,
        bags,
        only,
        reps,
        refs,
        image,
        compare,
        name,
        notes,
        text,
        on_ids,
        ref_ids,
    ):
        if not n:
            return no_update
        form = form_doc(
            [i["b"] for i in on_ids],
            on,
            bags,
            only,
            reps,
            [i["r"] for i in ref_ids],
            refs,
            image,
            compare,
            name,
            notes,
        )
        _, spec = preview(text, form)
        if spec is None:
            return html.Div("The spec has problems (see above).", className="callout")
        lz = launcher()
        try:
            p = su.plan(lz.manifest, spec)
        except co.CodeError as e:
            return html.Div(["Cannot pin the code: ", str(e)], className="callout")
        batch, ids = su.submit(lz.queue, p, trigger="web", requested_by=current_user())
        pinned = ", ".join(
            f"{k} {r['ref']} = {r['sha'][:12]}" for k, r in p.code["resolved"].items()
        )
        return html.Div(
            [
                html.B(f"Queued {len(ids)} job(s)"),
                f" ({pinned}). ",
                dcc.Link("See the queue →", href="/launch/queue"),
            ],
            className="callout info",
        )

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
