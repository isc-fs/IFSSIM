"""Launch › New run: choose the code, the benchmarks and their bags, optionally a YAML; launch.

The page has three parts in the order people decide them, and a summary that
stays in view:

1. **Code**, per repository: a branch, a pull request or a commit, then which
   commit on it (the latest by default). Lists come from a mirror of each
   repository (``launch/refs.py``), so nothing is typed from memory. The
   pipeline defaults to the commit IFSSIM pins, and says which one that is.
2. **Benchmarks**: tick one and its bags (with length, date and size; bags
   without what the benchmark needs are greyed out), parts and repeats open.
3. **Settings, overrides and sweeps** (optional): a YAML, from a preset, a file
   or typed. The form's choices win over it.

The summary shows the code, the jobs (each with its spec id), any problem, and
the Launch button, which pins the chosen commits and queues the jobs.
"""

from __future__ import annotations

import base64
import time
from datetime import datetime
from typing import Any

import yaml
from dash import ALL, MATCH, Dash, Input, Output, State, ctx, dcc, html, no_update

from ..launch import checkout as co
from ..launch import manifest as mf
from ..launch import refs as rf
from ..launch import spec as sp
from ..launch import submit as su
from . import launch as L

MODES_ROOT = [("branch", "Branch"), ("pr", "Pull request"), ("commit", "Commit")]
MODES_SUB = [("pinned", "As pinned")] + MODES_ROOT
MAX_JOBS_SHOWN = 30


# ======================================================================== helpers
def ago(t: float | None) -> str:
    if not t:
        return ""
    s = max(0.0, time.time() - t)
    for unit, n in (("d", 86400), ("h", 3600), ("min", 60)):
        if s >= n:
            return f"{s / n:.0f} {unit} ago"
    return "just now"


def size(n: int) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024 or unit == "TB":
            return f"{n:.0f} {unit}" if unit in ("B", "KB") else f"{n:.1f} {unit}"
        n /= 1024
    return ""


def _root_name(m: mf.Manifest) -> str:
    for name, path in m.repos.items():
        if path.resolve() == m.root.resolve():
            return name
    return next(iter(m.repos))


def _rel(m: mf.Manifest, name: str) -> str:
    return m.repos[name].resolve().relative_to(m.root.resolve()).as_posix()


def _mirror(name: str, root_sha: str | None = None) -> rf.Mirror | None:
    """The mirror of repository ``name``. A submodule's URL is the one the chosen root
    commit's .gitmodules has (a submodule can move, as the pipeline did to IFS09)."""
    m = L.launcher().manifest
    urls = rf.repo_urls(m)
    root = _root_name(m)
    if name != root and root_sha and root in urls:
        rm = rf.mirror(urls[root])
        if rm.ensure():
            url = rm.submodule_url(root_sha, _rel(m, name))
            if url:
                return rf.mirror(url)
    url = urls.get(name)
    return rf.mirror(url) if url else None


def _opt(value: str, title: str, meta: str, search: str = "") -> dict:
    return {
        "value": value,
        "label": html.Div(
            [html.Div(title, className="opt-t"), html.Div(meta, className="opt-m")],
            className="opt",
        ),
        "search": f"{value} {title} {meta} {search}",
    }


def _commit_meta(c: rf.Commit) -> str:
    return f"{c.short} · {c.author} · {ago(c.when)} · {c.subject}"


# ======================================================================== layout
def page(search: str | None = None) -> list:
    lz = L.launcher()
    m = lz.manifest
    q = dict(x.split("=", 1) for x in (search or "").lstrip("?").split("&") if "=" in x)
    rerun = lz.queue.get(int(q["from"])) if q.get("from", "").isdigit() else None
    root = _root_name(m)
    for name in m.repos:  # start fetching while the page draws
        mi = _mirror(name)
        if mi is not None:
            mi.refresh()

    # a rerun starts from that job's commits and spec
    start_code = ((rerun.code or {}).get("resolved") or {}) if rerun else {}
    start_yaml = ""
    if rerun is not None:
        spec = dict((rerun.job or {}).get("spec") or {})
        spec.pop("code", None)
        start_yaml = yaml.safe_dump(spec, sort_keys=False)

    code_blocks = [
        _code_block(name, name == root, start_code.get(name), m) for name in m.repos
    ]
    image = html.Div(
        [
            html.Label("Image", className="lf-label"),
            html.Div(
                [
                    dcc.Input(
                        id="lf-image",
                        type="text",
                        placeholder="auto",
                        className="lf-text",
                        debounce=True,
                        value=((rerun.code or {}).get("requested") or {}).get("image")
                        if rerun
                        else None,
                    ),
                    html.Div(
                        "auto: the published image of the code's base, or one built from "
                        "the code when it changes the image. Or build, or an image tag.",
                        className="dim small",
                    ),
                ]
            ),
        ],
        className="lf-field",
        style={} if m.image is not None else {"display": "none"},
    )

    return [
        dcc.Interval(id="lc-tick", interval=60_000),
        dcc.Store(id="lc-root-sha"),
        html.Div(
            [
                html.Div(
                    [
                        _step(
                            "1",
                            "Code",
                            "What to test. Pick a version; nothing to type.",
                        ),
                        html.Div(code_blocks, className="lf-code"),
                        html.Details(
                            [html.Summary("Image"), image],
                            className="lf-adv",
                            open=bool(
                                rerun
                                and ((rerun.code or {}).get("requested") or {}).get(
                                    "image"
                                )
                            ),
                        ),
                        _step("2", "Benchmarks", "Tick what to run, then its bags."),
                        *[
                            _bench_card(name, b, m, rerun)
                            for name, b in m.benchmarks.items()
                        ],
                        _step(
                            "3",
                            "Settings, parameter overrides and sweeps",
                            "Optional. A YAML; what you picked above wins over it.",
                        ),
                        _yaml_card(m, start_yaml),
                        _step(
                            "4",
                            "Name",
                            "Optional, shown in the queue and on the results.",
                        ),
                        html.Div(
                            [
                                dcc.Input(
                                    id="lf-name",
                                    type="text",
                                    className="lf-text",
                                    placeholder="e.g. loop-closure gate retune",
                                    debounce=True,
                                    value=(rerun.name if rerun else None),
                                ),
                                dcc.Textarea(
                                    id="lf-notes",
                                    className="lf-notes",
                                    placeholder="Notes (optional)",
                                ),
                            ],
                            className="card lf-card",
                        ),
                    ],
                    className="lf-form",
                ),
                html.Aside(
                    html.Div(
                        [
                            html.Div("What will run", className="lf-sum-title"),
                            html.Div(id="lf-summary"),
                            html.Button(
                                "Launch",
                                id="lf-go",
                                className="btn primary lf-go",
                                disabled=True,
                            ),
                            html.Div(
                                f"as {L.current_user()}", className="dim small lf-as"
                            ),
                            html.Div(id="lf-result"),
                        ],
                        className="card lf-sum",
                    ),
                    className="lf-side",
                ),
            ],
            className="lf",
        ),
    ]


def _step(n: str, title: str, sub: str) -> html.Div:
    return html.Div(
        [
            html.Span(n, className="lf-n"),
            html.Div(
                [
                    html.Div(title, className="lf-st"),
                    html.Div(sub, className="dim small"),
                ]
            ),
        ],
        className="lf-step",
    )


def _code_block(
    name: str, is_root: bool, start: dict | None, m: mf.Manifest
) -> html.Div:
    modes = MODES_ROOT if is_root else MODES_SUB
    role = "root" if is_root else "sub"
    mode = "branch" if is_root else "pinned"
    sha = None
    if start:  # a rerun: that exact commit
        mode, sha = "commit", start.get("sha")
    return html.Div(
        [
            html.Div(
                [
                    html.Div(name, className="lc-name"),
                    dcc.RadioItems(
                        id={"type": "lc-mode", "r": name, "role": role},
                        options=[{"label": lab, "value": v} for v, lab in modes],
                        value=mode,
                        className="seg",
                        inputClassName="seg-in",
                        labelClassName="seg-opt",
                    ),
                ],
                className="lc-head",
            ),
            html.Div(
                [
                    dcc.Dropdown(
                        id={"type": "lc-ref", "r": name, "role": role},
                        clearable=False,
                        placeholder="Loading…",
                        className="lc-dd",
                        optionHeight=52,
                        maxHeight=420,
                    ),
                    dcc.Dropdown(
                        id={"type": "lc-commit", "r": name, "role": role},
                        clearable=False,
                        placeholder="Commit",
                        className="lc-dd",
                        optionHeight=52,
                        maxHeight=420,
                    ),
                ],
                id={"type": "lc-refbox", "r": name, "role": role},
                className="lc-pick",
            ),
            html.Div(
                dcc.Input(
                    id={"type": "lc-sha", "r": name, "role": role},
                    type="text",
                    placeholder="commit id (at least 7 characters)",
                    className="lf-text mono",
                    debounce=True,
                    value=sha,
                ),
                id={"type": "lc-shabox", "r": name, "role": role},
                style={"display": "none"},
            ),
            dcc.Store(id={"type": "lc-value", "r": name, "role": role}),
            html.Div(
                id={"type": "lc-card", "r": name, "role": role}, className="lc-card"
            ),
        ],
        className="card lf-card lc-block",
    )


def _bench_card(name: str, b: mf.Benchmark, m: mf.Manifest, rerun) -> html.Div:
    body: list = []
    if b.bags:
        opts = []
        for bag in m.bags(b.bags):
            info = mf.bag_info(m.bag_path(b.bags, bag))
            missing = [
                t for t in b.needs_topics if info.topics and t not in info.topics
            ]
            meta = " · ".join(
                x
                for x in (
                    f"{info.duration_s:.0f} s" if info.duration_s else None,
                    datetime.fromtimestamp(info.recorded).strftime("%d %b %Y")
                    if info.recorded
                    else None,
                    size(info.size_bytes),
                    f"no {', '.join(missing)}" if missing else None,
                )
                if x
            )
            opts.append(
                {
                    "label": html.Span(
                        [
                            html.Span(bag, className="bag-n"),
                            html.Span(meta, className="bag-m"),
                        ],
                        className="bag",
                    ),
                    "value": bag,
                    "disabled": bool(missing),
                }
            )
        body.append(
            html.Div(
                [
                    html.Div(
                        [
                            html.Span("Bags", className="lf-label"),
                            html.Button(
                                "All",
                                id={"type": "lf-bags-all", "b": name},
                                className="btn small ghost",
                            ),
                            html.Button(
                                "None",
                                id={"type": "lf-bags-none", "b": name},
                                className="btn small ghost",
                            ),
                            html.Span(
                                str(m.bag_dirs[b.bags]), className="dim small bag-dir"
                            ),
                        ],
                        className="bag-head",
                    ),
                    dcc.Checklist(
                        id={"type": "lf-bags", "b": name},
                        options=opts,
                        value=[],
                        className="bag-list",
                    )
                    if opts
                    else html.Div("No bags in that folder.", className="callout"),
                ]
            )
        )
    if b.parts:
        body.append(
            L._field(
                "Parts",
                dcc.Checklist(
                    id={"type": "lf-only", "b": name},
                    options=[{"label": f" {p}", "value": p} for p in b.parts],
                    value=list(b.parts),
                    inline=True,
                    className="lf-inline",
                ),
            )
        )
    body.append(
        L._field(
            "Repeats",
            dcc.Input(
                id={"type": "lf-rep", "b": name},
                type="number",
                min=1,
                step=1,
                value=1,
                className="lf-num",
            ),
            "the same job again, to see the spread",
        )
    )
    return html.Div(
        [
            dcc.Checklist(
                id={"type": "lf-on", "b": name},
                options=[
                    {
                        "label": html.Span(
                            [
                                html.B(b.title),
                                html.Span(b.description, className="dim small lf-desc"),
                            ]
                        ),
                        "value": "on",
                    }
                ],
                value=[],
                className="lf-title",
            ),
            html.Div(
                body,
                id={"type": "lf-body", "b": name},
                className="lf-body",
                style={"display": "none"},
            ),
        ],
        className="card lf-card",
    )


def _yaml_card(m: mf.Manifest, start: str) -> html.Details:
    ref = []
    for name, b in m.benchmarks.items():
        rows = [
            html.Tr(
                [
                    html.Td(html.Code(f"benchmarks.{name}.settings.{s.name}")),
                    html.Td(s.type),
                    html.Td(s.help),
                ]
            )
            for s in b.settings.values()
        ]
        rows += [
            html.Tr(
                [
                    html.Td(html.Code(f"pipeline.{c}.<parameter>")),
                    html.Td(""),
                    html.Td(f"overrides for the {p} part"),
                ]
            )
            for p, c in b.parts.items()
            if c
        ]
        ref.append(html.Table(html.Tbody(rows), className="mt compact lf-ref"))
    presets = m.preset_names()
    return html.Details(
        [
            html.Summary("Open the YAML editor"),
            html.Div(
                [
                    html.Span("Start from", className="dim small"),
                    *[
                        html.Button(
                            p, id={"type": "lf-preset", "p": p}, className="btn small"
                        )
                        for p in presets
                    ],
                    dcc.Upload(
                        html.Button("Upload a file…", className="btn small ghost"),
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
                value=start,
            ),
            html.Details([html.Summary("What can be set"), *ref], className="small"),
        ],
        className="card lf-card lf-adv",
        open=bool(start),
    )


_YAML_HINT = """# Optional. For example:
benchmarks:
  sim_bag:
    settings: {max_frames: 200}
pipeline:
  cone_detection: {residual_gate_mse: 0.05}
  slam_node: {motion_model: imu}
sweep:                  # one job per value
  pipeline.cone_detection.residual_gate_mse: [0.02, 0.05]
"""


# ======================================================================== the spec
def form_doc(
    on: dict[str, list[str]],
    bags: dict[str, list[str] | None],
    only: dict[str, list[str] | None],
    reps: dict[str, int | None],
    code: dict[str, str],
    image: str | None,
    name: str | None,
    notes: str | None,
) -> dict[str, Any]:
    """The page's choices as a spec (by benchmark name: not every benchmark has bags or parts)."""
    m = L.launcher().manifest
    doc: dict[str, Any] = {}
    benches = {}
    for b, is_on in on.items():
        bench = m.benchmarks[b]
        bg, parts, rep = bags.get(b), only.get(b), reps.get(b)
        if not is_on:
            continue
        entry: dict[str, Any] = {"bags": list(bg or [])} if bench.bags else {}
        if bench.parts and parts is not None and set(parts) != set(bench.parts):
            entry["only"] = list(parts)
        if rep and int(rep) > 1:
            entry["repeats"] = int(rep)
        benches[b] = entry
    if benches:
        doc["benchmarks"] = benches
    code = dict(code)
    if image and image.strip():
        code["image"] = image.strip()
    if code:
        doc["code"] = code
    if name and name.strip():
        doc["name"] = name.strip()
    if notes and notes.strip():
        doc["notes"] = notes.strip()
    return doc


def build(
    yaml_text: str | None, form: dict[str, Any]
) -> tuple[dict | None, list[str], list[str], list[str]]:
    """(the spec to launch or None, problems, notes, form keys that overrode the YAML)."""
    m = L.launcher().manifest
    notes: list[str] = []
    try:
        ydoc = yaml.safe_load(yaml_text or "") or {}
    except yaml.YAMLError as e:
        return None, [f"YAML: {e}"], notes, []
    if not isinstance(ydoc, dict):
        return None, ["The YAML must be a mapping (key: value)."], notes, []
    if "code" in ydoc:
        ydoc = {k: v for k, v in ydoc.items() if k != "code"}
        notes.append("The YAML's code: is ignored; the Code section picks it.")
    merged = sp.merge(ydoc, form)
    if not merged.get("benchmarks"):
        return None, ["Tick a benchmark."], notes, []
    try:
        s = sp.validate(merged, m)
    except sp.SpecError as e:
        return None, str(e).splitlines(), notes, []
    over = L.overridden(ydoc, {k: v for k, v in form.items() if k != "code"})
    return s, [], notes, over


# ======================================================================== callbacks
def register(app: Dash) -> None:
    # ------------------------------------------------------------ code pickers
    # Registered twice: for the root repository, and for submodules, whose lists and
    # "as pinned" commit follow the root commit picked (separate, so there is no cycle).
    for role in ("root", "sub"):
        _register_picker(app, role)

    # ------------------------------------------------------------ benchmarks
    @app.callback(
        Output({"type": "lf-body", "b": MATCH}, "style"),
        Input({"type": "lf-on", "b": MATCH}, "value"),
    )
    def open_card(on):
        return {} if on else {"display": "none"}

    @app.callback(
        Output({"type": "lf-bags", "b": MATCH}, "value"),
        Input({"type": "lf-bags-all", "b": MATCH}, "n_clicks"),
        Input({"type": "lf-bags-none", "b": MATCH}, "n_clicks"),
        State({"type": "lf-bags", "b": MATCH}, "options"),
        prevent_initial_call=True,
    )
    def all_or_none(_a, _n, opts):
        if ctx.triggered_id and ctx.triggered_id["type"] == "lf-bags-none":
            return []
        return [o["value"] for o in opts or [] if not o.get("disabled")]

    # ------------------------------------------------------------ YAML
    @app.callback(
        Output("lf-yaml", "value"),
        Input({"type": "lf-preset", "p": ALL}, "n_clicks"),
        Input("lf-upload", "contents"),
        prevent_initial_call=True,
    )
    def load_yaml(_clicks, upload):
        t = ctx.triggered_id
        if t == "lf-upload" and upload:
            _, data = upload.split(",", 1)
            return base64.b64decode(data).decode("utf-8", errors="replace")
        if isinstance(t, dict) and t.get("type") == "lf-preset" and any(_clicks or []):
            p = L.launcher().manifest.preset(t["p"])
            return p.read_text() if p else no_update
        return no_update

    # ------------------------------------------------------------ summary + launch
    form_inputs = [
        Input({"type": "lf-on", "b": ALL}, "value"),
        Input({"type": "lf-bags", "b": ALL}, "value"),
        Input({"type": "lf-only", "b": ALL}, "value"),
        Input({"type": "lf-rep", "b": ALL}, "value"),
        Input({"type": "lc-value", "r": ALL, "role": ALL}, "data"),
        Input("lf-image", "value"),
        Input("lf-name", "value"),
        Input("lf-notes", "value"),
        Input("lf-yaml", "value"),
        State({"type": "lf-on", "b": ALL}, "id"),
        State({"type": "lc-value", "r": ALL, "role": ALL}, "id"),
        State({"type": "lf-bags", "b": ALL}, "id"),
        State({"type": "lf-only", "b": ALL}, "id"),
        State({"type": "lf-rep", "b": ALL}, "id"),
    ]

    def _by(ids, values):
        return {i["b"]: v for i, v in zip(ids, values, strict=True)}

    def _form(
        on,
        bags,
        only,
        reps,
        codes,
        image,
        name,
        notes,
        on_ids,
        code_ids,
        bag_ids,
        only_ids,
        rep_ids,
    ):
        code = {}
        missing = []
        for i, v in zip(code_ids, codes, strict=True):
            v = v or {}
            if v.get("spec"):
                code[i["r"]] = v["spec"]
            elif v.get("mode") != "pinned":
                missing.append(i["r"])
        doc = form_doc(
            _by(on_ids, on),
            _by(bag_ids, bags),
            _by(only_ids, only),
            _by(rep_ids, reps),
            code,
            image,
            name,
            notes,
        )
        return doc, missing

    @app.callback(
        Output("lf-summary", "children"),
        Output("lf-go", "disabled"),
        Output("lf-go", "children"),
        *form_inputs,
    )
    def summary(on, bags, only, reps, codes, image, name, notes, text, *ids):
        form, missing = _form(on, bags, only, reps, codes, image, name, notes, *ids)
        spec, problems, extra, over = build(text, form)
        problems = [f"Pick a version of {r}." for r in missing] + problems
        out: list = [_code_summary(codes, ids[1])]
        jobs: list[sp.Job] = []
        if spec is not None:
            try:
                jobs = sp.expand(spec, L.launcher().manifest)
            except sp.SpecError as e:
                problems += str(e).splitlines()
        if jobs:
            rows = [
                html.Tr(
                    [html.Td(i), html.Td(_job_cell(j)), html.Td(html.Code(j.spec_id))]
                )
                for i, j in enumerate(jobs[:MAX_JOBS_SHOWN], 1)
            ]
            out += [
                html.Div(
                    [
                        html.B(f"{len(jobs)} job{'s' if len(jobs) != 1 else ''}"),
                        html.Span(", one at a time, in this order", className="dim"),
                    ],
                    className="lf-sum-jobs",
                ),
                html.Div(
                    html.Table(html.Tbody(rows), className="mt compact lf-jobs"),
                    className="lf-sum-jobs-wrap",
                ),
            ]
            if len(jobs) > MAX_JOBS_SHOWN:
                out.append(
                    html.Div(
                        f"… and {len(jobs) - MAX_JOBS_SHOWN} more",
                        className="dim small",
                    )
                )
        if problems:
            out.append(html.Ul([html.Li(p) for p in problems], className="lf-errors"))
        for n in extra:
            out.append(html.Div(n, className="dim small"))
        if over:
            out.append(
                html.Div(
                    ["Your picks replaced the YAML's ", ", ".join(over), "."],
                    className="dim small",
                )
            )
        if spec is not None and not problems:
            out.append(
                html.Details(
                    [
                        html.Summary("The spec"),
                        html.Pre(
                            yaml.safe_dump(spec, sort_keys=False), className="lf-pre"
                        ),
                    ],
                    className="small",
                )
            )
        ok = bool(jobs) and not problems
        label = (
            f"Launch {len(jobs)} job{'s' if len(jobs) != 1 else ''}" if ok else "Launch"
        )
        return out, not ok, label

    @app.callback(
        Output("lf-result", "children"),
        Output("url", "pathname", allow_duplicate=True),
        Input("lf-go", "n_clicks"),
        *[State(i.component_id, i.component_property) for i in form_inputs],
        prevent_initial_call=True,
    )
    def go(n, on, bags, only, reps, codes, image, name, notes, text, *ids):
        if not n:
            return no_update, no_update
        form, missing = _form(on, bags, only, reps, codes, image, name, notes, *ids)
        spec, problems, _, _ = build(text, form)
        if spec is None or problems or missing:
            return html.Div(
                "Fix the problems above first.", className="callout"
            ), no_update
        lz = L.launcher()
        try:
            p = su.plan(lz.manifest, spec)
        except (co.CodeError, sp.SpecError) as e:
            return html.Div(["Cannot launch: ", str(e)], className="callout"), no_update
        try:
            su.submit(lz.queue, p, trigger="web", requested_by=L.current_user())
        except Exception as e:  # noqa: BLE001  (the queue database: say so on the page)
            return html.Div(
                ["Could not queue the jobs: ", f"{type(e).__name__}: {e}"],
                className="callout",
            ), no_update
        return no_update, "/launch/queue"


# ======================================================================== code pickers
def _id(kind: str, role: str, r=MATCH) -> dict:
    return {"type": kind, "r": r, "role": role}


ROOT_VALUES = {"type": "lc-value", "r": ALL, "role": "root"}


def _root_sha(values: list | None) -> str | None:
    return next(
        ((v or {}).get("sha") for v in values or [] if (v or {}).get("sha")), None
    )


def _register_picker(app: Dash, role: str) -> None:
    # submodules follow the root commit through one plain store: Dash doesn't re-run a
    # MATCH callback when the trigger is a wildcard (ALL) input
    follow = [Input("lc-root-sha", "data")] if role == "sub" else []
    if role == "root":

        @app.callback(Output("lc-root-sha", "data"), Input(ROOT_VALUES, "data"))
        def root_sha(values):
            return _root_sha(values)

    @app.callback(
        Output(_id("lc-ref", role), "options"),
        Output(_id("lc-ref", role), "value"),
        Output(_id("lc-ref", role), "placeholder"),
        Output(_id("lc-refbox", role), "style"),
        Output(_id("lc-shabox", role), "style"),
        Input(_id("lc-mode", role), "value"),
        Input("lc-tick", "n_intervals"),
        *follow,
        State(_id("lc-ref", role), "value"),
        State(_id("lc-mode", role), "id"),
    )
    def ref_options(mode, _tick, *rest):
        root_sha = rest[0] if role == "sub" else None
        current, ident = rest[-2], rest[-1]
        return _ref_options(ident["r"], mode, current, root_sha)

    @app.callback(
        Output(_id("lc-commit", role), "options"),
        Output(_id("lc-commit", role), "value"),
        Input(_id("lc-ref", role), "value"),
        *follow,
        State(_id("lc-mode", role), "value"),
        State(_id("lc-ref", role), "id"),
    )
    def commit_options(ref, *rest):
        root_sha = rest[0] if role == "sub" else None
        mode, ident = rest[-2], rest[-1]
        mi = _mirror(ident["r"], root_sha)
        if not ref or mi is None or mode not in ("branch", "pr") or not mi.ensure():
            return [], None
        rev = f"refs/pull/{ref[1:]}/head" if mode == "pr" else f"refs/heads/{ref}"
        commits = mi.commits(rev)
        opts = [
            _opt(
                c.sha,
                ("Latest · " if i == 0 else "") + c.subject,
                f"{c.short} · {c.author} · {ago(c.when)}",
            )
            for i, c in enumerate(commits)
        ]
        return opts, (commits[0].sha if commits else None)

    @app.callback(
        Output(_id("lc-value", role), "data"),
        Input(_id("lc-mode", role), "value"),
        Input(_id("lc-ref", role), "value"),
        Input(_id("lc-commit", role), "value"),
        Input(_id("lc-sha", role), "value"),
        *follow,
        State(_id("lc-mode", role), "id"),
    )
    def code_value(mode, ref, sha, typed, *rest):
        """What the spec gets: ``<branch or #PR>@<commit>``, a commit, or nothing (as pinned)."""
        root_sha = rest[0] if role == "sub" else None
        ident = rest[-1]
        if mode == "pinned":
            return {"mode": "pinned"}
        if mode == "commit":
            mi = _mirror(ident["r"], root_sha)
            c = mi.resolve((typed or "").strip()) if mi and mi.ensure() else None
            if c is None:
                return {
                    "mode": "commit",
                    "error": "Not a commit of this repository (new commits show up within a minute)."
                    if typed
                    else None,
                }
            return {"mode": "commit", "spec": c.sha, "sha": c.sha}
        if not ref or not sha:
            return {"mode": mode}
        return {"mode": mode, "spec": f"{ref}@{sha}", "sha": sha, "ref": ref}

    @app.callback(
        Output(_id("lc-card", role), "children"),
        Input(_id("lc-value", role), "data"),
        *follow,
        State(_id("lc-value", role), "id"),
    )
    def code_card(v, *rest):
        root_sha = rest[0] if role == "sub" else None
        return _card(rest[-1]["r"], v or {}, root_sha)


def _ref_options(name: str, mode: str, current: str | None, root_sha: str | None):
    show, hide = {}, {"display": "none"}
    if mode in ("pinned", "commit"):
        return [], None, "", hide, show if mode == "commit" else hide
    mi = _mirror(name, root_sha)
    if mi is None or not mi.ensure():
        err = (mi.error if mi else None) or "no remote URL for this repository"
        return [], None, f"Can't list: {err}", show, hide
    mi.refresh()
    if mode == "branch":
        refs = mi.branches()
        default = L.launcher().manifest.default_refs.get(name)
        first = [r for r in refs if r.name == default]
        refs = first + [r for r in refs if r.name != default]
        opts = [_opt(r.name, r.name, _commit_meta(r.commit)) for r in refs]
        value = (
            current
            if current in {o["value"] for o in opts}
            else (refs[0].name if refs else None)
        )
        return opts, value, "Branch", show, hide
    gh = rf.github_pulls(mi.url)
    prs = mi.pulls(gh)
    if gh is not None:
        prs = [p for p in prs if p.open]
    opts = [
        _opt(
            p.name,
            f"{p.name}  {p.title or p.commit.subject}",
            f"{(gh or {}).get(int(p.name[1:]), {}).get('author') or p.commit.author} · "
            f"updated {ago(p.commit.when)} · head {p.commit.short}",
        )
        for p in prs
    ]
    value = current if current in {o["value"] for o in opts} else None
    hint = "Pull request" + (
        "" if gh is not None else " (GitHub unreachable: all PRs, newest first)"
    )
    return opts, value, hint, show, hide


def _job_cell(j: sp.Job) -> html.Div:
    """One job on one line: the bag (cut with … when long, full on hover), then the rest."""
    meta = [j.benchmark] + [f"{k.rsplit('.', 1)[-1]}={v}" for k, v in j.point.items()]
    if j.repeat > 1:
        meta.append(f"repeat {j.repeat}")
    return html.Div(
        [
            html.Div(j.bag or j.benchmark, className="lf-job-t", title=j.bag or ""),
            html.Div(" · ".join(meta), className="lf-job-m"),
        ],
        className="lf-job",
    )


# ======================================================================== code cards
def _card(name: str, v: dict, root_sha: str | None) -> list:
    mi = _mirror(name, root_sha)
    if v.get("error"):
        return [html.Div(v["error"], className="lc-warn")]
    if v.get("mode") == "pinned":
        m = L.launcher().manifest
        root = _root_name(m)
        rmi = _mirror(root)
        if not root_sha or rmi is None:
            return [
                html.Div(
                    f"The commit {root} pins, once {root} is picked.",
                    className="dim small",
                )
            ]
        rel = _rel(m, name)
        pinned = rmi.pin(root_sha, rel)
        if not pinned:
            return [
                html.Div(f"{root} {root_sha[:7]} pins no {rel}.", className="lc-warn")
            ]
        c = mi.commit(pinned) if mi and mi.ensure() else None
        where = f"pinned by {root} {root_sha[:7]}"
        if mi is not None and rf.github_slug(mi.url):
            where += f" · {rf.github_slug(mi.url)}"
        return [_commit_line(c, pinned, mi, where)]
    sha = v.get("sha")
    if not sha:
        return []
    c = mi.commit(sha) if mi else None
    return [_commit_line(c, sha, mi, None)]


def _commit_line(
    c: rf.Commit | None, sha: str, mi: rf.Mirror | None, why: str | None
) -> html.Div:
    link = rf.web_url(mi.url, sha) if mi else None
    bits = [
        html.Code(sha[:7], className="lc-sha"),
        html.Span(
            c.subject if c else "(commit not in the copy yet)", className="lc-subj"
        ),
    ]
    meta = " · ".join(
        x for x in ((c.author if c else None), (ago(c.when) if c else None), why) if x
    )
    return html.Div(
        [
            html.Div(bits, className="lc-line"),
            html.Div(
                [
                    html.Span(meta, className="dim small"),
                    html.A("GitHub ↗", href=link, target="_blank", className="small")
                    if link
                    else None,
                ],
                className="lc-meta",
            ),
        ]
    )


def _code_summary(codes: list[dict | None], ids: list[dict]) -> html.Div:
    rows = []
    for i, v in zip(ids, codes, strict=True):
        v = v or {}
        if v.get("mode") == "pinned":
            what = "as pinned"
        elif v.get("sha"):
            what = (f"{v['ref']} @ " if v.get("ref") else "") + v["sha"][:7]
        else:
            what = "—"
        rows.append(
            html.Div(
                [html.Span(i["r"], className="dim"), html.Span(what, className="mono")],
                className="lf-sum-code",
            )
        )
    return html.Div(rows, className="lf-sum-codes")
