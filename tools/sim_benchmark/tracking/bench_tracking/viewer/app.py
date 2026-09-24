"""bench-view: a benchmark viewer on top of MLflow.

    uv run --extra viewer bench-view                      # http://127.0.0.1:8050, MLflow at $MLFLOW_TRACKING_URI
    uv run --extra viewer bench-view --uri http://mlflow.team:5000 --port 8050

MLflow keeps the data (runs, params, metrics, the ``bundle/`` artifact) and its
own UI. This app is a different way of looking at it:

1. the left navigation picks the kind of report (bag benchmarks, live runs, sim
   benchmarks, nightly, sweeps) and the part of it (summary, perception, SLAM…);
2. the selection bar says what is on screen. "Change…" and "+ Compare" open a
   run browser (runs by day, with their key numbers) where each run has *Open*
   and *+ Compare*. One run = one report; two or more = a comparison, shown
   overlaid or side by side;
3. the page shows full-width charts. Time charts share one playhead, driven by
   hovering, by the playback bar or by clicking the map, like a Foxglove layout.

Every state is in the URL (``/bag/slam?mode=side&r=…&vs=…``), so a view can be
shared as a link. See :mod:`.pages` for what each page shows and
``assets/sync.js`` for the playhead, which runs in the browser.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from urllib.parse import parse_qs

from dash import ALL, Dash, Input, Output, Patch, State, ctx, dcc, html, no_update

from . import pages as P
from . import replay as R
from . import sim as S
from .data import Catalog

CAT: Catalog | None = None


def cat() -> Catalog:
    assert CAT is not None
    return CAT


# ======================================================================== layout
def serve_layout():
    return html.Div(
        [
            dcc.Location(id="url", refresh=False),
            dcc.Store(id="sel"),
            dcc.Store(id="memory", storage_type="session", data={}),
            dcc.Store(id="cat-ver", data=0),
            dcc.Store(id="view"),
            dcc.Store(id="url-mirror"),
            dcc.Store(id="open-from-plot"),
            html.Nav(
                [
                    html.Div(
                        [
                            html.Div("IFSSIM Bench", className="brand"),
                            html.Div("benchmark viewer", className="sub"),
                        ],
                        className="brand-box",
                    ),
                    html.Div(id="nav"),
                    html.Div(
                        [
                            html.Button(
                                "↻ Reload runs",
                                id="refresh",
                                className="btn small ghost",
                                title="Re-read runs from MLflow",
                            ),
                            html.Div(id="src", className="src"),
                        ],
                        className="nav-foot",
                    ),
                ],
                className="nav",
            ),
            html.Main(
                [
                    html.Div(id="page-head", className="page-head"),
                    html.Div(
                        [
                            html.Div(id="selbar", className="selbar"),
                            html.Div(
                                html.Div(id="browser", className="browser"),
                                id="browser-wrap",
                                className="browser-wrap",
                            ),
                        ],
                        className="sel-area",
                    ),
                    dcc.Loading(
                        html.Div(id="content", className="content"),
                        type="dot",
                        delay_show=300,
                        color="#2563eb",
                        overlay_style={"visibility": "visible", "opacity": 0.45},
                    ),
                ],
                className="main",
            ),
            html.Div(
                html.Div(
                    [
                        html.Button("✕ close", id="modal-close", className="btn close"),
                        dcc.Graph(
                            id="modal-graph",
                            style={"height": "100%"},
                            config={"displaylogo": False},
                        ),
                    ],
                    className="box",
                ),
                id="modal",
                className="modal",
            ),
            html.Div(id="toast", className="toast"),
        ],
        className="app",
    )


# ======================================================================== selection state
def _q(search: str | None) -> dict:
    q = parse_qs((search or "").lstrip("?"))
    return {
        "mode": (q.get("mode") or [None])[0],
        "r": (q.get("r") or [None])[0],
        "vs": [x for x in (q.get("vs") or [""])[0].split(",") if x],
        "scen": (q.get("scen") or [None])[0],
    }


def _sanitize(c: Catalog, kind: str, st: dict, valid: list[str]) -> dict:
    r = st.get("r") if st.get("r") in valid else P.default_report(c, kind)
    vs = list(dict.fromkeys(x for x in (st.get("vs") or []) if x in valid and x != r))[
        : P.MAX_COMPARE
    ]
    mode = st.get("mode") if st.get("mode") in ("overlay", "side") else "overlay"
    return {"kind": kind, "mode": mode, "r": r, "vs": vs, "scen": st.get("scen") or "*"}


def _apply(st: dict, pid: dict, val) -> dict:
    """One user action on the selection."""
    st = dict(st, vs=list(st["vs"]))
    t, key = pid.get("type"), pid.get("key")
    if t == "open" and val:
        st["r"], st["vs"] = key, []
    elif t == "add" and val:
        if key in st["vs"]:
            st["vs"].remove(key)
        elif key != st["r"]:
            st["vs"].append(key)
            if len(st["vs"]) > 1 and st["mode"] == "side":
                # side by side shows two; a third run means overlay
                st["mode"] = "overlay"
    elif t == "rm" and val:
        if key == st["r"] and st["vs"]:
            st["r"], st["vs"] = st["vs"][0], st["vs"][1:]
        elif key in st["vs"]:
            st["vs"].remove(key)
    elif t == "swap" and val and st["vs"]:
        st["r"], st["vs"][0] = st["vs"][0], st["r"]
    elif t == "cmode" and val:
        st["mode"] = val
    elif t == "scen" and val:
        st["scen"] = val
    return st


# ======================================================================== app + callbacks
def build_app(catalog: Catalog) -> Dash:
    global CAT
    CAT = catalog
    app = Dash(
        __name__,
        title="IFSSIM Bench",
        suppress_callback_exceptions=True,
        assets_folder=str(Path(__file__).parent / "assets"),
    )
    app.layout = serve_layout

    # ---------------------------------------------------------------- the one place that owns the selection
    @app.callback(
        Output("sel", "data"),
        Output("memory", "data"),
        Input("url", "pathname"),
        Input("cat-ver", "data"),
        Input({"type": "open", "key": ALL}, "n_clicks"),
        Input({"type": "add", "key": ALL}, "n_clicks"),
        Input({"type": "rm", "key": ALL}, "n_clicks"),
        Input({"type": "swap", "i": ALL}, "n_clicks"),
        Input({"type": "cmode", "i": ALL}, "value"),
        Input({"type": "scen", "i": ALL}, "value"),
        Input("open-from-plot", "data"),
        State("url", "search"),
        State("sel", "data"),
        State("memory", "data"),
    )
    def controller(
        pathname, _ver, _o, _a, _r, _s, _m, _sc, from_plot, search, sel, memory
    ):
        c = cat()
        kind, _ = P.parse_path(pathname)
        valid = [o["value"] for o in P.options(c, kind)]
        memory = dict(memory or {})
        if not sel or sel.get("kind") != kind:
            # a new page: a link wins over what this tab last had open there; anything it leaves out is kept
            q = {k: val for k, val in _q(search).items() if val}
            st = dict(memory.get(kind) or {}) | q
            if q.get("r") in valid and "vs" not in q:
                st["vs"] = []
        else:
            st = dict(sel)
            for t in ctx.triggered or []:
                pid, val = t["prop_id"].rsplit(".", 1)[0], t["value"]
                if pid.startswith("{"):
                    st = _apply(st, json.loads(pid), val)
                elif pid == "open-from-plot" and val and val.get("key") in valid:
                    st["r"], st["vs"] = val["key"], []
        st = _sanitize(c, kind, st, valid)
        memory[kind] = st
        if sel == st:
            return no_update, no_update
        return st, memory

    @app.callback(
        Output("selbar", "children"),
        Output("browser", "children"),
        Input("sel", "data"),
        Input("cat-ver", "data"),
        State("url", "pathname"),
    )
    def bar(sel, _ver, pathname):
        if not sel or not sel.get("r"):
            return [], []
        c = cat()
        kind, section = P.parse_path(pathname)
        if sel["kind"] != kind:
            return no_update, no_update
        v = P.make_view(c, kind, section, sel["mode"], sel["r"], sel["vs"], sel["scen"])
        return P.selection_bar(v, c, sel["mode"]), P.browser(c, kind, v.r, v.vs, v.col)

    # ---------------------------------------------------------------- the page
    @app.callback(
        Output("content", "children"),
        Output("view", "data"),
        Output("page-head", "children"),
        Input("url", "pathname"),
        Input("sel", "data"),
    )
    def render(pathname, sel):
        c = cat()
        kind, section = P.parse_path(pathname)
        k = P.KINDS[kind]
        head = [
            html.Div(
                [
                    html.Span(k.title, className="crumb"),
                    html.Span(" › ", className="dim"),
                    html.Span(dict(k.sections)[section]),
                ],
                className="title",
            ),
            html.Div(k.blurb, className="blurb"),
        ]
        if not P.options(c, kind):
            return P.note(f"No {k.title.lower()} in MLflow yet."), None, head
        if not sel or sel.get("kind") != kind or not sel.get("r"):
            # the controller is about to set the selection
            return no_update, no_update, head
        v = P.make_view(c, kind, section, sel["mode"], sel["r"], sel["vs"], sel["scen"])
        try:
            body = P.page(c, v)
        except Exception:  # noqa: BLE001  (show the error in place instead of a blank page)
            import traceback

            body = [
                html.Pre(
                    traceback.format_exc(),
                    className="callout",
                    style={"whiteSpace": "pre-wrap"},
                )
            ]
        return body, v.to_store(), head

    @app.callback(
        Output("nav", "children"),
        Output("src", "children"),
        Input("url", "pathname"),
        Input("cat-ver", "data"),
    )
    def nav(pathname, _ver):
        c = cat()
        kind, section = P.parse_path(pathname)
        out = []
        for key, k in P.KINDS.items():
            n = len(P.options(c, key))
            active = key == kind
            out.append(
                dcc.Link(
                    [html.Span(k.title), html.Span(str(n), className="count")],
                    href=f"/{key}",
                    className="nav-kind" + (" active" if active else ""),
                )
            )
            if active:
                out.append(
                    html.Div(
                        [
                            dcc.Link(
                                lab,
                                href=f"/{key}/{s}",
                                className="nav-sec"
                                + (" active" if s == section else ""),
                            )
                            for s, lab in k.sections
                        ],
                        className="nav-secs",
                    )
                )
        src = [
            f"{len(c.rows)} runs · read {c.loaded_at:%H:%M} UTC · ",
            html.A("MLflow ↗", href=c.uri, target="_blank"),
        ]
        return out, src

    # the URL mirrors the selection (replaceState: picking runs does not flood the back button)
    app.clientside_callback(
        """function(sel, path){
            if(!sel || !sel.r) return window.dash_clientside.no_update;
            const q = new URLSearchParams();
            if(sel.vs && sel.vs.length){ if(sel.mode === 'side') q.set('mode', 'side'); q.set('r', sel.r);
                                         q.set('vs', sel.vs.join(',')); }
            else q.set('r', sel.r);
            if(sel.scen && sel.scen !== '*') q.set('scen', sel.scen);
            const url = (path && path !== '/' ? path : '/bag') + '?' + q.toString();
            if(window.location.pathname + window.location.search !== url) window.history.replaceState(null, '', url);
            return url;
        }""",
        Output("url-mirror", "data"),
        Input("sel", "data"),
        Input("url", "pathname"),
    )

    @app.callback(
        Output("cat-ver", "data"),
        Output("toast", "children"),
        Input("refresh", "n_clicks"),
        Input({"type": "pin", "i": ALL}, "n_clicks"),
        State("cat-ver", "data"),
        State("sel", "data"),
        prevent_initial_call=True,
    )
    def reload_or_pin(_n1, pins, ver, sel):
        c = cat()
        if ctx.triggered_id == "refresh":
            c.refresh()
            msg = f"Re-read {len(c.rows)} runs from MLflow."
        else:
            if not any(pins or []) or not sel:
                return no_update, no_update
            kind, r = sel["kind"], sel["r"]
            if kind == "sim":
                aggs = [a for a in c.collection("matrix") if a.commit == r]
                for a in aggs:
                    c.pin_baseline(a.run_id)
                msg = f"Commit {r} is now the team baseline for {len(aggs)} scenarios (MLflow tag bench.pinned_baseline)."
            else:
                c.pin_baseline(r)
                msg = f"{P.names(c, kind, [r])[r]} is now the team baseline (MLflow tag bench.pinned_baseline)."
        return (ver or 0) + 1, _toast(msg)

    # ---------------------------------------------------------------- full-screen chart
    app.clientside_callback(
        """function(clicks, figs, ids){
            const nu = window.dash_clientside.no_update;
            const t = window.dash_clientside.callback_context.triggered;
            if(!t.length || !t[0].value) return [nu, nu];
            const id = JSON.parse(t[0].prop_id.split('.')[0]);
            const i = ids.findIndex(x => x.index === id.index);
            if(i < 0) return [nu, nu];
            const fig = JSON.parse(JSON.stringify(figs[i]));
            fig.layout = fig.layout || {};
            fig.layout.height = Math.round(window.innerHeight * 0.9);
            fig.layout.width = null; fig.layout.autosize = true; fig.layout.showlegend = true;
            return [fig, 'modal open'];
        }""",
        Output("modal-graph", "figure"),
        Output("modal", "className"),
        Input({"type": "expand", "index": ALL}, "n_clicks"),
        State({"type": "g", "index": ALL}, "figure"),
        State({"type": "g", "index": ALL}, "id"),
        prevent_initial_call=True,
    )
    app.clientside_callback(
        "function(n){ return 'modal'; }",
        Output("modal", "className", allow_duplicate=True),
        Input("modal-close", "n_clicks"),
        prevent_initial_call=True,
    )

    app.clientside_callback(
        "function(v){ return (v && v.length) ? 'only-changes' : ''; }",
        Output("sc-wrap", "className"),
        Input("sc-only", "value"),
    )

    register_replay_callbacks(app)
    register_sim_callbacks(app)
    return app


def _toast(msg: str) -> html.Div:
    return html.Div(msg, key=str(time.time()), className="toast-msg")


# ======================================================================== bag replay: panel stack + map
def _view_runs(view: dict):
    """Played bundles of the current view (labelled as the page labels them) and the reference bundle."""
    c = cat()
    v = P.View.from_store(view)
    ids = [k for k in v.keys if k in c.rows and c.rows[k].status == "finished"]
    B = c.bundles(ids)
    for b in B:
        b.display = v.names.get(b.run_id)  # type: ignore[attr-defined]
    ref = (
        next((b for b in B if b.run_id == v.ref), None) if v.mode == "overlay" else None
    )
    return v, B, ref


def register_replay_callbacks(app: Dash) -> None:
    # hovering, the playhead, zoom windows and map clicks all run in the browser (assets/sync.js);
    # the server only rebuilds when the set of panels or map options changes
    @app.callback(
        Output("lane-stack", "children"),
        Input("lanes", "value"),
        Input("tl-opts", "value"),
        State("view", "data"),
        prevent_initial_call=True,
    )
    def lanes_changed(lanes, opts, view):
        v, B, ref = _view_runs(view)
        order = [la.key for la in R.LANES]
        return P.lane_cards(
            v,
            B,
            ref,
            sorted(lanes or ["speed"], key=lambda k: (k != "events", order.index(k))),
            smooth="smooth" in (opts or []),
        )

    @app.callback(
        Output("mapbox", "children"),
        Input("tl-opts", "value"),
        State("view", "data"),
        prevent_initial_call=True,
    )
    def map_opts(opts, view):
        v, B, ref = _view_runs(view)
        return P.map_box(
            B,
            ref,
            v.col,
            all_maps="maps" in (opts or []),
            show_events="events" in (opts or []),
        )


# ======================================================================== sim: track, nightly trend, sweep
def register_sim_callbacks(app: Dash) -> None:
    @app.callback(
        Output("trk-body", "children"),
        Input("trk-scen", "value"),
        Input("trk-metric", "value"),
        State("view", "data"),
    )
    def track(scen, metric, view):
        if not view or not scen:
            return no_update
        return P.track_body(cat(), P.View.from_store(view), scen, metric)

    @app.callback(
        Output({"type": "g", "index": "trk-map"}, "figure", allow_duplicate=True),
        Input({"type": "g", "index": "trk-prof"}, "hoverData"),
        State("trk-store", "data"),
        prevent_initial_call=True,
    )
    def trk_cursor(hover, st):
        if not hover or not st or st.get("cursor") is None or st.get("cl") is None:
            return no_update
        s = hover["points"][0].get("x")
        if not isinstance(s, (int, float)):
            return no_update
        import numpy as np

        x, y = S.xy_at(tuple(np.array(a) for a in st["cl"]), [s])
        p = Patch()
        p["data"][st["cursor"]]["x"] = [float(x[0])]
        p["data"][st["cursor"]]["y"] = [float(y[0])]
        return p

    @app.callback(
        Output({"type": "g", "index": "nt-trend"}, "figure"),
        Input("nt-metrics", "value"),
        State("view", "data"),
        prevent_initial_call=True,
    )
    def nt_metrics(metrics, view):
        c, v = cat(), P.View.from_store(view)
        base = next((c.rows[k] for k in v.base if k in c.rows), None)
        return P.tidy(
            S.nightly_trend(
                c.collection("nightly"),
                metrics or P.NIGHTLY_METRICS[:1],
                base,
                v.keys,
                v.col,
            ),
            legend=False,
            top=40,
        )

    @app.callback(
        Output({"type": "g", "index": "sw-grid"}, "figure"),
        Input("sw-metric", "value"),
        State("view", "data"),
        prevent_initial_call=True,
    )
    def sw_metric(metric, view):
        return P.tidy(
            S.sweep_grid(
                cat().collection("sweep"), metric, P.View.from_store(view).keys
            ),
            legend=False,
        )

    # clicking a night or a trial opens its summary (one callback per graph: they are never on screen together)
    for gid in ("nt-trend", "sw-grid", "sw-pareto"):
        app.callback(
            Output("open-from-plot", "data", allow_duplicate=True),
            Output("url", "pathname", allow_duplicate=True),
            Input({"type": "g", "index": gid}, "clickData"),
            State("url", "pathname"),
            prevent_initial_call=True,
        )(open_from_plot)


def open_from_plot(click, pathname):
    if not click:
        return no_update, no_update
    cd = click["points"][0].get("customdata")
    rid = cd[0] if isinstance(cd, list) else cd
    if not isinstance(rid, str) or rid not in cat().rows:
        return no_update, no_update
    kind, _ = P.parse_path(pathname)
    return {"key": rid, "ts": time.time()}, f"/{kind}/summary"


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(
        prog="bench-view",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--uri",
        help="MLflow tracking URI (default $MLFLOW_TRACKING_URI or http://127.0.0.1:5005)",
    )
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8050)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args(argv)
    app = build_app(Catalog(args.uri))
    print(
        f"bench-view on http://{args.host}:{args.port}  (MLflow: {cat().uri}, {len(cat().rows)} runs)"
    )
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
