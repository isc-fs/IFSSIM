"""The run spec as a JSON Schema with UI hints: what the benchweb's form is generated from.

    python3 tools/sim_benchmark/run_spec.py --print-schema

One source for both launchers (plan "Benchweb v1", decision B4): the spec format is
``spec.py``, what each benchmark can set is the manifest (``bench.yaml``), and this file
adds only how the form shows them, in the order and groups of the sim's Benchmarks page
(``Plugins/FSDSPlugin/.../UI/SFSDSBenchPage.cpp``). The page registers the fields it sets
in ``SFSDSBenchPageFields.inl`` and ``tests/test_schema.py`` fails when the two drift.

Validation stays ``spec.validate`` (the backend's check): the schema's types, enums and
patterns are for the form, and a spec that passes them can still be refused (a bag that
isn't there, an override for a component no chosen benchmark uses).

Each form field carries ``x-ui``:

    group, order     where it goes (``x-ui.groups`` lists the groups, in page order)
    widget           how the page draws it (see WIDGETS)
    options_from     a list the backend fills at run time (see ``x-ui.lists``)
    options          labels of an enum's values, in page order
    needs            the rig part the field needs (``run_spec.py --rig``: latte_panda, hil,
                     vhil); option_needs: the same per value of an enum
    visible_if       {dotted path: value or [values]}: shown only then; "benchmark" in a
                     path stands for each benchmark that has that setting
    in_run           a board's firmware: shown when the board is in the run, i.e. on the
                     HIL bench (hil: true) or in one of these vHIL systems (vhil: [...])
    targets          the page's targets it is offered on (default: both)
    form: false      not on the form: it rides along from a preset or a file
    fill: backend    set by the backend, never by a person
    status: planned  accepted by spec.py, not used yet (plan B8); not on the form

Versions: ``x-schema-version`` goes up when a field is renamed, removed or changes
meaning; new fields and hints keep it. ``x-fingerprint`` changes with any change, so a
cached form can tell it is stale.
"""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Any

from . import spec as sp
from .manifest import Manifest

VERSION = 1
SCHEMA_ID = f"urn:ifssim:run-spec-form:{VERSION}"

TABS = [
    {"id": "run", "title": "Run"},
    {"id": "setup", "title": "Setup"},
]
GROUPS = [
    {"id": "what", "tab": "run", "title": "What to test"},
    {"id": "benchmark", "tab": "run", "title": "", "about": "the chosen benchmark's own choices"},
    {"id": "common", "tab": "run", "title": ""},
    {"id": "more", "tab": "run", "title": "More settings", "collapsed": True},
    {"id": "sim", "tab": "setup", "title": "Sim"},
    {"id": "pipeline", "tab": "setup", "title": "Pipeline"},
    {"id": "boards", "tab": "setup", "title": "ECU, AMS, uDV"},
]
WIDGETS = {
    "pick-one": "one choice among the enum's values (the page tests one benchmark at a time)",
    "multi-csv": "several of options_from, stored as 'a,b'; all lit (unset) or all_value is every one",
    "multi": "several of the values, a list; all lit (unset) is every one",
    "bags": "'all', or several of options_from",
    "count": "a whole number from 1; 1 is unset",
    "yes-no": "No (unset) or Yes",
    "default-yes-no": "Default (unset), Yes or No",
    "choice": "Default (unset) or one of the values",
    "text": "free text or a number; empty is unset",
    "code-ref": "a branch, PR, commit or snapshot of the repository (x-ui.repo); empty: the checkout's own",
    "override-files": "files of pipeline parameter changes (options_from), merged in the order picked",
    "board-mode": "None (unset) or one of the values",
}
LISTS = {
    "missions": "the missions in tools/sim_benchmark/missions.yaml (GET /api/bench/manifest: missions)",
    "mission_tracks": "each mission's own tracks, shown when no tracks are picked (manifest: mission_tracks)",
    "tracks": "the track CSVs in Content/tracks (manifest: tracks)",
    "bags:*": "the bags of that kind on the machine that runs it (manifest: bags.<kind>)",
    "overrides": "files in tools/sim_benchmark/specs/overrides (GET /api/bench/presets: overrides)",
    "refs:*": "the repository's branches and PRs (GET /api/bench/refs/<owner>/<repo>)",
}
CAPABILITIES = {
    "latte_panda": {"title": "LattePanda", "about": "answers on SSH (rig descriptor latte_panda)"},
    "hil": {"title": "HIL bench", "about": "real boards on IFS_HIL's bench (car-in-the-loop, not wired yet)"},
    "vhil": {"title": "vHIL", "about": "Docker with IFS-vHIL's image: emulated boards"},
}

# Settings the backend fills and no person sets (Mission Control points the mission
# benchmark at itself). bench_queue.py leaves them off the page.
BACKEND_FILLED = frozenset({"mc_url"})

# A setting's words on the form, by name (any benchmark); others: its name, and bench.yaml's help.
SETTING_WORDS: dict[str, tuple[str, str]] = {
    "laps": ("Laps", "For every mission. Empty: each mission's own."),
    "timeout_s": ("Time limit (s)", "Per run, for every mission. Empty: each mission's own."),
    "stack": ("Pipeline stack", "Its own: started from the code being tested. The running one: the stack that's already up."),
    "replace_stack": ("Stop the running stack", "Stop a running pipeline stack for the benchmark, and start it again after."),
    "panda_start": ("LattePanda: start", "The command that starts the pipeline on the LattePanda."),
    "panda_stop": ("LattePanda: stop", "The command that stops it."),
    "panda_sha": ("LattePanda: commit", "The command that prints the pipeline commit there."),
    "gt_range_m": ("Ground truth range (m)", "Cones within this distance count."),
    "gt_hfov_deg": ("Ground truth view (deg)", "Half the horizontal field of view."),
    "gt_min_range_m": ("Ground truth from (m)", "Cones closer than this are left out."),
    "gt_scan_period_ms": ("LiDAR period (ms)", "Used to time the ground-truth cones."),
    "max_frames": ("First frames only", "Perception on the first N LiDAR frames only."),
    "profile": ("Time each stage", "Perception's per-stage timings."),
    "strategy": ("SLAM strategy", "Empty: trackdrive."),
    "resynth_odom": ("Rebuild odometry", "SLAM rebuilds /odom from the bag's sensors."),
}
# A string setting with no choices in bench.yaml that the form still offers as a choice.
SETTING_OPTIONS: dict[str, dict[str, str]] = {
    "stack": {"own": "Its own", "running": "The running one"},
}
BENCHMARK_WORDS: dict[str, tuple[str, str]] = {
    "missions": ("Drive the missions", "The pipeline drives the sim through the missions, scored by the referee. "
                 "The sim and Mission Control must be running."),
    "sim_bag": ("Replay recorded bags", "Perception and SLAM run offline on recorded simulator bags, scored against "
                "the sim's ground truth."),
}
# Benchmarks shown first, in this order; then the rest in bench.yaml's.
BENCHMARK_ORDER = ("missions",)

# The settings a benchmark shows above More settings, in order: (name, label, help, widget, extra x-ui).
FEATURED: dict[str, list[tuple[str, str, str, str, dict[str, Any]]]] = {
    "missions": [
        ("missions", "Missions", "All lit: every mission.", "multi-csv",
         {"options_from": "missions", "all_value": sp.ALL_BAGS}),
        ("tracks", "Tracks", "The tracks picked here replace every mission's own: each picked mission runs on all of them.",
         "multi-csv", {"options_from": "tracks", "unset_shows": "mission_tracks"}),
        ("repeats", "Runs per track", "Each run is scored on its own; run i uses seed i.", "count", {}),
        ("start_noise", "Vary the start", "Start each run from a slightly different, seeded position.", "yes-no", {}),
    ],
}
# In the Setup tab's Pipeline box, not under More settings.
PIPELINE_ON = "pipeline_on"
ON_THIS_PC, ON_PANDA = "bench_pc", "latte_panda"
PIPELINE_ON_WORDS = ("Where the pipeline runs", "Not now: the run waits in the queue until it's free.")
PIPELINE_ON_OPTIONS = {ON_THIS_PC: "This PC", ON_PANDA: "LattePanda"}
# Settings that only matter when the pipeline runs on the LattePanda.
PANDA_PREFIX = "panda_"

# Fixed rows and boxes.
WORDS: dict[str, tuple[str, str]] = {
    "benchmarks": ("What to test", "Pick one, or start from a preset above."),
    "bags": ("Bags", "Pick one or more, or all."),
    "only": ("Measure", "All lit: all of them."),
    "repeats": ("Runs per bag", "Each run is a job of its own."),
    "repeats.missions": ("Repeat everything", "The whole benchmark again, each time a job of its own."),
    "pipeline": ("Parameter changes", "Files of pipeline parameter changes, over the preset's own; a file picked later wins."),
    "name": ("Name", "How it shows in the queue."),
    "code.ifssim": ("Sim (IFSSIM)", "The sim's code on the runner. A local run uses this sim, as it is."),
    "code.pipeline": ("Pipeline (IFS09)", "Fetched when the run starts. Empty: the commit this checkout's pipeline/ has."),
    "boards.mode": ("Boards", "None, the real boards on a HIL bench, or emulated ones (IFS-vHIL) in Docker."),
    "boards.system": ("System", "IFS-vHIL's systems/<id>.yaml: which boards are emulated."),
    "boards.firmware": ("Firmware", "A branch, tag, #PR or commit of the board's firmware. Empty: bench.yaml's default."),
    "boards.bench": ("HIL bench", "IFS_HIL's bench id, e.g. bench-01. Empty: the rig's."),
    "lattepanda.deploy": ("Deploy on the LattePanda", "Check the pipeline out on the LattePanda, build it and apply the parameter "
                          "changes there (IFS09's deploy/bench scripts), instead of the LattePanda commands."),
}
BOARD_MODE_OPTIONS = {"hil": "Real (HIL)", "vhil": "Emulated (vHIL)"}

REF_PATTERN = (
    r"^(?:(?:#|pr/|pull/)\d{1,7}"
    r"|refs/bench/[A-Za-z0-9][A-Za-z0-9-]{0,38}/[A-Za-z0-9][A-Za-z0-9._-]{0,99}"
    r"|(?!refs/)[A-Za-z0-9][A-Za-z0-9._/+-]{0,199})"
    r"(?:@[0-9a-f]{7,40})?$"
)


def human(name: str) -> str:
    """``gt_range_m`` as ``Gt range m``: a setting's label when SETTING_WORDS has none."""
    s = name.replace("_", " ")
    return s[:1].upper() + s[1:]


def preset_about(text: str) -> str:
    """A spec file's leading comment, up to its first example or command."""
    lines = []
    for line in text.splitlines():
        if not line.startswith("#"):
            break
        line = line[1:].strip()
        if not line or line.endswith(":") or line.startswith(("python3 ", "--")):
            break
        lines.append(line)
    return " ".join(lines)


class _Order:
    def __init__(self) -> None:
        self.n = 0

    def __call__(self) -> int:
        self.n += 1
        return self.n


def _ui(group: str, order: int, widget: str | None = None, **extra: Any) -> dict[str, Any]:
    out: dict[str, Any] = {"group": group, "order": order}
    if widget:
        out["widget"] = widget
    out.update({k: v for k, v in extra.items() if v is not None})
    return out


def _off(**extra: Any) -> dict[str, Any]:
    """A key the form doesn't show."""
    return {"form": False, **extra}


def _setting_schema(s: Any) -> dict[str, Any]:
    t = {"number": "number", "integer": "integer", "boolean": "boolean", "string": "string"}[s.type]
    out: dict[str, Any] = {"type": t}
    if s.choices:
        out["enum"] = list(s.choices)
    if s.default is not None:
        out["default"] = s.default
    return out


def _benchmark_schema(m: Manifest, b: Any, order: _Order) -> dict[str, Any]:
    props: dict[str, Any] = {}
    featured = {f[0]: f for f in FEATURED.get(b.name, [])}
    settings: dict[str, Any] = {}

    # the benchmark's own rows, in page order
    if b.bags:
        label, help_ = WORDS["bags"]
        props["bags"] = {
            "title": label, "description": help_,
            "anyOf": [{"const": sp.ALL_BAGS}, {"type": "string"},
                      {"type": "array", "items": {"type": "string"}, "minItems": 1, "uniqueItems": True}],
            "x-ui": _ui("benchmark", order(), "bags", options_from=f"bags:{b.bags}", all_value=sp.ALL_BAGS),
        }
    if b.parts:
        label, help_ = WORDS["only"]
        props["only"] = {
            "title": label, "description": help_,
            "anyOf": [{"enum": list(b.parts)},
                      {"type": "array", "items": {"enum": list(b.parts)}, "minItems": 1, "uniqueItems": True}],
            "x-ui": _ui("benchmark", order(), "multi", options={p: human(p) for p in b.parts}),
        }
    for name, (sname, label, help_, widget, extra) in featured.items():
        s = b.settings.get(sname)
        if s is None:
            continue
        settings[sname] = {"title": label, "description": help_, **_setting_schema(s),
                           "x-ui": _ui("benchmark", order(), widget, **extra)}
    repeats_label, repeats_help = WORDS["repeats.missions" if b.name == "missions" else "repeats"]
    repeats = {"title": repeats_label, "description": repeats_help, "type": "integer", "minimum": 1, "default": 1}
    if b.name != "missions":  # in More settings for the missions
        props["repeats"] = repeats | {"x-ui": _ui("benchmark", order(), "count")}

    # the Setup tab and More settings
    for s in b.settings.values():
        if s.name in settings:
            continue
        node = _setting_schema(s)
        if s.name in BACKEND_FILLED:
            settings[s.name] = {"title": human(s.name), "description": s.help, **node,
                                "x-ui": _off(fill="backend")}
            continue
        if s.name == PIPELINE_ON:
            label, help_ = PIPELINE_ON_WORDS
            settings[s.name] = {
                "title": label, "description": help_, **node,
                "enum": node.get("enum") or list(PIPELINE_ON_OPTIONS), "default": ON_THIS_PC,
                "x-ui": _ui("pipeline", 0, "choice", options=PIPELINE_ON_OPTIONS,
                            option_needs={ON_PANDA: "latte_panda"}, unset=ON_THIS_PC),
            }
            continue
        label, help_ = SETTING_WORDS.get(s.name, (human(s.name), s.help))
        options = SETTING_OPTIONS.get(s.name) if not s.choices and s.type == "string" else None
        if s.choices:
            options = {str(c): str(c) for c in s.choices}
        widget = ("default-yes-no" if s.type == "boolean" else "choice" if options else "text")
        if options and "enum" not in node:
            node["enum"] = list(options)
        extra: dict[str, Any] = {"options": options}
        if s.name.startswith(PANDA_PREFIX):
            extra |= {"needs": "latte_panda",
                      "visible_if": {f"benchmarks.benchmark.settings.{PIPELINE_ON}": ON_PANDA}}
        settings[s.name] = {"title": label, "description": help_, **node,
                            "x-ui": _ui("more", order(), widget, **extra)}
    if b.name == "missions":
        props["repeats"] = repeats | {"x-ui": _ui("more", order(), "count")}
    if "repeats" not in props:
        props["repeats"] = repeats | {"x-ui": _ui("more", order(), "count")}

    t = b.timeout_s
    props["timeout_s"] = {"title": "Time limit per job (s)", "type": "number", "exclusiveMinimum": 0,
                          "default": t, "x-ui": _off()}
    props["settings"] = {"type": "object", "additionalProperties": False, "properties": settings}
    title, about = BENCHMARK_WORDS.get(b.name, (b.title, b.description))
    return {
        "title": title,
        "description": about,
        "type": ["object", "null"],
        "additionalProperties": False,
        "properties": props,
        "x-manifest": {"title": b.title, "description": b.description, "parts": dict(b.parts),
                       "components": sorted(b.components()), "bags": b.bags},
    }


def _presets(m: Manifest) -> list[dict[str, Any]]:
    out = []
    for name in m.preset_names():
        path = m.preset(name)
        assert path is not None
        text = path.read_text()
        try:
            doc = sp.load_file(path)
        except sp.SpecError as e:
            out.append({"name": name, "error": str(e)})
            continue
        out.append({"name": name, "title": doc.get("name") or name, "about": preset_about(text), "spec": doc})
    return out


def build(m: Manifest) -> dict[str, Any]:
    """The JSON Schema of a run spec on this manifest, with the form's hints."""
    order = _Order()
    names = [n for n in BENCHMARK_ORDER if n in m.benchmarks] + [n for n in m.benchmarks if n not in BENCHMARK_ORDER]
    benches = {n: _benchmark_schema(m, m.benchmarks[n], order) for n in names}
    label, help_ = WORDS["benchmarks"]
    components = sorted({c for b in m.benchmarks.values() for c in b.components()})

    code_props: dict[str, Any] = {}
    for repo in m.repos:
        if repo == "ifssim":
            g, o, targets = "sim", 1, ["runner"]
        elif repo == "pipeline":
            g, o, targets = "pipeline", 1, None
        else:
            g, o, targets = "pipeline", 2, None
        rl, rh = WORDS.get(f"code.{repo}", (repo, ""))
        code_props[repo] = {
            "title": rl, "description": rh,
            "type": ["string", "null"], "pattern": REF_PATTERN,
            "x-ui": _ui(g, o, "code-ref", repo=repo, options_from=f"refs:{repo}", targets=targets,
                        default=m.default_refs.get(repo), github=m.repo_urls.get(repo)),
        }
    code_props[sp.CODE_IMAGE] = {
        "title": "Image", "description": "auto: the published image of the code's dev base, or one built from it.",
        "type": "string", "x-ui": _off(),
    }

    boards = m.boards or {}
    firmware = boards.get("firmware") or {}
    vhil = boards.get("vhil") or {}
    fw_props = {
        board: {
            "title": board.upper(),
            "description": WORDS["boards.firmware"][1],
            "type": "string",
            "x-ui": _ui("boards", 3, "text", repo=info["repo"], default=info.get("default") or None,
                        options_from=f"refs:{info['repo']}",
                        in_run={"hil": board in (boards.get("hil") or []),
                                          "vhil": sorted(s for s, bs in vhil.items() if board in bs)}),
        }
        for board, info in firmware.items()
    }
    ml, mh = WORDS["boards.mode"]
    sl, sh = WORDS["boards.system"]
    bl, bh = WORDS["boards.bench"]
    fl, fh = WORDS["boards.firmware"]
    dl, dh = WORDS["lattepanda.deploy"]
    pl, ph = WORDS["pipeline"]
    nl, nh = WORDS["name"]

    doc: dict[str, Any] = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": SCHEMA_ID,
        "title": "IFSSIM run spec",
        "description": "What run_spec.py runs (tools/sim_benchmark/launch/spec.py), with the form's hints (schema.py).",
        "x-schema-version": VERSION,
        "x-spec-schema": sp.SCHEMA,
        "type": "object",
        "additionalProperties": False,
        "required": ["benchmarks"],
        "properties": {
            "schema": {"const": sp.SCHEMA, "default": sp.SCHEMA, "x-ui": _off()},
            "name": {"title": nl, "description": nh, "type": "string", "x-ui": _ui("common", 2, "text")},
            "notes": {"title": "Notes", "type": "string", "x-ui": _off()},
            "experiment": {"title": "Experiment", "enum": list(sp.EXPERIMENTS), "default": sp.DEFAULT_EXPERIMENT,
                           "x-ui": _off(fill="launcher")},
            "compare_to": {"title": "Compare to", "x-ui": _off()},
            "code": {"type": "object", "additionalProperties": False, "properties": code_props},
            "benchmarks": {
                "title": label, "description": help_,
                "type": "object", "additionalProperties": False, "minProperties": 1,
                "properties": benches,
                "x-ui": _ui("what", 0, "pick-one", options={n: benches[n]["title"] for n in names}),
            },
            "pipeline": {
                "title": pl, "description": ph,
                "type": "object",
                "propertyNames": {"enum": components},
                "additionalProperties": {"type": "object"},
                "x-ui": _ui("common", 1, "override-files", options_from="overrides"),
            },
            "sweep": {"title": "Sweep", "type": "object",
                      "propertyNames": {"pattern": r"^(pipeline|benchmarks)\."},
                      "additionalProperties": {"type": "array", "minItems": 1}, "x-ui": _off()},
            "boards": {
                "type": "object",
                "additionalProperties": False,
                "required": ["mode"],
                "properties": {
                    "mode": {"title": ml, "description": mh, "enum": list(sp.BOARD_MODES),
                             "x-ui": _ui("boards", 1, "board-mode", options=BOARD_MODE_OPTIONS,
                                         option_needs={m_: m_ for m_ in sp.BOARD_MODES})},
                    "system": {"title": sl, "description": sh, "enum": list(vhil),
                               "x-ui": _ui("boards", 2, "choice", visible_if={"boards.mode": "vhil"},
                                           options={s: s for s in vhil}, default=list(vhil)[-1] if vhil else None,
                                           needs="vhil")},
                    "firmware": {"title": fl, "description": fh, "type": "object", "additionalProperties": False,
                                 "properties": fw_props},
                    "bench": {"title": bl, "description": bh, "type": "string",
                              "pattern": r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$",
                              "x-ui": _off(status="planned", needs="hil", visible_if={"boards.mode": "hil"})},
                },
            },
            "lattepanda": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "deploy": {"title": dl, "description": dh, "type": "boolean",
                               "x-ui": _off(status="planned", needs="latte_panda",
                                            visible_if={f"benchmarks.benchmark.settings.{PIPELINE_ON}": ON_PANDA})},
                },
            },
        },
        "x-ui": {
            "tabs": TABS,
            "groups": GROUPS,
            "widgets": WIDGETS,
            "lists": LISTS,
            "capabilities": CAPABILITIES,
            "code_ref": {"help": sp.REF_HELP, "kinds": list(sp.REF_KINDS), "pattern": REF_PATTERN},
            "presets": _presets(m),
        },
    }
    assert set(doc["properties"]) == sp.TOP, "spec.TOP and the schema's keys differ"
    body = json.dumps(doc, sort_keys=True, default=str)
    doc["x-fingerprint"] = hashlib.sha256(body.encode()).hexdigest()[:16]
    return doc


# ------------------------------------------------------------------ fields
def fields(doc: dict[str, Any]) -> list[dict[str, Any]]:
    """Every field of the schema that carries ``x-ui``, as ``{path, title, description, ui,
    user_facing}``, in form order (tab, group, order)."""
    out: list[dict[str, Any]] = []

    def walk(node: dict[str, Any], path: str) -> None:
        if path and "x-ui" in node:
            ui = node["x-ui"]
            out.append({
                "path": path,
                "title": node.get("title", ""),
                "description": node.get("description", ""),
                "ui": copy.deepcopy(ui),
                "user_facing": ui.get("form", True) is not False and "fill" not in ui and "status" not in ui,
            })
        for k, v in (node.get("properties") or {}).items():
            if isinstance(v, dict):
                walk(v, f"{path}.{k}" if path else k)

    walk(doc, "")
    tab_of = {g["id"]: g["tab"] for g in GROUPS}
    group_i = {g["id"]: i for i, g in enumerate(GROUPS)}
    tab_i = {t["id"]: i for i, t in enumerate(TABS)}
    shown = [f for f in out if f["user_facing"]]
    rest = [f for f in out if not f["user_facing"]]
    shown.sort(key=lambda f: (tab_i[tab_of[f["ui"]["group"]]], group_i[f["ui"]["group"]], f["ui"]["order"]))
    for f in shown:
        f["tab"] = tab_of[f["ui"]["group"]]
    return shown + rest


def write(m: Manifest, path: Path | None = None) -> str:
    text = json.dumps(build(m), indent=2, default=str) + "\n"
    if path is not None:
        Path(path).write_text(text)
    return text
