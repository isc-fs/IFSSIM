"""The run spec's JSON Schema (launch/schema.py) and its parity with the sim's Benchmarks page.

The page registers the fields it sets, with their words, in SFSDSBenchPageFields.inl (it
compiles that file in and takes its labels from it). These tests read that list as JSON and
check it against ``run_spec.py --print-schema`` on this checkout's bench.yaml, both ways.
"""

from __future__ import annotations

import copy
import fnmatch
import json
import re
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch import manifest as mf  # noqa: E402
from launch import rig as rg  # noqa: E402
from launch import schema as sc  # noqa: E402
from launch import spec as sp  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
PAGE_DIR = REPO / "Plugins/FSDSPlugin/Source/FSDSPlugin/Private/UI"
FIELDS_INL = PAGE_DIR / "SFSDSBenchPageFields.inl"


def page_fields() -> dict:
    """The page's registry: the JSON between the .inl's first and last lines."""
    lines = FIELDS_INL.read_text().strip().splitlines()
    assert lines[0] == 'R"JSON(' and lines[-1] == ')JSON"', "the .inl wraps its JSON in R\"JSON( ... )JSON\""
    return json.loads("\n".join(lines[1:-1]))


@pytest.fixture(scope="module")
def manifest() -> mf.Manifest:
    return mf.load(REPO)


@pytest.fixture(scope="module")
def schema(manifest: mf.Manifest) -> dict:
    return sc.build(manifest)


def _segment_score(pattern: str) -> int:
    return 0 if pattern == "*" else 1 if any(c in pattern for c in "*?[") else 2


def matches(pattern: str, path: str) -> bool:
    p, q = pattern.split("."), path.split(".")
    return len(p) == len(q) and all(fnmatch.fnmatchcase(b, a) for a, b in zip(p, q))


def best(entries: list[dict], path: str) -> dict | None:
    """The most specific registry entry for a schema path, as the page picks it."""
    hits = [e for e in entries if matches(e["path"], path)]
    return max(hits, key=lambda e: sum(_segment_score(s) for s in e["path"].split(".")), default=None)


# ------------------------------------------------------------------ the schema itself
def test_print_schema_is_json_and_matches_build(manifest: mf.Manifest) -> None:
    out = subprocess.run(
        [sys.executable, str(REPO / "tools/sim_benchmark/run_spec.py"), "--print-schema", "--repo", str(REPO)],
        capture_output=True, text=True, check=True,
    ).stdout
    doc = json.loads(out)
    assert doc == json.loads(json.dumps(sc.build(manifest), default=str))
    assert doc["$schema"].endswith("2020-12/schema")
    assert doc["x-schema-version"] == sc.VERSION and doc["$id"].endswith(f":{sc.VERSION}")
    assert doc["x-spec-schema"] == sp.SCHEMA
    assert re.fullmatch(r"[0-9a-f]{16}", doc["x-fingerprint"])


def test_fingerprint_follows_content(manifest: mf.Manifest, tmp_path: Path) -> None:
    a = sc.build(manifest)["x-fingerprint"]
    assert sc.build(manifest)["x-fingerprint"] == a  # stable
    m2 = mf.Manifest(**{**manifest.__dict__, "presets": tmp_path})  # no presets
    assert sc.build(m2)["x-fingerprint"] != a


def test_top_level_keys_are_spec_py_s(schema: dict) -> None:
    assert set(schema["properties"]) == sp.TOP
    assert schema["additionalProperties"] is False


def test_every_manifest_benchmark_and_setting_is_there(schema: dict, manifest: mf.Manifest) -> None:
    benches = schema["properties"]["benchmarks"]["properties"]
    assert list(benches)[0] == "missions"
    assert set(benches) == set(manifest.benchmarks)
    for name, b in manifest.benchmarks.items():
        props = benches[name]["properties"]
        assert set(props["settings"]["properties"]) == set(b.settings)
        assert set(props) >= {"repeats", "timeout_s", "settings"}
        assert ("bags" in props) == bool(b.bags)
        assert ("only" in props) == bool(b.parts)
        for s in b.settings.values():
            node = props["settings"]["properties"][s.name]
            want = {"number": "number", "integer": "integer", "boolean": "boolean", "string": "string"}[s.type]
            assert node["type"] == want, s.name


def test_enums_are_spec_py_s_and_bench_yaml_s(schema: dict, manifest: mf.Manifest) -> None:
    p = schema["properties"]
    assert p["experiment"]["enum"] == list(sp.EXPERIMENTS)
    boards = p["boards"]["properties"]
    assert boards["mode"]["enum"] == list(sp.BOARD_MODES)
    assert set(boards) == sp.BOARD_KEYS
    assert boards["system"]["enum"] == list(manifest.boards["vhil"])
    assert set(boards["firmware"]["properties"]) == set(manifest.boards["firmware"])
    assert set(p["lattepanda"]["properties"]) == sp.LATTEPANDA_KEYS
    assert set(p["code"]["properties"]) == set(manifest.repos) | {sp.CODE_IMAGE}
    comps = sorted({c for b in manifest.benchmarks.values() for c in b.components()})
    assert p["pipeline"]["propertyNames"]["enum"] == comps


def test_presets_are_in_the_schema_and_valid(schema: dict, manifest: mf.Manifest) -> None:
    presets = schema["x-ui"]["presets"]
    assert [x["name"] for x in presets] == manifest.preset_names()
    for x in presets:
        assert "error" not in x and x["title"] and x["about"], x["name"]
        top = set(x["spec"]) - sp.TOP
        assert not top, (x["name"], top)
    assert any(x["name"] == "missions-extensive" for x in presets)


def test_every_field_names_a_known_group_widget_list_and_capability(schema: dict) -> None:
    groups = {g["id"] for g in sc.GROUPS}
    lists = schema["x-ui"]["lists"]
    caps = set(schema["x-ui"]["capabilities"])
    for f in sc.fields(schema):
        ui = f["ui"]
        if f["user_facing"]:
            assert ui["group"] in groups, f["path"]
            assert ui.get("widget") in sc.WIDGETS, f["path"]
            assert f["title"], f["path"]
        src = ui.get("options_from")
        if src:
            assert src in lists or f"{src.split(':')[0]}:*" in lists, (f["path"], src)
        for need in [ui.get("needs"), *(ui.get("option_needs") or {}).values()]:
            assert need is None or need in caps, (f["path"], need)


def test_backend_lists_and_capability_fields(schema: dict) -> None:
    by = {f["path"]: f for f in sc.fields(schema)}
    assert by["benchmarks.missions.settings.missions"]["ui"]["options_from"] == "missions"
    assert by["benchmarks.missions.settings.tracks"]["ui"]["options_from"] == "tracks"
    assert by["benchmarks.sim_bag.bags"]["ui"]["options_from"] == "bags:simulator"
    assert by["pipeline"]["ui"]["options_from"] == "overrides"
    assert by["benchmarks.missions.settings.pipeline_on"]["ui"]["option_needs"] == {"latte_panda": "latte_panda"}
    assert by["boards.mode"]["ui"]["option_needs"] == {"hil": "hil", "vhil": "vhil"}
    assert by["boards.system"]["ui"]["needs"] == "vhil"
    for k in ("panda_start", "panda_stop", "panda_sha"):
        assert by[f"benchmarks.missions.settings.{k}"]["ui"]["needs"] == "latte_panda"
    # B8's fields: in the schema, not on the form yet
    for path, need in (("boards.bench", "hil"), ("lattepanda.deploy", "latte_panda")):
        assert by[path]["ui"]["status"] == "planned" and by[path]["ui"]["needs"] == need
        assert not by[path]["user_facing"]
    assert not by["benchmarks.missions.settings.mc_url"]["user_facing"]
    assert by["code.ifssim"]["ui"]["targets"] == ["runner"]


def test_capability_hints_agree_with_the_rig_check(schema: dict) -> None:
    """What the form greys out is what rig.needs asks the runner for."""
    base = {"benchmarks": {"missions": {}}}
    assert "latte_panda" in rg.needs(sp.merge(base, {"benchmarks": {"missions": {"settings": {"pipeline_on": "latte_panda"}}}}))
    for mode in sp.BOARD_MODES:
        assert mode in rg.needs(sp.merge(base, {"boards": {"mode": mode}}))
    assert "latte_panda_deploy" in rg.needs(sp.merge(base, {"lattepanda": {"deploy": True}}))
    st = rg.check({"id": "x"}, run=lambda cmd, t: subprocess.CompletedProcess(cmd, 1, "", "no"))
    assert st["parts"]["latte_panda_deploy"]["status"] == rg.ABSENT


def test_ref_pattern_agrees_with_parse_ref() -> None:
    pat = re.compile(sc.REF_PATTERN)
    good = ["dev", "feat/88-loop", "#88", "pr/88", "pull/88", "4f1c2ab", "a" * 40,
            "refs/bench/alberto/k3x9", "refs/bench/Alberto-cd/2026.11.20_x", "dev@4f1c2ab",
            "#88@4f1c2ab", "refs/bench/alberto/k3x9@" + "b" * 40]
    bad = ["", " ", "refs/heads/dev", "refs/bench/alberto", "refs/bench//x", "-x", "a b", "dev@xyz@4f1c2ab"]
    for r in good:
        assert pat.fullmatch(r), r
        sp.parse_ref(r)
    for r in bad:
        with pytest.raises(sp.SpecError):
            sp.parse_ref(r)
    for r in bad[:-1]:
        assert not pat.fullmatch(r), r


# ------------------------------------------------------------------ parity with the page
def test_page_registry_is_valid() -> None:
    reg = page_fields()
    assert reg["version"] == 1
    paths = [e["path"] for e in reg["fields"]]
    assert len(paths) == len(set(paths)), "a path registered twice"
    for e in reg["fields"]:
        assert e["tab"] in {t["id"] for t in sc.TABS}, e
        assert set(e) <= {"path", "tab", "label", "help", "options", "needs", "targets"}, e


def test_every_field_the_page_sets_is_in_the_schema(schema: dict) -> None:
    shown = [f for f in sc.fields(schema) if f["user_facing"]]
    for e in page_fields()["fields"]:
        hits = [f["path"] for f in shown if matches(e["path"], f["path"])]
        assert hits, f"the page sets {e['path']}, which the schema has no form field for"


def test_every_form_field_of_the_schema_is_on_the_page(schema: dict) -> None:
    entries = page_fields()["fields"]
    for f in sc.fields(schema):
        if f["user_facing"]:
            assert best(entries, f["path"]), f"{f['path']} is on the schema's form but not on the page"


def test_page_and_schema_use_the_same_words_tabs_and_capabilities(schema: dict) -> None:
    entries = page_fields()["fields"]
    for f in sc.fields(schema):
        if not f["user_facing"]:
            continue
        e = best(entries, f["path"])
        where = f"{f['path']} (page entry {e['path']})"
        assert e["tab"] == f["tab"], where
        if e["label"]:
            assert e["label"] == f["title"], where
        if e["help"]:
            assert e["help"] == f["description"], where
        if "options" in e:
            assert e["options"] == f["ui"].get("options"), where
        assert e.get("needs") == f["ui"].get("needs"), where
        assert e.get("targets") == f["ui"].get("targets"), where


def test_page_falls_back_like_the_schema(schema: dict, manifest: mf.Manifest) -> None:
    """Settings the page has no words for: the name (BenchUi::Human) and bench.yaml's help."""
    entries = page_fields()["fields"]
    for f in sc.fields(schema):
        if f["user_facing"] and best(entries, f["path"])["path"] == "benchmarks.*.settings.*":
            _, bench, _, name = f["path"].split(".")
            assert f["title"] == sc.human(name)
            assert f["description"] == manifest.benchmarks[bench].settings[name].help


def test_benchmark_words_match(schema: dict) -> None:
    benches = schema["properties"]["benchmarks"]["properties"]
    for name, (title, about) in page_fields()["benchmarks"].items():
        assert (benches[name]["title"], benches[name]["description"]) == (title, about), name
    assert schema["properties"]["benchmarks"]["x-ui"]["options"] == {n: b["title"] for n, b in benches.items()}


def test_parity_catches_drift(schema: dict) -> None:
    """The checks above fail when a setting appears on one side only."""
    doc = copy.deepcopy(schema)
    doc["properties"]["boards"]["properties"]["bench"]["x-ui"] = sc._ui("boards", 9, "text")
    entries = page_fields()["fields"]
    missing = [f["path"] for f in sc.fields(doc) if f["user_facing"] and not best(entries, f["path"])]
    assert missing == ["boards.bench"]
    shown = [f for f in sc.fields(schema) if f["user_facing"]]
    assert not [f for f in shown if matches("benchmarks.*.settings.gear", f["path"])]


# ------------------------------------------------------------------ backend uses the same words
def test_backend_hides_what_the_schema_marks_backend_filled() -> None:
    src = (REPO / "tools/mission_control/backend/bench_queue.py").read_text()
    assert "BACKEND_FILLED" in src and "FORM_HIDDEN = frozenset" not in src


def test_existing_specs_still_validate(manifest: mf.Manifest) -> None:
    for name in manifest.preset_names():
        doc = sp.load_file(manifest.preset(name))
        if name.startswith("sim-bag"):
            continue  # need bags on this machine
        sp.validate(doc, manifest)
    for p in sorted((REPO / "tools/sim_benchmark/specs/overrides").glob("*.yaml")):
        assert isinstance(yaml.safe_load(p.read_text()), dict)
