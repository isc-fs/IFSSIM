"""Run specs and run_spec.py, against a small manifest and fake bags (nothing real runs)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch import manifest as mf  # noqa: E402
from launch import run  # noqa: E402
from launch import spec as sp  # noqa: E402

REPO = Path(__file__).resolve().parents[3]  # the IFSSIM checkout these tests live in

# the "benchmark": writes what it was given, the way a real one records spec.json
FAKE = """
import json, os, sys
from pathlib import Path
out = Path(sys.argv[sys.argv.index("--results-root") + 1]) / "ran.jsonl"
with out.open("a") as f:
    f.write(json.dumps({"argv": sys.argv[1:], "spec": os.environ.get("BENCH_SPEC")}) + "\\n")
sys.exit(int(os.environ.get("FAKE_RC", "0")))
"""


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    (tmp_path / "fake.py").write_text(FAKE)
    for b in ("bag_a", "bag_b"):
        (tmp_path / "bags" / b).mkdir(parents=True)
        (tmp_path / "bags" / b / "metadata.yaml").write_text("x: 1\n")
    (tmp_path / "bags" / "not_a_bag").mkdir()
    (tmp_path / "specs").mkdir()
    (tmp_path / "specs" / "base.yaml").write_text(
        "benchmarks:\n  sim_bag:\n    bags: [bag_a]\n"
    )
    (tmp_path / "bench.yaml").write_text(
        yaml.safe_dump(
            {
                "schema": 1,
                "results": "results",
                "presets": "specs",
                "repos": {"ifssim": ".", "pipeline": "pipeline"},
                "bags": {"simulator": "bags"},
                "benchmarks": {
                    "sim_bag": {
                        "command": [
                            sys.executable,
                            "fake.py",
                            "{bag}",
                            "--results-root",
                            "{results}",
                        ],
                        "bags": "simulator",
                        "parts": {"perception": "cone_detection", "slam": "slam_node"},
                        "parts_flag": "--only",
                        "overrides_flag": "--pipeline-overrides",
                        "settings": {
                            "gt_range_m": {"type": "number", "flag": "--gt-range-m"},
                            "max_frames": {"type": "integer", "flag": "--max-frames"},
                            "profile": {"type": "boolean", "flag": "--profile"},
                        },
                    }
                },
            }
        )
    )
    return tmp_path


def jobs_of(repo: Path, doc: dict) -> list[sp.Job]:
    m = mf.load(repo)
    return sp.expand(sp.validate(doc, m), m)


def test_manifest_lists_only_real_bags(repo: Path) -> None:
    m = mf.load(repo)
    assert m.bags("simulator") == ["bag_a", "bag_b"]
    assert m.preset_names() == ["base"]


def test_later_sources_win_and_mappings_merge() -> None:
    got = sp.merge(
        {"benchmarks": {"sim_bag": {"bags": ["a", "b"], "repeats": 2}}},
        {"benchmarks": {"sim_bag": {"bags": ["c"]}}},
        sp.assignment("pipeline.slam_node.motion_model=imu"),
    )
    assert got == {
        "benchmarks": {"sim_bag": {"bags": ["c"], "repeats": 2}},
        "pipeline": {"slam_node": {"motion_model": "imu"}},
    }


def test_every_problem_is_reported_at_once(repo: Path) -> None:
    with pytest.raises(sp.SpecError) as e:
        sp.validate(
            {
                "benchmarks": {
                    "sim_bag": {
                        "bags": ["nope"],
                        "only": ["planning"],
                        "settings": {"gt_range_m": "far", "typo": 1},
                    },
                    "sim_e2e": {},
                },
                "pipeline": {"control_node": {"v_max": 5}},
                "code": {"lichtblick": "main"},
            },
            mf.load(repo),
        )
    msg = str(e.value)
    for bit in (
        "not in",  # bag
        "only: planning",
        "gt_range_m: expected number",
        "settings.typo: unknown setting",
        "sim_e2e: unknown benchmark",
        "pipeline.control_node: no chosen benchmark uses it",
        "code.lichtblick",
    ):
        assert bit in msg


def test_overrides_must_reach_a_chosen_part(repo: Path) -> None:
    doc = {
        "benchmarks": {"sim_bag": {"bags": ["bag_a"], "only": ["perception"]}},
        "pipeline": {"slam_node": {"motion_model": "imu"}},
    }
    with pytest.raises(sp.SpecError, match="slam_node: no chosen benchmark uses it"):
        sp.validate(doc, mf.load(repo))


def _add_missions(repo: Path, **extra) -> None:
    """A benchmark with no parts that runs the whole pipeline, like IFSSIM's missions."""
    doc = yaml.safe_load((repo / "bench.yaml").read_text())
    doc["benchmarks"]["missions"] = {
        "command": [sys.executable, "fake.py", "--results-root", "{results}"],
        "overrides_flag": "--pipeline-overrides",
        "overrides_for": ["cone_detection", "control_node"],
        **extra,
    }
    (repo / "bench.yaml").write_text(yaml.safe_dump(doc))


def test_overrides_for_a_benchmark_without_parts(repo: Path) -> None:
    _add_missions(repo)
    m = mf.load(repo)
    assert m.benchmarks["missions"].components() == {"cone_detection", "control_node"}
    pipeline = {
        "control_node": {"v_max": 5},
        "cone_detection": {"residual_gate_mse": 0.1},
    }
    [job] = jobs_of(repo, {"benchmarks": {"missions": {}}, "pipeline": pipeline})
    assert job.pipeline == pipeline
    # sim_bag doesn't take control_node: it doesn't get it, and isn't refused for it
    jobs = jobs_of(
        repo,
        {
            "benchmarks": {"missions": {}, "sim_bag": {"bags": ["bag_a"]}},
            "pipeline": pipeline,
        },
    )
    by = {j.benchmark: j.pipeline for j in jobs}
    assert by["sim_bag"] == {"cone_detection": {"residual_gate_mse": 0.1}}
    assert by["missions"] == pipeline
    with pytest.raises(sp.SpecError, match="slam_node: no chosen benchmark uses it"):
        jobs_of(
            repo, {"benchmarks": {"missions": {}}, "pipeline": {"slam_node": {"x": 1}}}
        )


def test_overrides_for_needs_a_flag(repo: Path) -> None:
    _add_missions(repo, overrides_flag=None)
    with pytest.raises(mf.ManifestError, match="overrides_for needs an overrides_flag"):
        mf.load(repo)


def test_jobs_are_bags_times_repeats_times_sweep(repo: Path) -> None:
    jobs = jobs_of(
        repo,
        {
            "benchmarks": {"sim_bag": {"bags": "all", "repeats": 2}},
            "pipeline": {"cone_detection": {"residual_gate_mse": 0.2}},
            "sweep": {"pipeline.cone_detection.residual_gate_mse": [0.02, 0.05]},
        },
    )
    assert len(jobs) == 2 * 2 * 2
    assert {j.pipeline["cone_detection"]["residual_gate_mse"] for j in jobs} == {
        0.02,
        0.05,
    }
    # repeats and bags share a spec id; sweep points don't
    assert len({j.spec_id for j in jobs}) == 2


def test_spec_id(repo: Path) -> None:
    (plain,) = jobs_of(repo, {"benchmarks": {"sim_bag": {"bags": ["bag_a"]}}})
    assert plain.spec_id == sp.DEFAULT_ID
    base = {
        "benchmarks": {"sim_bag": {"bags": ["bag_a"], "settings": {"gt_range_m": 20}}}
    }
    (a,) = jobs_of(repo, base)
    (b,) = jobs_of(
        repo, {**base, "name": "renamed", "notes": "x", "compare_to": "pinned"}
    )
    (c,) = jobs_of(
        repo,
        {
            "benchmarks": {
                "sim_bag": {"bags": ["bag_b"], "settings": {"gt_range_m": 20.0}}
            }
        },
    )
    assert (
        a.spec_id == b.spec_id == c.spec_id != sp.DEFAULT_ID
    )  # names, bags, 20 vs 20.0
    (d,) = jobs_of(
        repo,
        {
            "benchmarks": {
                "sim_bag": {"bags": ["bag_a"], "settings": {"gt_range_m": 25}}
            }
        },
    )
    assert d.spec_id != a.spec_id


def test_dry_run_runs_nothing(repo: Path, capsys) -> None:
    rc = run.main(
        [
            "base",
            "--repo",
            str(repo),
            "--dry-run",
            "--set",
            "benchmarks.sim_bag.settings.profile=true",
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out
    assert "fake.py" in out and "--profile" in out
    assert not (repo / "results").exists()


def test_run_spec_hands_each_job_its_spec_and_overrides(repo: Path) -> None:
    (repo / "results").mkdir()
    rc = run.main(
        [
            "base",
            "--repo",
            str(repo),
            "--set",
            "benchmarks.sim_bag.only=[slam]",
            "--set",
            "pipeline.slam_node.motion_model=imu",
            "--set",
            "benchmarks.sim_bag.settings.max_frames=10",
        ]
    )
    assert rc == 0
    (line,) = (repo / "results" / "ran.jsonl").read_text().splitlines()
    got = json.loads(line)
    argv = got["argv"]
    assert argv[0] == str(repo / "bags" / "bag_a")
    assert argv[argv.index("--only") + 1] == "slam"
    assert argv[argv.index("--max-frames") + 1] == "10"
    overrides = Path(argv[argv.index("--pipeline-overrides") + 1])
    assert json.loads(overrides.read_text()) == {"slam_node": {"motion_model": "imu"}}
    spec = json.loads(Path(got["spec"]).read_text())
    assert spec["bag"] == "bag_a" and spec["spec_id"] != sp.DEFAULT_ID
    assert spec["spec"]["benchmarks"]["sim_bag"]["only"] == ["slam"]


def test_bad_spec_exits_2(repo: Path, capsys) -> None:
    rc = run.main(
        ["base", "--repo", str(repo), "--set", "benchmarks.sim_bag.bags=[zzz]"]
    )
    assert rc == 2
    assert "zzz" in capsys.readouterr().err


def test_a_failed_job_fails_the_run(repo: Path, monkeypatch) -> None:
    (repo / "results").mkdir()
    monkeypatch.setenv("FAKE_RC", "3")
    assert run.main(["base", "--repo", str(repo)]) == 1


def test_bags_without_what_the_benchmark_needs_are_refused(repo: Path) -> None:
    doc = yaml.safe_load((repo / "bench.yaml").read_text())
    doc["benchmarks"]["sim_bag"]["needs_topics"] = ["/testing_only/track"]
    (repo / "bench.yaml").write_text(yaml.safe_dump(doc))
    (repo / "bags" / "bag_b" / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  topics_with_message_count:\n"
        "    - topic_metadata: {name: /imu}\n"
    )
    with pytest.raises(sp.SpecError, match="bag_b has no /testing_only/track"):
        sp.validate({"benchmarks": {"sim_bag": {"bags": ["bag_b"]}}}, mf.load(repo))
    # a bag whose metadata lists no topics is not judged
    sp.validate({"benchmarks": {"sim_bag": {"bags": ["bag_a"]}}}, mf.load(repo))


def _only_spec(repo: Path) -> dict:
    (line,) = (repo / "results" / "ran.jsonl").read_text().splitlines()
    return json.loads(Path(json.loads(line)["spec"]).read_text())


def test_a_local_run_records_who_and_where(repo: Path, monkeypatch) -> None:
    import socket

    (repo / "results").mkdir()
    for k in ("BENCH_TRIGGER", "BENCH_REQUESTED_BY", "GITHUB_ACTIONS"):
        monkeypatch.delenv(k, raising=False)
    assert run.main(["base", "--repo", str(repo)]) == 0
    spec = _only_spec(repo)
    assert spec["trigger"] == "local" and spec["job_id"].startswith("local:")
    assert spec["host"] == socket.gethostname()
    assert spec["requested_by"].endswith("@" + socket.gethostname())
    assert "github" not in spec


def test_a_runner_run_records_the_person_and_the_github_run(
    repo: Path, monkeypatch
) -> None:
    (repo / "results").mkdir()
    monkeypatch.setenv("BENCH_TRIGGER", "runner")
    monkeypatch.setenv("BENCH_REQUESTED_BY", "Alberto-cd")
    monkeypatch.setenv("GITHUB_ACTIONS", "true")
    monkeypatch.setenv("GITHUB_SERVER_URL", "https://github.com")
    monkeypatch.setenv("GITHUB_REPOSITORY", "isc-fs/IFSSIM")
    monkeypatch.setenv("GITHUB_RUN_ID", "123")
    monkeypatch.setenv("GITHUB_RUN_ATTEMPT", "2")
    monkeypatch.setenv("GITHUB_WORKFLOW", "Bench")
    monkeypatch.setenv("GITHUB_ACTOR", "Alberto-cd")
    monkeypatch.setenv("RUNNER_NAME", "bench-pc")
    assert run.main(["base", "--repo", str(repo)]) == 0
    spec = _only_spec(repo)
    assert spec["trigger"] == "runner" and spec["job_id"].startswith("runner:")
    assert spec["requested_by"] == "Alberto-cd"
    assert spec["github"] == {
        "run_id": "123",
        "run_attempt": "2",
        "workflow": "Bench",
        "actor": "Alberto-cd",
        "runner": "bench-pc",
        "url": "https://github.com/isc-fs/IFSSIM/actions/runs/123",
    }


def test_find_root_is_this_checkout() -> None:
    assert mf.find_root() == REPO


def test_ifssim_bench_yaml_loads() -> None:
    m = mf.load(REPO)
    assert {"sim_bag", "missions"} <= set(m.benchmarks)


def test_ifssim_presets_are_valid_specs() -> None:
    # bags live on each machine, so a preset may only fail on its bags here
    m = mf.load(REPO)
    assert m.preset_names()
    for name in m.preset_names():
        try:
            sp.validate(sp.load_file(m.preset(name)), m)
        except sp.SpecError as e:
            assert all("bags" in line for line in str(e).splitlines()), f"{name}: {e}"
