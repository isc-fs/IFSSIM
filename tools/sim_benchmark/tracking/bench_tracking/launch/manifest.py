"""The manifest: what a repository lets bench-run launch (``bench.yaml`` at its root).

This is the only thing the launcher knows about the repository it runs
benchmarks for, so the launcher can live in a repository of its own. The
manifest says:

* where results go, where the preset specs are, which folders hold bags;
* which repositories a spec's ``code:`` can pin (the repo itself, submodules);
* per benchmark: the command, its settings (name, type, flag), its parts, and
  which pipeline component each part takes overrides for.

Example (IFSSIM's is ``bench.yaml``)::

    schema: 1
    results: tools/sim_benchmark/results
    presets: tools/sim_benchmark/specs
    repos: {ifssim: {path: ., default: dev}, pipeline: pipeline}
    bags: {simulator: tools/sim_benchmark/results/capture}
    image:
      env: IFSSIM_DV_IMAGE
      registry: ghcr.io/isc-fs/ifssim-dv_pipeline_stack
      paths: [docker/dv_pipeline_stack, ros2/src]
      build: [docker, build, -f, docker/dv_pipeline_stack/Dockerfile, -t, "{tag}", .]
    benchmarks:
      sim_bag:
        title: Simulator bag benchmarks
        command: [python3, tools/sim_benchmark/run_sim_bag_benchmark.py, "{bag}",
                  --results-root, "{results}"]
        bags: simulator
        parts: {perception: cone_detection, slam: slam_node}
        parts_flag: --only
        overrides_flag: --pipeline-overrides
        settings:
          gt_range_m: {type: number, flag: --gt-range-m, help: ...}

Paths are relative to the manifest's folder.
"""

from __future__ import annotations

import dataclasses
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

FILE = "bench.yaml"
SCHEMA = 1
TYPES = {
    "number": (int, float),
    "integer": (int,),
    "boolean": (bool,),
    "string": (str,),
}


class ManifestError(ValueError):
    pass


@dataclass(frozen=True)
class Setting:
    name: str
    type: str
    flag: str
    help: str = ""
    choices: tuple[Any, ...] = ()
    default: Any = None

    def check(self, value: Any, where: str) -> Any:
        ok = TYPES[self.type]
        # bool is an int in Python; a number setting must not take true/false
        if isinstance(value, bool) and self.type != "boolean":
            ok = ()
        if not isinstance(value, ok):
            raise ManifestError(f"{where}: expected {self.type}, got {value!r}")
        if self.choices and value not in self.choices:
            raise ManifestError(
                f"{where}: {value!r} is not one of {', '.join(map(str, self.choices))}"
            )
        return float(value) if self.type == "number" else value

    def args(self, value: Any) -> list[str]:
        if self.type == "boolean":
            return [self.flag] if value else []
        return [self.flag, str(value)]


@dataclass(frozen=True)
class Benchmark:
    name: str
    title: str
    command: tuple[str, ...]
    description: str = ""
    bags: str | None = None
    parts: dict[str, str | None] = field(default_factory=dict)
    parts_flag: str | None = None
    overrides_flag: str | None = None
    settings: dict[str, Setting] = field(default_factory=dict)
    timeout_s: float = 7200.0
    needs_topics: tuple[str, ...] = ()  # a bag without these can't be used

    def components(self, only: list[str] | None = None) -> set[str]:
        """Pipeline components the chosen parts take overrides for."""
        return {c for p, c in self.parts.items() if c and (only is None or p in only)}


@dataclass(frozen=True)
class Image:
    """The image the benchmarks run in, and how the worker picks it (``code.image: auto``):
    the registry image of the newest published ``base_branch`` commit the tested code
    starts from, unless the code changes ``paths``, in which case ``build`` makes one."""

    env: str  # the variable the benchmarks read the image from
    registry: str | None = None  # e.g. ghcr.io/isc-fs/ifssim-dv_pipeline_stack
    tag: str = "sha-{short}"  # the registry tag of one commit
    base_branch: str = "dev"
    paths: tuple[
        str, ...
    ] = ()  # a change here needs an image built from the code itself
    build: tuple[
        str, ...
    ] = ()  # command, run in the checkout; {tag} is the image to make
    local_tag: str = "bench-local:{sha}"


@dataclass(frozen=True)
class Manifest:
    root: Path
    results: Path
    presets: Path | None
    repos: dict[str, Path]
    bag_dirs: dict[str, Path]
    benchmarks: dict[str, Benchmark]
    # where the Launch page starts, per repository; and URLs git can't tell
    default_refs: dict[str, str] = field(default_factory=dict)
    repo_urls: dict[str, str] = field(default_factory=dict)
    image: Image | None = None

    def with_paths(
        self, results: Path | None = None, bag_dirs: dict[str, Path] | None = None
    ) -> Manifest:
        """The same manifest with results and bags elsewhere (the central machine keeps
        them outside the checkout it tests)."""
        return dataclasses.replace(
            self,
            results=Path(results) if results else self.results,
            bag_dirs={**self.bag_dirs, **(bag_dirs or {})},
        )

    def bags(self, kind: str) -> list[str]:
        """Bag folder names of one kind, as they are on disk now."""
        d = self.bag_dirs[kind]
        if not d.is_dir():
            return []
        return sorted(
            p.name
            for p in d.iterdir()
            if p.is_dir() and (p / "metadata.yaml").is_file()
        )

    def bag_path(self, kind: str, name: str) -> Path:
        return self.bag_dirs[kind] / name

    def preset(self, name: str) -> Path | None:
        if self.presets is None:
            return None
        for cand in (self.presets / name, self.presets / f"{name}.yaml"):
            if cand.is_file():
                return cand
        return None

    def preset_names(self) -> list[str]:
        if self.presets is None or not self.presets.is_dir():
            return []
        return sorted(p.stem for p in self.presets.glob("*.yaml"))


@dataclass(frozen=True)
class BagInfo:
    name: str
    duration_s: float | None
    recorded: float | None  # unix time
    size_bytes: int
    topics: frozenset[str]


_bag_cache: dict[Path, tuple[float, BagInfo]] = {}


def bag_info(path: Path) -> BagInfo:
    """What a bag's ``metadata.yaml`` says (length, when, topics) and its size; cached."""
    meta_file = path / "metadata.yaml"
    mtime = meta_file.stat().st_mtime if meta_file.exists() else 0.0
    hit = _bag_cache.get(path)
    if hit and hit[0] == mtime:
        return hit[1]
    info: dict = {}
    try:
        loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
        info = (yaml.load(meta_file.read_text(), Loader=loader) or {}).get(
            "rosbag2_bagfile_information", {}
        )
    except (OSError, yaml.YAMLError, AttributeError):
        pass
    dur = (info.get("duration") or {}).get("nanoseconds")
    start = (info.get("starting_time") or {}).get("nanoseconds_since_epoch")
    topics = frozenset(
        (t.get("topic_metadata") or {}).get("name", "")
        for t in info.get("topics_with_message_count") or []
    )
    size = (
        sum(f.stat().st_size for f in path.iterdir() if f.is_file())
        if path.is_dir()
        else 0
    )
    out = BagInfo(
        path.name,
        dur / 1e9 if dur else None,
        start / 1e9 if start else None,
        size,
        topics,
    )
    _bag_cache[path] = (mtime, out)
    return out


def find_root(start: Path | None = None) -> Path:
    """The repository to run in: the git top level of ``start`` (default: cwd)."""
    start = Path(start or Path.cwd()).resolve()
    try:
        top = subprocess.run(
            ["git", "-C", str(start), "rev-parse", "--show-superproject-working-tree"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        if not top:
            top = subprocess.run(
                ["git", "-C", str(start), "rev-parse", "--show-toplevel"],
                capture_output=True,
                text=True,
                check=True,
            ).stdout.strip()
        return Path(top)
    except (OSError, subprocess.CalledProcessError):
        return start


def load(root: Path | str) -> Manifest:
    root = Path(root).resolve()
    path = root / FILE
    if not path.is_file():
        raise ManifestError(
            f"no {FILE} in {root}: bench-run needs the repository's manifest "
            f"(see bench_tracking/launch/manifest.py)"
        )
    try:
        doc = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise ManifestError(f"{path}: {e}") from e
    if doc.get("schema") != SCHEMA:
        raise ManifestError(f"{path}: schema must be {SCHEMA}")
    if "results" not in doc:
        raise ManifestError(f"{path}: 'results' is required")

    bag_dirs = {k: root / v for k, v in (doc.get("bags") or {}).items()}
    benchmarks = {}
    for name, b in (doc.get("benchmarks") or {}).items():
        where = f"{path}: benchmarks.{name}"
        if not b.get("command"):
            raise ManifestError(f"{where}: 'command' is required")
        if b.get("bags") and b["bags"] not in bag_dirs:
            raise ManifestError(f"{where}: bags {b['bags']!r} is not under 'bags'")
        settings = {}
        for sname, s in (b.get("settings") or {}).items():
            if s.get("type") not in TYPES or not s.get("flag"):
                raise ManifestError(
                    f"{where}.settings.{sname}: needs a flag and a type "
                    f"({', '.join(TYPES)})"
                )
            settings[sname] = Setting(
                name=sname,
                type=s["type"],
                flag=s["flag"],
                help=s.get("help", ""),
                choices=tuple(s.get("choices") or ()),
                default=s.get("default"),
            )
        benchmarks[name] = Benchmark(
            name=name,
            title=b.get("title", name),
            description=b.get("description", ""),
            command=tuple(str(c) for c in b["command"]),
            bags=b.get("bags"),
            parts=dict(b.get("parts") or {}),
            parts_flag=b.get("parts_flag"),
            overrides_flag=b.get("overrides_flag"),
            settings=settings,
            timeout_s=float(b.get("timeout_s", 7200)),
            needs_topics=tuple(b.get("needs_topics") or ()),
        )
    repos, default_refs, repo_urls = {}, {}, {}
    for k, v in (doc.get("repos") or {}).items():
        if isinstance(v, dict):
            repos[k] = root / v.get("path", ".")
            if v.get("default"):
                default_refs[k] = str(v["default"])
            if v.get("url"):
                repo_urls[k] = str(v["url"])
        else:
            repos[k] = root / v
    img = doc.get("image")
    image = None
    if img:
        if not img.get("env"):
            raise ManifestError(f"{path}: image.env is required")
        image = Image(
            env=img["env"],
            registry=img.get("registry"),
            tag=img.get("tag", "sha-{short}"),
            base_branch=img.get("base_branch", "dev"),
            paths=tuple(img.get("paths") or ()),
            build=tuple(str(c) for c in img.get("build") or ()),
            local_tag=img.get("local_tag", "bench-local:{sha}"),
        )
    return Manifest(
        root=root,
        results=root / doc["results"],
        presets=root / doc["presets"] if doc.get("presets") else None,
        repos=repos,
        bag_dirs=bag_dirs,
        benchmarks=benchmarks,
        default_refs=default_refs,
        repo_urls=repo_urls,
        image=image,
    )
