"""Record which code produced a benchmark run.

Every run dir made by ``common.make_run_dir`` gets a ``provenance.json``, plus

* ``ifssim.diff`` / ``pipeline.diff`` when either tree has changes that are not
  committed (untracked files included);
* ``ifssim.unpushed.bundle`` / ``pipeline.unpushed.bundle`` when HEAD has
  commits that no remote has (as far as the last ``git fetch`` knows): a git
  bundle of exactly those commits, so someone else can check the code out.

Two runs of the same commit can then be told apart, and any run can be rebuilt
on another machine::

    git fetch <run>/pipeline.unpushed.bundle HEAD   # only if the run has one
    git checkout <pipeline.sha>
    git apply <run>/pipeline.diff                    # only if the run has one

For runs in Docker the image is recorded too. An image pulled from the registry
has a digest that is the same on every machine; a locally built one does not,
so its contents cannot be matched across machines (a warning says so).

The code state is read with ``git`` on the host: the benchmark container has
no ``.git``. When ``maybe_reexec_in_docker`` hands a run to the container, it
captures first and stages the files under ``results/.provenance/<id>/``; the
container finds them through ``IFSSIM_PROVENANCE_DIR`` and copies them into
the run dir. A run started inside the container some other way records that
the code state is unknown rather than guessing.

Standard library only: this runs in the container's system Python.
"""

from __future__ import annotations

import getpass
import hashlib
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCHEMA = 1
ENV_DIR = "IFSSIM_PROVENANCE_DIR"
FILE = "provenance.json"
STAGING = ".provenance"  # under the results dir (git-ignored with the rest of results/)
# Untracked files bigger than this are listed, not copied into the diff.
MAX_UNTRACKED_BYTES = 512 * 1024
# Unpushed commits are saved as a bundle unless there are more, or it gets bigger, than this.
MAX_UNPUSHED_COMMITS = 200
MAX_BUNDLE_BYTES = 50 * 1024 * 1024
OCI_REVISION = "org.opencontainers.image.revision"
OCI_SOURCE = "org.opencontainers.image.source"


def _git(repo: Path, *args: str, ok: tuple[int, ...] = (0,)) -> str | None:
    try:
        p = subprocess.run(
            ["git", "-C", str(repo), *args],
            capture_output=True,
            text=True,
            errors="replace",
            check=False,
        )
    except OSError:
        return None
    return p.stdout if p.returncode in ok else None


def _is_repo_root(repo: Path) -> bool:
    """True only if ``repo`` is itself a checkout. An uninitialised submodule is an
    empty dir, and ``git -C`` there would silently answer for the parent repo."""
    top = _git(repo, "rev-parse", "--show-toplevel")
    return top is not None and Path(top.strip()).resolve() == repo.resolve()


def git_state(repo: Path) -> tuple[dict[str, Any], str]:
    """(state, diff). The diff covers staged, unstaged and untracked changes against HEAD."""
    blank = {
        "sha": None,
        "branch": None,
        "dirty": None,
        "diff_sha": None,
        "subject": None,
        "unpushed": None,
    }
    if not repo.is_dir() or not _is_repo_root(repo):
        return blank, ""
    sha = (_git(repo, "rev-parse", "HEAD") or "").strip() or None
    if sha is None:
        return blank, ""
    diff = _git(repo, "diff", "HEAD", "--binary", "--ignore-submodules=dirty") or ""
    skipped = []
    for rel in (
        _git(repo, "ls-files", "--others", "--exclude-standard", "-z") or ""
    ).split("\0"):
        if not rel:
            continue
        f = repo / rel
        try:
            size = f.stat().st_size
        except OSError:
            continue
        if size > MAX_UNTRACKED_BYTES:
            skipped.append(rel)
            continue
        # `git diff --no-index` exits 1 when the files differ, which is the point
        diff += (
            _git(
                repo,
                "diff",
                "--binary",
                "--no-index",
                "--",
                "/dev/null",
                rel,
                ok=(0, 1),
            )
            or ""
        )
    return {
        "sha": sha,
        "branch": (_git(repo, "rev-parse", "--abbrev-ref", "HEAD") or "").strip()
        or None,
        "subject": (_git(repo, "log", "-1", "--format=%s") or "").strip() or None,
        "dirty": bool(diff),
        "diff_sha": hashlib.sha1(diff.encode()).hexdigest()[:8] if diff else None,
        "untracked_not_saved": skipped,
        **unpushed(repo),
    }, diff


def unpushed(repo: Path) -> dict[str, Any]:
    """Commits in HEAD that no remote-tracking branch contains.

    Uses the remote refs from the last fetch, so it never touches the network;
    a stale checkout can report commits that someone else has since pushed.
    ``unpushed`` is None when the repo has no remote at all.
    """
    if not (_git(repo, "remote") or "").strip():
        return {"unpushed": None, "unpushed_commits": []}
    out = _git(repo, "rev-list", "HEAD", "--not", "--remotes")
    shas = out.split() if out is not None else []
    return {"unpushed": len(shas), "unpushed_commits": [c[:12] for c in shas[:20]]}


def unpushed_bundle(repo: Path, state: dict[str, Any]) -> bytes | None:
    """A git bundle of the unpushed commits (``git fetch <file> HEAD`` restores them)."""
    n = state.get("unpushed") or 0
    if not n:
        return None
    if n > MAX_UNPUSHED_COMMITS:
        state["unpushed_not_saved"] = f"{n} commits (limit {MAX_UNPUSHED_COMMITS})"
        return None
    fd, tmp = tempfile.mkstemp(suffix=".bundle")
    os.close(fd)
    try:
        if _git(repo, "bundle", "create", tmp, "HEAD", "--not", "--remotes") is None:
            state["unpushed_not_saved"] = "git bundle failed"
            return None
        data = Path(tmp).read_bytes()
    finally:
        Path(tmp).unlink(missing_ok=True)
    if len(data) > MAX_BUNDLE_BYTES:
        state["unpushed_not_saved"] = f"bundle is {len(data) >> 20} MB"
        return None
    return data


def code_id(code: dict[str, Any]) -> str | None:
    """One id for "this exact code": both commits and both diffs. Same id = same code."""
    parts = [
        f"{(code.get(n) or {}).get('sha') or '-'}:{(code.get(n) or {}).get('diff_sha') or ''}"
        for n in ("pipeline", "ifssim")
    ]
    if all(x.startswith("-:") for x in parts):
        return None
    # the image holds the compiled rest of the stack; only a registry digest names it
    # the same way on every machine (a local build's id changes with every rebuild)
    digest = (code.get("image") or {}).get("digest")
    if digest:
        parts.append(f"image:{digest}")
    return hashlib.sha1("|".join(parts).encode()).hexdigest()[:8]


def label(code: dict[str, Any]) -> str:
    """What people read: the pipeline commit, and a mark when anything was uncommitted.

    ``a64350a`` · ``a64350a-dirty.3f2c1a9e`` (the suffix is the code id, so two different
    dirty states of one commit get different labels).
    """
    p = code.get("pipeline") or {}
    i = code.get("ifssim") or {}
    if not p.get("sha") and not i.get("sha"):
        return "unknown"
    head = p["sha"][:7] if p.get("sha") else "no-pipeline"
    if p.get("dirty") or i.get("dirty"):
        return f"{head}-dirty.{code_id(code) or 'x'}"
    return head


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent.parent


def _is_registry_repo(repo: str) -> bool:
    """``ghcr.io/isc-fs/x`` is in a registry; ``ifssim-dv_pipeline_stack`` (a local build) is not."""
    first = repo.split("/", 1)[0]
    return "/" in repo and ("." in first or ":" in first or first == "localhost")


def docker_image(image: str) -> dict[str, Any]:
    """What the benchmark image is: its registry digest when it came from one, else its local id."""
    info: dict[str, Any] = {"tag": image, "id": None, "digest": None}
    try:
        out = subprocess.run(
            ["docker", "image", "inspect", "--format", "{{json .}}", image],
            capture_output=True,
            text=True,
            check=False,
        )
        doc = json.loads(out.stdout) if out.returncode == 0 else None
    except (OSError, ValueError):
        doc = None
    if not isinstance(doc, dict):
        return info
    labels = (doc.get("Config") or {}).get("Labels") or {}
    digests = [
        d for d in doc.get("RepoDigests") or [] if _is_registry_repo(d.split("@")[0])
    ]
    info.update(
        {
            "id": doc.get("Id"),
            "digest": digests[0] if digests else None,
            "created": doc.get("Created"),
            "revision": labels.get(OCI_REVISION),
            "source": labels.get(OCI_SOURCE),
            "local_build": not digests,
        }
    )
    return info


def capture(image: str | None = None) -> tuple[dict[str, Any], dict[str, str]]:
    """(provenance, {file name: diff}) for the checkout this file lives in. Host only."""
    root = repo_root()
    ifssim, ifssim_diff = git_state(root)
    pipeline, pipeline_diff = git_state(root / "pipeline")
    code = {"ifssim": ifssim, "pipeline": pipeline}
    if image:
        code["image"] = docker_image(image)
    bundles = {
        f"{name}.unpushed.bundle": data
        for name, repo, st in (
            ("ifssim", root, ifssim),
            ("pipeline", root / "pipeline", pipeline),
        )
        if (data := unpushed_bundle(repo, st)) is not None
    }
    meta = {
        "schema": SCHEMA,
        "captured": True,
        "captured_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "capture_id": uuid.uuid4().hex[:12],
        "code": code,
        "code_id": code_id(code),
        "label": label(code),
        "command": [Path(sys.argv[0]).name, *sys.argv[1:]],
        "host": socket.gethostname(),
        "user": getpass.getuser(),
    }
    diffs: dict[str, str | bytes] = {
        name: text
        for name, text in (
            ("ifssim.diff", ifssim_diff),
            ("pipeline.diff", pipeline_diff),
        )
        if text
    }
    diffs.update(bundles)
    for line in warnings(code):
        print(f"warning: {line}", file=sys.stderr)
    return meta, diffs


def warnings(code: dict[str, Any]) -> list[str]:
    """What would stop someone else from getting this exact code."""
    out = []
    for name in ("pipeline", "ifssim"):
        st = code.get(name) or {}
        if st.get("unpushed"):
            saved = (
                f"not saved: {st['unpushed_not_saved']}"
                if st.get("unpushed_not_saved")
                else f"saved to {name}.unpushed.bundle in the run dir"
            )
            out.append(
                f"{name} HEAD has {st['unpushed']} commit(s) that are on no remote ({saved}). "
                "Push them so others can check this code out."
            )
    img = code.get("image") or {}
    if img.get("local_build"):
        out.append(
            f"image {img.get('tag')} was built locally, so other machines cannot tell whether "
            "theirs matches. For runs to compare across machines, pull a published one "
            "(IFSSIM_DV_IMAGE=ghcr.io/isc-fs/ifssim-dv_pipeline_stack:sha-<commit>)."
        )
    return out


def write(dest: Path, meta: dict[str, Any], diffs: dict[str, str | bytes]) -> None:
    dest.mkdir(parents=True, exist_ok=True)
    for name, data in diffs.items():
        if isinstance(data, bytes):
            (dest / name).write_bytes(data)
        else:
            (dest / name).write_text(data)
    meta = dict(meta, diff_files=sorted(diffs))
    (dest / FILE).write_text(json.dumps(meta, indent=2, sort_keys=True))


def stage_for_docker(results: Path, image: str) -> Path:
    """Capture on the host and park it where the container can read it (``/results/...``)."""
    meta, diffs = capture(image)
    staged = results / STAGING / meta["capture_id"]
    write(staged, meta, diffs)
    return staged


def unstage(staged: Path) -> None:
    shutil.rmtree(staged, ignore_errors=True)


def record(run_dir: Path) -> None:
    """Put provenance into a new run dir. Called by ``make_run_dir``; never fails the benchmark."""
    try:
        staged = os.environ.get(ENV_DIR)
        if staged and (Path(staged) / FILE).is_file():
            for f in Path(staged).iterdir():
                if f.is_file():
                    shutil.copy2(f, run_dir / f.name)
            return
        if os.environ.get("IFSSIM_BENCHMARK_IN_DOCKER") == "1":
            write(
                run_dir,
                {
                    "schema": SCHEMA,
                    "captured": False,
                    "reason": "started inside the benchmark container without host provenance",
                    "label": "unknown",
                    "code_id": None,
                    "command": [Path(sys.argv[0]).name, *sys.argv[1:]],
                },
                {},
            )
            return
        meta, diffs = capture()
        write(run_dir, meta, diffs)
    except Exception as e:  # noqa: BLE001  (provenance must never break a benchmark)
        print(
            f"warning: could not record provenance in {run_dir}: {e}", file=sys.stderr
        )


def read(run_dir: Path) -> dict[str, Any] | None:
    p = Path(run_dir) / FILE
    try:
        return json.loads(p.read_text()) if p.is_file() else None
    except ValueError:
        return None


if __name__ == "__main__":
    # `python run_provenance.py` prints what a run started now would record.
    m, d = capture()
    print(json.dumps(dict(m, diff_bytes={k: len(v) for k, v in d.items()}), indent=2))
