"""Which code a job runs: branch/PR/commit -> exact commits, a checkout of them, and the image.

At launch, refs are resolved to commits (``resolve``) so a job runs what was
asked for even if the branch moves while it waits. The worker then checks those
commits out in a fresh git worktree of its own clone (``prepare``), with the
submodules at the commit IFSSIM pins unless the spec pins them too, and picks
the image (``choose_image``, the manifest's ``image:`` section).
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path
from typing import Callable

from .manifest import Image, Manifest

SHA = re.compile(r"^[0-9a-f]{7,40}$")
PR = re.compile(r"^(?:#|pr/|pull/)(\d+)$")
SCAN_BACK = 50  # commits of base_branch searched for a published image


class CodeError(ValueError):
    pass


def git(repo: Path | str, *args: str, check: bool = True) -> str:
    r = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True)
    if check and r.returncode != 0:
        raise CodeError(f"git {' '.join(args)}: {r.stderr.strip() or r.stdout.strip()}")
    return r.stdout.strip()


def remote_url(repo: Path) -> str:
    return git(repo, "remote", "get-url", "origin")


def _pr_ref(ref: str) -> str | None:
    m = PR.match(ref.strip())
    return f"refs/pull/{m.group(1)}/head" if m else None


def resolve_ref(repo: Path, ref: str) -> dict[str, str]:
    """``{"ref": what was asked, "sha": the commit, "fetch": what to fetch}``.

    Branches, tags and PRs (``#123``) are looked up on the remote, so it is what is
    pushed that counts. A commit id is taken as it is (the checkout fails if no
    remote has it)."""
    ref = ref.strip()
    if not ref:
        raise CodeError("empty ref")
    if "@" in ref:
        # "<branch or #PR>@<commit>": that commit, reached through that branch or PR (the
        # Launch page sends this: the version picked, not whatever the branch is at later)
        name, sha = ref.rsplit("@", 1)
        if not SHA.match(sha):
            raise CodeError(f"{ref!r}: {sha!r} is not a commit id")
        pr = _pr_ref(name)
        return {"ref": name, "sha": sha, "fetch": pr or f"refs/heads/{name}"}
    if SHA.match(ref):
        return {"ref": ref, "sha": ref, "fetch": ref}
    url = remote_url(repo)
    pr = _pr_ref(ref)
    wanted = (
        [pr]
        if pr
        else [f"refs/heads/{ref}", f"refs/tags/{ref}^{{}}", f"refs/tags/{ref}"]
    )
    out = git(repo, "ls-remote", url, *wanted)
    found = dict(
        reversed(line.split("\t", 1)) for line in out.splitlines() if "\t" in line
    )
    for w in wanted:
        if w in found:
            return {"ref": ref, "sha": found[w], "fetch": pr or w.removesuffix("^{}")}
    raise CodeError(f"{ref!r} is not a branch, tag or PR of {url}")


def resolve(m: Manifest, requested: dict[str, str]) -> dict[str, dict[str, str]]:
    """``code:`` of a spec -> the commits, per repository. Unknown repos are an error;
    ``image`` is not a repository and is left out."""
    out = {}
    for name, ref in requested.items():
        if name == "image" or ref in (None, ""):
            continue
        if name not in m.repos:
            raise CodeError(f"code.{name}: not a repository in the manifest")
        out[name] = resolve_ref(m.repos[name], str(ref))
    return out


def _root_repo(m: Manifest) -> str:
    for name, path in m.repos.items():
        if path.resolve() == m.root.resolve():
            return name
    raise CodeError("the manifest names no repository at its own root ('.')")


def prepare(
    m: Manifest,
    resolved: dict[str, dict[str, str]],
    dest: Path,
    log: Callable[[str], None] = print,
) -> Path:
    """A worktree of ``m.root``'s repository at the resolved commits, in ``dest``."""
    root_name = _root_repo(m)
    top = resolved.get(root_name)
    if top is None:
        raise CodeError(f"code.{root_name} is needed to check the code out")
    if dest.exists():
        remove(m.root, dest)
    log(f"fetch {root_name} {top['fetch']} ({top['sha'][:12]})")
    git(m.root, "fetch", "--quiet", "origin", top["fetch"])
    git(m.root, "worktree", "add", "--quiet", "--detach", str(dest), top["sha"])
    for name, path in m.repos.items():
        if name == root_name:
            continue
        rel = path.resolve().relative_to(m.root.resolve()).as_posix()
        log(f"submodule {rel}")
        git(dest, "submodule", "update", "--init", "--quiet", rel)
        if name in resolved:
            sub, r = dest / rel, resolved[name]
            log(f"fetch {name} {r['fetch']} ({r['sha'][:12]})")
            git(sub, "fetch", "--quiet", "origin", r["fetch"])
            git(sub, "checkout", "--quiet", "--detach", r["sha"])
    return dest


def remove(repo: Path, dest: Path) -> None:
    git(repo, "worktree", "remove", "--force", str(dest), check=False)
    if dest.exists():
        shutil.rmtree(dest, ignore_errors=True)
    git(repo, "worktree", "prune", check=False)


# --------------------------------------------------------------------- image
def _docker(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], capture_output=True, text=True)


def published(ref: str) -> bool:
    return _docker("manifest", "inspect", ref).returncode == 0


def choose_image(
    img: Image,
    checkout: Path,
    want: str | None,
    log: Callable[[str], None] = print,
    run: Callable[[list[str], Path], int] | None = None,
) -> tuple[str, str]:
    """(image to use, why). ``want``: ``auto`` (default), ``build``, or a tag/reference."""
    want = (want or "auto").strip()
    head = git(checkout, "rev-parse", "HEAD")
    if want not in ("auto", "build"):
        ref = want if ("/" in want or not img.registry) else f"{img.registry}:{want}"
        return ref, "asked for"

    reason = "asked for a build"
    if want == "auto":
        git(checkout, "fetch", "--quiet", "origin", img.base_branch, check=False)
        base = git(checkout, "merge-base", "HEAD", "FETCH_HEAD", check=False)
        if not base:
            reason = f"no common commit with {img.base_branch}"
        else:
            changed = (
                git(checkout, "diff", "--name-only", base, "HEAD", "--", *img.paths)
                if img.paths
                else ""
            )
            if changed:
                n = len(changed.splitlines())
                reason = f"{n} file(s) under {', '.join(img.paths)} changed since {img.base_branch}"
            elif img.registry:
                for sha in git(
                    checkout, "rev-list", "--first-parent", f"-n{SCAN_BACK}", base
                ).split():
                    ref = f"{img.registry}:{img.tag.format(short=sha[:7], sha=sha)}"
                    if published(ref):
                        log(f"image {ref} (published for {img.base_branch} {sha[:7]})")
                        pull = _docker("pull", "--quiet", ref)
                        if pull.returncode != 0:
                            raise CodeError(f"docker pull {ref}: {pull.stderr.strip()}")
                        return ref, f"published image of {img.base_branch} {sha[:7]}"
                reason = f"no published image in the last {SCAN_BACK} {img.base_branch} commits"
            else:
                reason = "the manifest names no registry"

    if not img.build:
        raise CodeError(
            f"an image must be built ({reason}) but the manifest has no build command"
        )
    tag = img.local_tag.format(sha=head[:12], short=head[:7])
    if _docker("image", "inspect", tag).returncode == 0:
        log(f"image {tag} (built before)")
        return tag, f"built from this code ({reason})"
    cmd = [c.format(tag=tag) for c in img.build]
    log(f"building {tag}: {reason}")
    rc = (run or _run)(cmd, checkout)
    if rc != 0:
        raise CodeError(f"image build failed (exit {rc})")
    return tag, f"built from this code ({reason})"


def _run(cmd: list[str], cwd: Path) -> int:
    return subprocess.call(cmd, cwd=cwd)
