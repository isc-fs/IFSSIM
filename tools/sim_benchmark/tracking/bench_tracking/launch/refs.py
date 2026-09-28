"""What code there is to test: branches, pull requests and commits of each repository.

The Launch page lists them so a version is picked, not typed. It reads a small
mirror of each repository (``git clone --bare --filter=tree:0``: commits and
refs only, about 2 MB for IFSSIM) that it refreshes in the background, at most
once a minute. Git has no request limit and needs no login for a public
repository, and the same mirror answers "which pipeline commit does this IFSSIM
commit pin" (``pin``).

Only PR titles, and which PRs are still open, come from the GitHub API
(``github_pulls``), cached for a few minutes. Without it (not GitHub, no
network, rate limit: 60 requests an hour without ``GITHUB_TOKEN``) the page
still lists the PRs, by their head commit's subject.

    BENCH_CACHE     where the mirrors live (default ~/.cache/ifssim-bench)
    GITHUB_TOKEN    optional; a read-only token lifts the API limit to 5000 an hour
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import threading
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from .manifest import Manifest

FETCH_EVERY_S = 60.0
GITHUB_TTL_S = 300.0
SEP = "\x1f"
FMT = SEP.join(
    [
        "%(refname)",
        "%(objectname)",
        "%(committerdate:unix)",
        "%(authorname)",
        "%(subject)",
    ]
)
LOG_FMT = SEP.join(["%H", "%ct", "%an", "%s"])
GITHUB = re.compile(r"github\.com[:/]+([^/]+)/([^/]+?)(?:\.git)?/?$")


@dataclass(frozen=True)
class Commit:
    sha: str
    when: float  # unix time
    author: str
    subject: str

    @property
    def short(self) -> str:
        return self.sha[:7]


@dataclass(frozen=True)
class Ref:
    kind: str  # "branch" | "pr"
    name: str  # branch name, or "#123"
    commit: Commit
    title: str | None = None  # a PR's title (GitHub), when known
    open: bool | None = None  # a PR's state (GitHub), when known


def cache_root() -> Path:
    return Path(os.environ.get("BENCH_CACHE") or "~/.cache/ifssim-bench").expanduser()


def github_slug(url: str) -> str | None:
    m = GITHUB.search(url.strip())
    return f"{m.group(1)}/{m.group(2)}" if m else None


def web_url(url: str, sha: str | None = None) -> str | None:
    slug = github_slug(url)
    if not slug:
        return None
    return f"https://github.com/{slug}" + (f"/commit/{sha}" if sha else "")


def _git(d: Path, *args: str, check: bool = True, timeout: float = 120) -> str:
    r = subprocess.run(
        ["git", "-C", str(d), *args], capture_output=True, text=True, timeout=timeout
    )
    if check and r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout


class Mirror:
    """A refs-and-commits-only copy of one remote repository."""

    def __init__(self, url: str, root: Path | None = None) -> None:
        self.url = url
        name = re.sub(r"[^A-Za-z0-9._-]+", "_", github_slug(url) or url).strip("_")
        self.path = (root or cache_root()) / "mirrors" / f"{name}.git"
        self._lock = threading.Lock()
        self._fetching = False
        self.error: str | None = None

    # ------------------------------------------------------------ keeping it fresh
    @property
    def fetched_at(self) -> float | None:
        f = self.path / "FETCH_HEAD"
        return f.stat().st_mtime if f.exists() else None

    def ensure(self) -> bool:
        """Cloned (the first time: a second or two)? Failures are kept in ``error``."""
        if (self.path / "HEAD").exists():
            return True
        with self._lock:
            if (self.path / "HEAD").exists():
                return True
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.path.with_suffix(".tmp")
                subprocess.run(["rm", "-rf", str(tmp)], check=False)
                r = subprocess.run(
                    [
                        "git",
                        "clone",
                        "--quiet",
                        "--bare",
                        "--filter=tree:0",
                        self.url,
                        str(tmp),
                    ],
                    capture_output=True,
                    text=True,
                    timeout=300,
                )
                if r.returncode != 0:
                    raise RuntimeError(r.stderr.strip())
                tmp.rename(self.path)
                self._fetch()
                self.error = None
                return True
            except Exception as e:  # noqa: BLE001
                self.error = f"cannot copy {self.url}: {e}"
                return False

    def _fetch(self) -> None:
        _git(
            self.path,
            "fetch",
            "--quiet",
            "--prune",
            "--filter=tree:0",
            "origin",
            "+refs/heads/*:refs/heads/*",
            "+refs/pull/*/head:refs/pull/*/head",
            timeout=300,
        )

    def refresh(self, max_age: float = FETCH_EVERY_S, wait: bool = False) -> None:
        """Fetch if the copy is older than ``max_age``; in the background unless ``wait``."""
        if not self.ensure():
            return
        age = time.time() - (self.fetched_at or 0)
        if age < max_age or self._fetching:
            return
        self._fetching = True

        def run() -> None:
            try:
                self._fetch()
                self.error = None
            except Exception as e:  # noqa: BLE001
                self.error = f"cannot update {self.url}: {e}"
            finally:
                self._fetching = False

        if wait:
            run()
        else:
            threading.Thread(target=run, daemon=True).start()

    # ------------------------------------------------------------ reading
    def _refs(self, pattern: str) -> list[tuple[str, Commit]]:
        out = _git(
            self.path,
            "for-each-ref",
            "--sort=-committerdate",
            f"--format={FMT}",
            pattern,
            check=False,
        )
        rows = []
        for line in out.splitlines():
            ref, sha, when, author, subject = (line.split(SEP) + [""] * 5)[:5]
            rows.append((ref, Commit(sha, float(when or 0), author, subject)))
        return rows

    def branches(self) -> list[Ref]:
        return [
            Ref("branch", ref.removeprefix("refs/heads/"), c)
            for ref, c in self._refs("refs/heads")
        ]

    def pulls(self, github: dict[int, dict] | None = None) -> list[Ref]:
        out = []
        for ref, c in self._refs("refs/pull"):
            m = re.match(r"refs/pull/(\d+)/head$", ref)
            if not m:
                continue
            n = int(m.group(1))
            gh = (github or {}).get(n)
            out.append(
                Ref(
                    "pr",
                    f"#{n}",
                    c,
                    title=gh.get("title") if gh else None,
                    open=(gh is not None) if github else None,
                )
            )
        return out

    def commits(self, rev: str, n: int = 40) -> list[Commit]:
        out = _git(
            self.path, "log", f"-n{n}", f"--format={LOG_FMT}", rev, "--", check=False
        )
        rows = []
        for line in out.splitlines():
            sha, when, author, subject = (line.split(SEP) + [""] * 4)[:4]
            rows.append(Commit(sha, float(when or 0), author, subject))
        return rows

    def commit(self, rev: str) -> Commit | None:
        rows = self.commits(rev, 1) if rev else []
        return rows[0] if rows else None

    def resolve(self, prefix: str) -> Commit | None:
        """A commit from a (short) id, if the copy has it."""
        if not re.fullmatch(r"[0-9a-fA-F]{4,40}", prefix or ""):
            return None
        sha = _git(
            self.path,
            "rev-parse",
            "--verify",
            "--quiet",
            f"{prefix}^{{commit}}",
            check=False,
        ).strip()
        return self.commit(sha) if sha else None

    def submodule_url(self, rev: str, path: str) -> str | None:
        """The URL a submodule at ``path`` had at ``rev`` (from that commit's .gitmodules)."""
        out = _git(
            self.path,
            "config",
            "--blob",
            f"{rev}:.gitmodules",
            "--get",
            f"submodule.{path}.url",
            check=False,
            timeout=60,
        )
        return out.strip() or None

    def pin(self, rev: str, path: str) -> str | None:
        """The commit a submodule at ``path`` is pinned to at ``rev`` (trees are fetched on demand)."""
        out = _git(self.path, "ls-tree", rev, "--", path, check=False, timeout=60)
        parts = out.split()
        return parts[2] if len(parts) >= 3 and parts[1] == "commit" else None


# ------------------------------------------------------------------ GitHub (PR titles)
_gh_cache: dict[str, tuple[float, dict[int, dict]]] = {}


def github_pulls(url: str) -> dict[int, dict] | None:
    """Open PRs by number (title, author, head branch), or None when GitHub can't say."""
    slug = github_slug(url)
    if not slug:
        return None
    hit = _gh_cache.get(slug)
    if hit and time.time() - hit[0] < GITHUB_TTL_S:
        return hit[1]
    req = urllib.request.Request(
        f"https://api.github.com/repos/{slug}/pulls?state=open&per_page=100",
        headers={"Accept": "application/vnd.github+json", "User-Agent": "bench-view"},
    )
    token = os.environ.get("GITHUB_TOKEN")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            data = json.load(r)
    except Exception:  # noqa: BLE001  (rate limit, offline: the page manages without)
        return hit[1] if hit else None
    out = {
        int(p["number"]): {
            "title": p.get("title"),
            "author": (p.get("user") or {}).get("login"),
            "branch": (p.get("head") or {}).get("ref"),
            "draft": p.get("draft"),
        }
        for p in data
        if isinstance(p, dict) and "number" in p
    }
    _gh_cache[slug] = (time.time(), out)
    return out


# ------------------------------------------------------------------ per repository
def repo_urls(m: Manifest) -> dict[str, str]:
    """Remote URL of each repository in the manifest: the root's ``origin``, and each
    submodule's URL from the root's ``.gitmodules``."""
    out = {}
    for name, path in m.repos.items():
        if name in m.repo_urls:
            out[name] = m.repo_urls[name]
            continue
        try:
            if path.resolve() == m.root.resolve():
                out[name] = _git(m.root, "remote", "get-url", "origin").strip()
            else:
                rel = path.resolve().relative_to(m.root.resolve()).as_posix()
                out[name] = _git(
                    m.root,
                    "config",
                    "-f",
                    ".gitmodules",
                    "--get",
                    f"submodule.{rel}.url",
                ).strip()
        except Exception:  # noqa: BLE001
            continue
    return out


_mirrors: dict[str, Mirror] = {}


def mirror(url: str) -> Mirror:
    if url not in _mirrors:
        _mirrors[url] = Mirror(url)
    return _mirrors[url]
