#!/usr/bin/env python3
"""Which IFSSIM commit a bench run tests (.github/workflows/bench.yml, its first job).

    GH_TOKEN=... SPEC='{"code": {"ifssim": "#880"}, ...}' python3 tools/ci/bench_resolve.py

The workflow's ``ifssim`` input, else the ``code.ifssim`` of a spec given as JSON: a
branch, a PR of this repository ("#880") or a commit. Without either (a preset that
pins nothing) the run tests the commit the workflow was started on. A PR from a fork is refused: the bench PC only runs
this repository's code (the roadmap's Security section).

Writes ``sha`` and ``what`` to ``$GITHUB_OUTPUT`` (or prints them) and exits 1 with a
reason when the ref can't be used.
"""

from __future__ import annotations

import json
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable

Api = Callable[[str], Any]


class ResolveError(RuntimeError):
    pass


def github_api(repo: str, token: str) -> Api:
    def get(path: str) -> Any:
        req = urllib.request.Request(
            f"https://api.github.com/repos/{repo}/{path}",
            headers={
                "Accept": "application/vnd.github+json",
                "Authorization": f"Bearer {token}",
                "User-Agent": "ifssim-bench",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                return json.load(r)
        except urllib.error.HTTPError as e:
            raise ResolveError(f"GitHub said {e.code} for {path}") from e

    return get


def spec_ref(spec_input: str) -> str | None:
    """``code.ifssim`` of a spec given as JSON; None for a preset name or no pin."""
    text = (spec_input or "").strip()
    if not text.startswith("{"):
        return None
    try:
        spec = json.loads(text)
    except ValueError as e:
        raise ResolveError(f"the spec isn't valid JSON: {e}") from e
    ref = ((spec.get("code") or {}) if isinstance(spec, dict) else {}).get("ifssim")
    return str(ref).strip() if ref not in (None, "") else None


def resolve(ref: str | None, default_sha: str, repo: str, api: Api) -> tuple[str, str]:
    """(commit, what it is) for a branch, "#PR" or commit; the default without a ref."""
    if not ref:
        return default_sha, f"the workflow's commit {default_sha[:7]}"
    pr = re.fullmatch(r"(?:#|pr/|pull/)(\d+)", ref)
    if pr:
        p = api(f"pulls/{pr.group(1)}")
        head = p.get("head") or {}
        if (head.get("repo") or {}).get("full_name") != repo:
            raise ResolveError(
                f"PR #{pr.group(1)} comes from a fork: the bench PC only runs {repo}'s own branches"
            )
        return head["sha"], f"PR #{pr.group(1)} ({head.get('ref')}) at {head['sha'][:7]}"
    if re.fullmatch(r"[0-9a-f]{7,40}", ref):
        c = api(f"commits/{ref}")
        return c["sha"], f"commit {c['sha'][:7]}"
    b = api(f"branches/{urllib.parse.quote(ref, safe='')}")
    sha = b["commit"]["sha"]
    return sha, f"branch {ref} at {sha[:7]}"


def main() -> int:
    repo = os.environ["GITHUB_REPOSITORY"]
    try:
        sha, what = resolve(
            os.environ.get("IFSSIM", "").strip() or spec_ref(os.environ.get("SPEC", "")),
            os.environ["GITHUB_SHA"],
            repo,
            github_api(repo, os.environ.get("GH_TOKEN", "")),
        )
    except ResolveError as e:
        print(f"::error::code.ifssim: {e}")
        return 1
    print(f"IFSSIM: {what}")
    out = os.environ.get("GITHUB_OUTPUT")
    if out:
        with open(out, "a") as f:
            f.write(f"sha={sha}\nwhat={what}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
