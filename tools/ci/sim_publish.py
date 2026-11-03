#!/usr/bin/env python3
"""Puts a sim check's result where people read it.

    python3 tools/ci/sim_publish.py --report-dir <dir> --run-url <url> --run-id <id>
        [--pr-repo owner/name --pr <number>] [--source "PR #12"]
        [--baseline <dev report.json>] [--save-baseline <dir>]

1. The track maps (<report dir>/maps/*.png, from sim_plot.py) go to the
   `ci-assets` branch of this repo, which only holds these images: a PR
   comment can't carry an image of its own.
2. The PR (in this repo or the pipeline repo) gets one comment with the
   result, the comparison with dev and the maps. Later runs edit that same
   comment instead of adding new ones.
3. The `sim-report` issue gets a row in its history table; a dev run also
   replaces its "Latest dev" section. The issue is created the first time.
4. --save-baseline keeps this report (with its map links) as the dev result
   that later checks compare with.

Env: GH_TOKEN (this repo: contents, issues and pull requests write),
PR_TOKEN (the PR's repo, if that's another one; default GH_TOKEN),
GITHUB_REPOSITORY (this repo).

Every step is best effort: one that fails is reported and the rest still run,
and the exit code is 1 if any failed.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_report  # noqa: E402

API = "https://api.github.com"
ASSETS_BRANCH = "ci-assets"
ISSUE_LABEL = "sim-report"
ISSUE_TITLE = "Sim check report"
HISTORY_ROWS = 100
ISSUE_MARKER = "<!-- sim-report -->"


class ApiError(Exception):
    pass


def api(method: str, path: str, token: str, body: dict | None = None, ok404: bool = False):
    req = urllib.request.Request(
        API + path,
        method=method,
        data=None if body is None else json.dumps(body).encode(),
        headers={
            "Authorization": f"Bearer {token}",
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
            "User-Agent": "ifssim-sim-check",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            raw = r.read()
            return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as e:
        if ok404 and e.code == 404:
            return None
        detail = e.read().decode(errors="replace")[:300]
        raise ApiError(f"{method} {path}: HTTP {e.code} {detail}") from None


# ------------------------------------------------------------------ 1. maps
def ensure_assets_branch(repo: str, token: str) -> None:
    if api("GET", f"/repos/{repo}/branches/{ASSETS_BRANCH}", token, ok404=True) is not None:
        return
    readme = ("Images the sim check (.github/workflows/sim-pr.yml) links from PR comments and the "
              "`sim-report` issue. Nothing else lives here; old runs can be pruned.\n")
    tree = api("POST", f"/repos/{repo}/git/trees", token,
               {"tree": [{"path": "README.md", "mode": "100644", "type": "blob", "content": readme}]})
    commit = api("POST", f"/repos/{repo}/git/commits", token,
                 {"message": "Images for the sim check", "tree": tree["sha"], "parents": []})
    api("POST", f"/repos/{repo}/git/refs", token,
        {"ref": f"refs/heads/{ASSETS_BRANCH}", "sha": commit["sha"]})


def upload_maps(repo: str, token: str, maps_dir: Path, prefix: str) -> dict[str, str]:
    pngs = sorted(maps_dir.glob("*.png")) if maps_dir.is_dir() else []
    if not pngs:
        return {}
    ensure_assets_branch(repo, token)
    urls = {}
    for png in pngs:
        path = f"{prefix}/{png.name}"
        api("PUT", f"/repos/{repo}/contents/{path}", token, {
            "message": f"sim check map {path}",
            "content": base64.b64encode(png.read_bytes()).decode(),
            "branch": ASSETS_BRANCH,
        })
        urls[png.stem] = f"https://raw.githubusercontent.com/{repo}/{ASSETS_BRANCH}/{path}"
    return urls


# ------------------------------------------------------------------ 2. PR comment
def upsert_comment(repo: str, pr: int, token: str, body: str) -> str:
    for page in range(1, 11):
        comments = api("GET", f"/repos/{repo}/issues/{pr}/comments?per_page=100&page={page}", token)
        for c in comments:
            if (c.get("body") or "").startswith(sim_report.COMMENT_MARKER):
                return api("PATCH", f"/repos/{repo}/issues/comments/{c['id']}", token, {"body": body})["html_url"]
        if len(comments) < 100:
            break
    return api("POST", f"/repos/{repo}/issues/{pr}/comments", token, {"body": body})["html_url"]


# ------------------------------------------------------------------ 3. report issue
INTRO = """{marker}
Every sim check on the GPU box adds a row to the history below: PRs in this repo and in \
isc-fs/IFS09-DV-PIPELINE with the `sim-test` label, and pushes to `dev`. **Latest dev** is \
what the PR checks compare with. Written by `tools/ci/sim_publish.py`; edits here are overwritten.

Required missions decide the check; *italic* ones are informational.
"""


def section(body: str, name: str) -> str:
    start, end = f"<!-- {name} -->", f"<!-- /{name} -->"
    if start in body and end in body:
        return body.split(start, 1)[1].split(end, 1)[0].strip("\n")
    return ""


def issue_body(old: str, row: str, latest: str | None) -> str:
    rows = [ln for ln in section(old, "history").splitlines() if ln.startswith("| 2")]
    rows = [r for r in [row] + rows if r][:HISTORY_ROWS]
    latest = latest if latest is not None else (section(old, "latest") or "No dev run yet.")
    return "\n".join([
        INTRO.format(marker=ISSUE_MARKER),
        "## Latest dev", "<!-- latest -->", latest, "<!-- /latest -->", "",
        "## History", f"Newest first, the last {HISTORY_ROWS} runs.", "",
        "<!-- history -->",
        "| When (UTC) | What | IFSSIM | Pipeline | Result | Missions | Run |",
        "|---|---|---|---|---|---|---|",
        *rows,
        "<!-- /history -->",
    ])


def find_or_create_issue(repo: str, token: str) -> dict:
    found = api("GET", f"/repos/{repo}/issues?labels={ISSUE_LABEL}&state=open&per_page=10", token)
    for i in found:
        if "pull_request" not in i and (i.get("body") or "").startswith(ISSUE_MARKER):
            return i
    try:
        api("POST", f"/repos/{repo}/labels", token,
            {"name": ISSUE_LABEL, "color": "5319e7", "description": "The sim check's running report"})
    except ApiError as e:
        if "already_exists" not in str(e):
            raise
    return api("POST", f"/repos/{repo}/issues", token, {
        "title": ISSUE_TITLE, "labels": [ISSUE_LABEL],
        "body": issue_body("", "", None),
    })


def update_issue(repo: str, token: str, report: dict, source: str, run_url: str,
                 latest: str | None) -> str:
    issue = find_or_create_issue(repo, token)
    when = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M")
    row = "| {} | {} | `{}` | `{}` | {} | {} | [run]({}) |".format(
        when, source, report.get("ifssim") or "?", report.get("pipeline") or "?",
        {"pass": "✅ passed", "fail": "❌ failed"}.get(report["verdict"], "⚠️ couldn't run"),
        sim_report.history_cell(report), run_url,
    )
    body = issue_body(issue.get("body") or "", row, latest)
    api("PATCH", f"/repos/{repo}/issues/{issue['number']}", token, {"body": body})
    return issue["html_url"]


# ------------------------------------------------------------------ main
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--report-dir", type=Path, required=True)
    ap.add_argument("--run-url", default="")
    ap.add_argument("--run-id", default="local")
    ap.add_argument("--pr-repo")
    ap.add_argument("--pr", type=int)
    ap.add_argument("--source", default="run")
    ap.add_argument("--baseline", type=Path)
    ap.add_argument("--save-baseline", type=Path)
    args = ap.parse_args(argv)

    repo = os.environ.get("GITHUB_REPOSITORY", "isc-fs/IFSSIM")
    token = os.environ.get("GH_TOKEN", "")
    pr_token = os.environ.get("PR_TOKEN") or token
    if not token:
        print("::warning::GH_TOKEN isn't set, so the result isn't published")
        return 0
    report = json.loads((args.report_dir / "report.json").read_text())
    baseline = None
    if args.baseline and args.baseline.exists():
        baseline = json.loads(args.baseline.read_text())
    failures = 0

    images: dict[str, str] = {}
    try:
        images = upload_maps(repo, token, args.report_dir / "maps", f"runs/{args.run_id}")
        print(f"maps: {len(images)} uploaded")
    except (ApiError, OSError) as e:
        failures += 1
        print(f"::warning::the maps weren't uploaded: {e}")
    report["images"] = images

    base_note = ""
    if baseline:
        base_note = "compared with dev at IFSSIM `{}`, pipeline `{}`".format(
            baseline.get("ifssim") or "?", baseline.get("pipeline") or "?")
    body = sim_report.comment(report, baseline, images, args.run_url, base_note)

    if args.pr_repo and args.pr:
        try:
            print("PR comment:", upsert_comment(args.pr_repo, args.pr, pr_token, body))
        except ApiError as e:
            failures += 1
            print(f"::warning::the PR comment wasn't posted: {e}")

    latest = None
    if args.save_baseline and report["verdict"] in ("pass", "fail"):
        latest = "\n".join([sim_report.headline(report) + f" · [run]({args.run_url})", "",
                            sim_report.tables(report, None), sim_report.maps(images)])
    try:
        print("report issue:", update_issue(repo, token, report, args.source, args.run_url, latest))
    except ApiError as e:
        failures += 1
        print(f"::warning::the report issue wasn't updated: {e}")

    if args.save_baseline and report["verdict"] in ("pass", "fail"):
        args.save_baseline.mkdir(parents=True, exist_ok=True)
        tmp = args.save_baseline / "report.json.tmp"
        tmp.write_text(json.dumps(report, indent=2))
        tmp.replace(args.save_baseline / "report.json")
        print(f"saved as the dev baseline: {args.save_baseline / 'report.json'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
