"""The Benchmarks tab's *Runner* target: specs run on the bench PC through GitHub.

Launching starts ``.github/workflows/bench.yml`` with the spec as its input (the
same ``run_spec.py`` a Local run uses, on the GPU runner). The queue is GitHub's:
every workflow that waits for the GPU runner (bench runs, the sim check, the
nightly suites), newest first, because they all wait for the same machine. Fetching
results downloads a run's ``bench-<id>`` artifact into this machine's results folder,
where a Local run's results go, so both show the same way.

Signing in: GitHub's device flow (Mission Control shows a code, you enter it on
github.com) with the OAuth or GitHub App client id in ``IFSSIM_GITHUB_CLIENT_ID``,
or a token in ``IFSSIM_GITHUB_TOKEN`` / ``GH_TOKEN`` (e.g. ``gh auth token``). The
token stays on this machine (``<results>/.bench/github.json``, readable only by
you). GitHub decides who may launch: anyone with write access to the repository.
Reading the queue needs no sign-in (the repository is public).
"""

from __future__ import annotations

import io
import json
import os
import re
import statistics
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

REPO = os.environ.get("IFSSIM_GITHUB_REPO", "isc-fs/IFSSIM")
WORKFLOW = "bench.yml"
DISPATCH_REF = os.environ.get("IFSSIM_BENCH_REF", "dev")  # where bench.yml is read from
# The workflows that wait for the GPU runner; the queue shows them all.
GPU_WORKFLOWS = ("bench.yml", "sim-pr.yml", "nightly.yml")
TOKEN_FILE = "github.json"
QUEUE_TTL_S = 15.0  # signed in: 5000 calls an hour
ANON_TTL_S = 120.0  # not: 60 an hour, shared by everyone on this address
OFFLINE_AFTER_S = 300.0  # queued this long with nothing running: is the runner online?
ID_RE = re.compile(r"\[([0-9a-z-]{4,64})\]\s*$")
# what a fetched artifact keeps out of the results folder: the runner's own job folders
PRIVATE_DIRS = (".bench", "headless")


class RunnerError(RuntimeError):
    """A runner request that failed; ``status_code`` is the HTTP status to answer with."""

    def __init__(self, message: str, status_code: int = 400):
        super().__init__(message)
        self.status_code = status_code


# ------------------------------------------------------------------ HTTP
Http = Callable[..., Any]


def _http(method: str, url: str, *, token: str | None = None, data: Any = None,
          form: bool = False, raw: bool = False, timeout: float = 15.0) -> Any:
    """One request; JSON back (or bytes with ``raw``), None for an empty answer."""
    headers = {"Accept": "application/vnd.github+json", "User-Agent": "ifssim-mission-control"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    body = None
    if data is not None:
        if form:
            body = urllib.parse.urlencode(data).encode()
            headers["Accept"] = "application/json"
        else:
            body = json.dumps(data).encode()
            headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            payload = r.read()
    except urllib.error.HTTPError as e:
        detail = ""
        try:
            detail = json.loads(e.read() or b"{}").get("message", "")
        except ValueError:
            pass
        raise RunnerError(f"GitHub said {e.code}{f': {detail}' if detail else ''}", _status(e.code)) from e
    except (urllib.error.URLError, OSError) as e:
        raise RunnerError(f"couldn't reach GitHub ({e})", 502) from e
    if raw:
        return payload
    return json.loads(payload) if payload else None


def _status(code: int) -> int:
    # GitHub's 401/403/404 on a write mean "not allowed": say so as 403
    return {401: 401, 403: 403, 404: 404, 422: 400}.get(code, 502)


http: Http = _http  # tests replace it


def api(method: str, path: str, **kw: Any) -> Any:
    return http(method, f"https://api.github.com/{path.lstrip('/')}", **kw)


# ------------------------------------------------------------------ signing in
def _token_path(bench_dir: Path) -> Path:
    return bench_dir / TOKEN_FILE


def stored(bench_dir: Path) -> dict[str, Any]:
    try:
        return json.loads(_token_path(bench_dir).read_text())
    except (OSError, ValueError):
        return {}


def token(bench_dir: Path) -> tuple[str | None, str]:
    """(token, where it came from): the environment wins over a sign-in."""
    for env in ("IFSSIM_GITHUB_TOKEN", "GH_TOKEN"):
        if os.environ.get(env, "").strip():
            return os.environ[env].strip(), env
    t = stored(bench_dir).get("token")
    return (t, "sign-in") if t else (None, "")


def _save(bench_dir: Path, doc: dict[str, Any]) -> None:
    bench_dir.mkdir(parents=True, exist_ok=True)
    p = _token_path(bench_dir)
    tmp = p.with_suffix(".tmp")
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        json.dump(doc, f)
    os.replace(tmp, p)


def client_id() -> str:
    return os.environ.get("IFSSIM_GITHUB_CLIENT_ID", "").strip()


_device: dict[str, Any] = {}
_lock = threading.Lock()


def sign_in_start() -> dict[str, Any]:
    """Step 1 of the device flow: the code to enter on github.com."""
    cid = client_id()
    if not cid:
        raise RunnerError(
            "sign-in isn't set up: set IFSSIM_GITHUB_CLIENT_ID (the team's GitHub app), "
            "or give a token in IFSSIM_GITHUB_TOKEN (e.g. `gh auth token`)",
            501,
        )
    out = http("POST", "https://github.com/login/device/code", data={"client_id": cid, "scope": "public_repo"}, form=True)
    if not out or "device_code" not in out:
        raise RunnerError(f"GitHub didn't start the sign-in: {out}", 502)
    with _lock:
        _device.clear()
        _device.update(code=out["device_code"], interval=int(out.get("interval", 5)),
                       expires=time.time() + int(out.get("expires_in", 900)), next=0.0)
    return {
        "user_code": out["user_code"],
        "verification_uri": out.get("verification_uri", "https://github.com/login/device"),
        "expires_in": int(out.get("expires_in", 900)),
        "interval": int(out.get("interval", 5)),
    }


def sign_in_poll(bench_dir: Path) -> dict[str, Any]:
    """Step 2: ask GitHub whether the code was entered; ``pending`` until it is."""
    with _lock:
        if not _device:
            return {"state": "none"}
        if time.time() > _device["expires"]:
            _device.clear()
            return {"state": "expired"}
        if time.time() < _device["next"]:
            return {"state": "pending"}
        _device["next"] = time.time() + _device["interval"]
        code = _device["code"]
    out = http("POST", "https://github.com/login/oauth/access_token", form=True, data={
        "client_id": client_id(), "device_code": code,
        "grant_type": "urn:ietf:params:oauth:grant-type:device_code",
    }) or {}
    err = out.get("error")
    if err == "authorization_pending":
        return {"state": "pending"}
    if err == "slow_down":
        with _lock:
            _device["interval"] = int(out.get("interval", _device.get("interval", 5) + 5))
        return {"state": "pending"}
    if err or "access_token" not in out:
        with _lock:
            _device.clear()
        return {"state": "failed", "error": out.get("error_description") or err or "no token"}
    user = api("GET", "user", token=out["access_token"]) or {}
    _save(bench_dir, {"token": out["access_token"], "login": user.get("login"), "at": time.time()})
    with _lock:
        _device.clear()
    return {"state": "signed_in", "login": user.get("login")}


def sign_out(bench_dir: Path) -> dict[str, Any]:
    _token_path(bench_dir).unlink(missing_ok=True)
    return {"signed_in": False}


def account(bench_dir: Path) -> dict[str, Any]:
    """Who Mission Control launches as, and whether they may (write access)."""
    t, source = token(bench_dir)
    out: dict[str, Any] = {"signed_in": bool(t), "source": source, "repo": REPO,
                           "sign_in_available": bool(client_id())}
    if not t:
        return out
    login = stored(bench_dir).get("login") if source == "sign-in" else None
    try:
        login = login or (api("GET", "user", token=t) or {}).get("login")
        perms = (api("GET", f"repos/{REPO}", token=t) or {}).get("permissions") or {}
        out["can_launch"] = bool(perms.get("push") or perms.get("admin") or perms.get("maintain"))
    except RunnerError as e:
        out["error"] = str(e)
    out["login"] = login
    return out


def _need_token(bench_dir: Path) -> str:
    t, _ = token(bench_dir)
    if not t:
        raise RunnerError("sign in to GitHub first (Benchmarks → Setup → Runner)", 401)
    return t


# ------------------------------------------------------------------ launching
def new_id() -> str:
    return uuid.uuid4().hex[:8]


def launch(bench_dir: Path, spec: dict[str, Any], name: str | None = None) -> dict[str, Any]:
    """Start bench.yml with this spec; the run's name carries ``[<id>]`` so the queue
    finds it again."""
    t = _need_token(bench_dir)
    jid = new_id()
    blob = json.dumps(spec, separators=(",", ":"), default=str)
    if len(blob) > 60000:
        raise RunnerError("the spec is too big for a workflow input (60 kB)")
    name = (name or spec.get("name") or "custom spec").strip()[:100]
    api("POST", f"repos/{REPO}/actions/workflows/{WORKFLOW}/dispatches", token=t,
        data={"ref": DISPATCH_REF, "inputs": {"spec": blob, "name": name, "id": jid}})
    _cache.clear()
    return {
        "id": jid,
        "target": "runner",
        "name": name,
        "url": f"https://github.com/{REPO}/actions/workflows/{WORKFLOW}",
        "hint": "started on GitHub: it shows in the queue once GitHub has created the run (a few seconds)",
    }


# ------------------------------------------------------------------ the queue
_cache: dict[str, tuple[float, Any]] = {}


def _cached(key: str, ttl: float, fetch: Callable[[], Any]) -> Any:
    hit = _cache.get(key)
    if hit and time.time() - hit[0] < ttl:
        return hit[1]
    val = fetch()
    _cache[key] = (time.time(), val)
    return val


def _ts(s: str | None) -> float | None:
    if not s:
        return None
    return datetime.strptime(s, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc).timestamp()


def _state(run: dict[str, Any]) -> str:
    st = run.get("status")
    if st == "completed":
        c = run.get("conclusion")
        return {"success": "done", "cancelled": "cancelled", "skipped": "cancelled"}.get(c, "failed")
    return "running" if st == "in_progress" else "queued"


def _durations(runs: list[dict[str, Any]]) -> dict[str, float]:
    """Median seconds of the finished runs of each workflow: the time-left estimate."""
    by: dict[str, list[float]] = {}
    for r in runs:
        if r.get("status") == "completed" and r.get("conclusion") == "success":
            a, b = _ts(r.get("run_started_at")), _ts(r.get("updated_at"))
            if a and b and b > a:
                by.setdefault(r["_workflow"], []).append(b - a)
    return {k: statistics.median(v) for k, v in by.items()}


def _current_step(run_id: int, t: str | None) -> str | None:
    def fetch() -> str | None:
        jobs = (api("GET", f"repos/{REPO}/actions/runs/{run_id}/jobs", token=t) or {}).get("jobs") or []
        for j in jobs:
            if j.get("status") == "in_progress":
                for s in j.get("steps") or []:
                    if s.get("status") == "in_progress":
                        return f"{j.get('name')}: {s.get('name')}"
                return j.get("name")
        return None

    return _cached(f"steps:{run_id}", QUEUE_TTL_S, fetch)


def queue(bench_dir: Path, limit: int = 30, now: float | None = None) -> dict[str, Any]:
    """GitHub's runs of the workflows that use the GPU runner, newest first, with the
    state, who and what, the current step, and a time-left estimate."""
    t, _ = token(bench_dir)
    now = time.time() if now is None else now
    # One call for every workflow, cached: without a sign-in GitHub allows 60 calls an
    # hour, and the panel asks every few seconds.
    ttl = QUEUE_TTL_S if t else ANON_TTL_S

    def fetch() -> list[dict[str, Any]]:
        got = api("GET", f"repos/{REPO}/actions/runs?per_page=100", token=t) or {}
        out = []
        for r in got.get("workflow_runs") or []:
            wf = (r.get("path") or "").rsplit("/", 1)[-1].split("@")[0]
            if wf in GPU_WORKFLOWS:
                out.append(r | {"_workflow": wf})
        return out

    try:
        runs = list(_cached("runs", ttl, fetch))
    except RunnerError as e:
        return {"error": str(e), "runs": [], "runner_offline": False}

    runs.sort(key=lambda r: r.get("created_at") or "", reverse=True)
    med = _durations(runs)
    active = [r for r in runs if r.get("status") != "completed"]
    running = [r for r in active if r.get("status") == "in_progress"]
    waiting = sorted((r for r in active if r.get("status") != "in_progress"), key=lambda r: r.get("created_at") or "")
    out = []
    for r in runs[:limit]:
        state = _state(r)
        title = r.get("display_title") or r.get("name") or ""
        m = ID_RE.search(title)
        item: dict[str, Any] = {
            "run_id": r["id"],
            "id": m.group(1) if m else None,
            "workflow": r["_workflow"],
            "name": title,
            "state": state,
            "event": r.get("event"),
            "actor": (r.get("triggering_actor") or r.get("actor") or {}).get("login"),
            "branch": r.get("head_branch"),
            "sha": (r.get("head_sha") or "")[:7],
            "created": r.get("created_at"),
            "started": r.get("run_started_at"),
            "url": r.get("html_url"),
            "current": None,
            "eta_s": None,
        }
        typical = med.get(r["_workflow"])
        if state == "running":
            item["current"] = _current_step(r["id"], t) if t else None
            started = _ts(r.get("run_started_at"))
            if typical and started:
                item["eta_s"] = round(max(0.0, typical - (now - started)))
        elif state == "queued":
            ahead = waiting[: waiting.index(r)] if r in waiting else []
            left = sum(med.get(x["_workflow"], 0.0) for x in ahead)
            for x in running:
                s = _ts(x.get("run_started_at"))
                left += max(0.0, med.get(x["_workflow"], 0.0) - (now - s)) if s else 0.0
            item["ahead"] = len(ahead) + len(running)
            if typical:
                item["eta_s"] = round(left + typical)
        out.append(item)
    oldest = min((_ts(r.get("created_at")) or now for r in waiting), default=now)
    offline = bool(waiting) and not running and now - oldest > OFFLINE_AFTER_S
    return {"error": None, "runs": out, "runner_offline": offline}


def cancel(bench_dir: Path, run_id: int) -> dict[str, Any]:
    t = _need_token(bench_dir)
    api("POST", f"repos/{REPO}/actions/runs/{int(run_id)}/cancel", token=t)
    _cache.clear()
    return {"run_id": run_id, "state": "cancelling"}


# ------------------------------------------------------------------ fetching results
def fetch_results(bench_dir: Path, results: Path, run_id: int) -> dict[str, Any]:
    """Download a run's ``bench-*`` artifact into ``results``: the measurements land
    where a Local run writes them (``mission/...``), and the runner's own job folders
    (``.bench``, ``headless``) under ``runner/<run_id>/``."""
    t = _need_token(bench_dir)  # GitHub serves artifacts only to a signed-in user
    arts = (api("GET", f"repos/{REPO}/actions/runs/{int(run_id)}/artifacts", token=t) or {}).get("artifacts") or []
    art = next((a for a in arts if a.get("name", "").startswith("bench-") and not a.get("expired")), None)
    if art is None:
        raise RunnerError(f"run {run_id} has no results to fetch (not a bench run, not finished, or expired)", 404)
    blob = http("GET", art["archive_download_url"], token=t, raw=True, timeout=120)
    files = _extract(blob, results, Path("runner") / str(int(run_id)))
    return {"run_id": run_id, "artifact": art["name"], "files": files, "results": str(results)}


def _extract(blob: bytes, results: Path, private: Path) -> int:
    root = results.resolve()
    n = 0
    with zipfile.ZipFile(io.BytesIO(blob)) as z:
        for info in z.infolist():
            if info.is_dir():
                continue
            rel = Path(info.filename)
            if rel.is_absolute() or ".." in rel.parts:
                continue  # never outside the results folder
            if rel.parts and rel.parts[0] in PRIVATE_DIRS:
                rel = private / rel
            dest = (root / rel).resolve()
            if root not in dest.parents:
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            with z.open(info) as src, open(dest, "wb") as out:
                out.write(src.read())
            n += 1
    return n
