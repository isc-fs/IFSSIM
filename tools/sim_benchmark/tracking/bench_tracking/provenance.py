"""Provenance: code state and bag identity.

Captured on the host (the benchmark container has no ``.git``). For runs
imported after the fact the code state is unknown and recorded as such —
never guessed into the ``code.*`` fields.
"""

from __future__ import annotations

import getpass
import hashlib
import json
import os
import socket
import subprocess
from pathlib import Path
from typing import Any

import yaml

CACHE = Path(
    os.environ.get("BENCH_TRACKING_CACHE", Path.home() / ".cache" / "bench_tracking")
)


def _git(repo: Path, *args: str) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), *args], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except Exception:
        return None


def git_state(repo: Path) -> dict[str, Any]:
    sha = _git(repo, "rev-parse", "HEAD")
    if sha is None:
        return {"sha": None, "branch": None, "dirty": None, "diff_sha": None}
    porcelain = _git(repo, "status", "--porcelain") or ""
    diff = _git(repo, "diff", "HEAD") or ""
    return {
        "sha": sha[:10],
        "branch": _git(repo, "rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(porcelain),
        "diff_sha": hashlib.sha1(diff.encode()).hexdigest()[:8] if diff else None,
    }


def capture_code(ifssim_root: Path) -> dict[str, Any]:
    return {
        "ifssim": git_state(ifssim_root),
        "pipeline": git_state(ifssim_root / "pipeline"),
    }


def unknown_code() -> dict[str, Any]:
    blank = {"sha": None, "branch": None, "dirty": None, "diff_sha": None}
    return {
        "ifssim": dict(blank),
        "pipeline": dict(blank),
        "image": {"id": None, "tag": None},
    }


def env_info() -> dict[str, Any]:
    return {"user": getpass.getuser(), "host": socket.gethostname()}


# ---------------------------------------------------------------- bags
def bag_metadata(bag_dir: Path) -> dict[str, Any]:
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.is_file():
        return {}
    info = yaml.safe_load(meta_path.read_text())["rosbag2_bagfile_information"]
    topics = {
        t["topic_metadata"]["name"]: int(t["message_count"])
        for t in info.get("topics_with_message_count", [])
    }
    return {
        "storage": info.get("storage_identifier"),
        "duration_s": info["duration"]["nanoseconds"] / 1e9,
        "start_ns": info["starting_time"]["nanoseconds_since_epoch"],
        "message_count": info.get("message_count"),
        "topics": topics,
    }


def bag_identity(bag_dir: Path, *, full_hash: bool = True) -> dict[str, Any]:
    """Content id of a rosbag2 dir, cached by (path, size, mtime) of each file.

    ``full_hash`` streams every storage file through SHA-256 (~10 s per 7.5 GB
    on NVMe) once; later calls hit the cache. Without it the id is derived
    from metadata.yaml + file sizes, which is enough to tell bags apart but
    wouldn't notice an in-place edit.
    """
    files = sorted(
        p for p in bag_dir.iterdir() if p.is_file() and not p.name.startswith(".")
    )
    stat_key = [(p.name, p.stat().st_size, int(p.stat().st_mtime)) for p in files]
    CACHE.mkdir(parents=True, exist_ok=True)
    cache_file = CACHE / "bag_ids.json"
    cache = json.loads(cache_file.read_text()) if cache_file.is_file() else {}
    ck = f"{bag_dir.resolve()}::{'full' if full_hash else 'meta'}"
    if ck in cache and cache[ck]["stat"] == stat_key:
        return cache[ck]["id"]

    h = hashlib.sha256()
    for p in files:
        h.update(p.name.encode())
        if p.name == "metadata.yaml" or not full_hash:
            h.update(
                p.read_bytes()
                if p.name == "metadata.yaml"
                else str(p.stat().st_size).encode()
            )
        else:
            fh = hashlib.sha256()
            with p.open("rb") as f:
                while chunk := f.read(8 << 20):
                    fh.update(chunk)
            h.update(fh.digest())
    ident = {
        "name": bag_dir.name,
        "id": h.hexdigest()[:16],
        "id_method": "sha256(files)" if full_hash else "sha256(metadata+sizes)",
        "size_bytes": sum(s for _, s, _ in stat_key),
    }
    cache[ck] = {"stat": stat_key, "id": ident}
    cache_file.write_text(json.dumps(cache, indent=1))
    return ident


def scenario_id(fields: dict[str, Any]) -> str:
    return hashlib.sha1(
        json.dumps(fields, sort_keys=True, default=str).encode()
    ).hexdigest()[:10]
