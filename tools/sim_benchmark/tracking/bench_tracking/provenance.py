"""Provenance: code state and bag identity.

The code state is recorded when a benchmark runs (``tools/sim_benchmark/run_provenance.py``
writes ``provenance.json`` into the run dir); this module only reads it. For
runs made before that existed the code state is unknown and recorded as such —
never guessed into the ``code.*`` fields.
"""

from __future__ import annotations

import getpass
import hashlib
import json
import os
import socket
import time
from pathlib import Path
from typing import Any

import yaml

CACHE = Path(
    os.environ.get("BENCH_TRACKING_CACHE", Path.home() / ".cache" / "bench_tracking")
)


# spec_id of a run that set no settings or overrides (bench_tracking/launch/spec.py), and of
# every run made without bench-run
DEFAULT_SPEC = "default"


def unknown_code() -> dict[str, Any]:
    blank = {"sha": None, "branch": None, "dirty": None, "diff_sha": None}
    return {
        "ifssim": dict(blank),
        "pipeline": dict(blank),
        "image": {"id": None, "tag": None},
        "id": None,
        "label": "unknown",
        "spec_id": DEFAULT_SPEC,
    }


def variant_of(code: dict[str, Any]) -> str:
    """What makes two runs of a scenario reruns of each other: the same code and the
    same spec (settings and parameter overrides, ``default`` when there were none)."""
    key = code.get("id") or "unknown"
    spec = code.get("spec_id") or DEFAULT_SPEC
    return key if spec == DEFAULT_SPEC else f"{key}+{spec}"


def read_run(run_dir: Path) -> dict[str, Any] | None:
    """The ``provenance.json`` a benchmark wrote at run time (``run_provenance.py``), if any."""
    p = Path(run_dir) / "provenance.json"
    try:
        return json.loads(p.read_text()) if p.is_file() else None
    except ValueError:
        return None


def code_of(prov: dict[str, Any] | None) -> dict[str, Any]:
    """``config.code`` for a run: what was recorded at run time, else explicitly unknown."""
    if not prov or not prov.get("captured"):
        return unknown_code()
    code = unknown_code()
    for k in ("ifssim", "pipeline", "image"):
        code[k].update(
            {
                kk: vv
                for kk, vv in (prov.get("code", {}).get(k) or {}).items()
                if not isinstance(vv, list)
            }
        )
    code["id"] = prov.get("code_id")
    code["label"] = prov.get("label") or "unknown"
    # which run this was and where it ran (identifies the run on every machine)
    code["capture_id"] = prov.get("capture_id")
    code["host"] = prov.get("host")
    code["spec_id"] = prov.get("spec_id") or DEFAULT_SPEC
    return code


# A run that started but has not written its results for this long is taken as crashed.
STALE_AFTER_S = 6 * 3600


def in_progress(run_dir: Path, finished: bool | None = None) -> bool:
    """True for a run that is still going: it recorded its code at the start
    (``provenance.json``, only runs made since that exists), has not finished
    (``finished``, default: it has no ``results.json`` yet), and something in it
    changed recently. Uploading it now would upload half a run."""
    d = Path(run_dir)
    if finished is None:
        finished = (d / "results.json").is_file()
    if finished or not (d / "provenance.json").is_file():
        return False
    newest = max((p.stat().st_mtime for p in d.rglob("*")), default=d.stat().st_mtime)
    return time.time() - newest < STALE_AFTER_S


def provenance_files(run_dir: Path) -> list[Path]:
    return [
        p
        for p in (
            [Path(run_dir) / "provenance.json", Path(run_dir) / "spec.json"]
            + sorted(Path(run_dir).glob("*.diff"))
            + sorted(Path(run_dir).glob("*.unpushed.bundle"))
        )
        if p.is_file()
    ]


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
    # lists, not tuples: the cache goes through JSON and must compare equal when read back
    stat_key = [[p.name, p.stat().st_size, int(p.stat().st_mtime)] for p in files]
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
