"""What this machine can run, and whether each part can be used right now.

A rig is a machine that runs benchmarks: the bench PC, or your desk with a sim and a
LattePanda. Its descriptor says what it has; ``check`` tests each part now. Mission
Control colours its component boxes from the result:

    ready        usable now
    unavailable  the rig has it, but not now (yellow; a queued job waits)
    absent       the rig doesn't have it (grey)

The descriptor is ``$IFSSIM_RIG``, else ``~/.config/ifssim/rig.yaml``; without one,
the rig is this machine with no LattePanda and no HIL bench. Example::

    id: alberto-desk
    latte_panda: {host: 192.168.50.2, user: isc}   # checked over SSH
    hil: {bench: bench-01}                         # IFS_HIL's descriptor id

The bench PC's descriptor is ``rigs/bench-pc.yaml`` (its runner sets ``IFSSIM_RIG`` to it).
"""

from __future__ import annotations

import os
import socket
import subprocess
import time
from pathlib import Path
from typing import Any, Callable

import yaml

ENV = "IFSSIM_RIG"
USER_FILE = Path("~/.config/ifssim/rig.yaml")
READY, UNAVAILABLE, ABSENT = "ready", "unavailable", "absent"
# Whether a sim run can include boards (the roadmap's Phase 3). Turns vHIL on for any
# rig with Docker; real boards also need a rig whose descriptor declares a HIL bench.
CAR_IN_THE_LOOP = False

Runner = Callable[[list[str], float], subprocess.CompletedProcess]


def _run(cmd: list[str], timeout: float) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False
        )
    except FileNotFoundError:
        return subprocess.CompletedProcess(cmd, 127, "", f"{cmd[0]}: not installed")
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", f"{cmd[0]}: no answer in {timeout:.0f} s")


def descriptor_path() -> Path | None:
    p = os.environ.get(ENV)
    if p:
        return Path(p).expanduser()
    user = USER_FILE.expanduser()
    return user if user.is_file() else None


def load(path: Path | None = None) -> dict[str, Any]:
    """The rig's descriptor; without one, this machine with nothing attached."""
    path = path or descriptor_path()
    doc: dict[str, Any] = {}
    if path is not None:
        doc = yaml.safe_load(Path(path).read_text()) or {}
        if not isinstance(doc, dict):
            raise ValueError(f"{path}: a rig descriptor is a mapping")
    doc.setdefault("id", socket.gethostname())
    return doc


def _first_line(r: subprocess.CompletedProcess) -> str:
    text = (r.stderr or r.stdout or "").strip().splitlines()
    return text[0][:200] if text else f"exit {r.returncode}"


def _part(status: str, reason: str = "", **extra: Any) -> dict[str, Any]:
    return {"status": status, "reason": reason, **extra}


def check(rig: dict[str, Any] | None = None, run: Runner = _run) -> dict[str, Any]:
    """Each part of the rig, checked now."""
    rig = load() if rig is None else rig
    parts: dict[str, dict[str, Any]] = {}

    r = run(["nvidia-smi", "-L"], 10)
    gpu = r.stdout.strip().splitlines()[0] if r.returncode == 0 and r.stdout.strip() else ""
    parts["sim"] = (
        _part(READY, gpu=gpu)
        if gpu
        else _part(UNAVAILABLE, f"no NVIDIA GPU: {_first_line(r)}")
    )

    r = run(["docker", "info", "--format", "{{.ServerVersion}}"], 15)
    version = r.stdout.strip() if r.returncode == 0 else ""
    # `docker info --format` exits 0 even when it can't reach the daemon (no access to
    # its socket): only a server version means Docker answered.
    parts["docker"] = (
        _part(READY, version=version)
        if version
        else _part(UNAVAILABLE, f"Docker: {_first_line(r) if (r.stderr or r.returncode) else 'no answer from the daemon'}")
    )
    # the pipeline on this machine runs in the stack's container
    parts["pipeline_bench_pc"] = (
        _part(READY)
        if parts["docker"]["status"] == READY
        else _part(UNAVAILABLE, "needs Docker")
    )

    panda = rig.get("latte_panda")
    if not panda:
        parts["latte_panda"] = _part(ABSENT, "no LattePanda on this rig")
    else:
        host = panda.get("host", "")
        target = f"{panda['user']}@{host}" if panda.get("user") else host
        r = run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", target, "true"], 10
        )
        parts["latte_panda"] = (
            _part(READY, host=host)
            if r.returncode == 0
            else _part(UNAVAILABLE, f"LattePanda {host} not answering: {_first_line(r)}", host=host)
        )

    hil = rig.get("hil")
    parts["hil"] = (
        _part(ABSENT, "no HIL bench on this rig")
        if not hil
        else _part(ABSENT, "car-in-the-loop not wired yet", bench=hil.get("bench"))
    )
    # Emulated boards (IFS-vHIL) run in Docker, so once car-in-the-loop joins them to a
    # sim run, any rig with Docker can have them; until then no rig can.
    if not CAR_IN_THE_LOOP:
        parts["vhil"] = _part(ABSENT, "car-in-the-loop not wired yet")
    elif parts["docker"]["status"] == READY:
        parts["vhil"] = _part(READY)
    else:
        parts["vhil"] = _part(UNAVAILABLE, "needs Docker")

    return {
        "id": rig["id"],
        "host": socket.gethostname(),
        "checked_at": time.time(),
        "parts": parts,
    }


# which part each choice in a spec needs
def needs(spec: dict[str, Any]) -> list[str]:
    """The rig parts a spec needs: every benchmark drives the sim in its own stack."""
    out = ["sim", "docker"]
    for b in (spec.get("benchmarks") or {}).values():
        settings = (b or {}).get("settings") or {}
        where = settings.get("pipeline_on", "bench_pc")
        out.append("latte_panda" if where == "latte_panda" else "pipeline_bench_pc")
    mode = (spec.get("boards") or {}).get("mode") if isinstance(spec.get("boards"), dict) else None
    if mode in ("hil", "vhil"):
        out.append(mode)
    return list(dict.fromkeys(out))


def problems(spec: dict[str, Any], status: dict[str, Any]) -> list[dict[str, str]]:
    """What the spec needs that the rig can't give now: ``{part, status, reason}`` each.
    ``unavailable`` means wait; ``absent`` means this rig never can."""
    parts = status.get("parts") or {}
    out = []
    for name in needs(spec):
        p = parts.get(name) or {"status": ABSENT, "reason": "not checked"}
        if p["status"] != READY:
            out.append(
                {"part": name, "status": p["status"], "reason": p.get("reason") or ""}
            )
    return out


def describe(probs: list[dict[str, str]]) -> str:
    return "; ".join(f"{p['part']} {p['status']}: {p['reason']}" for p in probs)
