"""Emulated boards (IFS-vHIL) for a sim run: start them, wire them, stop them.

A spec's ``boards: {mode: vhil, system: ecu-ams-udv, firmware: {ecu: <ref>, ...}}`` runs
the car's real ECU, AMS and uDV firmware on IFS-vHIL's emulated MainLites in Docker, in
place of the sim's fake uDV (``sim_supervisor``). This module is the one place that does
it, so the sim's Local target and every runner (T11) start them the same way:

1. the IFS-vHIL checkout (``$IFSSIM_VHIL_REPO``, a ``IFS-vHIL`` folder next to this
   repository, or a clone kept in ``~/.cache/ifssim``) and its published Docker image;
2. the firmware, built by IFS-vHIL's own recipe at the refs the spec names
   (``python -m vhil.system build``), cached in a Docker volume;
3. one container running ``tools/vhil_boards/server.py``, which paces the emulation to the
   wall clock and answers on ``127.0.0.1:<port>``;
4. ``IFSSIM_VHIL_BOARDS`` for the benchmark, which hands it to the pipeline stack: its
   ``sim_supervisor`` node then publishes what the firmware says (``vhil_link.py``).

Stopping removes the container. Nothing else of the host is touched: it needs Docker and
the vHIL image, not vcan, root or the host network's CAN.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

ENV_REPO = "IFSSIM_VHIL_REPO"
ENV_BOARDS = "IFSSIM_VHIL_BOARDS"
ENV_TORQUE = "IFSSIM_VHIL_TORQUE"
CONTAINER = "ifssim-vhil-boards"
VOLUME = os.environ.get("IFSSIM_VHIL_VOLUME", "vhil-data")  # IFS-vHIL's own firmware cache
REPO_URL = "https://github.com/isc-fs/IFS-vHIL"
REGISTRY = os.environ.get("VHIL_REGISTRY", "ghcr.io/isc-fs")
PORT = 47800
SYSTEMS = ("ecu-ams-udv",)  # what the sim's uDV node can run against (the car's three boards)
READY = "vhil-boards ready on"
START_TIMEOUT_S = 240.0
BUILD_TIMEOUT_S = 1800.0
PULL_TIMEOUT_S = 1800.0

Run = Callable[..., subprocess.CompletedProcess]


class BoardsError(RuntimeError):
    pass


def _run(cmd: list[str], timeout: float = 60.0, **kw) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False, **kw)
    except FileNotFoundError:
        return subprocess.CompletedProcess(cmd, 127, "", f"{cmd[0]}: not installed")
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", f"{cmd[0]}: no answer in {timeout:.0f} s")


def vhil_repo(root: Path) -> Path | None:
    """The IFS-vHIL checkout this machine uses, if there is one."""
    cands = [os.environ.get(ENV_REPO), root.parent / "IFS-vHIL", Path("~/.cache/ifssim/IFS-vHIL").expanduser()]
    for c in cands:
        if c and (Path(c) / "vhil" / "sim.py").is_file():
            return Path(c).resolve()
    return None


def image_key(repo: Path, run: Run = _run) -> str | None:
    r = run(["bash", str(repo / "scripts" / "image-key.sh")], 30)
    m = re.search(r"^base=(\w+)$", r.stdout or "", re.M)
    return m.group(1) if m else None


def image_ref(repo: Path, run: Run = _run) -> tuple[str, str] | None:
    """``(local tag, published tag)`` of IFS-vHIL's base image for this checkout."""
    key = image_key(repo, run)
    return (f"ifs-vhil:{key}", f"{REGISTRY}/ifs-vhil:{key}") if key else None


def image_present(repo: Path, run: Run = _run) -> bool:
    refs = image_ref(repo, run)
    return bool(refs) and run(["docker", "image", "inspect", refs[0]], 20).returncode == 0


def availability(root: Path, run: Run = _run) -> tuple[bool, str]:
    """Whether this machine can run emulated boards now, and if not, why."""
    repo = vhil_repo(root)
    if repo is None:
        return False, (f"IFS-vHIL not found: clone {REPO_URL} next to this repository or set ${ENV_REPO}")
    if not (root / "tools" / "vhil_boards" / "server.py").is_file():
        return False, "this checkout has no tools/vhil_boards/server.py"
    if image_ref(repo, run) is None:
        return False, f"cannot read IFS-vHIL's image key ({repo}/scripts/image-key.sh)"
    if not image_present(repo, run):
        local, published = image_ref(repo, run) or ("", "")
        return False, f"the vHIL image is not pulled: docker pull {published}"
    return True, ""


def _docker(args: list[str], timeout: float = 60.0, run: Run = _run) -> subprocess.CompletedProcess:
    return run(["docker", *args], timeout)


def ensure_image(repo: Path, log: Callable[[str], None], run: Run = _run) -> str:
    refs = image_ref(repo, run)
    if refs is None:
        raise BoardsError(f"cannot read IFS-vHIL's image key ({repo}/scripts/image-key.sh)")
    local, published = refs
    if _docker(["image", "inspect", local], 20, run).returncode == 0:
        return local
    log(f"pulling the vHIL image {published}")
    r = _docker(["pull", published], PULL_TIMEOUT_S, run)
    if r.returncode != 0:
        raise BoardsError(f"cannot pull {published}: {(r.stderr or r.stdout).strip()[:300]}")
    _docker(["tag", published, local], 30, run)
    return local


def parse_images(text: str) -> dict[str, str]:
    """``board=/path.elf`` lines of ``python -m vhil.system build``."""
    out = {}
    for line in text.splitlines():
        m = re.match(r"^([\w.\-]+)=(/\S+\.elf)\s*$", line.strip())
        if m:
            out[m.group(1)] = m.group(2)
    return out


def build_firmware(
    repo: Path, image: str, system: str, refs: dict[str, str], log: Callable[[str], None], run: Run = _run
) -> dict[str, str]:
    """The system's firmware images (inside the container) at the refs the spec names."""
    cmd = ["python", "-m", "vhil.system", "build", f"systems/{system}.yaml", "--workdir", "/vhil/fw"]
    for board, ref in sorted(refs.items()):
        cmd += ["--ref", f"{board}={ref}"]
    log(f"building the firmware ({', '.join(f'{b}@{r}' for b, r in sorted(refs.items())) or 'catalogue refs'})")
    r = _docker(
        ["run", "--rm", "-v", f"{repo}:/work", "-v", f"{VOLUME}:/vhil", "-e", "PYTHONPATH=/work",
         "-w", "/work", image, "bash", "-c", "mkdir -p /vhil/fw && " + shlex.join(cmd)],
        BUILD_TIMEOUT_S, run,
    )
    images = parse_images(r.stdout or "")
    if r.returncode != 0 or not images:
        tail = "\n".join((r.stderr or r.stdout or "").strip().splitlines()[-8:])
        raise BoardsError(f"the firmware build failed (exit {r.returncode}):\n{tail}")
    return images


@dataclass
class Boards:
    """Running emulated boards. ``env`` is what the benchmark's stack needs."""

    port: int
    log_path: Path | None = None
    env: dict[str, str] = field(default_factory=dict)
    run: Run = _run

    def logs(self) -> str:
        r = _docker(["logs", CONTAINER], 30, self.run)
        return (r.stdout or "") + (r.stderr or "")

    def alive(self) -> bool:
        r = _docker(["inspect", "-f", "{{.State.Running}}", CONTAINER], 15, self.run)
        return r.returncode == 0 and r.stdout.strip() == "true"

    def summary(self) -> dict:
        """The emulation's pace over the run, from its own log: the worst lag behind the wall
        clock and the last real-time factor (a lag above a few hundred ms means the run was
        not real time)."""
        lags, last = [], {}
        for line in self.logs().splitlines():
            if line.startswith("{"):
                try:
                    last = json.loads(line)
                    lags.append(float(last.get("lag_s", 0.0)))
                except ValueError:
                    pass
        return {"max_lag_s": max(lags, default=0.0), "rtf": last.get("rtf"), "last": last}

    def stop(self) -> None:
        summary = self.summary() if self.alive() else {}
        if self.log_path is not None:
            try:
                self.log_path.write_text(self.logs())
            except OSError:
                pass
        _docker(["rm", "-f", CONTAINER], 60, self.run)
        if summary:
            print(f"vhil boards stopped: rtf {summary['rtf']}, worst lag {summary['max_lag_s']:.2f} s", file=sys.stderr)


def start(
    root: Path,
    boards: dict,
    *,
    log_path: Path | None = None,
    log: Callable[[str], None] = lambda s: print(s, file=sys.stderr),
    port: int = PORT,
    run: Run = _run,
) -> Boards:
    """Start the emulated boards for one job; raise ``BoardsError`` with the reason if it can't."""
    system = boards.get("system")
    if system not in SYSTEMS:
        raise BoardsError(f"vHIL system {system!r}: the sim's uDV node runs against {', '.join(SYSTEMS)} (the car's three boards)")
    repo = vhil_repo(root)
    if repo is None:
        raise BoardsError(f"IFS-vHIL not found: clone {REPO_URL} next to this repository or set ${ENV_REPO}")
    server = root / "tools" / "vhil_boards"
    if not (server / "server.py").is_file():
        raise BoardsError("tools/vhil_boards/server.py is missing from this checkout")
    image = ensure_image(repo, log, run)
    images = build_firmware(repo, image, system, dict(boards.get("firmware") or {}), log, run)
    _docker(["rm", "-f", CONTAINER], 60, run)
    cmd = [
        "run", "-d", "--name", CONTAINER, "--network", "host",
        "-v", f"{repo}:/work:ro", "-v", f"{server}:/boards:ro", "-v", f"{VOLUME}:/vhil",
        "-e", "PYTHONPATH=/work", "-e", "VHIL_MODELS_CACHE=/vhil/renode-models", "-w", "/work",
        image, "python", "/boards/server.py", "--system", system, "--port", str(port),
    ]
    for b, p in sorted(images.items()):
        cmd += ["--fw", f"{b}={p}"]
    r = _docker(cmd, 120, run)
    if r.returncode != 0:
        raise BoardsError(f"cannot start the boards container: {(r.stderr or r.stdout).strip()[:300]}")
    started = Boards(port, log_path, {ENV_BOARDS: f"127.0.0.1:{port}"}, run)
    deadline = time.monotonic() + START_TIMEOUT_S
    while time.monotonic() < deadline:
        if READY in started.logs():
            log(f"emulated boards ready on 127.0.0.1:{port} ({system})")
            return started
        if not started.alive():
            tail = "\n".join(started.logs().strip().splitlines()[-8:])
            started.stop()
            raise BoardsError(f"the boards container stopped while starting:\n{tail}")
        time.sleep(1.0)
    started.stop()
    raise BoardsError(f"the boards were not ready in {START_TIMEOUT_S:.0f} s")
