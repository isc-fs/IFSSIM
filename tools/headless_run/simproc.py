"""Start the simulator without a window, wait until it answers, stop it.

Three ways to point at a sim (``--sim``):

  * a packaged build: ``.../IFSSIM.sh`` or ``.../Binaries/Linux/IFSSIM``
    (Linux), ``.../IFSSIM.app`` (macOS), ``...\\IFSSIM.exe`` (Windows);
  * the editor binary, ``.../UnrealEditor`` (with ``--project``), which runs
    the project as a game (``-game``), handy on a dev box;
  * ``attach``: a sim someone already started; nothing is launched or
    stopped.

Offscreen means ``-RenderOffscreen``, not ``-nullrhi``: the GPU LiDAR renders
depth with a scene capture, so the sim needs a real GPU and RHI even with no
window. On Linux that is a GPU with Vulkan drivers (see docs/HEADLESS.md).
"""

from __future__ import annotations

import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

RPC_PORT = 41451

# -RenderOffscreen: no window, still a GPU (LiDAR needs it).
# -NoLauncher: no in-game Mission Control panel.
# -unattended: never block on a message box (a headless run has nobody to
#   click OK; it fails and exits instead).
BASE_ARGS = [
    "-RenderOffscreen", "-unattended", "-nosound", "-NoSplash", "-NoLauncher",
    "-ResX=640", "-ResY=480", "-stdout", "-FullStdOutLogOutput",
]


class SimError(RuntimeError):
    pass


@dataclass
class SimCommand:
    argv: List[str]
    cwd: Optional[str] = None


def resolve_command(sim: str, project: Optional[str] = None, extra: Optional[List[str]] = None,
                    settings_dir: Optional[str] = None) -> SimCommand:
    """Turn ``--sim`` into an argv. Pure, so it is unit-tested."""
    extra = list(extra or [])
    p = Path(sim).expanduser()
    # Absolute, because the sim starts in --settings-dir: a path relative to
    # the caller's folder would point nowhere from there. A bare name with no
    # folder is left for a PATH lookup.
    if p.parent != Path(".") or p.exists():
        p = p.absolute()
    if p.suffix == ".app":
        p = p / "Contents" / "MacOS" / p.stem
    if p.name.startswith("UnrealEditor"):
        if not project:
            raise SimError("--sim is the editor: pass --project path/to/IFSSIM.uproject")
        argv = [str(p), str(Path(project).expanduser().resolve()), "-game"]
    else:  # a packaged build
        argv = [str(p)]
    # settings.json is read from the launch directory first (FSDSSettings::
    # AutoLoad), so running from the folder that holds it selects it.
    cwd = str(Path(settings_dir).expanduser().absolute()) if settings_dir else None
    return SimCommand(argv=argv + BASE_ARGS + extra, cwd=cwd)


def rpc(host: str, command: str, port: int = RPC_PORT, timeout: float = 5.0) -> str:
    """One command on the sim's line protocol: a line out, a line back."""
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall((command + "\n").encode())
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
    return buf.decode(errors="replace").strip()


def port_open(host: str, port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


@dataclass
class SimProcess:
    """One launched sim. ``attach`` mode wraps a sim that's already up."""

    command: Optional[SimCommand]
    log_path: Optional[Path] = None
    host: str = "127.0.0.1"
    port: int = RPC_PORT
    proc: Optional[subprocess.Popen] = field(default=None, repr=False)
    _log: Optional[object] = field(default=None, repr=False)

    @property
    def attached(self) -> bool:
        return self.command is None

    def start(self) -> None:
        if self.attached:
            if not port_open(self.host, self.port):
                raise SimError(f"--sim attach: nothing is listening on {self.host}:{self.port}")
            return
        if port_open(self.host, self.port):
            raise SimError(f"port {self.port} is already taken: another sim is running "
                           "(stop it, or use --sim attach)")
        exe = self.command.argv[0]
        if not (os.path.exists(exe) or shutil.which(exe)):
            raise SimError(f"sim binary not found: {exe}")
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log = open(self.log_path, "wb")
        try:
            self.proc = subprocess.Popen(
                self.command.argv, cwd=self.command.cwd,
                stdout=self._log or subprocess.DEVNULL, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                # Own process group, so stop() takes the engine's children with it.
                start_new_session=(sys.platform != "win32"),
            )
        except OSError as e:     # not executable, missing --settings-dir, ...
            if self._log:
                self._log.close()
                self._log = None
            raise SimError(f"couldn't start the sim ({' '.join(self.command.argv[:1])}): {e}") from e

    def wait_ready(self, timeout_s: float = 600.0,
                   should_stop: Callable[[], bool] = lambda: False) -> float:
        """Block until the RPC port answers. First runs compile shaders: be generous.

        Raises KeyboardInterrupt as soon as ``should_stop()`` is true, so a stop
        request doesn't wait for a slow first boot."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if should_stop():
                raise KeyboardInterrupt
            if self.proc is not None and self.proc.poll() is not None:
                raise SimError(f"the sim exited during startup (code {self.proc.returncode}); "
                               f"see {self.log_path}")
            try:
                if rpc(self.host, "ping", port=self.port, timeout=2.0) == "true":
                    return time.monotonic() - t0
            except OSError:
                pass
            time.sleep(1.0)
        raise SimError(f"the sim didn't open port {self.port} within {timeout_s:.0f} s")

    def alive(self) -> bool:
        if self.attached:
            return port_open(self.host, self.port)
        return self.proc is not None and self.proc.poll() is None

    def stop(self, grace_s: float = 20.0) -> None:
        """Stop the sim and everything it started, within ``grace_s``.

        Waits on the whole process group, not just its leader: IFSSIM.sh can
        exit on SIGTERM while the engine it launched is still shutting down and
        holding the RPC port, which would make the next run's start fail."""
        if self.proc is None:
            return
        if sys.platform == "win32":
            if self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=grace_s)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=10)
        else:
            pgid = self.proc.pid
            deadline = time.monotonic() + grace_s
            _signal_group(pgid, signal.SIGTERM)
            while _group_alive(pgid, self.proc) and time.monotonic() < deadline:
                time.sleep(0.2)
            if _group_alive(pgid, self.proc):
                _signal_group(pgid, signal.SIGKILL)
                kill_deadline = time.monotonic() + 10.0
                while _group_alive(pgid, self.proc) and time.monotonic() < kill_deadline:
                    time.sleep(0.1)
            if self.proc.poll() is None:
                self.proc.wait(timeout=5)
        if self._log:
            self._log.close()
            self._log = None


def _signal_group(pgid: int, sig: int) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


def _group_alive(pgid: int, leader: subprocess.Popen) -> bool:
    """Is any process of the group still running? Reaps the leader first, so
    its zombie doesn't count. Orphaned children are reaped by init, so once
    the leader is gone a successful signal means a live process."""
    leader.poll()
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True
