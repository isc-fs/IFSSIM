"""launch/boards.py: starting the emulated boards for a job (Docker faked)."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch import boards as bd  # noqa: E402

BUILD_OUT = """\
ecu=/vhil/fw/ecu@dev/build/ECU08.elf
ecu.bootloader=/vhil/fw/can-bootloader@v1.7.0/build/Release/CAN_BL.elf
udv=/vhil/fw/udv@feat-x/build/binaries/uDV.elf
some other line
"""


class FakeDocker:
    def __init__(self, ready=True, build_rc=0, image=True):
        self.calls: list[list[str]] = []
        self.ready, self.build_rc, self.image, self.running = ready, build_rc, image, False
        self.log = ""
        self.heads = {"dev": "d" * 40, "feat-x": "f" * 40}
        self.pulls = {"123": "9" * 40}
        self.all = ["a1b2c3d" + "0" * 33]

    def remote(self, cmd):
        pats = cmd[5:]
        lines = []
        for pat in pats:
            if pat.startswith("refs/heads/") and pat[11:] in self.heads:
                lines.append(f"{self.heads[pat[11:]]}\t{pat}")
            if pat.startswith("refs/pull/"):
                n = pat.split("/")[2]
                if n in self.pulls:
                    lines.append(f"{self.pulls[n]}\t{pat}")
        if not pats:
            lines += [f"{c}\trefs/heads/old" for c in self.all]
        return "\n".join(lines) + "\n"

    def __call__(self, cmd, timeout=60.0, **kw):
        self.calls.append(cmd)
        out = lambda rc=0, so="", se="": subprocess.CompletedProcess(cmd, rc, so, se)
        if cmd[0] == "bash":
            return out(0, "base=k1\nclean=true\n")
        if cmd[0] == "git":                       # git ls-remote https://github.com/<repo> [refs...]
            return out(0, self.remote(cmd))
        sub = cmd[1] if len(cmd) > 1 else ""
        if sub == "image":
            return out(0 if self.image else 1)
        if sub == "run" and "-d" in cmd:
            self.running = True
            self.log = ("vhil-boards ready on 127.0.0.1:47800\n" if self.ready else "") + json.dumps({"lag_s": 0.4, "rtf": 1.0}) + "\n"
            return out(0, "cid\n")
        if sub == "run":
            return out(self.build_rc, BUILD_OUT, "boom" if self.build_rc else "")
        if sub == "logs":
            return out(0, self.log)
        if sub == "inspect":
            return out(0, "true\n" if self.running else "false\n")
        if sub == "rm":
            self.running = False
        return out()


@pytest.fixture
def tree(tmp_path):
    root = tmp_path / "IFSSIM"
    (root / "tools" / "vhil_boards").mkdir(parents=True)
    (root / "tools" / "vhil_boards" / "server.py").write_text("")
    (tmp_path / "IFS-vHIL" / "vhil").mkdir(parents=True)
    (tmp_path / "IFS-vHIL" / "vhil" / "sim.py").write_text("")
    return root


def test_parse_images_reads_the_builders_board_elf_lines():
    assert bd.parse_images(BUILD_OUT) == {
        "ecu": "/vhil/fw/ecu@dev/build/ECU08.elf",
        "ecu.bootloader": "/vhil/fw/can-bootloader@v1.7.0/build/Release/CAN_BL.elf",
        "udv": "/vhil/fw/udv@feat-x/build/binaries/uDV.elf",
    }


def test_start_builds_the_firmware_at_the_specs_refs_and_runs_the_server(tree):
    d = FakeDocker()
    boards = bd.start(tree, {"mode": "vhil", "system": "ecu-ams-udv", "firmware": {"udv": "feat-x"}}, run=d, log=lambda s: None,
                      manifest_boards={"firmware": {"udv": {"repo": "isc-fs/udv", "default": "dev"}}})
    assert boards.env == {"IFSSIM_VHIL_BOARDS": "127.0.0.1:47800"}
    build = next(c for c in d.calls if c[1] == "run" and "-d" not in c)
    assert "--ref udv=feat-x --commit udv=" + "f" * 40 in build[-1] and "systems/ecu-ams-udv.yaml" in build[-1]
    run = next(c for c in d.calls if c[1] == "run" and "-d" in c)
    assert run[run.index("--network") + 1] == "host" and "ifs-vhil:k1" in run
    assert "--fw" in run and "udv=/vhil/fw/udv@feat-x/build/binaries/uDV.elf" in run
    boards.stop()
    assert ["docker", "rm", "-f", bd.CONTAINER] in d.calls and not d.running


def test_start_says_why_it_cannot(tree, tmp_path):
    with pytest.raises(bd.BoardsError, match="ecu-ams-udv"):
        bd.start(tree, {"system": "udv"}, run=FakeDocker(), log=lambda s: None)
    with pytest.raises(bd.BoardsError, match="firmware build failed"):
        bd.start(tree, {"system": "ecu-ams-udv"}, run=FakeDocker(build_rc=2), log=lambda s: None)
    bad = tmp_path / "other" / "IFSSIM"
    bad.mkdir(parents=True)
    with pytest.raises(bd.BoardsError, match="IFS-vHIL not found"):
        bd.start(bad, {"system": "ecu-ams-udv"}, run=FakeDocker(), log=lambda s: None)


def test_a_container_that_dies_while_starting_is_reported_with_its_log(tree, monkeypatch):
    d = FakeDocker(ready=False)
    orig = d.__call__
    def dies(cmd, timeout=60.0, **kw):
        r = orig(cmd, timeout)
        if len(cmd) > 1 and cmd[1] == "logs":
            d.running = False
        return r
    with pytest.raises(bd.BoardsError, match="stopped while starting"):
        bd.start(tree, {"system": "ecu-ams-udv"}, run=dies, log=lambda s: None)


def test_the_summary_reports_the_worst_lag_of_the_run(tree):
    d = FakeDocker()
    boards = bd.start(tree, {"system": "ecu-ams-udv"}, run=d, log=lambda s: None)
    d.log += json.dumps({"lag_s": 1.5, "rtf": 0.9}) + "\n"
    assert boards.summary()["max_lag_s"] == 1.5 and boards.summary()["rtf"] == 0.9


def test_the_image_counts_as_present_under_the_published_tag_too(tmp_path, monkeypatch):
    """The page says `docker pull <published>`; that alone must make the part ready."""
    monkeypatch.setattr(bd, "image_ref", lambda repo, run=None: ("ifs-vhil:k", "ghcr.io/isc-fs/ifs-vhil:k"))

    def run(cmd, timeout=60.0):
        have = cmd[-1] == "ghcr.io/isc-fs/ifs-vhil:k"
        return subprocess.CompletedProcess(cmd, 0 if have else 1, "", "")

    assert bd.image_present(tmp_path, run)


FW = {"ecu": {"repo": "isc-fs/ecu", "default": "dev"}, "ams": {"repo": "isc-fs/ams", "default": "dev"},
      "udv": {"repo": "isc-fs/udv", "default": "dev"}}


def test_a_ref_is_a_branch_a_pull_request_or_a_commit_and_resolves_to_a_full_commit():
    d = FakeDocker()
    assert bd.resolve_ref("isc-fs/udv", "feat-x", d) == ("feat-x", "f" * 40)
    assert bd.resolve_ref("isc-fs/udv", "#123", d) == ("pr-123", "9" * 40)
    assert bd.resolve_ref("isc-fs/udv", "pr/123", d) == ("pr-123", "9" * 40)
    assert bd.resolve_ref("isc-fs/udv", "c" * 40, d) == ("commit-" + "c" * 12, "c" * 40)
    assert bd.resolve_ref("isc-fs/udv", "a1b2c3d", d) == ("commit-a1b2c3d" + "0" * 5, "a1b2c3d" + "0" * 33)
    for bad, why in (("nope", "no branch"), ("#9", "no pull request"), ("deadbeef", "short commit id")):
        with pytest.raises(bd.BoardsError, match=why):
            bd.resolve_ref("isc-fs/udv", bad, d)


def test_every_board_is_pinned_to_a_commit_the_spec_or_the_manifest_default():
    pinned = bd.resolve_firmware(["ecu", "udv"], {"udv": "#123"}, FW, FakeDocker())
    assert pinned["ecu"] == {"ref": "dev", "label": "dev", "commit": "d" * 40, "repo": "isc-fs/ecu"}
    assert pinned["udv"]["commit"] == "9" * 40 and pinned["udv"]["label"] == "pr-123"


def test_the_run_records_what_ran(tree, tmp_path):
    d = FakeDocker()
    log = tmp_path / "job" / "boards.log"
    log.parent.mkdir()
    boards = bd.start(tree, {"system": "ecu-ams-udv", "firmware": {"udv": "#123"}}, run=d, log=lambda s: None,
                      log_path=log, manifest_boards={"firmware": FW, "vhil": {"ecu-ams-udv": ["ecu", "ams", "udv"]}})
    build = next(c for c in d.calls if c[1] == "run" and "-d" not in c)
    assert "--ref udv=pr-123 --commit udv=" + "9" * 40 in build[-1] and "--commit ecu=" + "d" * 40 in build[-1]
    boards.stop()
    rec = json.loads((log.parent / "boards.json").read_text())
    assert rec["system"] == "ecu-ams-udv" and rec["firmware"]["udv"]["commit"] == "9" * 40


def test_a_choice_for_a_board_with_no_repository_is_refused_not_dropped():
    with pytest.raises(bd.BoardsError, match="no repository"):
        bd.resolve_firmware(["udv"], {"udv": "feat-x"}, {}, FakeDocker())
