from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run_provenance as rp  # noqa: E402


def _git(repo: Path, *args: str) -> None:
    subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        env={
            **os.environ,
            "GIT_AUTHOR_NAME": "t",
            "GIT_AUTHOR_EMAIL": "t@t",
            "GIT_COMMITTER_NAME": "t",
            "GIT_COMMITTER_EMAIL": "t@t",
        },
    )


def _repo(root: Path) -> Path:
    root.mkdir(parents=True)
    _git(root, "init", "-q", "-b", "main")
    (root / "a.py").write_text("x = 1\n")
    _git(root, "add", "a.py")
    _git(root, "commit", "-q", "-m", "first")
    return root


class GitStateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp())
        self.repo = _repo(self.tmp / "repo")

    def test_clean_checkout(self) -> None:
        st, diff = rp.git_state(self.repo)
        self.assertEqual(len(st["sha"]), 40)
        self.assertEqual(st["branch"], "main")
        self.assertEqual(st["subject"], "first")
        self.assertFalse(st["dirty"])
        self.assertIsNone(st["diff_sha"])
        self.assertEqual(diff, "")

    def test_edits_and_new_files_are_in_the_diff(self) -> None:
        (self.repo / "a.py").write_text("x = 2\n")
        (self.repo / "new.py").write_text("y = 3\n")
        st, diff = rp.git_state(self.repo)
        self.assertTrue(st["dirty"])
        self.assertIn("+x = 2", diff)
        self.assertIn("+y = 3", diff)  # untracked file, not just a flag

    def test_different_changes_give_different_ids(self) -> None:
        (self.repo / "a.py").write_text("x = 2\n")
        a = rp.git_state(self.repo)[0]
        (self.repo / "a.py").write_text("x = 3\n")
        b = rp.git_state(self.repo)[0]
        self.assertNotEqual(a["diff_sha"], b["diff_sha"])

    def test_uninitialised_submodule_does_not_report_the_parent(self) -> None:
        empty = self.repo / "pipeline"
        empty.mkdir()
        st, diff = rp.git_state(empty)
        self.assertIsNone(st["sha"])
        self.assertEqual(diff, "")

    def test_big_untracked_files_are_listed_not_copied(self) -> None:
        (self.repo / "big.bin").write_bytes(b"0" * (rp.MAX_UNTRACKED_BYTES + 1))
        st, diff = rp.git_state(self.repo)
        self.assertEqual(st["untracked_not_saved"], ["big.bin"])
        self.assertNotIn("big.bin", diff)


class UnpushedTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp())
        self.repo = _repo(self.tmp / "repo")
        remote = self.tmp / "remote.git"
        subprocess.run(["git", "init", "-q", "--bare", str(remote)], check=True)
        _git(self.repo, "remote", "add", "origin", str(remote))
        _git(self.repo, "push", "-q", "origin", "main")

    def commit(self, text: str) -> None:
        (self.repo / "a.py").write_text(text)
        _git(self.repo, "commit", "-qam", text)

    def test_pushed_head_has_nothing_to_save(self) -> None:
        st, _ = rp.git_state(self.repo)
        self.assertEqual(st["unpushed"], 0)
        self.assertIsNone(rp.unpushed_bundle(self.repo, st))

    def test_unpushed_commits_are_bundled_and_restorable(self) -> None:
        self.commit("x = 2\n")
        self.commit("x = 3\n")
        st, _ = rp.git_state(self.repo)
        self.assertEqual(st["unpushed"], 2)
        data = rp.unpushed_bundle(self.repo, st)
        self.assertIsNotNone(data)
        self.assertTrue(any("2 commit(s)" in w for w in rp.warnings({"ifssim": st})))
        # someone else, with only what the remote has, gets the exact commit back
        other = self.tmp / "other"
        subprocess.run(
            ["git", "clone", "-q", str(self.tmp / "remote.git"), str(other)], check=True
        )
        bundle = self.tmp / "run.bundle"
        bundle.write_bytes(data)
        _git(other, "fetch", "-q", str(bundle), "HEAD")
        _git(other, "checkout", "-q", st["sha"])
        self.assertEqual((other / "a.py").read_text(), "x = 3\n")

    def test_no_remote_is_not_called_unpushed(self) -> None:
        _git(self.repo, "remote", "remove", "origin")
        self.assertIsNone(rp.git_state(self.repo)[0]["unpushed"])


class ImageTest(unittest.TestCase):
    def test_registry_repos(self) -> None:
        self.assertTrue(rp._is_registry_repo("ghcr.io/isc-fs/ifssim-dv_pipeline_stack"))
        self.assertTrue(rp._is_registry_repo("localhost:5000/x"))
        self.assertFalse(rp._is_registry_repo("ifssim-dv_pipeline_stack"))
        self.assertFalse(rp._is_registry_repo("library/ubuntu"))

    def test_registry_digest_is_part_of_the_code(self) -> None:
        code = {"pipeline": {"sha": "a64350a"}, "ifssim": {"sha": "6210d09"}}
        plain = rp.code_id(code)
        local = rp.code_id(dict(code, image={"id": "sha256:1", "digest": None}))
        pulled = rp.code_id(dict(code, image={"digest": "ghcr.io/x@sha256:2"}))
        self.assertEqual(plain, local)  # a local build's id says nothing across machines
        self.assertNotEqual(plain, pulled)

    def test_local_build_warns(self) -> None:
        w = rp.warnings({"image": {"tag": "x:latest", "local_build": True}})
        self.assertTrue(w and "built locally" in w[0])


class LabelTest(unittest.TestCase):
    def code(self, p_dirty: bool, diff: str | None = None) -> dict:
        return {
            "pipeline": {"sha": "a64350a5704543", "dirty": p_dirty, "diff_sha": diff},
            "ifssim": {"sha": "6210d096284af2", "dirty": False, "diff_sha": None},
        }

    def test_clean_is_the_short_commit(self) -> None:
        self.assertEqual(rp.label(self.code(False)), "a64350a")

    def test_dirty_names_the_exact_state(self) -> None:
        a = rp.label(self.code(True, "11111111"))
        b = rp.label(self.code(True, "22222222"))
        self.assertTrue(a.startswith("a64350a-dirty."))
        self.assertNotEqual(a, b)

    def test_same_code_same_id(self) -> None:
        self.assertEqual(
            rp.code_id(self.code(True, "1")), rp.code_id(self.code(True, "1"))
        )

    def test_nothing_known(self) -> None:
        self.assertEqual(rp.label({}), "unknown")
        self.assertIsNone(rp.code_id({}))


class RecordTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp())
        self.run = self.tmp / "run"
        self.run.mkdir()

    def test_container_copies_what_the_host_staged(self) -> None:
        staged = self.tmp / "staged"
        rp.write(staged, {"captured": True, "label": "a64350a"}, {"pipeline.diff": "d"})
        env = {rp.ENV_DIR: str(staged), "IFSSIM_BENCHMARK_IN_DOCKER": "1"}
        with mock.patch.dict(os.environ, env):
            rp.record(self.run)
        self.assertEqual(rp.read(self.run)["label"], "a64350a")
        self.assertEqual((self.run / "pipeline.diff").read_text(), "d")

    def test_container_without_host_record_says_unknown(self) -> None:
        with mock.patch.dict(os.environ, {"IFSSIM_BENCHMARK_IN_DOCKER": "1"}):
            os.environ.pop(rp.ENV_DIR, None)
            rp.record(self.run)
        got = rp.read(self.run)
        self.assertFalse(got["captured"])
        self.assertEqual(got["label"], "unknown")

    def test_never_raises(self) -> None:
        with mock.patch.object(rp, "capture", side_effect=RuntimeError("boom")):
            with mock.patch.dict(os.environ, {}, clear=False):
                os.environ.pop(rp.ENV_DIR, None)
                os.environ.pop("IFSSIM_BENCHMARK_IN_DOCKER", None)
                rp.record(self.run)  # prints a warning, does not fail the benchmark

    def test_written_file_lists_the_diffs(self) -> None:
        rp.write(self.run, {"captured": True}, {"ifssim.diff": "x"})
        self.assertEqual(
            json.loads((self.run / "provenance.json").read_text())["diff_files"],
            ["ifssim.diff"],
        )


if __name__ == "__main__":
    unittest.main()
