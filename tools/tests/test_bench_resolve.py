"""tools/ci/bench_resolve.py: which IFSSIM commit a bench run tests."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ci"))

import bench_resolve as br  # noqa: E402

REPO = "isc-fs/IFSSIM"
SHA = "a" * 40


def fake_api(answers):
    def get(path):
        if path not in answers:
            raise br.ResolveError(f"GitHub said 404 for {path}")
        return answers[path]

    return get


class SpecRef(unittest.TestCase):
    def test_a_preset_pins_nothing(self):
        self.assertIsNone(br.spec_ref("missions-basic"))

    def test_a_json_spec_without_code_pins_nothing(self):
        self.assertIsNone(br.spec_ref('{"benchmarks": {"missions": {}}}'))
        self.assertIsNone(br.spec_ref('{"code": {"ifssim": ""}}'))

    def test_a_json_spec_pins_its_ifssim(self):
        self.assertEqual(br.spec_ref('{"code": {"ifssim": " #880 "}}'), "#880")

    def test_broken_json_is_an_error(self):
        with self.assertRaises(br.ResolveError):
            br.spec_ref("{nope")


class Resolve(unittest.TestCase):
    def test_no_ref_is_the_workflows_commit(self):
        sha, what = br.resolve(None, SHA, REPO, fake_api({}))
        self.assertEqual(sha, SHA)
        self.assertIn("workflow", what)

    def test_a_branch(self):
        api = fake_api({"branches/feat%2Fx": {"commit": {"sha": "b" * 40}}})
        self.assertEqual(br.resolve("feat/x", SHA, REPO, api)[0], "b" * 40)

    def test_a_commit(self):
        api = fake_api({"commits/abc1234": {"sha": "c" * 40}})
        self.assertEqual(br.resolve("abc1234", SHA, REPO, api)[0], "c" * 40)

    def test_a_pr_of_this_repository(self):
        api = fake_api({"pulls/880": {"head": {"sha": "d" * 40, "ref": "feat/880", "repo": {"full_name": REPO}}}})
        sha, what = br.resolve("#880", SHA, REPO, api)
        self.assertEqual(sha, "d" * 40)
        self.assertIn("PR #880", what)

    def test_a_pr_from_a_fork_is_refused(self):
        api = fake_api({"pulls/9": {"head": {"sha": "e" * 40, "ref": "main", "repo": {"full_name": "someone/IFSSIM"}}}})
        with self.assertRaises(br.ResolveError):
            br.resolve("#9", SHA, REPO, api)

    def test_a_pr_whose_fork_was_deleted_is_refused(self):
        api = fake_api({"pulls/9": {"head": {"sha": "e" * 40, "ref": "main", "repo": None}}})
        with self.assertRaises(br.ResolveError):
            br.resolve("#9", SHA, REPO, api)

    def test_an_unknown_branch_is_an_error(self):
        with self.assertRaises(br.ResolveError):
            br.resolve("nope", SHA, REPO, fake_api({}))


if __name__ == "__main__":
    unittest.main()
