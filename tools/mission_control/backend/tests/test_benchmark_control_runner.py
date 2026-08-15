"""Tests for benchmark_control_runner path resolution."""
from __future__ import annotations

import benchmark_control_runner as bcr


def test_tools_dir_does_not_raise_when_env_set(monkeypatch, tmp_path):
    tools = tmp_path / "tools"
    tools.mkdir()
    (tools / "track_driver.py").write_text("# stub\n", encoding="utf-8")
    monkeypatch.setenv("IFSSIM_TOOLS_DIR", str(tools))
    bcr._tools_dir_cache = None
    assert bcr._tools_dir() == tools
