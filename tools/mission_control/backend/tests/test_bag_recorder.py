"""Unit tests for tools/mission_control/backend/bag_recorder.py (#465).

Pure-Python tests — mock subprocess so the suite runs anywhere, no
docker on the runner required. Covers the three public helpers
(`compose_bag_name`, `check_free_disk`, `start_recording`,
`stop_recording`) at the contracts MC's session lifecycle depends on.
"""
from __future__ import annotations

import sys
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir))

import bag_recorder as br  # noqa: E402


# ----- compose_bag_name --------------------------------------------------

def test_compose_bag_name_sanitises_and_orders_segments():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("trackdrive", "TrainingMap", now=when)
    assert out == "trackdrive_TrainingMap_20260514_153022"


def test_compose_bag_name_strips_csv_extension():
    # The track name MC sees is the CSV filename — must NOT end up
    # with `.csv` in the bag dir.
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("trackdrive", "track_20260512_151240.csv", now=when)
    assert out == "trackdrive_track_20260512_151240_20260514_153022"
    assert ".csv" not in out


def test_compose_bag_name_replaces_unsafe_chars():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("track drive!", "../etc/passwd", now=when)
    # No spaces, slashes, dots, exclamation marks in the result.
    assert " " not in out
    assert "/" not in out
    assert ".." not in out
    assert "!" not in out


def test_compose_bag_name_fallback_when_track_missing():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("trackdrive", None, now=when)
    assert out == "trackdrive_no-track_20260514_153022"


def test_compose_bag_name_fallback_when_event_unknown():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("", "TrainingMap", now=when)
    assert out == "unknown_TrainingMap_20260514_153022"


# ----- check_free_disk ---------------------------------------------------

def _df_output(avail_gib: int) -> str:
    """Synthesise a `df -PB1G` row matching the real format."""
    return (
        "Filesystem 1G-blocks Used Available Capacity Mounted on\n"
        f"/dev/sdX     500    100  {avail_gib}      40%       /bags\n"
    )


def _run_mock(returncode=0, stdout="", stderr=""):
    """Return a callable suitable to inject as `_run`."""
    def _fn(cmd, **kw):
        result = MagicMock()
        result.returncode = returncode
        result.stdout = stdout
        result.stderr = stderr
        return result
    return _fn


def test_check_free_disk_above_floor():
    ok, free = br.check_free_disk(
        "ifssim-dv_pipeline_stack-1",
        path="/bags",
        min_gib=10,
        _run=_run_mock(stdout=_df_output(avail_gib=42)),
    )
    assert ok is True
    assert free == 42


def test_check_free_disk_below_floor():
    ok, free = br.check_free_disk(
        "ifssim-dv_pipeline_stack-1",
        path="/bags",
        min_gib=10,
        _run=_run_mock(stdout=_df_output(avail_gib=3)),
    )
    assert ok is False
    assert free == 3


def test_check_free_disk_docker_exec_failure_raises():
    with pytest.raises(br.DockerExecError):
        br.check_free_disk(
            "ifssim-dv_pipeline_stack-1",
            _run=_run_mock(returncode=1, stderr="No such container"),
        )


def test_check_free_disk_unparseable_output_raises():
    with pytest.raises(br.DockerExecError):
        br.check_free_disk(
            "ifssim-dv_pipeline_stack-1",
            _run=_run_mock(stdout="something weird\n"),
        )


# ----- start_recording ---------------------------------------------------

def _multi_run(*responses):
    """Sequence of mock responses for successive `_run` calls."""
    iterator = iter(responses)

    def _fn(cmd, **kw):
        try:
            spec = next(iterator)
        except StopIteration:
            spec = {"returncode": 0, "stdout": "", "stderr": ""}
        result = MagicMock()
        result.returncode = spec.get("returncode", 0)
        result.stdout = spec.get("stdout", "")
        result.stderr = spec.get("stderr", "")
        # Capture the cmd for assertion if the test needs it
        _fn.last_cmd = cmd
        return result
    _fn.last_cmd = None
    return _fn


def test_start_recording_refuses_when_disk_full():
    runner = _run_mock(stdout=_df_output(avail_gib=3))
    with pytest.raises(br.DiskFullError):
        br.start_recording(
            "ifssim-dv_pipeline_stack-1", "trackdrive_X_20260514_153022",
            min_free_gib=10, _run=runner,
        )


def test_start_recording_returns_state_dict_with_pid():
    # Three calls: df (free), docker exec -d (start), pgrep (find pid).
    runner = _multi_run(
        {"returncode": 0, "stdout": _df_output(avail_gib=42)},
        {"returncode": 0, "stdout": ""},
        {"returncode": 0, "stdout": "12345\n"},
    )
    state = br.start_recording(
        "ifssim-dv_pipeline_stack-1", "trackdrive_X_20260514_153022",
        min_free_gib=10, _run=runner,
    )
    assert state["name"] == "trackdrive_X_20260514_153022"
    assert state["container"] == "ifssim-dv_pipeline_stack-1"
    assert state["pid"] == 12345
    assert state["state"] == "recording"
    assert state["path_in_container"] == "/bags/trackdrive_X_20260514_153022"


def test_start_recording_docker_exec_failure_raises():
    runner = _multi_run(
        {"returncode": 0, "stdout": _df_output(avail_gib=42)},
        {"returncode": 125, "stderr": "Error: No such container"},
    )
    with pytest.raises(br.DockerExecError):
        br.start_recording(
            "ifssim-dv_pipeline_stack-1", "x_y_20260514_153022",
            min_free_gib=10, _run=runner,
        )


def test_start_recording_pid_not_found_marks_state_starting():
    # All pgrep polls return rc=1 (no match). start_recording should
    # still return a state dict but with state="starting" so the
    # caller can surface the partial start to the operator.
    runner = _multi_run(
        {"returncode": 0, "stdout": _df_output(avail_gib=42)},
        {"returncode": 0, "stdout": ""},
        # 15 successive pgrep calls all empty
        *[{"returncode": 1, "stdout": ""} for _ in range(15)],
    )
    state = br.start_recording(
        "ifssim-dv_pipeline_stack-1", "x_y_20260514_153022",
        min_free_gib=10, _run=runner,
    )
    assert state["pid"] is None
    assert state["state"] == "starting"


# ----- stop_recording ----------------------------------------------------

def test_stop_recording_skips_terminal_state():
    state = {"state": "stopped", "name": "x"}
    out = br.stop_recording(state, Path("/tmp/never"), _run=lambda *a, **k: None)
    assert out["state"] == "stopped"  # unchanged


def test_stop_recording_sigints_pid_then_copies_bag(tmp_path: Path):
    state = {
        "name": "trackdrive_X_20260514_153022",
        "path_in_container": "/bags/trackdrive_X_20260514_153022",
        "container": "ifssim-dv_pipeline_stack-1",
        "pid": 12345,
        "state": "recording",
    }
    # Sequence of mocked calls:
    #   1. docker exec kill -INT 12345   -> ok
    #   2. docker exec pgrep ...         -> rc=1 (gone)
    #   3. docker cp ...                 -> ok
    runner = _multi_run(
        {"returncode": 0},
        {"returncode": 1},
        {"returncode": 0},
    )
    out = br.stop_recording(state, tmp_path, _run=runner, _sleep=lambda s: None)
    assert out["state"] == "stopped"
    assert "stopped_at" in out
    assert out["host_path"].startswith(str(tmp_path))


def test_stop_recording_falls_back_to_pkill_when_no_pid(tmp_path: Path):
    state = {
        "name": "trackdrive_X_20260514_153022",
        "path_in_container": "/bags/trackdrive_X_20260514_153022",
        "container": "ifssim-dv_pipeline_stack-1",
        "pid": None,
        "state": "starting",
    }
    captured_cmds = []

    def _fn(cmd, **kw):
        captured_cmds.append(cmd)
        result = MagicMock()
        # First call: pkill — ok. Second: pgrep — empty (gone). Third: cp — ok.
        result.returncode = 1 if "pgrep" in cmd else 0
        result.stdout = ""
        result.stderr = ""
        return result

    out = br.stop_recording(state, tmp_path, _run=_fn, _sleep=lambda s: None)
    # The first call must be pkill -INT -f (not kill PID).
    assert "pkill" in captured_cmds[0]
    assert "-INT" in captured_cmds[0]
    assert out["state"] == "stopped"


def test_stop_recording_docker_cp_failure_marks_failed(tmp_path: Path):
    state = {
        "name": "x_y_20260514_153022",
        "path_in_container": "/bags/x_y_20260514_153022",
        "container": "ifssim-dv_pipeline_stack-1",
        "pid": 99999,
        "state": "recording",
    }
    runner = _multi_run(
        {"returncode": 0},                       # kill -INT
        {"returncode": 1},                       # pgrep — gone
        {"returncode": 1, "stderr": "No such container:path"},  # cp fails
    )
    out = br.stop_recording(state, tmp_path, _run=runner, _sleep=lambda s: None)
    assert out["state"] == "failed"
    assert "error" in out
