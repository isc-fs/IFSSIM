"""Unit tests for tools/mission_control/backend/bag_recorder.py (#465 v2).

bag_recorder is now a thin client over the /bag_recorder/{start,stop}
ROS services hosted by dv_pipeline_stack's bag_recorder_node. The
tests mock out the rclpy plumbing so the suite stays pure-Python.

Subprocess management lives in pipeline/bag_recorder_node/recorder.py
and has its own test suite there. This file tests just the
ROS-service-client wrapper.
"""
from __future__ import annotations

import os
import sys
from datetime import datetime, timezone
from types import SimpleNamespace
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
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("trackdrive", "track_20260512_151240.csv", now=when)
    assert out == "trackdrive_track_20260512_151240_20260514_153022"
    assert ".csv" not in out


def test_compose_bag_name_replaces_unsafe_chars():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("track drive!", "../etc/passwd", now=when)
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


# ----- request_start / request_stop --------------------------------------
#
# These exercise the rclpy service-client path. We monkey-patch the
# lazy-init `_ensure_clients` to drop in mocks that match the rclpy
# service-client surface (wait_for_service, call_async, the response
# fields the wrapper reads). _reset_clients_for_test clears the module
# globals between tests so each one starts from a clean slate.


def _make_fake_bridge():
    """Return a stand-in for the RosBridge singleton."""
    bridge = MagicMock()
    bridge._node = MagicMock()
    return bridge


def _install_fake_clients(monkeypatch, start_resp, stop_resp,
                          start_reachable=True, stop_reachable=True,
                          start_call_returns_none=False,
                          stop_call_returns_none=False):
    """Monkey-patch bag_recorder's lazy client init to inject mocks.

    `start_resp`/`stop_resp` are SimpleNamespace objects with the
    response fields the wrapper reads (ok / state / bag_path / error).
    None on `*_call_returns_none` simulates a future that timed out.
    """
    br._reset_clients_for_test()

    fake_start_client = MagicMock()
    fake_start_client.wait_for_service = MagicMock(return_value=start_reachable)
    fake_stop_client = MagicMock()
    fake_stop_client.wait_for_service = MagicMock(return_value=stop_reachable)

    # Stand-in Srv types whose .Request() returns a settable object.
    class _FakeStartReq:
        def __init__(self):
            self.bag_name = ""

    class _FakeStopReq:
        pass

    class _FakeStartSrv:
        Request = _FakeStartReq

    class _FakeStopSrv:
        Request = _FakeStopReq

    def _ensure(_bridge):
        br._start_client = fake_start_client
        br._stop_client = fake_stop_client
        br._start_srv_type = _FakeStartSrv
        br._stop_srv_type = _FakeStopSrv

    monkeypatch.setattr(br, "_ensure_clients", _ensure)

    # `_call_sync` does the future-wait dance. Mock it directly — the
    # actual rclpy future plumbing is library-internal, not interesting
    # for the wrapper's contract.
    def _fake_call_sync(client, request, timeout_s):
        if client is fake_start_client:
            return None if start_call_returns_none else start_resp
        if client is fake_stop_client:
            return None if stop_call_returns_none else stop_resp
        raise AssertionError("unexpected client passed to _call_sync")

    monkeypatch.setattr(br, "_call_sync", _fake_call_sync)


def test_request_start_happy_path(monkeypatch):
    start_resp = SimpleNamespace(
        ok=True, state="recording",
        bag_path="/bags/trackdrive_X_20260514_153022",
        error="",
    )
    _install_fake_clients(monkeypatch, start_resp, stop_resp=None)

    out = br.request_start(_make_fake_bridge(), "trackdrive_X_20260514_153022")

    assert out["ok"] is True
    assert out["state"] == "recording"
    assert out["name"] == "trackdrive_X_20260514_153022"
    assert out["path"] == "/bags/trackdrive_X_20260514_153022"
    assert out["error"] == ""


def test_request_start_propagates_disk_full(monkeypatch):
    # bag_recorder_node returns ok=False, state="failed", with an error
    # explaining the disk situation. The client must surface that
    # verbatim so the UI can show it.
    start_resp = SimpleNamespace(
        ok=False, state="failed", bag_path="",
        error="only 3 GiB free on /bags (need ≥10 GiB)",
    )
    _install_fake_clients(monkeypatch, start_resp, stop_resp=None)

    out = br.request_start(_make_fake_bridge(), "x_y_20260514_153022")

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "3 GiB" in out["error"]


def test_request_start_handles_unreachable_service(monkeypatch):
    _install_fake_clients(
        monkeypatch, start_resp=None, stop_resp=None,
        start_reachable=False,
    )

    out = br.request_start(
        _make_fake_bridge(), "x_y_20260514_153022", timeout_s=0.1,
    )

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "not reachable" in out["error"]


def test_request_start_handles_timeout(monkeypatch):
    _install_fake_clients(
        monkeypatch, start_resp=None, stop_resp=None,
        start_call_returns_none=True,
    )

    out = br.request_start(
        _make_fake_bridge(), "x_y_20260514_153022", timeout_s=0.1,
    )

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "timed out" in out["error"]


def test_request_stop_happy_path(monkeypatch):
    stop_resp = SimpleNamespace(
        ok=True, state="stopped",
        bag_path="/bags/trackdrive_X_20260514_153022",
        error="",
    )
    _install_fake_clients(monkeypatch, start_resp=None, stop_resp=stop_resp)

    out = br.request_stop(_make_fake_bridge())

    assert out["ok"] is True
    assert out["state"] == "stopped"
    assert out["path"] == "/bags/trackdrive_X_20260514_153022"
    assert out["error"] == ""


def test_request_stop_idempotent_when_nothing_active(monkeypatch):
    stop_resp = SimpleNamespace(ok=True, state="none", bag_path="", error="")
    _install_fake_clients(monkeypatch, start_resp=None, stop_resp=stop_resp)

    out = br.request_stop(_make_fake_bridge())

    assert out["ok"] is True
    assert out["state"] == "none"
    assert out["path"] == ""


def test_request_stop_surfaces_failure(monkeypatch):
    stop_resp = SimpleNamespace(
        ok=False, state="failed",
        bag_path="/tmp/ifssim_bag_active/x_y_20260514_153022",
        error="staging→final move failed: cross-device link not permitted",
    )
    _install_fake_clients(monkeypatch, start_resp=None, stop_resp=stop_resp)

    out = br.request_stop(_make_fake_bridge())

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "move failed" in out["error"]


def test_request_stop_handles_timeout(monkeypatch):
    _install_fake_clients(
        monkeypatch, start_resp=None, stop_resp=None,
        stop_call_returns_none=True,
    )

    out = br.request_stop(_make_fake_bridge(), timeout_s=0.1)

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "timed out" in out["error"]
