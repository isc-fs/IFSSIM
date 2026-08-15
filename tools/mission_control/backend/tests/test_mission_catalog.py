"""Unit tests for mission_catalog."""
from __future__ import annotations

import mission_catalog as mc


def test_list_missions_includes_sim_only_benchmark():
    names = {m.name for m in mc.list_missions()}
    assert "trackdrive" in names
    assert "scruti" in names
    assert "benchmark_control" in names


def test_sim_event_mapping_accel():
    assert mc.sim_event_for_mission("accel") == "acceleration"
    assert mc.mission_for_sim_event("acceleration") == "accel"


def test_get_benchmark_control_kind():
    spec = mc.get_mission("benchmark_control")
    assert spec is not None
    assert spec.kind == "sim_benchmark_control"
    assert spec.mission_id is None


def test_missions_for_api_shape():
    rows = mc.missions_for_api()
    assert rows
    assert "name" in rows[0] and "label" in rows[0]
