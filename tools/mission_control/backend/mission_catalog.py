"""Mission list for Mission Control — pipeline registry + sim-only modes.

Pipeline modes are read from ``mode_manager.mode_registry`` when that
package is importable (docker / sourced workspace). A static fallback
keeps unit tests and bare-metal dev working without the full stack.

Sim-only missions (e.g. benchmark control) are defined here so they never
land in the car ``pipeline/`` submodule.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Optional

MissionKind = Literal["pipeline", "sim_benchmark_control"]


@dataclass(frozen=True)
class MissionSpec:
    name: str
    label: str
    kind: MissionKind
    sim_event_type: str
    mission_id: Optional[int] = None
    supports_laps: bool = False
    description: str = ""


# FS referee event names differ from pipeline mode names in one case.
_MISSION_TO_SIM_EVENT: dict[str, str] = {
    "accel": "acceleration",
}

_SIM_EVENT_TO_MISSION: dict[str, str] = {
    "acceleration": "accel",
}

# Fallback mirrors pipeline/mode_manager/mode_manager/mode_registry.py
_FALLBACK_PIPELINE: tuple[MissionSpec, ...] = (
    MissionSpec("trackdrive", "Trackdrive", "pipeline", "trackdrive", 1, True),
    MissionSpec("autocross", "Autocross", "pipeline", "autocross", 2, True),
    MissionSpec("accel", "Acceleration", "pipeline", "acceleration", 3, True),
    MissionSpec("skidpad", "Skidpad", "pipeline", "skidpad", 4, True),
    MissionSpec("scruti", "Scruti", "pipeline", "scruti", 5, True),
)

SIM_ONLY_MISSIONS: tuple[MissionSpec, ...] = (
    MissionSpec(
        name="benchmark_control",
        label="Benchmark control",
        kind="sim_benchmark_control",
        sim_event_type="trackdrive",
        mission_id=None,
        supports_laps=True,
        description=(
            "Sim-only: GT pure-pursuit driver plus control error harness. "
            "Does not start the autonomy pipeline."
        ),
    ),
)


def _label_for_mode(mode_name: str) -> str:
    if mode_name == "accel":
        return "Acceleration"
    return mode_name.replace("_", " ").title()


def _load_pipeline_from_registry() -> tuple[MissionSpec, ...] | None:
    try:
        from mode_manager.mode_registry import MODE_REGISTRY
    except ImportError:
        return None
    specs: list[MissionSpec] = []
    for mode in sorted(MODE_REGISTRY.values(), key=lambda m: m.mission_id):
        sim_ev = _MISSION_TO_SIM_EVENT.get(mode.mode_name, mode.mode_name)
        specs.append(
            MissionSpec(
                name=mode.mode_name,
                label=_label_for_mode(mode.mode_name),
                kind="pipeline",
                sim_event_type=sim_ev,
                mission_id=mode.mission_id,
                supports_laps=mode.mode_name == "trackdrive",
            )
        )
    return tuple(specs)


def list_missions() -> list[MissionSpec]:
    pipeline = _load_pipeline_from_registry() or _FALLBACK_PIPELINE
    names = {m.name for m in pipeline}
    sim_extra = tuple(m for m in SIM_ONLY_MISSIONS if m.name not in names)
    return list(pipeline) + list(sim_extra)


def get_mission(name: str) -> Optional[MissionSpec]:
    for m in list_missions():
        if m.name == name:
            return m
    return None


def mission_name_to_id() -> dict[str, int]:
    return {
        m.name: m.mission_id
        for m in list_missions()
        if m.mission_id is not None
    }


def sim_event_for_mission(mission_name: str) -> str:
    spec = get_mission(mission_name)
    if spec is None:
        return mission_name
    return spec.sim_event_type


def mission_for_sim_event(sim_event: str) -> str:
    return _SIM_EVENT_TO_MISSION.get(sim_event, sim_event)


def missions_for_api() -> list[dict]:
    out: list[dict] = []
    for m in list_missions():
        out.append(
            {
                "name": m.name,
                "label": m.label,
                "kind": m.kind,
                "mission_id": m.mission_id,
                "sim_event_type": m.sim_event_type,
                "supports_laps": m.supports_laps,
                "description": m.description,
            }
        )
    return out
