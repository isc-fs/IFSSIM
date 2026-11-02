"""Pipeline parameter overrides for the benchmarks (``--pipeline-overrides FILE``).

The file is JSON, one section per pipeline component, each a mapping of
parameter name to value::

    {"cone_detection": {"residual_gate_mse": 0.05},
     "slam_node": {"motion_model": "imu"}}

``bench-run`` writes it from the ``pipeline:`` section of a run spec
(``bench.yaml`` at the repo root lists which benchmark takes which component).
Each benchmark applies the sections it uses and rejects the rest, so a typo
fails the run before it starts instead of silently measuring the defaults:

* ``cone_detection``: fields of ``ConeDetectionConfig`` (perception).
* ``slam_node``: ROS parameters of ``ConeGraphSlamNode`` (SLAM).

What was in effect is written to ``<run>/params/<component>.json``: every
value, defaults included, plus which ones were overridden.

Standard library only: this runs in the container's system Python.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
from typing import Any

COMPONENTS = ("cone_detection", "slam_node")
PARAMS_DIR = "params"


class OverrideError(ValueError):
    pass


def load(path: str | Path | None) -> dict[str, dict[str, Any]]:
    """The overrides in ``path`` ({} for none), checked for shape."""
    if not path:
        return {}
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, ValueError) as e:
        raise OverrideError(f"cannot read pipeline overrides {path}: {e}") from e
    if not isinstance(data, dict):
        raise OverrideError(f"{path}: expected an object of components")
    for comp, values in data.items():
        if comp not in COMPONENTS:
            raise OverrideError(
                f"{path}: unknown component {comp!r} (known: {', '.join(COMPONENTS)})"
            )
        if not isinstance(values, dict):
            raise OverrideError(f"{path}: {comp} must map parameter names to values")
    return data


def only(overrides: dict[str, dict[str, Any]], *used: str) -> None:
    """Refuse sections this benchmark would ignore."""
    extra = sorted(set(overrides) - set(used))
    if extra:
        raise OverrideError(
            f"this benchmark does not use {', '.join(extra)} "
            f"(it takes: {', '.join(used) or 'nothing'})"
        )


def apply_dataclass(base: Any, values: dict[str, Any], component: str) -> Any:
    """``base`` with ``values`` replaced; unknown fields are an error."""
    known = {f.name for f in dataclasses.fields(base)}
    unknown = sorted(set(values) - known)
    if unknown:
        raise OverrideError(
            f"{component}: unknown parameter(s) {', '.join(unknown)} "
            f"(see {type(base).__name__})"
        )
    return dataclasses.replace(base, **values)


def ros_parameters(node: Any, values: dict[str, Any], component: str) -> None:
    """Set ``values`` on a ROS node; parameters it doesn't declare are an error."""
    from rclpy.parameter import Parameter

    unknown = sorted(k for k in values if not node.has_parameter(k))
    if unknown:
        raise OverrideError(
            f"{component}: {type(node).__name__} has no parameter(s) {', '.join(unknown)}"
        )
    results = node.set_parameters([Parameter(k, value=v) for k, v in values.items()])
    bad = [
        f"{k} ({r.reason})"
        for k, r in zip(values, results, strict=True)
        if not r.successful
    ]
    if bad:
        raise OverrideError(f"{component}: rejected {', '.join(bad)}")


def ros_values(node: Any) -> dict[str, Any]:
    return {
        name: p.value for name, p in sorted(node.get_parameters_by_prefix("").items())
    }


def write_effective(
    run_dir: Path, component: str, values: dict[str, Any], overridden: dict[str, Any]
) -> Path:
    out = Path(run_dir) / PARAMS_DIR / f"{component}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(
            {"values": values, "overridden": sorted(overridden)},
            indent=2,
            sort_keys=True,
            default=str,
        )
    )
    return out
