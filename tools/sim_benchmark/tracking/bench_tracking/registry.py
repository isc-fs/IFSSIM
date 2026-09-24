"""Metric registry: canonical names, units, direction and regression thresholds."""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Iterable, Literal

import yaml

Direction = Literal["min", "max", "none"]


@dataclass(frozen=True)
class MetricSpec:
    name: str
    unit: str
    direction: Direction
    desc: str
    threshold: float | None = None
    headline: bool = False

    @property
    def domain(self) -> str:
        return self.name.split("/", 1)[0]


@lru_cache(maxsize=1)
def registry() -> dict[str, MetricSpec]:
    raw = yaml.safe_load((Path(__file__).parent / "metrics.yaml").read_text())
    return {
        name: MetricSpec(
            name=name,
            unit=str(spec.get("unit", "")),
            direction=spec.get("direction", "none"),
            desc=spec.get("desc", ""),
            threshold=spec.get("threshold"),
            headline=bool(spec.get("headline", False)),
        )
        for name, spec in raw.items()
    }


def spec(name: str) -> MetricSpec | None:
    return registry().get(name)


def headline(names: Iterable[str]) -> list[str]:
    reg = registry()
    return [n for n in names if n in reg and reg[n].headline]


def check_names(names: Iterable[str], *, context: str) -> list[str]:
    """Warn (don't fail) on metrics that aren't registered — keeps the vocabulary honest."""
    suffixes = (".std", ".min", ".max", ".delta", ".baseline")
    base = lambda n: next((n[: -len(x)] for x in suffixes if n.endswith(x)), n)  # noqa: E731
    unknown = sorted(n for n in names if base(n) not in registry())
    if unknown:
        warnings.warn(f"{context}: unregistered metrics {unknown}", stacklevel=2)
    return unknown


def delta(
    name: str, value: float | None, baseline: float | None
) -> dict[str, float | bool | None]:
    """Signed delta vs baseline and whether it is a regression per the registry."""
    if (
        value is None
        or baseline is None
        or not all(map(math.isfinite, (value, baseline)))
    ):
        return {"delta": None, "regression": None, "improvement": None}
    s = spec(name)
    d = value - baseline
    if s is None or s.direction == "none":
        return {"delta": d, "regression": None, "improvement": None}
    worse = d > 0 if s.direction == "min" else d < 0
    tol = s.threshold if s.threshold is not None else 0.0
    regression = worse and abs(d) > tol
    improvement = (not worse) and abs(d) > tol and d != 0
    return {"delta": d, "regression": regression, "improvement": improvement}
