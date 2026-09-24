"""Backend-neutral description of one benchmark run.

Adapters turn a run directory into a :class:`RunBundle`; backends turn a
RunBundle into tracker calls. Nothing in here knows about W&B, MLflow or
ClearML, so the same bundle is what every backend receives — the comparison
between trackers is apples to apples.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

import numpy as np

SCHEMA_VERSION = 1

JobType = Literal[
    "onboard_replay",  # bag replayed through the live pipeline, --report
    "live_replay",  # bag replayed with --live (Lichtblick), logs only
    "perception_gt",
    "slam_gt",
    "control_gt",
    "sim_e2e",  # simulator-in-the-loop, full pipeline, one seed
    "sim_aggregate",  # N seeds of one scenario at one commit
    "sim_sweep_trial",  # one sweep trial (aggregate over its seeds)
]
Status = Literal["finished", "failed", "aborted"]
FileKind = Literal["report", "data", "log", "provenance", "config"]


@dataclass
class Series:
    """A family of per-sample metrics sharing one x axis.

    ``step`` is the x axis (seconds since replay start, metres along the
    track, lap number, ...). Every entry of ``values`` has the same length as
    ``step``; NaN marks a missing sample.
    """

    family: str
    step_name: str  # e.g. "t/perception_s", "track/s_m"
    step: np.ndarray
    values: dict[str, np.ndarray]

    def __post_init__(self) -> None:
        self.step = np.asarray(self.step, dtype=float)
        for k, v in list(self.values.items()):
            v = np.asarray(v, dtype=float)
            if v.shape != self.step.shape:
                raise ValueError(
                    f"series {self.family}/{k}: {v.shape} != step {self.step.shape}"
                )
            self.values[k] = v

    def __len__(self) -> int:
        return int(self.step.shape[0])

    def decimated(self, max_points: int) -> "Series":
        """Stride-decimate, always keeping the last sample."""
        n = len(self)
        if n <= max_points:
            return self
        idx = np.unique(np.r_[np.linspace(0, n - 1, max_points).astype(int), n - 1])
        return Series(
            self.family,
            self.step_name,
            self.step[idx],
            {k: v[idx] for k, v in self.values.items()},
        )


@dataclass
class Table:
    name: str
    columns: list[str]
    rows: list[list[Any]]
    description: str = ""

    def column(self, name: str) -> list[Any]:
        i = self.columns.index(name)
        return [r[i] for r in self.rows]

    def decimated(self, max_rows: int, *, by: str | None = None) -> "Table":
        """Keep at most ``max_rows`` rows per distinct value of ``by``."""
        if by is None:
            groups = {None: self.rows}
        else:
            i = self.columns.index(by)
            groups: dict[Any, list] = {}
            for r in self.rows:
                groups.setdefault(r[i], []).append(r)
        out: list[list[Any]] = []
        for rows in groups.values():
            if len(rows) <= max_rows:
                out.extend(rows)
            else:
                idx = np.unique(np.linspace(0, len(rows) - 1, max_rows).astype(int))
                out.extend(rows[j] for j in idx)
        return Table(self.name, self.columns, out, self.description)


@dataclass
class Event:
    """Something discrete worth a row in the events table (and maybe an alert)."""

    t_s: float | None
    node: str
    kind: str  # e.g. "pose_jump_rejected", "crash", "dnf_off_track"
    severity: Literal["info", "warn", "error"]
    detail: str
    s_m: float | None = None
    x: float | None = None
    y: float | None = None


@dataclass
class RunBundle:
    job_type: JobType
    name: str
    group: str
    source_dir: Path | None
    started_at: datetime
    status: Status = "finished"
    tags: list[str] = field(default_factory=list)
    config: dict[str, Any] = field(default_factory=dict)
    summary: dict[str, float | int | str | None] = field(default_factory=dict)
    series: dict[str, Series] = field(default_factory=dict)
    tables: dict[str, Table] = field(default_factory=dict)
    events: list[Event] = field(default_factory=list)
    files: list[tuple[Path, FileKind]] = field(default_factory=list)
    html_report: Path | None = None
    notes: str = ""

    @property
    def run_key(self) -> str:
        """Stable id: same source run -> same key, so re-uploads are detectable."""
        basis = json.dumps(
            {
                "job": self.job_type,
                "name": self.name,
                "group": self.group,
                "src": str(self.source_dir) if self.source_dir else None,
            },
            sort_keys=True,
        )
        return hashlib.sha1(basis.encode()).hexdigest()[:12]

    @property
    def scenario_id(self) -> str:
        return str(self.config.get("scenario", {}).get("id", "unknown"))

    def add_series(self, s: Series) -> None:
        self.series[s.family] = s

    def add_table(self, t: Table) -> None:
        self.tables[t.name] = t

    def events_table(self) -> Table:
        return Table(
            "events",
            ["t_s", "node", "kind", "severity", "detail", "s_m", "x", "y"],
            [
                [e.t_s, e.node, e.kind, e.severity, e.detail, e.s_m, e.x, e.y]
                for e in self.events
            ],
            "Discrete events: warnings, rejections, crashes, penalties, DNFs",
        )


def flatten(d: dict[str, Any], prefix: str = "", sep: str = ".") -> dict[str, Any]:
    """{'a': {'b': 1}} -> {'a.b': 1}; lists become JSON strings."""
    out: dict[str, Any] = {}
    for k, v in d.items():
        key = f"{prefix}{sep}{k}" if prefix else str(k)
        if isinstance(v, dict):
            out.update(flatten(v, key, sep))
        elif isinstance(v, (list, tuple)):
            out[key] = json.dumps(v)
        else:
            out[key] = v
    return out


def finite_or_none(v: Any) -> Any:
    if isinstance(v, (float, np.floating)):
        return None if not np.isfinite(v) else float(v)
    if isinstance(v, np.integer):
        return int(v)
    return v
