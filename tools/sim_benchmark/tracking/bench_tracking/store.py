"""RunBundle <-> a directory of Parquet + JSON files.

This is the full-resolution copy of a run that a tracker keeps as an artifact
(``bundle/`` in MLflow). The viewer reads it back into a RunBundle, so every
figure builder works on tracker data exactly as it does on local run dirs.

Layout::

    bundle/
      meta.json                 job_type, name, group, status, tags, config, summary, notes, started_at,
                                series index {family: step_name}, table index {name: description}
      series/<family>.parquet   column "step" + one column per value, float64, full resolution
      tables/<name>.parquet     one file per table, incl. "events"

Tracker metric histories are decimated and (in MLflow) integer-stepped. The
bundle is what keeps float x axes and every sample.
"""

from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from .bundle import SCHEMA_VERSION, Event, RunBundle, Series, Table

BUNDLE_DIR = "bundle"


def _json_default(o: Any) -> Any:
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, (Path, datetime)):
        return str(o)
    raise TypeError(type(o))


def _clean(v: Any) -> Any:
    if isinstance(v, float) and not math.isfinite(v):
        return None
    return v


def _table_df(t: Table) -> pd.DataFrame:
    df = pd.DataFrame(t.rows, columns=t.columns)
    for c in df.columns:
        if df[c].dtype == object:
            kinds = {
                type(v)
                for v in df[c]
                if v is not None and not (isinstance(v, float) and math.isnan(v))
            }
            if len(kinds) > 1 and not kinds <= {int, float}:
                df[c] = df[c].map(lambda v: None if v is None else str(v))
    return df


def write_bundle(b: RunBundle, out: Path) -> Path:
    """Write ``b`` under ``out`` (created). Returns ``out``."""
    (out / "series").mkdir(parents=True, exist_ok=True)
    (out / "tables").mkdir(parents=True, exist_ok=True)
    tables = dict(b.tables)
    if b.events:
        tables["events"] = b.events_table()
    meta = {
        "schema_version": SCHEMA_VERSION,
        "job_type": b.job_type,
        "name": b.name,
        "group": b.group,
        "status": b.status,
        "started_at": b.started_at.isoformat(),
        "tags": b.tags,
        "notes": b.notes,
        "config": b.config,
        "summary": {k: _clean(v) for k, v in b.summary.items()},
        "series": {f: s.step_name for f, s in b.series.items()},
        "tables": {n: t.description for n, t in tables.items()},
    }
    (out / "meta.json").write_text(
        json.dumps(meta, default=_json_default, allow_nan=False)
    )
    for fam, s in b.series.items():
        pd.DataFrame({"step": s.step, **s.values}).to_parquet(
            out / "series" / f"{fam}.parquet", index=False
        )
    for name, t in tables.items():
        _table_df(t).to_parquet(out / "tables" / f"{name}.parquet", index=False)
    return out


def _nan_to_none(v: Any) -> Any:
    return None if isinstance(v, float) and math.isnan(v) else v


def read_bundle(d: Path) -> RunBundle:
    meta = json.loads((d / "meta.json").read_text())
    b = RunBundle(
        job_type=meta["job_type"],
        name=meta["name"],
        group=meta["group"],
        source_dir=None,
        started_at=datetime.fromisoformat(meta["started_at"]),
        status=meta["status"],
        tags=list(meta["tags"]),
        config=meta["config"],
        summary=meta["summary"],
        notes=meta.get("notes", ""),
    )
    for fam, step_name in meta["series"].items():
        df = pd.read_parquet(d / "series" / f"{fam}.parquet")
        b.add_series(
            Series(
                fam,
                step_name,
                df["step"].to_numpy(float),
                {c: df[c].to_numpy(float) for c in df.columns if c != "step"},
            )
        )
    for name, desc in meta["tables"].items():
        df = pd.read_parquet(d / "tables" / f"{name}.parquet")
        rows = [
            [_nan_to_none(v.item() if hasattr(v, "item") else v) for v in r]
            for r in df.itertuples(index=False)
        ]
        t = Table(name, list(df.columns), rows, desc)
        if name == "events":
            b.events = [
                Event(
                    t_s=r[0],
                    node=r[1],
                    kind=r[2],
                    severity=r[3],
                    detail=r[4],
                    s_m=r[5],
                    x=r[6],
                    y=r[7],
                )
                for r in rows
            ]
        else:
            b.add_table(t)
    return b
