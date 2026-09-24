"""Backend interface: one call uploads one RunBundle.

The harness in the design doc sketches a fine-grained Tracker API
(log_config / log_series / ...). Here each backend implements the same
steps as private helpers behind ``upload(bundle)``. That keeps the backend-specific
decisions (int steps in MLflow, iterations in ClearML, step metrics in W&B)
in one readable place per tracker.
"""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from ..bundle import RunBundle, Series

# Same decimation budget for every backend so the comparison is fair.
MAX_SERIES_POINTS = 3000
MAX_TABLE_ROWS_PER_GROUP = 2500


@dataclass
class UploadResult:
    backend: str
    run_id: str
    url: str | None
    name: str


class Backend(ABC):
    name: str

    @abstractmethod
    def upload(
        self,
        b: RunBundle,
        *,
        figures: dict[str, Any],
        parent: UploadResult | None = None,
    ) -> UploadResult: ...

    def finish(self) -> None:  # flush / close sessions
        pass


# Off by default: imports read run dirs that belong to someone else's checkout.
WRITE_RECORDS = False


def record(run_dir: Path | None, res: UploadResult) -> None:
    """tracking.json next to the run files, when enabled (``--write-records``)."""
    if run_dir is None or not WRITE_RECORDS:
        return
    p = run_dir / "tracking.json"
    try:
        data = json.loads(p.read_text()) if p.is_file() else {}
        data[res.backend] = {
            "run_id": res.run_id,
            "url": res.url,
            "name": res.name,
            "uploaded_at": datetime.now(timezone.utc).isoformat(),
        }
        p.write_text(json.dumps(data, indent=2))
    except OSError:
        pass


def series_rows(
    s: Series, max_points: int = MAX_SERIES_POINTS
) -> tuple[list[float], dict[str, list[float | None]]]:
    d = s.decimated(max_points)
    # NaN -> None
    vals = {
        k: [None if v != v else float(v) for v in arr] for k, arr in d.values.items()
    }
    return [float(x) for x in d.step], vals
