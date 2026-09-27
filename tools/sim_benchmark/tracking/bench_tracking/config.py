"""Where the tracker is: one small file per machine, read by bench-track and bench-view.

``~/.config/ifssim-bench/tracking.env`` (or ``$IFSSIM_BENCH_CONFIG``), ``KEY=VALUE``
lines, for example::

    MLFLOW_TRACKING_URI=https://bench.<tailnet>.ts.net
    MLFLOW_TRACKING_USERNAME=alice
    MLFLOW_TRACKING_PASSWORD=...

Variables already set in the environment win over the file. The password can
also live in ``~/.mlflow/credentials``, which MLflow reads on its own. Nothing
here belongs in the repository.
"""

from __future__ import annotations

import os
from pathlib import Path

DEFAULT = "~/.config/ifssim-bench/tracking.env"


def path() -> Path:
    return Path(os.environ.get("IFSSIM_BENCH_CONFIG", DEFAULT)).expanduser()


def load() -> Path | None:
    """Put the file's variables into the environment (without overriding). Returns the file if read."""
    p = path()
    if not p.is_file():
        return None
    for line in p.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        os.environ.setdefault(k.strip(), v.strip().strip("'\""))
    return p
