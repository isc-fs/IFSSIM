#!/usr/bin/env python3
"""Run the sim perception benchmark from the IFSSIM repo root (no cd).

Example::

    python run_perception_benchmark.py results/capture/sim_benchmark_20260527_135117 \\
        --profile --profile-frames 80 --bev-samples 8

Bag paths like ``results/capture/...`` are resolved under ``tools/sim_benchmark/``.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_BENCH = Path(__file__).resolve().parent / "tools" / "sim_benchmark"

if __name__ == "__main__":
    sys.path.insert(0, str(_BENCH))
    runpy.run_path(str(_BENCH / "run_perception_benchmark.py"), run_name="__main__")
