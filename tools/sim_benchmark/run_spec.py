#!/usr/bin/env python3
"""Run a benchmark spec on this checkout: ``python3 tools/sim_benchmark/run_spec.py --help``.

The same command runs on a laptop, on the bench PC's runner and from Mission
Control. The code is in ``launch/`` (manifest, spec format, runner).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from launch.run import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
