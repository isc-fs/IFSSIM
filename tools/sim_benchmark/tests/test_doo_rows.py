"""events.csv's kind=doo rows (the referee's per-cone hits)."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_mission_benchmark as b  # noqa: E402


class TestDooRows:
    def test_doo_events_become_events_csv_rows(self):
        res = b.RunResult(run=None, outcome="finished", started=None, seconds=10.0, sim_start=100.0,
            referee={"doo_events": [{"t": 112.5, "cone": 7, "name": "Cone 7", "color": 1, "x": 1.0,
                                     "y": 2.0, "displacement": 0.3, "tilt_deg": 50.0,
                                     "reason": "tipped", "car_speed": 3.0}]})
        (row,) = b.doo_rows(res)
        assert (row["kind"], row["t"]) == ("doo", 12.5)
        assert "color=blue" in row["detail"]
        assert "name=Cone_7" in row["detail"]
        assert "why=tipped" in row["detail"]
        assert "car_x" not in row["detail"]


