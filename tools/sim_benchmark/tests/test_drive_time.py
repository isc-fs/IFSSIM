"""race/drive_time_s: what the drive took, whatever the start sequence before the car moved."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_mission_benchmark as b  # noqa: E402


def result(seconds, set_off, lap):
    return b.RunResult(run=None, outcome="finished", started=None, seconds=seconds, set_off=set_off,
                       referee={"lap_times": [lap], "doo_counter": 0, "oc_counter": 0})


def test_drive_time_counts_from_the_cars_first_motion():
    # the referee's acceleration clock starts when the event is set: both runs drove 8 s, and the
    # one that waited 3.4 s longer for the car to be ready (the ECU's own sequence) reads slower
    quick = b.race_metrics(result(seconds=13.4, set_off=5.4, lap=12.9))
    slow = b.race_metrics(result(seconds=16.6, set_off=8.7, lap=16.5))
    assert quick["race/drive_time_s"] == 8.0 and slow["race/drive_time_s"] == 7.9
    assert quick["race/total_time_s"] == 12.9 and slow["race/total_time_s"] == 16.5   # the referee's, unchanged
    assert (quick["race/set_off_s"], slow["race/set_off_s"]) == (5.4, 8.7)


def test_no_motion_seen_leaves_the_fields_out():
    assert "race/drive_time_s" not in b.race_metrics(result(seconds=10.0, set_off=None, lap=9.0))
