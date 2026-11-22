"""The emulated boards' pace over a run, in the run's results."""
from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_mission_benchmark as b  # noqa: E402

T0 = datetime(2026, 11, 15, 20, 0, 0, tzinfo=timezone.utc)


def rows(rtf, lag0=0.0, lag_per_s=0.0, n=6):
    """server.py's log lines: epoch seconds, the emulation's clock and how far behind it is."""
    return [{"wall": T0.timestamp() + 2 * i, "t": 10 + 2 * i * rtf, "lag_s": lag0 + 2 * i * lag_per_s} for i in range(n)]


def test_a_run_the_boards_kept_up_with_is_real_time():
    p = b.boards_pace_of(rows(rtf=0.998, lag0=0.3))
    assert p == {"rtf": 0.998, "max_lag_s": 0.3, "real_time_ok": True}


def test_boards_that_fell_behind_are_flagged_by_pace_or_by_lag():
    assert b.boards_pace_of(rows(rtf=0.89, lag_per_s=0.11))["real_time_ok"] is False        # slow
    assert b.boards_pace_of(rows(rtf=1.0, lag0=2.5))["real_time_ok"] is False               # far behind from the start
    assert b.boards_pace_of(rows(rtf=1.0)[:1]) is None                                       # too few lines to say


def test_the_pace_is_read_from_the_boards_log_for_the_runs_window(monkeypatch):
    monkeypatch.setenv("IFSSIM_VHIL_BOARDS", "127.0.0.1:47800")
    lines = ["vhil-boards ready on 127.0.0.1:47800", *[json.dumps(r) for r in rows(rtf=1.0)], "{not json"]
    seen = {}

    def fake(cmd, **kw):
        seen["cmd"] = cmd
        return subprocess.CompletedProcess(cmd, 0, "\n".join(lines), "")

    p = b.boards_pace(T0, T0 + timedelta(seconds=10), run=fake)
    assert p and p["real_time_ok"] and seen["cmd"][:2] == ["docker", "logs"] and seen["cmd"][-1] == b.BOARDS_CONTAINER


def test_no_boards_no_pace(monkeypatch):
    monkeypatch.delenv("IFSSIM_VHIL_BOARDS", raising=False)
    assert b.boards_pace(T0, T0 + timedelta(seconds=5), run=lambda *a, **k: 1 / 0) is None


def test_the_results_carry_it():
    res = b.RunResult(run=None, outcome="finished", started=None, seconds=10.0,
                      boards={"rtf": 0.89, "max_lag_s": 4.2, "real_time_ok": False},
                      referee={"lap_times": [9.0], "doo_counter": 0, "oc_counter": 0})
    m = b.race_metrics(res)
    assert (m["box/boards_rtf"], m["box/boards_max_lag_s"], m["race/boards_real_time_ok"]) == (0.89, 4.2, 0)
    assert "box/boards_rtf" not in b.race_metrics(b.RunResult(run=None, outcome="finished", started=None, seconds=1.0))


def test_a_restart_inside_the_window_does_not_spoil_the_pace():
    """The server restarts the car when the operator goes OFF, at the end of the run: its clock
    goes back to zero, and the lines after it must not be mixed with the run's."""
    run = rows(rtf=1.0, n=12)                                    # 22 s of the run, in step with the wall
    after = [{"wall": T0.timestamp() + 24 + 2 * i, "t": 0.5 + 2 * i, "lag_s": 0.0} for i in range(2)]
    p = b.boards_pace_of(run + after)
    assert p == {"rtf": 1.0, "max_lag_s": 0.0, "real_time_ok": True}
