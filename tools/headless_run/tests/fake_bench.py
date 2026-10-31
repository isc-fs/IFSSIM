"""Stands in for run_mission_benchmark.py: writes one run's manifest, sleeps, exits."""
import argparse, pathlib, sys, time

ap = argparse.ArgumentParser()
ap.add_argument("--sim"); ap.add_argument("--mc-url"); ap.add_argument("--results-root")
ap.add_argument("--mc-key"); ap.add_argument("--exit", type=int, default=0)
ap.add_argument("--sleep", type=float, default=0.0)
ap.add_argument("--crash", action="store_true")
a, _ = ap.parse_known_args()
if a.crash:
    raise ImportError("No module named 'yaml'")   # exits 1, like the real thing did
d = pathlib.Path(a.results_root) / "mission" / "accel_acceleration" / "sha" / "seed1"
d.mkdir(parents=True, exist_ok=True)
(d / "manifest.json").write_text("{}")
time.sleep(a.sleep)
sys.exit(a.exit)
