"""Stands in for tools/characterise: writes a result, notes its arguments, can fail or hang."""
import argparse, json, pathlib, sys, time

ap = argparse.ArgumentParser()
ap.add_argument("--sim"); ap.add_argument("--out"); ap.add_argument("--sim-log")
ap.add_argument("--status", default="pass"); ap.add_argument("--sleep", type=float, default=0.0)
a = ap.parse_args()
time.sleep(a.sleep)
out = pathlib.Path(a.out)
out.mkdir(parents=True, exist_ok=True)
(out / "result.json").write_text(json.dumps({"status": a.status, "args": vars(a), "t": time.time()}))
sys.exit({"pass": 0, "fail": 1}.get(a.status, 2))
