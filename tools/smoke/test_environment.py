#!/usr/bin/env python3
"""
Track environment sidecars against a running sim (RPC only; no Mission Control,
no Docker). See docs/environment_sidecar.md.

The sim's settings.json decides which half runs (getEnvironment reports it):

  Environment.Enabled false (the default):
    loading tools/smoke/fixtures/env_gate.csv, which has a sidecar, reports
    "environment": "off", and getEnvironment holds nothing.

  Environment.Enabled true:
    env_gate.csv loads its sidecar (seed 7, profile "smoke", 3 props, ground
    extent); ramp_gate.csv, which has none, reports "none"; env_bad.csv's
    sidecar has a misspelt key and reports "invalid" with that reason; and
    env_gate.csv loads again afterwards.

Either way the sim is left on acceleration.csv.

    python tools/smoke/test_environment.py                     # whichever mode the sim is in
    python tools/smoke/test_environment.py --expect-enabled    # fail unless enabled
"""
import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))
from ifssim.client import IFSSIMClient  # noqa: E402

FIXTURES = REPO / "tools" / "smoke" / "fixtures"


def rpc(client, cmd):
    resp = client._text_cmd(cmd)
    try:
        return json.loads(resp)
    except (TypeError, ValueError):
        return {"raw": resp}


def load(client, path: Path) -> dict:
    reply = rpc(client, f"loadTrack {path.as_posix()}")
    if "error" in reply or "raw" in reply:
        raise RuntimeError(f"loadTrack {path.name}: {reply}")
    return reply


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default="127.0.0.1:41451")
    ap.add_argument("--expect-enabled", action="store_true",
                    help="fail if the sim's Environment.Enabled is false")
    args = ap.parse_args()

    host, port = args.sim.rsplit(":", 1)
    client = IFSSIMClient(ip=host, port=int(port))
    if not client.ping():
        print(f"FAIL: no sim at {args.sim}")
        sys.exit(1)

    failures = []

    def check(cond, msg):
        print(f"  {'ok  ' if cond else 'FAIL'} {msg}")
        if not cond:
            failures.append(msg)

    enabled = rpc(client, "getEnvironment").get("enabled")
    print(f"Environment.Enabled = {enabled}")
    if args.expect_enabled and not enabled:
        print("FAIL: --expect-enabled, but the sim runs with Environment.Enabled false")
        sys.exit(1)

    reply = load(client, FIXTURES / "env_gate.csv")
    env = rpc(client, "getEnvironment")
    print(f"env_gate.csv -> {reply.get('environment')}; getEnvironment {env}")

    if not enabled:
        check(reply.get("environment") == "off", "loadTrack reports environment 'off'")
        check(env.get("status") == "off", "getEnvironment status 'off'")
        check("props" not in env and "sidecar" not in env, "nothing loaded while off")
    else:
        check(reply.get("environment") == "loaded", "loadTrack reports environment 'loaded'")
        check(env.get("status") == "loaded", "getEnvironment status 'loaded'")
        check(env.get("seed") == 7 and env.get("profile") == "smoke", "seed 7, profile 'smoke'")
        check(env.get("props") == 3 and env.get("classes") == {"bollard": 2, "tripod": 1},
              "3 props: 2 bollards, 1 tripod")
        check(env.get("ground_extent") == {"x_min": -60, "y_min": -60, "x_max": 75, "y_max": 60},
              "ground extent as authored")
        check(str(env.get("sidecar", "")).endswith("env_gate.env.json"), "sidecar path recorded")

        reply = load(client, FIXTURES / "ramp_gate.csv")
        env = rpc(client, "getEnvironment")
        print(f"ramp_gate.csv -> {reply.get('environment')}; getEnvironment {env}")
        check(reply.get("environment") == "none" and env.get("status") == "none",
              "a track without a sidecar reports 'none'")
        check("props" not in env, "the previous track's environment is gone")

        reply = load(client, FIXTURES / "env_bad.csv")
        env = rpc(client, "getEnvironment")
        print(f"env_bad.csv -> {reply.get('environment')}; getEnvironment {env}")
        check(reply.get("environment") == "invalid", "an invalid sidecar reports 'invalid'")
        check("props[0]: unknown key 'yaw'" in env.get("error", ""), "with the reason")
        check(reply.get("cones") == 10, "and the track itself still loads (10 cones)")

        reply = load(client, FIXTURES / "env_gate.csv")
        check(reply.get("environment") == "loaded", "a valid sidecar loads again afterwards")

    load(client, REPO / "Content" / "tracks" / "acceleration.csv")

    if failures:
        print("\nFAIL:\n  " + "\n  ".join(failures))
        sys.exit(1)
    print("\nPASS")


if __name__ == "__main__":
    main()
