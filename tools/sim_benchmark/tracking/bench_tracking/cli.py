"""bench-track: put benchmark runs into MLflow.

    bench-track sync      [--results-root <results>]   (upload every finished run MLflow does not have yet)
    bench-track import    --replay-root <results> --sim-root ../results/sim_mock
    bench-track import    --sim-bag-root <results> --only sim_bag   (simulator bag sessions)
    bench-track admin     init | add-user <name> | set-password <name> | remove-user <name>
                          (central server with logins, as its admin; see DEPLOY.md)
    bench-track purge     [--only replay|sim|sim_bag]   (delete runs this machine uploaded)
    bench-track mock-sim  --out ../results/sim_mock
    bench-track summary   --replay-root ... --sim-root ...   (no upload; prints what would be logged)

Nothing here runs a benchmark. Replays and simulator bag sessions are read
from existing run dirs, simulator runs from the mock generator's output.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import warnings
from pathlib import Path

from . import config
from . import figures as F
from .backends import base as backend_base
from .suite import Suite, load_suite

HERE = Path(__file__).resolve().parent.parent
REPO = HERE.parent.parent.parent
STATE_DIR = HERE / ".state"
DEFAULT_TRACKS = REPO / "Content" / "tracks"
BACKEND = "mlflow"


def get_backend(name: str = BACKEND):
    if name != BACKEND:
        raise SystemExit(f"unknown backend {name}")
    from .backends.mlflow_backend import MlflowBackend

    return MlflowBackend()


def state_path(backend: str) -> Path:
    """One state file per tracker, and for MLflow per server (a local one and the team's)."""
    if backend == "mlflow":
        import os
        import re

        uri = os.environ.get("MLFLOW_TRACKING_URI", "http://127.0.0.1:5005")
        if (
            uri.rstrip("/") != "http://127.0.0.1:5005"
        ):  # the local default keeps mlflow.json
            return (
                STATE_DIR
                / f"mlflow@{re.sub(r'[^A-Za-z0-9.-]+', '_', uri.split('://')[-1]).strip('_')}.json"
            )
    return STATE_DIR / f"{backend}.json"


def load_state(backend: str) -> dict[str, dict]:
    p = state_path(backend)
    return json.loads(p.read_text()) if p.is_file() else {}


def save_state(backend: str, state: dict[str, dict]) -> None:
    STATE_DIR.mkdir(exist_ok=True)
    state_path(backend).write_text(json.dumps(state, indent=1, sort_keys=True))


def _suite(args) -> Suite:
    warnings.simplefilter("ignore", RuntimeWarning)
    return load_suite(
        replay_root=Path(args.replay_root) if args.replay_root else None,
        sim_root=Path(args.sim_root) if args.sim_root else None,
        sim_bag_root=Path(args.sim_bag_root) if args.sim_bag_root else None,
        full_bag_hash=not args.fast_bag_id,
    )


class Uploader:
    """Uploads runs the tracker does not have yet.

    MLflow is asked first (the ``bench.run_key`` tag), so a run is uploaded once
    however many machines, or copies of the results, see it. The state file is a
    local record of what was seen; ``uploaded_here`` marks what this machine
    uploaded, which is all ``purge`` may delete.
    """

    def __init__(self, backend: str, *, fresh=False, force=False, figures=True):
        self.name = backend
        self.be = get_backend(backend)
        self.state = {} if fresh else load_state(backend)
        self.force, self.figures = force, figures
        self.counts = {"uploaded": 0, "already there": 0, "failed": 0}

    def _known(self, b, *, claim: bool = True) -> backend_base.UploadResult | None:
        old = self.state.get(b.run_key) or self.state.get(b.legacy_run_key)
        if not self.be.can_find:  # trackers that cannot be asked: trust the state file
            return (
                backend_base.UploadResult(
                    self.name, old["run_id"], old.get("url"), b.name
                )
                if old
                else None
            )
        hit = self.be.find(b.run_key)
        if hit is None and old and not claim:
            return backend_base.UploadResult(
                self.name, old["run_id"], old.get("url"), b.name
            )
        if hit is None and old and self.be.claim(old["run_id"], b.run_key):
            hit = backend_base.UploadResult(
                self.name, old["run_id"], old.get("url"), b.name
            )
        return hit

    def _remember(self, b, res, here: bool) -> None:
        prev = self.state.pop(b.legacy_run_key, None) or self.state.get(b.run_key)
        # entries from before the flag existed were all uploaded from here
        was_mine = bool(prev) and prev.get("uploaded_here", True)
        self.state[b.run_key] = {
            "run_id": res.run_id,
            "url": res.url,
            "name": b.name,
            "job_type": b.job_type,
            "group": b.group,
            "scenario_id": b.scenario_id,
            "tags": b.tags,
            "uploaded_here": here or was_mine,
        }
        save_state(self.name, self.state)

    def up(self, b, parent=None) -> backend_base.UploadResult | None:
        if not self.force and (hit := self._known(b)) is not None:
            self._remember(b, hit, here=False)
            self.counts["already there"] += 1
            return hit
        t0 = time.time()
        try:
            figs = F.per_run(b) if self.figures else {}
            res = self.be.upload(b, figures=figs, parent=parent)
        except OSError:  # the server went away: stop, the next sync retries
            raise
        except Exception as e:  # noqa: BLE001  (one bad run must not stop the rest)
            self.counts["failed"] += 1
            print(f"  FAILED {b.name}: {type(e).__name__}: {e}", flush=True)
            return None
        self._remember(b, res, here=True)
        self.counts["uploaded"] += 1
        print(
            f"  [{self.name}] {b.name}  ({time.time() - t0:.1f}s, {len(figs)} figs)",
            flush=True,
        )
        return res


def cmd_import(args) -> None:
    suite = _suite(args)
    u = Uploader(
        BACKEND, fresh=args.fresh, force=args.force, figures=not args.no_figures
    )
    be, up = u.be, u.up
    only = set(args.only or [])

    if not only or "replay" in only:
        print(f"== replays: {len(suite.replays)}")
        for b in sorted(suite.replays, key=lambda b: b.started_at):
            up(b)
    if not only or "sim" in only:
        seeds = suite.seeds_by_group()
        print(f"== sim: {len(suite.sim_aggs)} aggregates, {len(suite.sim_seeds)} seeds")
        for a in sorted(suite.sim_aggs, key=lambda a: a.started_at):
            pr = up(a)
            for s in seeds.get(a.group, []) if pr else []:
                up(s, parent=pr)
    if not only or "sim_bag" in only:
        print(f"== simulator bag sessions: {len(suite.sim_bags)}")
        for b in sorted(suite.sim_bags, key=lambda b: b.started_at):
            up(b)
    be.finish()
    print(", ".join(f"{v} {k}" for k, v in u.counts.items()))
    print(f"state: {state_path(BACKEND)}")
    if u.counts["failed"]:
        raise SystemExit(1)


def cmd_sync(args) -> None:
    """Upload every finished run under the results dir that MLflow does not have yet.

    Safe to run any time, as often as you like: runs already on the server are
    skipped, runs still going are left for next time, and nothing is lost when the
    server cannot be reached (the next sync picks it up).
    """
    root = Path(args.results_root).resolve()
    if not root.is_dir():
        raise SystemExit(f"no results dir at {root}")
    from .backends.mlflow_backend import MlflowBackend

    uri = MlflowBackend.default_uri()
    print(f"sync {root} -> {uri}")
    if not reachable(uri):
        # checked up front: MLflow's own retries take minutes to give up
        raise SystemExit(
            f"MLflow at {uri} cannot be reached. Nothing is lost: "
            "the next sync uploads whatever is missing."
        )
    try:
        warnings.simplefilter("ignore", RuntimeWarning)
        suite = load_suite(
            replay_root=root,
            sim_root=None,
            sim_bag_root=root,
            full_bag_hash=not args.fast_bag_id,
        )
        u = Uploader(BACKEND, figures=not args.no_figures)
        for b in sorted(suite.replays + suite.sim_bags, key=lambda b: b.started_at):
            if args.dry_run:
                hit = u._known(b, claim=False)
                print(f"  {'on server ' if hit else 'to upload '} {b.name}")
                continue
            u.up(b)
    except Exception as e:  # noqa: BLE001
        if _unreachable(e):
            raise SystemExit(
                f"MLflow at {uri} cannot be reached ({type(e).__name__}). Nothing is lost: "
                "the next sync uploads whatever is missing."
            ) from None
        raise
    if not args.dry_run:
        print(", ".join(f"{v} {k}" for k, v in u.counts.items()))
        if u.counts["failed"]:
            raise SystemExit(1)


def reachable(uri: str, timeout: float = 5.0) -> bool:
    """The server answers its health check (no login needed for that)."""
    import urllib.request

    if not uri.startswith(("http://", "https://")):
        return True  # a local file store
    try:
        with urllib.request.urlopen(f"{uri.rstrip('/')}/health", timeout=timeout):
            return True
    except Exception:  # noqa: BLE001
        return False


def _unreachable(e: BaseException) -> bool:
    """A connection problem (retry later), as opposed to a bug."""
    text = f"{type(e).__name__} {e}"
    return isinstance(e, (ConnectionError, OSError)) or any(
        s in text
        for s in (
            "ConnectionError",
            "Max retries exceeded",
            "Connection refused",
            "Failed to resolve",
            "timed out",
            "API request to endpoint",
        )
    )


def cmd_purge(args) -> None:
    """Delete uploaded runs (by job family) from MLflow and forget them in the state file."""
    from mlflow.tracking import MlflowClient

    from .backends.mlflow_backend import MlflowBackend

    state = load_state(BACKEND)
    fams = {
        "replay": ("onboard_replay", "live_replay"),
        "sim": ("sim_e2e", "sim_aggregate", "sim_sweep_trial"),
        "sim_bag": ("sim_bag",),
    }
    jobs = {j for f in (args.only or list(fams)) for j in fams[f]}
    # only runs this machine uploaded: the state file also lists runs found on the server
    doomed = {
        k: v
        for k, v in state.items()
        if v.get("job_type") in jobs and v.get("uploaded_here", True)
    }
    c = MlflowClient(MlflowBackend().uri)
    for k, v in doomed.items():
        try:
            c.delete_run(v["run_id"])
        except Exception as e:  # noqa: BLE001
            print(f"  could not delete {v['name']}: {e}")
        state.pop(k)
        print(f"  deleted {v['name']}")
    save_state(BACKEND, state)


def cmd_admin(args) -> None:
    """Set up and look after the central server (MLflow with --app-name basic-auth).

    Runs as the MLflow admin: MLFLOW_TRACKING_USERNAME / MLFLOW_TRACKING_PASSWORD.
    """
    import os
    import secrets

    import requests

    from .backends.mlflow_backend import EXPERIMENTS, MlflowBackend

    be = MlflowBackend()

    def users(method: str, action: str, **body) -> None:
        # MLflow's own auth client needs Flask-WTF; its REST API does not
        r = requests.request(
            method,
            f"{be.uri.rstrip('/')}/api/2.0/mlflow/users/{action}",
            json=body,
            auth=(
                os.environ.get("MLFLOW_TRACKING_USERNAME", ""),
                os.environ.get("MLFLOW_TRACKING_PASSWORD", ""),
            ),
            timeout=30,
        )
        if not r.ok:
            raise SystemExit(
                f"{action} {body.get('username')}: {r.status_code} {r.text[:300]}"
            )

    if args.action == "init":
        # the admin creates every experiment, so nobody else gets MANAGE (delete) on one
        for name in EXPERIMENTS:
            print(f"  experiment {name}: {be._experiment(name)}")
        print(
            "Users get the server's default permission (EDIT): upload runs and pin "
            "baselines, but not delete. Deleting is for the admin."
        )
        return
    if args.action == "remove-user":
        users("DELETE", "delete", username=args.name)
        print(f"removed {args.name}")
        return
    password = args.password or secrets.token_urlsafe(18)
    if args.action == "add-user":
        users("POST", "create", username=args.name, password=password)
    else:
        users("PATCH", "update-password", username=args.name, password=password)
    if not args.password:
        print(f"{args.name}: {password}")
        print("Shown once. Give it to them for ~/.config/ifssim-bench/tracking.env.")


def cmd_mock(args) -> None:
    from .mock_sim import generate_suite

    out = Path(args.out)
    runs = generate_suite(out, Path(args.tracks), seeds=args.seeds)
    print(f"wrote {len(runs)} mock sim runs under {out}")


def cmd_summary(args) -> None:
    suite = _suite(args)
    for b in suite.all:
        print(
            f"{b.job_type:16} {b.status:9} {b.name:55} series={len(b.series):2} tables={len(b.tables):2} "
            f"events={len(b.events):4} metrics={len(b.summary):3}"
        )
    print({k: v.name for k, v in suite.baselines.items()})


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(
        prog="bench-track",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    def data_args(p):
        p.add_argument(
            "--replay-root",
            help="tools/sim_benchmark/results dir holding onboard/ and capture/",
        )
        p.add_argument("--sim-root", help="mock (or real) sim results root")
        p.add_argument(
            "--sim-bag-root",
            help="simulator bag sessions (results/sim_bag/*), and perception/slam runs "
            "made one at a time under it (tools/sim_benchmark/results)",
        )
        p.add_argument(
            "--fast-bag-id",
            action="store_true",
            help="bag id from metadata+sizes, skip full hash",
        )

    p = sub.add_parser("import", help="upload runs from given result dirs to MLflow")
    data_args(p)
    p.add_argument("--only", nargs="*", choices=["replay", "sim", "sim_bag"])
    p.add_argument(
        "--force", action="store_true", help="re-upload runs already in the state file"
    )
    p.add_argument("--fresh", action="store_true", help="ignore the state file")
    p.add_argument("--no-figures", action="store_true")
    p.add_argument(
        "--write-records",
        action="store_true",
        help="write tracking.json into each run dir",
    )
    p.set_defaults(fn=cmd_import)

    p = sub.add_parser(
        "sync",
        help="upload every finished run under a results dir that MLflow does not have yet",
    )
    p.add_argument(
        "--results-root",
        default=str(HERE.parent / "results"),
        help="the benchmark results dir (default: tools/sim_benchmark/results)",
    )
    p.add_argument(
        "--fast-bag-id",
        action="store_true",
        help="bag id from metadata+sizes, skip full hash",
    )
    p.add_argument("--no-figures", action="store_true")
    p.add_argument("--dry-run", action="store_true", help="list what would be uploaded")
    p.set_defaults(fn=cmd_sync)

    p = sub.add_parser(
        "admin", help="central server: create experiments, add users (as its admin)"
    )
    p.add_argument(
        "action", choices=["init", "add-user", "set-password", "remove-user"]
    )
    p.add_argument("name", nargs="?", help="user name (all but init)")
    p.add_argument(
        "--password",
        help="default: a random one, printed once (add-user, set-password)",
    )
    p.set_defaults(fn=cmd_admin)

    p = sub.add_parser(
        "purge",
        help="delete runs this machine uploaded, of a family, from MLflow",
    )
    p.add_argument("--only", nargs="*", choices=["replay", "sim", "sim_bag"])
    p.set_defaults(fn=cmd_purge)

    p = sub.add_parser("mock-sim", help="generate MOCK simulator benchmark run dirs")
    p.add_argument("--out", default=str(HERE.parent / "results" / "sim_mock"))
    p.add_argument("--tracks", default=str(DEFAULT_TRACKS))
    p.add_argument("--seeds", type=int, default=5)
    p.set_defaults(fn=cmd_mock)

    p = sub.add_parser("summary", help="print what would be logged")
    data_args(p)
    p.set_defaults(fn=cmd_summary)

    args = ap.parse_args(argv)
    if args.cmd == "admin" and args.action != "init" and not args.name:
        ap.error(f"admin {args.action} needs a user name")
    config.load()
    if getattr(args, "write_records", False):
        backend_base.WRITE_RECORDS = True
    args.fn(args)


if __name__ == "__main__":
    main(sys.argv[1:])
