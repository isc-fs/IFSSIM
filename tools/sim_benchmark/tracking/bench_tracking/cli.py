"""bench-track: import runs into a tracker and build its dashboards.

    bench-track mock-sim  --out ../results/sim_mock
    bench-track import    --backend mlflow|clearml|wandb --replay-root <results> --sim-root ../results/sim_mock
    bench-track dashboards --backend mlflow|clearml|wandb --replay-root ... --sim-root ...
    bench-track attach-bundles --replay-root ... --sim-root ...   (MLflow: add bundle/ to already-imported runs)
    bench-track summary   --replay-root ... --sim-root ...        (no tracker; prints what would be logged)

Nothing here runs a benchmark. Replays are read from existing run dirs,
simulator runs from the mock generator's output.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import warnings
from pathlib import Path

from . import figures as F
from .backends import base as backend_base
from .suite import Suite, load_suite

HERE = Path(__file__).resolve().parent.parent
REPO = HERE.parent.parent.parent
STATE_DIR = HERE / ".state"
DEFAULT_TRACKS = REPO / "Content" / "tracks"


def get_backend(name: str):
    if name == "mlflow":
        from .backends.mlflow_backend import MlflowBackend

        return MlflowBackend()
    if name == "clearml":
        from .backends.clearml_backend import ClearMLBackend

        return ClearMLBackend()
    if name == "wandb":
        from .backends.wandb_backend import WandbBackend

        return WandbBackend(tracks_dir=DEFAULT_TRACKS)
    raise SystemExit(f"unknown backend {name}")


def state_path(backend: str) -> Path:
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
        full_bag_hash=not args.fast_bag_id,
    )


def cmd_import(args) -> None:
    suite = _suite(args)
    be = get_backend(args.backend)
    state = {} if args.fresh else load_state(args.backend)
    only = set(args.only or [])

    def up(b, parent=None):
        if b.run_key in state and not args.force:
            return backend_base.UploadResult(
                args.backend,
                state[b.run_key]["run_id"],
                state[b.run_key].get("url"),
                b.name,
            )
        t0 = time.time()
        figs = {} if args.no_figures else F.per_run(b)
        res = be.upload(b, figures=figs, parent=parent)
        state[b.run_key] = {
            "run_id": res.run_id,
            "url": res.url,
            "name": b.name,
            "job_type": b.job_type,
            "group": b.group,
            "scenario_id": b.scenario_id,
            "tags": b.tags,
        }
        save_state(args.backend, state)
        print(
            f"  [{args.backend}] {b.name}  ({time.time() - t0:.1f}s, {len(figs)} figs)",
            flush=True,
        )
        return res

    if not only or "replay" in only:
        print(f"== replays: {len(suite.replays)}")
        for b in sorted(suite.replays, key=lambda b: b.started_at):
            up(b)
    if not only or "sim" in only:
        seeds = suite.seeds_by_group()
        print(f"== sim: {len(suite.sim_aggs)} aggregates, {len(suite.sim_seeds)} seeds")
        for a in sorted(suite.sim_aggs, key=lambda a: a.started_at):
            pr = up(a)
            for s in seeds.get(a.group, []):
                up(s, parent=pr)
    be.finish()
    print(f"state: {state_path(args.backend)}")


def cmd_purge(args) -> None:
    """Delete uploaded runs (by job family) from a tracker and forget them in the state file."""
    state = load_state(args.backend)
    fams = {
        "replay": ("onboard_replay", "live_replay"),
        "sim": ("sim_e2e", "sim_aggregate", "sim_sweep_trial"),
    }
    jobs = {j for f in (args.only or ["replay", "sim"]) for j in fams[f]}
    doomed = {k: v for k, v in state.items() if v.get("job_type") in jobs}
    if args.backend == "mlflow":
        from mlflow.tracking import MlflowClient
        from .backends.mlflow_backend import MlflowBackend

        c = MlflowClient(MlflowBackend().uri)
        delete = c.delete_run
    elif args.backend == "clearml":
        from .backends.clearml_backend import configure_from_env_file

        configure_from_env_file()
        from clearml import Task

        delete = lambda rid: Task.get_task(task_id=rid).delete(raise_on_error=False)  # noqa: E731
    else:
        import wandb
        from .backends.wandb_backend import PROJECT

        api = wandb.Api()
        ent = api.default_entity
        delete = lambda rid: api.run(f"{ent}/{PROJECT}/{rid}").delete()  # noqa: E731
    for k, v in doomed.items():
        try:
            delete(v["run_id"])
        except Exception as e:  # noqa: BLE001
            print(f"  could not delete {v['name']}: {e}")
        state.pop(k)
        print(f"  deleted {v['name']}")
    save_state(args.backend, state)


def cmd_dashboards(args) -> None:
    suite = _suite(args)
    state = load_state(args.backend)
    if not state:
        raise SystemExit(f"no uploads recorded for {args.backend}; run `import` first")
    if args.backend == "mlflow":
        from .dashboards.mlflow_dash import build
    elif args.backend == "clearml":
        from .dashboards.clearml_dash import build
    else:
        from .dashboards.wandb_dash import build
    for line in build(suite, state):
        print(line)


def cmd_attach_bundles(args) -> None:
    """Add the bundle/ artifact to runs imported before it existed (MLflow only)."""
    from .backends.mlflow_backend import MlflowBackend, attach_bundle

    suite = _suite(args)
    state = load_state("mlflow")
    be = MlflowBackend()
    done = 0
    for b in suite.all:
        entry = state.get(b.run_key)
        if entry is None:
            continue
        if (
            not args.force
            and be.client.get_run(entry["run_id"]).data.tags.get("bench.bundle") == "1"
        ):
            continue
        attach_bundle(be.client, entry["run_id"], b)
        done += 1
        print(f"  bundle -> {b.name}", flush=True)
    print(f"attached {done} bundles")


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
            "--fast-bag-id",
            action="store_true",
            help="bag id from metadata+sizes, skip full hash",
        )

    p = sub.add_parser("import", help="upload runs to a tracker")
    p.add_argument("--backend", required=True, choices=["mlflow", "clearml", "wandb"])
    data_args(p)
    p.add_argument("--only", nargs="*", choices=["replay", "sim"])
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

    p = sub.add_parser("purge", help="delete uploaded runs of a family from a tracker")
    p.add_argument("--backend", required=True, choices=["mlflow", "clearml", "wandb"])
    p.add_argument("--only", nargs="*", choices=["replay", "sim"])
    p.set_defaults(fn=cmd_purge)

    p = sub.add_parser(
        "dashboards", help="build the comparison dashboards for a tracker"
    )
    p.add_argument("--backend", required=True, choices=["mlflow", "clearml", "wandb"])
    data_args(p)
    p.set_defaults(fn=cmd_dashboards)

    p = sub.add_parser(
        "attach-bundles",
        help="add the full-resolution bundle/ artifact to imported MLflow runs",
    )
    data_args(p)
    p.add_argument(
        "--force", action="store_true", help="re-attach even if the run already has one"
    )
    p.set_defaults(fn=cmd_attach_bundles)

    p = sub.add_parser("mock-sim", help="generate MOCK simulator benchmark run dirs")
    p.add_argument("--out", default=str(HERE.parent / "results" / "sim_mock"))
    p.add_argument("--tracks", default=str(DEFAULT_TRACKS))
    p.add_argument("--seeds", type=int, default=5)
    p.set_defaults(fn=cmd_mock)

    p = sub.add_parser("summary", help="print what would be logged")
    data_args(p)
    p.set_defaults(fn=cmd_summary)

    args = ap.parse_args(argv)
    if getattr(args, "write_records", False):
        backend_base.WRITE_RECORDS = True
    args.fn(args)


if __name__ == "__main__":
    main(sys.argv[1:])
