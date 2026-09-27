"""Load everything that exists on disk into bundles, aggregate, attach baselines."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from . import compare, provenance
from .adapters import sim_bag
from .adapters.replay import load_replay
from .adapters.sim import aggregate, group_runs, load_sim_run
from .bundle import RunBundle


@dataclass
class Suite:
    replays: list[RunBundle] = field(default_factory=list)
    sim_seeds: list[RunBundle] = field(default_factory=list)
    sim_aggs: list[RunBundle] = field(default_factory=list)
    sim_bags: list[RunBundle] = field(default_factory=list)
    baselines: dict[str, RunBundle] = field(default_factory=dict)

    @property
    def all(self) -> list[RunBundle]:
        return self.replays + self.sim_seeds + self.sim_aggs + self.sim_bags

    def seeds_by_group(self) -> dict[str, list[RunBundle]]:
        return group_runs(self.sim_seeds)

    def matrix_aggs(self) -> list[RunBundle]:
        return [a for a in self.sim_aggs if "matrix" in a.tags]

    def nightly_aggs(self) -> list[RunBundle]:
        return [a for a in self.sim_aggs if "nightly" in a.tags]

    def sweep_trials(self) -> list[RunBundle]:
        return [a for a in self.sim_aggs if a.job_type == "sim_sweep_trial"]


def load_suite(
    *,
    replay_root: Path | None,
    sim_root: Path | None,
    sim_bag_root: Path | None = None,
    full_bag_hash: bool = True,
) -> Suite:
    s = Suite()
    if replay_root is not None and (replay_root / "onboard").is_dir():
        onboard = replay_root / "onboard"
        for d in sorted(p for p in onboard.iterdir() if p.is_dir()):
            if provenance.in_progress(d):
                print(f"  skipping {d.name}: still running")
                continue
            s.replays.append(load_replay(d, replay_root, full_bag_hash=full_bag_hash))
    if sim_root is not None and sim_root.is_dir():
        s.sim_seeds = [
            load_sim_run(m.parent) for m in sorted(sim_root.rglob("manifest.json"))
        ]
        for group, seeds in group_runs(s.sim_seeds).items():
            jt = "sim_sweep_trial" if seeds[0].config.get("sweep") else "sim_aggregate"
            s.sim_aggs.append(aggregate(seeds, job_type=jt))
    if sim_bag_root is not None and sim_bag_root.is_dir():
        s.sim_bags = sim_bag.load_all(sim_bag_root, full_bag_hash=full_bag_hash)
    s.baselines = compare.choose_baselines(s.all)
    compare.apply(s.all, s.baselines)
    return s
