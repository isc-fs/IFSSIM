# Branches

Two long-lived lines with different jobs and different risk tolerances.

| branch | for | who merges here |
|---|---|---|
| **`dev`** | the driverless car: UE5 sim, ROS 2 pipeline, Mission Control. **Releases are cut from here.** | anything that must work |
| **`dev-manual`** | the MATLAB/Simulink FMU plant, and the kinematic and dynamic vehicle models | plant and vehicle-dynamics investigation |
| `main` | a stub. Its history is unrelated to `dev` and nothing is merged into it. | nothing |

`dev` is the safe one. If a change could affect a car that drives itself, it
goes through `dev` and through CI. `dev-manual` is where the plant is still
being figured out, and it moves faster because the blast radius is a MATLAB
model, not a vehicle.

They are **not** in a parent/child relationship and `dev-manual` is not a
feature branch. It carries its own line of work and will be merged back
deliberately, in one piece, when the plant is proven — not continuously.

## Protection, as actually configured

| rule | `dev` | `dev-manual` | `main` |
|---|---|---|---|
| pull request required | yes, **1 approval** | no — direct commits | yes, 1 approval |
| stale approvals dismissed on new commits | yes | — | yes |
| required status check | `CI summary` | none | none |
| branch must be up to date before merge | yes | — | no |
| conversation resolution required | yes | no | no |
| linear history | yes | no | no |
| force-push | **blocked** | **blocked** | **blocked** |
| deletion | **blocked** | **blocked** | **blocked** |
| rules apply to admins | yes | no | yes |

Two of those deserve their reasons written down.

**`dev-manual` allows direct commits on purpose.** The plant work lands as a
long series of small commits against a model that is being measured, rebuilt
and re-measured; routing each through a PR would cost more than it catches.
What it does *not* allow is losing that history — force-push and deletion are
blocked, which is the protection that actually matters for a branch nobody
reviews.

**`dev-manual` has no required status check on purpose.** A required check
blocks the push that would produce it, so requiring one on a branch that takes
direct commits deadlocks it. CI still *runs* on `dev-manual` (it is in the
workflow's trigger list) — it reports, it just does not gate.

## The required check has to be able to fail

`CI summary` aggregates the five CI jobs. It is required on `dev` instead of
requiring those five individually, because job display names change and every
rename would otherwise have to be mirrored in the rules of three branches.

That only works because the summary job **fails when any dependency fails**.
It did not always: it posted a comment, carried `if: always()`, and reported
success even when everything under it had failed. A required check that cannot
fail is worse than no rule at all — the branch looks gated and is not.

## Releases

Cut from `dev`. `main` is not involved.

Immutable releases are enabled, so a published release's assets cannot be
changed afterwards. Build every platform's artifact **before** publishing, or
the ones that come late need their own release.

Packaging workflows:

- `.github/workflows/package-windows.yml` — GitHub-hosted is not enough; needs
  a self-hosted Windows runner with UE5.
- `.github/workflows/package-mac.yml` — needs a self-hosted **macOS** runner
  with UE5. There isn't one registered, so this workflow cancels on tag pushes
  and the macOS artifact has to be built by hand until a runner exists.
- `.github/workflows/package-linux.yml`

## Working on the plant

Plant, FMU and vehicle-dynamics work lives in `matlab/` on `dev-manual` and
does not touch `pipeline/`. See `docs/vdb_plant_migration.md` for where that
work has got to.
