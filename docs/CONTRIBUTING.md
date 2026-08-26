# Contributing

Everything below is for **contributors** — people writing PRs against
this repository. If you only want to run the sim, see
[SETUP.md](SETUP.md) instead.

---

## Directory layout

Where things live, and who works on them. The simulator is split along
one seam — the platform owns the world, the plant owns the vehicle —
and the directories follow that split rather than cutting across it.

```
IFSSIM/
├── Plugins/FSDSPlugin/     PLATFORM. All simulation logic.
│   └── Source/FSDSPlugin/
│       ├── Public|Private/Plant/     the IFSDSPlant seam + both impls
│       ├── Public|Private/FMI/       FMI 3.0 importer (zip, ABI, package)
│       ├── Public|Private/Sensors/   LiDAR, IMU, GPS, GSS
│       ├── Public|Private/RPC/       TCP RPC server, UDP push
│       └── Public|Private/Test/      ramp/crown/step terrain for probe tests
├── Source/Blocks/          minimal UE game module — entry point only
├── Content/                maps, cone meshes, track CSVs
│
├── matlab/plant/           PLANT. The Simulink vehicle model.
│   ├── build_*.m           one builder per subsystem — the SOURCE
│   ├── models/             generated .slx — regenerate, don't hand-edit
│   ├── fmu/                exported IFSSIM_Plant.fmu
│   └── test_*_physics.m    per-subsystem physics checks
│
├── pipeline/               AUTONOMY. Git submodule, its own repo:
│                           isc-fs/IFS08-DV-PIPELINE
├── ros2/src/ifssim_bridge/ the bridge between sim and ROS 2
│
├── tools/mission_control/  session orchestration (FastAPI + React)
├── tools/                  operator scripts, benchmarks, FMU utilities
├── docker/                 the dv_pipeline_stack image
└── docs/                   what you are reading
```

**The one rule that is easy to get wrong.** `matlab/plant/models/*.slx`
is *generated*. Edit `build_*.m` and regenerate; a hand-edit to the
`.slx` is overwritten by the next build and is invisible in review,
because a `.slx` is a binary blob in a diff.

**Where a change belongs.** If it is about terrain, sensors, cones or
the referee, it is the platform. If it is about how the car responds to
a command, it is the plant. If it decides what command to send, it is
the autonomy and lives in the submodule, not here.

---

## Branch layout

The repo has two permanent branches and many short-lived ones.

**`main`** is the production branch. Contains only validated code.
Never work directly on it.

**`dev`** is the development branch. The integration point where
everyone's work comes together. Never work directly on it either —
all changes arrive through a feature branch.

```
main  ──────────────────●──────────────────────●──▶  (validated releases only)
                        ↑                      ↑
dev   ──────●───●───●───●───●───●───●───●───●──●──▶  (continuous integration)
            ↑   ↑       ↑   ↑   ↑       ↑   ↑
          feat/1 fix/1 feat/2 fix/2   feat/3 fix/3
```

### Feature branches

All work — new feature or bug fix — happens on a branch created from
`dev`. When the work is ready, a PR opens against `dev`, gets
reviewed, merged, and the branch is deleted.

Two branch types with **independent numeric counters**, plus a short
kebab-case title:

```
feat/<n>-<short-title>   →  new functionality  (feat/1-frame-layout, …)
fix/<n>-<short-title>    →  bug fix            (fix/1-wrp-race,      …)
perf/<short-title>       →  performance        (no number; rare)
chore/<short-title>      →  release/infra      (no number; rare)
```

The short title is 2–4 lowercase words joined by dashes. `feat` and
`fix` counters are independent: `feat/2-…` and `fix/2-…` can exist
at the same time.

### Tracking branch history

Feature branches are deleted after merge to keep the repo clean.
History is preserved in **GitHub Issues**: every branch has an
associated issue with the label `feat` or `fix`, with the branch
number in the title.

- See active branches: filter issues by label and status `open`.
- Browse closed: filter by label and status `closed`.
- Next number for each type: last closed issue of that type + 1.

> Example: if the last closed issue with label `feat` is
> `[feat/491-…]`, the next feature branch is `feat/492-<your-title>`.

---

## Continuous integration

Every PR to `dev` or `main` runs through a 4-job CI suite
(`.github/workflows/ci.yml`) on free GitHub-hosted Linux runners.
Total wall time < 4 min. PRs **cannot merge with a red status**.

| Job | What it checks |
|---|---|
| **Bridge — colcon build** | `ifssim_bridge` + `fs_msgs` compile cleanly on ROS 2 Humble |
| **Mission Control — frontend** | ESLint clean, `tsc --noEmit` clean, `npm run build` succeeds |
| **Mission Control — backend** | `compileall` syntax check, `pytest` for `track_validators` + `scoring` (~70 cases) |
| **Tracks — CSV validator** | every `Content/tracks/*.csv` parses (field count, known cone types, numeric coords within ±500 m) |

### Running CI locally

```bash
# Bridge build (inside dv_pipeline_stack)
docker compose exec dv_pipeline_stack bash -lc \
  'cd /dv_pipeline_stack_ws && source /opt/ros/humble/setup.bash && \
   colcon build --packages-select ifssim_bridge'

# Frontend
cd tools/mission_control/frontend && npx eslint src && npx tsc --noEmit && npm run build

# Backend
cd tools/mission_control/backend && pytest tests/ -v

# Track CSV validator
python3 tools/validate_tracks.py

# Markdown internal-link checker
npx lychee --offline ./readme.md ./CHANGELOG.md ./docs/**/*.md
```

If all five pass locally, the CI gate will pass too.

### Pre-commit hooks (optional)

Run CI's checks on every `git commit` instead of waiting for the
push. Install once per machine:

```bash
pip install pre-commit
pre-commit install     # in the repo root — installs the git hook
```

Every commit then runs ruff (Python), ESLint (frontend), the track
CSV validator (when CSVs change), plus hygiene (trailing whitespace,
EOL, large-file guard). Skip in an emergency with `git commit -n`;
the push-time CI still catches it. After a rebase:
`pre-commit run --all-files`.

---

## Release pipelines (per-platform)

Three workflows produce shipping builds when a version tag is pushed.
All run on **self-hosted runners** because UE 5.7 must be installed
on the host.

| Workflow | Runner label | Requirements | Local-dev script |
|---|---|---|---|
| `package-mac.yml` | `[self-hosted, macOS]` | UE 5.7 + Mac codesigning identity | `./package_mac.sh` |
| `package-windows.yml` | `[self-hosted, Windows]` | UE 5.7 + Visual Studio 2022 | `bash package_windows.sh` |
| `package-linux.yml` | `[self-hosted, Linux]` | UE 5.7 (built from source) | `./package_linux.sh` |

Each runner needs `UE_ROOT` exported (e.g.
`/Users/Shared/Epic Games/UE_5.7` on Mac,
`C:\Program Files\Epic Games\UE_5.7` on Windows). Register at
**Settings → Actions → Runners → New self-hosted runner**.

**No self-hosted runners registered yet?** That's OK. The CI gate
above runs on free GitHub-hosted runners and protects every PR. The
release workflows just sit idle until a tag is pushed AND a runner
is online; without runners, tag pushes won't produce release
artefacts but won't break anything else.

---

## Automation

A GitHub Actions workflow manages tracking issues automatically.
No setup required — works for every developer as soon as a `feat/*`
or `fix/*` branch is pushed.

### Automatic issue creation

When a `feat/*` or `fix/*` branch is pushed to GitHub, the workflow
opens an issue with:

- Title mirroring the branch name — `[feat/N-short-title]` or
  `[fix/N-short-title]`
- The correct label (`feat` or `fix`)
- A template with sections for describing the work
- The developer who created the branch

### Wrong-number warning

If the branch number isn't the next expected one (too low or too
high), the issue shows a warning and asks the developer to delete
and recreate with the right name.

### Auto-fill description from first commit

When the developer makes their first commit and pushes, the workflow
updates the *"What does this branch do?"* section with that commit
message. Manual edits before the first push are preserved; the
description is only auto-filled once.

---

## Step-by-step workflow

### 1. Create the branch

```bash
git checkout dev
git pull origin dev

# Pick the next number from closed issues (feat or fix)
git checkout -b feat/493-add-foo    # or fix/N-..., etc.
```

To find the right number: **Issues → filter by label `feat` or
`fix` → sort by newest** and read the last number.

### 2. Push the branch

```bash
git push -u origin feat/493-add-foo
```

The tracking issue opens automatically within seconds.

### 3. Work and commit

```bash
git add .
git commit -m "short description of what this commit does"
git push
```

The message of your **first commit** auto-fills the issue
description.

### 4. Open a Pull Request

When the work is ready, open a PR on GitHub from your branch toward
`dev`. In the PR description write `Closes #<issue-number>` so the
issue closes automatically when the PR merges.

Before requesting review, check:

- The CI gate is green (Bridge / Frontend / Backend / Tracks)
- You have tested the change if applicable
- The PR targets `dev`, not `main`
- If user-visible, **`CHANGELOG.md` has a one-liner under
  `## [Unreleased]`**

### 5. Review and merge

Another team member reviews. Once approved, the PR merges into `dev`
(merge commit, not squash — matches the repo's history pattern) and
the branch is deleted. The issue closes as the permanent record.

### 6. Merging into main + cutting a release

When `dev` holds a set of validated changes ready to ship, a
responsible team member opens a PR from `dev` into `main`.

To cut a release:

1. Open a `chore/v<X.Y.Z>-release-prep` branch.
2. Move the `[Unreleased]` block in `CHANGELOG.md` under the new
   `## [X.Y.Z] — <YYYY-MM-DD>` heading. Re-tag anything that
   slipped out of the cycle under "Planned for v<next>".
3. Bump `ProjectVersion=X.Y.Z` in `Config/DefaultGame.ini`.
4. Open PR → merge.
5. `git tag -a vX.Y.Z -m "..."` → `git push origin vX.Y.Z`.
6. The per-platform release workflows pick up the tag and produce
   shipping zips (provided self-hosted runners are registered).
7. `gh release create vX.Y.Z dist/*.zip --notes "..."`.

See [#491](https://github.com/isc-fs/IFSSIM/pull/491) for the v0.1.1
example.

---

## Conventions

### Commit message style

Standard `type(scope): subject` pattern. The `gh pr merge` flow keeps
PR titles as the merge commit subject, so keep PR titles in the same
shape.

Common types: `feat`, `fix`, `perf`, `chore`, `docs`, `refactor`,
`test`.

```
feat(slam): add proximity-veto on new-landmark spawn (#441)
fix(plugin): UPROPERTY() the referee's AActor* containers — closes #471
perf(docker): bag named volume, no source bind mounts, .dockerignore
```

### CHANGELOG entries

User-visible changes get a one-liner under `## [Unreleased]` before
the PR merges. Write for someone who hasn't read your code —
describe the observable behaviour, not the implementation. See
`CHANGELOG.md` for the prevailing voice.

### Memory + docs

If your PR changes something that contradicts an existing doc, fix
the doc in the same PR. Stale documentation is worse than no
documentation — the next person hits the wrong instruction, debugs
for an hour, then mistrusts everything.

The full doc set:

- `readme.md` — front door
- `docs/SETUP.md` — first-time-user setup
- `docs/OPERATING.md` — daily ops
- `docs/AUTONOMY.md` — autonomy integration contract
- `docs/REFERENCE.md` — exhaustive technical reference
- `docs/CONTRIBUTING.md` — this file
- `CHANGELOG.md` — release-level human-readable delta

`docs/archive/` and `docs/history/` hold per-PR design / post-mortem
docs. They're append-only — once landed, don't touch them; they're a
record of how a decision was reached. New design work goes in
`docs/history/<YYYY-MM-DD>_<short-name>.md`.
