# Running the benchmark tracker for the team

One central machine keeps every benchmark result. Anyone can run benchmarks on
their own machine or on the central one, and every run ends up on the central
server. It shows the same way everywhere: in the hosted viewer, in a viewer
running on your laptop, and in MLflow's own UI.

```
 your machine                                     central machine (Docker, 127.0.0.1 only)
 ─────────────                                    ─────────────────────────────────────────
 benchmark ──► results/<run>/ ──► bench-track sync ──►  MLflow (logins) ──► Postgres
                 provenance.json     (automatic)            │                + artifacts on disk
 bench-view (optional) ◄───────────────────────────────────┤
                                                           bench-view (gunicorn)
            ◄──────── Tailscale (tailscale serve, HTTPS) ─────────┘
```

Only the results go to the server, never the bags.

## What gets recorded, and why a run can be reproduced

Every run folder gets a `provenance.json` when the run starts, written by
`tools/sim_benchmark/run_provenance.py`. It holds:

- **The IFSSIM and pipeline commits and branches.**
- **Uncommitted changes** of each repo, as `<repo>.diff`. This includes new
  files up to 512 KB.
- **Commits that no remote has** (as far as the last `git fetch` knows), as
  `<repo>.unpushed.bundle`. The run prints a warning so you push them.
- **The Docker image.** A pulled image has a registry digest, which is the same
  on every machine. A locally built one can't be matched across machines, so
  the run prints a warning.
- **The machine and user**, the command, and a capture id that names this run
  on every machine.

The **code label** (`a64350a`, or `a64350a-dirty.3f2c1a9e`) and the **code id**
come from all of that. Runs with the same code id ran the same code. The viewer
shows reruns of one code together.

To rebuild a run's code on another machine:

```bash
cd pipeline
git fetch <run>/pipeline.unpushed.bundle HEAD     # only if the run has one
git checkout <pipeline sha from the run>
git apply <run>/pipeline.diff                      # only if the run has one
```

Do the same for IFSSIM at the repo root.

To run exactly the same stack as the central machine, pin the image. Use
`IFSSIM_DV_IMAGE=ghcr.io/isc-fs/ifssim-dv_pipeline_stack:sha-<commit>`. The
`sha-` tags are immutable, and are the reason the registry digest can be
compared across machines.

## 1. The central machine

It needs Docker (with compose) and Tailscale. Everything is in
`deploy/central/`.

```bash
cd tools/sim_benchmark/tracking/deploy/central
cp .env.example .env     # fill it in: hostname, three passwords and a secret key (openssl rand -hex 24)
./setup.sh               # builds, starts, creates the experiments and the viewer's login
```

It starts three containers, all on `127.0.0.1` only:

| service | what | port |
|---|---|---|
| `postgres` | runs, metrics, params, and the logins (a second database) | internal |
| `mlflow` | MLflow with logins (`--app-name basic-auth`); artifacts in `DATA_DIR/artifacts` | `MLFLOW_PORT` (5005) |
| `viewer` | bench-view under gunicorn, reading MLflow as the `viewer` login | `VIEWER_PORT` (8050) |

### Tailscale

Serve both ports to the tailnet over HTTPS:

```bash
sudo tailscale serve --bg --https=443  http://127.0.0.1:8050   # bench-view: https://<host>.<tailnet>.ts.net
sudo tailscale serve --bg --https=8443 http://127.0.0.1:5005   # MLflow:     https://<host>.<tailnet>.ts.net:8443
tailscale serve status
```

`BENCH_HOSTNAME` in `.env` must be that `<host>.<tailnet>.ts.net` name.
MLflow rejects requests addressed to any other name (DNS-rebinding
protection); run `./setup.sh` again after you change it. Nothing listens on
the LAN or the internet. Who can reach the machine is up to the tailnet's
access rules.

### Logins

Give each person their own login:

```bash
./admin.sh add-user alice         # prints a random password, once
./admin.sh set-password alice     # new random password
./admin.sh remove-user alice
```

Every login gets `EDIT`: it can upload runs and pin baselines, but not delete
anything. Only `admin` can delete. `setup.sh` creates the experiments as the
admin, so no one else ends up managing one. The hosted viewer has no login of
its own: anyone who can reach it through Tailscale can look at runs and pin
baselines.

### Backups

`backup.sh` dumps both databases (`pg_dump`, keeps the last 14) and mirrors
the artifacts with `rsync`. Run it every night, to a disk that isn't
`DATA_DIR`:

```cron
30 3 * * *  BACKUP_DIR=/mnt/backup/ifssim-bench /path/to/deploy/central/backup.sh >> /var/log/ifssim-bench-backup.log 2>&1
```

To restore, start `postgres` alone and load the dumps:

```bash
docker compose up -d postgres
docker compose exec -T postgres pg_restore -U mlflow -d mlflow --clean < mlflow_<ts>.dump
docker compose exec -T postgres pg_restore -U mlflow -d mlflow_auth --clean < mlflow_auth_<ts>.dump
```

Then copy `artifacts/` back into `DATA_DIR/artifacts` and run `docker compose up -d`.

### Updating

- **bench-view code:** `docker compose up -d --build viewer`.
- **MLflow:** change the version in `tracking/uv.lock` (`uv lock --upgrade-package mlflow`)
  and `MLFLOW_VERSION` in `.env` together, back up first, then
  `docker compose up -d --build mlflow`. MLflow upgrades its database schema on start.

### Moving existing results in

Results that are already on disk go up with a sync from the machine that has
them (section 2). That works the same for a laptop's old runs and for a
previous local MLflow: re-importing from the results folders is simpler than
copying MLflow databases.

## 2. Each person's machine (and the central one, for its own benchmarks)

Once, ask the admin for a login and write
`~/.config/ifssim-bench/tracking.env`:

```ini
MLFLOW_TRACKING_URI=https://<host>.<tailnet>.ts.net:8443
MLFLOW_TRACKING_USERNAME=alice
MLFLOW_TRACKING_PASSWORD=<from the admin>
# IFSSIM_AUTO_UPLOAD=0      # to keep runs local by default
```

It's read by the benchmarks, `bench-track` and `bench-view`. Variables set in
the environment win over the file. Keep the file out of the repo: the repo is
public.

Check it with:

```bash
cd tools/sim_benchmark/tracking
uv run bench-track sync --dry-run     # lists what is on the server and what would go up
```

From then on **every benchmark uploads when it finishes**. `common.py` runs
`bench-track sync` on the host after the container exits, and at exit for
`--no-docker` runs. The sim-bag runner uploads its whole session at the end,
never half of it. Sync uploads every finished run that the server doesn't have
yet, not just the last one:

- **Offline or server down:** nothing is lost; the next benchmark, or a manual
  `uv run bench-track sync`, uploads it.
- **Runs still going** (started, no results yet, changed in the last 6 hours):
  left for next time.
- **Duplicates:** each run has a key, stored on the server as the
  `bench.run_key` tag. It is built from the capture id or the run folder's
  name, never from an absolute path. A run seen from two machines, or from two
  copies of the results, goes up once.
- **No uv on the machine:** the run says so and prints the command to use later.

The central machine itself works the same way. Give it its own login (e.g.
`central`) and point it at `https://…:8443` or `http://127.0.0.1:5005`.

### The viewer on your own machine

```bash
cd tools/sim_benchmark/tracking
uv run --extra viewer bench-view            # the team server, from tracking.env
uv run --extra viewer bench-view --uri http://127.0.0.1:5005   # a local MLflow instead
```

It is the same code as the hosted one. Running it locally is useful while
changing the viewer, or to look at a private local MLflow
(`deploy/mlflow/run_server.sh`).

## 3. Looking after the data

- **Delete a run:** the admin deletes it in MLflow's UI (or with `purge` on the
  machine that uploaded it: `purge` only touches runs that the machine
  uploaded itself). A deleted run goes up again at the owner's next sync
  unless its folder is removed.
- **After an importer fix** (a change in `bench_tracking/adapters/`): the
  admin deletes the affected runs and each owner syncs again. The runs are
  rebuilt from the result folders, which only exist on the machine that ran
  them.
- **Disk:** each run stores its bundle (Parquet), its reports and its CSVs.
  Watch `DATA_DIR/artifacts`.

## Troubleshooting

| symptom | cause |
|---|---|
| `Invalid Host header` / 403 from MLflow | `BENCH_HOSTNAME` doesn't match the name used; fix `.env`, `./setup.sh` |
| 401 on upload | login missing or wrong in `tracking.env` |
| `MLflow at … cannot be reached` | not on the tailnet, or the server is down; runs stay on disk until the next sync |
| a run never uploads | `uv run bench-track sync --dry-run` says why (still running, or already on the server) |
| `image … was built locally` warning | runs on locally built images can't be matched across machines; pull a `sha-` tag for runs to compare |
