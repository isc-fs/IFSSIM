#!/usr/bin/env bash
# Back up the central server: the databases (pg_dump) and the artifacts (rsync).
# Run it from cron or a systemd timer (DEPLOY.md). Keeps the last KEEP dumps.
#
#   BACKUP_DIR=/mnt/backup/ifssim-bench ./backup.sh
set -euo pipefail
cd "$(dirname "$0")"
set -a; . ./.env; set +a
dest="${BACKUP_DIR:-./backups}"
keep="${KEEP:-14}"
ts="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$dest/db" "$dest/artifacts"

for db in mlflow mlflow_auth bench; do
  docker compose exec -T postgres pg_dump -U mlflow -Fc "$db" > "$dest/db/${db}_${ts}.dump.part"
  mv "$dest/db/${db}_${ts}.dump.part" "$dest/db/${db}_${ts}.dump"
  ls -1t "$dest/db/${db}_"*.dump | tail -n +"$((keep + 1))" | xargs -r rm --
done
# artifacts are written once and never changed, so a plain mirror is enough
rsync -a "${DATA_DIR:-./data}/artifacts/" "$dest/artifacts/"
echo "backup done: $dest (db ${ts})"
