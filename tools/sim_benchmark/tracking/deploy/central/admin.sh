#!/usr/bin/env bash
# bench-track admin as the MLflow admin, e.g.  ./admin.sh add-user alice
set -euo pipefail
cd "$(dirname "$0")"
set -a; . ./.env; set +a
exec docker compose run --rm --no-deps \
  -e MLFLOW_TRACKING_USERNAME=admin -e MLFLOW_TRACKING_PASSWORD="$MLFLOW_ADMIN_PASSWORD" \
  viewer bench-track admin "$@"
