#!/usr/bin/env bash
# First start of the central server: build, start, create the experiments and the
# viewer's login. Safe to run again (it skips what exists).
set -euo pipefail
cd "$(dirname "$0")"
[ -f .env ] || { echo "No .env: cp .env.example .env and fill it in." >&2; exit 1; }
set -a; . ./.env; set +a

docker compose up -d --build postgres mlflow
echo "waiting for MLflow ..."
for i in $(seq 90); do
  docker compose exec -T mlflow python -c "import urllib.request; urllib.request.urlopen('http://localhost:5000/health')" 2>/dev/null && break
  if [ "$i" = 90 ]; then
    docker compose logs --tail 40 mlflow >&2
    echo "MLflow did not come up (logs above)." >&2
    exit 1
  fi
  sleep 2
done

# bench-track from the viewer image, as the MLflow admin
admin() {
  docker compose run --rm --no-deps \
    -e MLFLOW_TRACKING_USERNAME=admin -e MLFLOW_TRACKING_PASSWORD="$MLFLOW_ADMIN_PASSWORD" \
    viewer bench-track admin "$@"
}
docker compose build viewer
admin init
admin add-user viewer --password "$VIEWER_PASSWORD" 2>/dev/null \
  || admin set-password viewer --password "$VIEWER_PASSWORD"
docker compose up -d viewer

echo
echo "MLflow:      http://127.0.0.1:${MLFLOW_PORT:-5005}  (admin / MLFLOW_ADMIN_PASSWORD)"
echo "bench-view:  http://127.0.0.1:${VIEWER_PORT:-8050}"
echo "Next: tailscale serve (DEPLOY.md), then add a login per person:"
echo "  ./admin.sh add-user <name>"
