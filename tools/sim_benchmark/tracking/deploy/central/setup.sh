#!/usr/bin/env bash
# First start of the central server: build, start, create the experiments, the viewer's
# and the worker's logins, and the worker's settings (worker.env). Safe to run again.
set -euo pipefail
cd "$(dirname "$0")"
[ -f .env ] || { echo "No .env: cp .env.example .env and fill it in." >&2; exit 1; }
set -a; . ./.env; set +a

: "${WORKER_PASSWORD:?set WORKER_PASSWORD in .env}"
# MLflow refuses passwords of 12 characters or fewer
for v in MLFLOW_ADMIN_PASSWORD VIEWER_PASSWORD WORKER_PASSWORD; do
  [ "${#v}" -gt 0 ] && [ "$(printf %s "${!v}" | wc -c)" -gt 12 ] \
    || { echo "$v must be longer than 12 characters (openssl rand -base64 24)." >&2; exit 1; }
done
for d in "$BENCH_REPO" "$BENCH_RESULTS" "$BENCH_BAGS"; do
  [ -d "$d" ] || { echo "$d does not exist (BENCH_REPO / BENCH_RESULTS / BENCH_BAGS in .env)." >&2; exit 1; }
done
[ -f "$BENCH_REPO/bench.yaml" ] || { echo "$BENCH_REPO has no bench.yaml: is it an IFSSIM clone?" >&2; exit 1; }

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

# the job queue's database, on servers set up before it existed
docker compose exec -T postgres psql -U mlflow -d mlflow -tAc \
  "SELECT 1 FROM pg_database WHERE datname = 'bench'" | grep -q 1 \
  || docker compose exec -T postgres createdb -U mlflow bench

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
admin add-user worker --password "$WORKER_PASSWORD" 2>/dev/null \
  || admin set-password worker --password "$WORKER_PASSWORD"
docker compose up -d viewer

# what bench-worker.service reads (git-ignored: it has passwords)
umask 077
cat > worker.env <<ENV
BENCH_QUEUE_URL=postgresql+psycopg://mlflow:${POSTGRES_PASSWORD}@127.0.0.1:${POSTGRES_PORT:-5432}/bench
MLFLOW_TRACKING_URI=http://127.0.0.1:${MLFLOW_PORT:-5005}
MLFLOW_TRACKING_USERNAME=worker
MLFLOW_TRACKING_PASSWORD=${WORKER_PASSWORD}
BENCH_REPO=${BENCH_REPO}
BENCH_RESULTS=${BENCH_RESULTS}
BENCH_BAGS=${BENCH_BAGS}
ENV

echo
echo "MLflow:      http://127.0.0.1:${MLFLOW_PORT:-5005}  (admin / MLFLOW_ADMIN_PASSWORD)"
echo "bench-view:  http://127.0.0.1:${VIEWER_PORT:-8050}"
echo "Next (DEPLOY.md): tailscale serve, then start the worker:"
echo "  sudo cp bench-worker.service /etc/systemd/system/ && sudo systemctl daemon-reload"
echo "  sudo systemctl enable --now bench-worker   # after editing User= and the paths in it"
