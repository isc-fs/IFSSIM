#!/usr/bin/env bash
# Local MLflow tracking server for the tracker evaluation (SQLite + local artifact store).
# UI: http://127.0.0.1:5005   Stop: Ctrl-C
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
data="${MLFLOW_DATA_DIR:-$here/data}"
mkdir -p "$data/artifacts"
cd "$here/../.."
exec uv run mlflow server \
  --backend-store-uri "sqlite:///$data/mlflow.db" \
  --artifacts-destination "$data/artifacts" \
  --host 127.0.0.1 --port "${MLFLOW_PORT:-5005}" \
  --workers 2
