#!/bin/sh
# Writes the login config from the environment, then starts MLflow.
set -eu
# explicit driver: which one plain postgresql:// means differs between SQLAlchemy versions
db="postgresql+psycopg://mlflow:${POSTGRES_PASSWORD}@postgres:5432"
mkdir -p /config
cat > /config/basic_auth.ini <<INI
[mlflow]
# Every login can upload runs and pin baselines; only the admin can delete.
default_permission = EDIT
database_uri = ${db}/mlflow_auth
admin_username = admin
authorization_function = mlflow.server.auth:authenticate_request_basic_auth
INI
export MLFLOW_AUTH_CONFIG_PATH=/config/basic_auth.ini
h="${BENCH_HOSTNAME}"
exec mlflow server \
  --app-name basic-auth \
  --backend-store-uri "${db}/mlflow" \
  --artifacts-destination /artifacts \
  --host 0.0.0.0 --port 5000 \
  --workers "${MLFLOW_WORKERS:-4}" \
  --allowed-hosts "${h},${h}:*,localhost,localhost:*,127.0.0.1,127.0.0.1:*,mlflow,mlflow:*,100.*"
