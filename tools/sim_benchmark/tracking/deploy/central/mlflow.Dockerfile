# MLflow tracking server with logins (basic-auth) and Postgres.
FROM python:3.12-slim
ARG MLFLOW_VERSION=3.16.1
RUN pip install --no-cache-dir "mlflow[auth]==${MLFLOW_VERSION}" "psycopg[binary]>=3.2,<4"
COPY mlflow-entrypoint.sh /usr/local/bin/mlflow-entrypoint.sh
EXPOSE 5000
ENTRYPOINT ["/usr/local/bin/mlflow-entrypoint.sh"]
