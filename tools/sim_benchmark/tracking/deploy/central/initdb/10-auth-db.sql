-- MLflow keeps its logins in a database of their own, and the launcher its job queue.
-- (First start only; setup.sh creates any that are missing on an existing server.)
CREATE DATABASE mlflow_auth OWNER mlflow;
CREATE DATABASE bench OWNER mlflow;
