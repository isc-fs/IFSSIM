#!/usr/bin/env bash
# Rebuild the Lichtblick web UI image from Git Bash on Windows.

set -euo pipefail

# Prevent Git Bash/MSYS from rewriting container paths such as
# /entrypoint.sh or Caddy's :8080 arguments before Docker sees them.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

echo "→ normalizing docker/lichtblick/Dockerfile to LF"
sed -i 's/\r$//' docker/lichtblick/Dockerfile

echo "→ stopping Lichtblick"
docker compose stop lichtblick >/dev/null 2>&1 || true

echo "→ rebuilding Lichtblick without cache"
docker compose build --no-cache lichtblick

echo "→ recreating Lichtblick"
docker compose up -d --force-recreate lichtblick

echo
echo "✓ Lichtblick rebuilt. Open http://localhost:8080"
