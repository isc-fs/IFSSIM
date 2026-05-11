#!/usr/bin/env bash
# Bring up Mission Control for local development:
#   - Backend (FastAPI / uvicorn) on :8000
#   - Frontend (Vite dev server) on :3000
#
# Both run in the foreground; Ctrl-C stops both via the trap. For a
# production-mode build, run `npm run build` in frontend/ and serve
# the dist/ directory via any static file server (nginx, caddy,
# whatever); the backend stays the same.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/backend"
FRONTEND_DIR="$SCRIPT_DIR/frontend"

# Best-effort: install backend deps if not present. Detected by the
# absence of `fastapi` in the Python `import` graph from this dir's
# venv (or the system Python, if no venv is in use). Operators who
# manage their own venv can skip this by exporting
# IFSSIM_MC_SKIP_INSTALL=1.
maybe_install_backend() {
    if [ -n "${IFSSIM_MC_SKIP_INSTALL:-}" ]; then return; fi
    if python3 -c "import fastapi" 2>/dev/null; then return; fi
    echo "→ installing backend deps (one-time, ~30 s)"
    cd "$BACKEND_DIR"
    python3 -m pip install -r requirements.txt
    cd "$SCRIPT_DIR"
}

maybe_install_frontend() {
    if [ -n "${IFSSIM_MC_SKIP_INSTALL:-}" ]; then return; fi
    if [ -d "$FRONTEND_DIR/node_modules" ]; then return; fi
    echo "→ installing frontend deps (one-time, ~1 m)"
    cd "$FRONTEND_DIR"
    npm install
    cd "$SCRIPT_DIR"
}

maybe_install_backend
maybe_install_frontend

# Start both, hold their PIDs for cleanup.
echo "→ starting backend on :8000"
(cd "$BACKEND_DIR" && python3 main.py) &
BACKEND_PID=$!

echo "→ starting frontend on :3000"
(cd "$FRONTEND_DIR" && npm run dev -- --host 0.0.0.0) &
FRONTEND_PID=$!

cleanup() {
    echo
    echo "→ stopping Mission Control..."
    kill "$BACKEND_PID" "$FRONTEND_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

echo
echo "✓ Mission Control running"
echo "  Backend:  http://localhost:8000"
echo "  Frontend: http://localhost:3000"
echo "  Ctrl-C to stop both."
echo

# Wait for either process to exit; if one dies, take the other down.
wait -n "$BACKEND_PID" "$FRONTEND_PID"
cleanup
