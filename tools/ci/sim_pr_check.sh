#!/usr/bin/env bash
# The sim PR check as the GPU runner runs it (.github/workflows/sim-pr.yml):
# a headless start-up check, then the acceleration mission $REPEATS times
# through this checkout's pipeline, scored by the referee.
#
# It holds the box's sim-session lock for the whole run, so it waits for any
# other sim run on the box (they share the RPC port, the pipeline container
# name and the GPU) instead of fighting it.
#
# Env:
#   IFSSIM_BUILD   the packaged sim (…/Linux/IFSSIM.sh); required
#   IFSSIM_SIM     the box's container wrapper for it (default /usr/local/bin/ifssim-sim)
#   RESULTS        results root (default tools/sim_benchmark/results/ci)
#   REPEATS        acceleration runs (default 3)
#   SESSION_LOCK   default /var/tmp/ifssim-session.lock
#   LOCK_WAIT_S    how long to wait for it (default 14400)
#
# Exit codes follow headless_run: 0 every run finished, 1 some didn't,
# 3 the machine failed (sim didn't start or crashed, lock never free).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM="${IFSSIM_SIM:-/usr/local/bin/ifssim-sim}"
RESULTS="${RESULTS:-$ROOT/tools/sim_benchmark/results/ci}"
REPEATS="${REPEATS:-3}"
LOCK="${SESSION_LOCK:-/var/tmp/ifssim-session.lock}"
LOCK_WAIT_S="${LOCK_WAIT_S:-14400}"
: "${IFSSIM_BUILD:?set IFSSIM_BUILD to the packaged IFSSIM.sh}"
export IFSSIM_BUILD
# Own compose project, so this run's images and containers never replace the
# ones a manual run on the box uses.
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-ifssim-ci}"

exec 9>>"$LOCK"
echo "waiting for the sim-session lock ($LOCK)"
flock -w "$LOCK_WAIT_S" 9 || { echo "the lock wasn't free after ${LOCK_WAIT_S} s" >&2; exit 3; }
echo "lock held"

# shellcheck disable=SC2329  # run by the EXIT trap
cleanup() {
  docker rm -f ifssim-sim bench-dv-stack >/dev/null 2>&1 || true
  (cd "$ROOT" && docker compose down --remove-orphans >/dev/null 2>&1) || true
}
trap cleanup EXIT

# Mission Control's backend listens on host port 8000, so stop another
# project's (a manual run's; its own --mc-up starts it again).
for id in $(docker ps -q --filter "name=mission_control_backend"); do
  project="$(docker inspect -f '{{index .Config.Labels "com.docker.compose.project"}}' "$id")"
  [ "$project" = "$COMPOSE_PROJECT_NAME" ] || docker stop "$id" >/dev/null
done

mkdir -p "$RESULTS"
cd "$ROOT/tools"

if ! python3 -m headless_run check --sim "$SIM" --settings-dir "$ROOT"; then
  echo "### Sim check: the sim didn't start" >> "${GITHUB_STEP_SUMMARY:-/dev/null}"
  exit 3
fi

rc=0
python3 -m headless_run run --sim "$SIM" --settings-dir "$ROOT" --mc-up \
  --results-root "$RESULTS" --max-run-s 1800 \
  -- --missions acceleration --repeats "$REPEATS" || rc=$?

python3 "$ROOT/tools/ci/sim_pr_summary.py" "$RESULTS" | tee -a "${GITHUB_STEP_SUMMARY:-/dev/null}" || true
exit "$rc"
