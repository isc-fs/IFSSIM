#!/usr/bin/env bash
# The sim PR check as the GPU runner runs it (.github/workflows/sim-pr.yml):
# a headless start-up check, then the missions tools/ci/sim_check.json lists,
# through this checkout's pipeline, scored by the referee; then the report
# (tools/ci/sim_report.py) and its track maps (tools/ci/sim_plot.py).
#
# It holds the box's sim-session lock for the whole run, so it waits for any
# other sim run on the box (they share the RPC port, the pipeline container
# name and the GPU) instead of fighting it.
#
# Env:
#   IFSSIM_BUILD   the packaged sim (…/Linux/IFSSIM.sh); required
#   IFSSIM_SIM     the box's container wrapper for it (default /usr/local/bin/ifssim-sim)
#   RESULTS        results root (default tools/sim_benchmark/results/ci)
#   REPEATS        acceleration runs (default: sim_check.json's)
#   REPORT         where the report and maps go (default $RESULTS/report)
#   BASELINE       the last dev run's report.json, to compare with
#                  (default /build/ifssim-ci/baseline/report.json)
#   DV_IMAGE       the pipeline image built for this run; draws the maps
#   SESSION_LOCK   default /var/tmp/ifssim-session.lock
#   LOCK_WAIT_S    how long to wait for it (default 14400)
#
# Exit codes: 0 every run of the required missions finished, 1 some didn't,
# 3 the machine failed (sim didn't start, nothing ran, lock never free).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM="${IFSSIM_SIM:-/usr/local/bin/ifssim-sim}"
RESULTS="${RESULTS:-$ROOT/tools/sim_benchmark/results/ci}"
REPORT="${REPORT:-$RESULTS/report}"
BASELINE="${BASELINE:-/build/ifssim-ci/baseline/report.json}"
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

# Marks the box's sim as this CI's while a check runs it. A cancelled job is
# killed without running its traps (the runner kills the step's process tree),
# which leaves the sim container up and this file behind: the next check then
# knows that sim is CI's own and removes it. A sim with no marker was started
# by someone else and is never touched.
OWNER="${SIM_OWNER_FILE:-/build/ifssim-ci/ci-sim.owner}"
ours=0

# shellcheck disable=SC2329  # run by the traps
cleanup() {
  [ "$ours" = 1 ] || return 0
  docker rm -f ifssim-sim bench-dv-stack >/dev/null 2>&1 || true
  (cd "$ROOT" && docker compose down --remove-orphans >/dev/null 2>&1) || true
  rm -f "$OWNER"
}
trap cleanup EXIT
# A cancel that does reach us (SIGINT/SIGTERM): stop the sim before the lock goes.
trap 'cleanup; exit 130' INT TERM

sim_up() { docker ps -q --filter name='^ifssim-sim$' | grep -q .; }
if sim_up && [ -f "$OWNER" ]; then
  echo "removing the sim a cancelled check left running"
  docker rm -f ifssim-sim bench-dv-stack >/dev/null 2>&1 || true
fi
# Anyone else's: it may be stopping on its own, so give it two minutes.
for _ in $(seq 60); do
  sim_up || break
  sleep 2
done
if sim_up; then
  echo "::error::another sim (container ifssim-sim) is running on the box; it isn't this CI's, so stop it by hand and re-run"
  exit 3
fi
ours=1
touch "$OWNER"

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

# The missions and their overrides; REPEATS (the workflow's input) changes acceleration's.
overrides="$RESULTS/mission_overrides.json"
missions="$(python3 - "$ROOT/tools/ci/sim_check.json" "$overrides" "${REPEATS:-}" <<'PY'
import json, sys
cfg = json.load(open(sys.argv[1]))
over = cfg.get("overrides", {})
if sys.argv[3]:
    over.setdefault("acceleration", {})["repeats"] = int(sys.argv[3])
json.dump(over, open(sys.argv[2], "w"))
print(",".join(cfg["required"] + cfg.get("informational", [])))
PY
)"
echo "missions: $missions"

# --poll-s 0.2: the car's path is sampled 5 times a second (the maps need it).
python3 -m headless_run run --sim "$SIM" --settings-dir "$ROOT" --mc-up \
  --results-root "$RESULTS" --max-run-s 2400 \
  -- --missions "$missions" --mission-overrides "$overrides" --poll-s 0.2 || true

rc=0
python3 "$ROOT/tools/ci/sim_report.py" "$RESULTS" --out "$REPORT" --baseline "$BASELINE" \
  > /dev/null || rc=$?
cat "$REPORT/summary.md" >> "${GITHUB_STEP_SUMMARY:-/dev/null}" 2>/dev/null || true

# The maps need matplotlib, which the pipeline image has and the box doesn't.
if [ -n "${DV_IMAGE:-}" ] && [ -f "$REPORT/report.json" ]; then
  docker run --rm --user "$(id -u):$(id -g)" --network none --entrypoint python3 \
    -v "$ROOT/tools/ci:/ci:ro" -v "$RESULTS:/results" -v "$REPORT:/report" -e MPLCONFIGDIR=/tmp \
    "$DV_IMAGE" /ci/sim_plot.py /report --results /results \
    || echo "::warning::the track maps weren't drawn"
fi
exit "$rc"
