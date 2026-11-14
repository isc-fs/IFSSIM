#!/usr/bin/env bash
# Runs a benchmark spec as the bench PC's runner does (.github/workflows/bench.yml):
# the same tools/sim_benchmark/run_spec.py a laptop runs, with a fresh headless sim
# and Mission Control's backend around it when the spec drives the sim (the mission
# benchmark). Bag benchmarks need no sim and run straight away.
#
#   tools/ci/bench_run.sh missions-basic                         # a preset (tools/sim_benchmark/specs)
#   tools/ci/bench_run.sh my-spec.json --set benchmarks.missions.repeats=3
#
# On your own machine, with a sim already open, you don't need this script:
# run_spec.py on its own drives that sim.
#
# Env:
#   IFSSIM_BUILD   the packaged sim (…/Linux/IFSSIM.sh); required when the spec drives the sim
#   IFSSIM_SIM     how to start the sim (as in sim_pr_check.sh; default the box's
#                  /usr/local/bin/ifssim-sim when it has one, else IFSSIM_BUILD)
#   RESULTS        where the runs go (default: bench.yaml's results folder)
#   CI_HOME        the box's CI folder, for the sim owner marker (default /build/ifssim-ci)
#   MC_URL         the backend the missions talk to (default http://127.0.0.1:8000)
#   MC_UP          1 (default): start the backend (docker compose); 0: use the one at MC_URL
#   SESSION_LOCK   the box's sim-session lock, shared with sim_pr_check.sh
#                  (default /var/tmp/ifssim-session.lock)
#   LOCK_WAIT_S    how long to wait for it (default 14400)
#   BENCH_TRIGGER, BENCH_REQUESTED_BY  recorded in each run's spec.json (run_spec.py)
#
# Exit codes: run_spec.py's (0 every job finished, 1 some failed, 2 the spec has
# problems), 3 the machine failed (sim or backend never up, lock never free).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[ $# -gt 0 ] || { echo "usage: $0 <spec file or preset>... [--set key=value]..." >&2; exit 2; }
spec_args=("$@")
results_args=()
[ -n "${RESULTS:-}" ] && { mkdir -p "$RESULTS"; results_args=(--results "$RESULTS"); }
run_spec=(python3 "$ROOT/tools/sim_benchmark/run_spec.py")

# The spec is checked before waiting for the lock or starting a sim.
spec_json="$("${run_spec[@]}" "${spec_args[@]}" --print-spec)" || exit 2
drives_sim="$(printf '%s' "$spec_json" | python3 -c \
  'import json, sys; print(int("missions" in (json.load(sys.stdin).get("benchmarks") or {})))')"
name="$(printf '%s' "$spec_json" | python3 -c 'import json, sys; print(json.load(sys.stdin).get("name") or "")')"
echo "spec: ${name:-unnamed} (drives the sim: $([ "$drives_sim" = 1 ] && echo yes || echo no))"

log="${RESULTS:-$ROOT/tools/sim_benchmark/results}/bench_run.log"
mkdir -p "$(dirname "$log")"
summary() {
  # run_spec.py's closing table, for the job summary
  [ -n "${GITHUB_STEP_SUMMARY:-}" ] || return 0
  {
    echo "### Benchmark: ${name:-unnamed}"
    echo '```'
    sed -n '/^Done:/,$p' "$log"
    echo '```'
  } >> "$GITHUB_STEP_SUMMARY"
}

if [ "$drives_sim" != 1 ]; then
  rc=0
  "${run_spec[@]}" "${spec_args[@]}" "${results_args[@]}" 2>&1 | tee "$log" || rc=${PIPESTATUS[0]}
  summary
  exit "$rc"
fi

[ -n "${IFSSIM_BUILD:-}" ] || { echo "set IFSSIM_BUILD to the packaged IFSSIM.sh (the spec drives the sim)" >&2; exit 2; }
export IFSSIM_BUILD
if [ -z "${IFSSIM_SIM:-}" ] && [ -x /usr/local/bin/ifssim-sim ]; then
  IFSSIM_SIM=/usr/local/bin/ifssim-sim
fi
SIM="${IFSSIM_SIM:-$IFSSIM_BUILD}"
MC_URL="${MC_URL:-http://127.0.0.1:8000}"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-ifssim-ci}"
CI_HOME="${CI_HOME:-/build/ifssim-ci}"
LOCK="${SESSION_LOCK:-/var/tmp/ifssim-session.lock}"
LOCK_WAIT_S="${LOCK_WAIT_S:-14400}"

exec 9>>"$LOCK"
echo "waiting for the sim-session lock ($LOCK)"
flock -w "$LOCK_WAIT_S" 9 || { echo "the lock wasn't free after ${LOCK_WAIT_S} s" >&2; exit 3; }
echo "lock held"

# The same owner marker as sim_pr_check.sh (only one of them holds the lock at a
# time): a sim with the marker was left by a cancelled CI job and is removed; one
# without it is someone else's and never touched.
OWNER="${SIM_OWNER_FILE:-$CI_HOME/ci-sim.owner}"
mkdir -p "$(dirname "$OWNER")"
ours=0
# shellcheck disable=SC2329  # run by the traps
cleanup() {
  [ "$ours" = 1 ] || return 0
  docker rm -f ifssim-sim bench-dv-stack >/dev/null 2>&1 || true
  (cd "$ROOT" && docker compose down --remove-orphans >/dev/null 2>&1) || true
  rm -f "$OWNER"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

sim_up() { docker ps -q --filter name='^ifssim-sim$' 2>/dev/null | grep -q .; }
if sim_up && [ -f "$OWNER" ]; then
  echo "removing the sim a cancelled job left running"
  docker rm -f ifssim-sim bench-dv-stack >/dev/null 2>&1 || true
fi
for _ in $(seq 60); do
  sim_up || break
  sleep 2
done
if sim_up; then
  echo "::error::another sim (container ifssim-sim) is running on the box; it isn't CI's, so stop it by hand and re-run"
  exit 3
fi
ours=1
touch "$OWNER"

# Mission Control's backend listens on host port 8000: stop another project's.
for id in $(docker ps -q --filter "name=mission_control_backend"); do
  project="$(docker inspect -f '{{index .Config.Labels "com.docker.compose.project"}}' "$id")"
  [ "$project" = "$COMPOSE_PROJECT_NAME" ] || docker stop "$id" >/dev/null
done

export IFSSIM_SIM_CPU_SHARES="${IFSSIM_SIM_CPU_SHARES-8192}"
export BENCH_CPU_SHARES="${BENCH_CPU_SHARES-8192}"
export DV_VIZ_RELAY="${DV_VIZ_RELAY:-false}"

cd "$ROOT/tools"
rc=0
mc_up=(--mc-up)
[ "${MC_UP:-1}" = 0 ] && mc_up=()
python3 -m headless_run exec --sim "$SIM" --settings-dir "$ROOT" "${mc_up[@]}" --mc-url "$MC_URL" \
  --results-root "${RESULTS:-$ROOT/tools/sim_benchmark/results}" -- \
  "${run_spec[@]}" "${spec_args[@]}" "${results_args[@]}" \
  --set "benchmarks.missions.settings.mc_url=$MC_URL" 2>&1 | tee "$log" || rc=${PIPESTATUS[0]}
summary
exit "$rc"
