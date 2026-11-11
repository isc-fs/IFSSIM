#!/usr/bin/env bash
# The sim PR check as the GPU runner runs it (.github/workflows/sim-pr.yml):
# a headless start-up check, then the car characterisation (tools/characterise)
# and the missions tools/ci/sim_check.json lists,
# through this checkout's pipeline, scored by the referee; then the report
# (tools/ci/sim_report.py) and its track maps (tools/ci/sim_plot.py).
#
# It holds the box's sim-session lock for the whole run, so it waits for any
# other sim run on the box (they share the RPC port, the pipeline container
# name and the GPU) instead of fighting it.
#
# Env:
#   IFSSIM_BUILD   the packaged sim (…/Linux/IFSSIM.sh); required
#   CI_HOME        the box's CI folder: baseline, owner marker (default /build/ifssim-ci)
#   IFSSIM_SIM     how to start the sim: the box's container wrapper
#                  (/usr/local/bin/ifssim-sim) when it has one, else IFSSIM_BUILD itself
#   RESULTS        results root (default tools/sim_benchmark/results/ci)
#   REPEATS        acceleration runs (default: sim_check.json's)
#   RANDOM_SEEDS   comma-separated seeds for the random tracks, to drive given
#                  ones again (default: new ones from GITHUB_RUN_ID)
#   REPORT         where the report and maps go (default $RESULTS/report)
#   BASELINE       the last dev run's report.json, to compare with
#                  (default $CI_HOME/baseline/report.json)
#   DV_IMAGE       the pipeline image built for this run; draws the maps
#   SESSION_LOCK   default /var/tmp/ifssim-session.lock
#   LOCK_WAIT_S    how long to wait for it (default 14400)
#
# Exit codes: 0 every run of the required missions finished (and the car
# characterisation's hard checks passed, when sim_check.json requires them), 1 not,
# 3 the machine failed (sim didn't start, nothing ran, lock never free).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
RESULTS="${RESULTS:-$ROOT/tools/sim_benchmark/results/ci}"
REPORT="${REPORT:-$RESULTS/report}"
CI_HOME="${CI_HOME:-/build/ifssim-ci}"
BASELINE="${BASELINE:-$CI_HOME/baseline/report.json}"
LOCK="${SESSION_LOCK:-/var/tmp/ifssim-session.lock}"
LOCK_WAIT_S="${LOCK_WAIT_S:-14400}"
: "${IFSSIM_BUILD:?set IFSSIM_BUILD to the packaged IFSSIM.sh}"
export IFSSIM_BUILD
# The container wrapper on a box that has one (gpu-1080's leases the GPU from its
# render farm), else the packaged build itself (tools/ci/README.md).
if [ -z "${IFSSIM_SIM:-}" ] && [ -x /usr/local/bin/ifssim-sim ]; then
  IFSSIM_SIM=/usr/local/bin/ifssim-sim
fi
SIM="${IFSSIM_SIM:-$IFSSIM_BUILD}"
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
OWNER="${SIM_OWNER_FILE:-$CI_HOME/ci-sim.owner}"
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

# Random tracks (tools/ci/gen_tracks.py), one per seed, written into this
# checkout's Content/tracks, where the sim reads tracks from. The generator and
# its dependencies are in the Mission Control image (built by the workflow).
rm -f "$ROOT"/Content/tracks/ci_rand_*.csv
rand_count="$(python3 -c 'import json, sys; print((json.load(open(sys.argv[1])).get("random_tracks") or {}).get("count", 0))' \
  "$ROOT/tools/ci/sim_check.json")"
seeds="${RANDOM_SEEDS:-}"
if [ -n "$seeds" ] && ! [[ "$seeds" =~ ^[0-9]+(,[0-9]+)*$ ]]; then
  echo "::error::RANDOM_SEEDS must be comma-separated numbers, got '$seeds'"
  exit 3
fi
if [ -z "$seeds" ] && [ "$rand_count" -gt 0 ]; then
  # 20 apart: a seed whose track is unusable is replaced by the next one up.
  base=$(( (${GITHUB_RUN_ID:-$(date +%s)} % 100000) * 100 ))
  seeds="$(seq -s, "$base" 20 $(( base + 20 * (rand_count - 1) )))"
fi
random_tracks=""
if [ -n "$seeds" ]; then
  echo "random track seeds: $seeds"
  if out="$(cd "$ROOT" && docker compose run --rm --no-deps -T --user "$(id -u):$(id -g)" \
      -e MPLCONFIGDIR=/tmp -e GEN_MIN_RADIUS_M -v "$ROOT/tools/ci:/ci:ro" --entrypoint python3 \
      mission_control_backend /ci/gen_tracks.py --gen /track_gen --out /tracks --seeds "$seeds")"; then
    random_tracks="$(printf '%s\n' "$out" | tail -n1)"
  else
    echo "::warning::the random tracks weren't generated; autocross runs on its fixed track only"
  fi
fi
echo "random tracks: ${random_tracks:-none}"

# The missions and their overrides; REPEATS (the workflow's input) changes
# acceleration's, and the random tracks join their mission's tracks.
overrides="$RESULTS/mission_overrides.json"
missions="$(python3 - "$ROOT/tools/ci/sim_check.json" "$overrides" "${REPEATS:-}" "$random_tracks" <<'PY'
import json, sys
cfg = json.load(open(sys.argv[1]))
over = cfg.get("overrides", {})
if sys.argv[3]:
    over.setdefault("acceleration", {})["repeats"] = int(sys.argv[3])
extra = [t for t in sys.argv[4].split(",") if t]
if extra:
    m = over.setdefault((cfg.get("random_tracks") or {}).get("mission", "autocross"), {})
    m["tracks"] = list(m.get("tracks", [])) + extra
json.dump(over, open(sys.argv[2], "w"))
print(",".join(cfg["required"] + cfg.get("informational", [])))
PY
)"
echo "missions: $missions"

# The car characterisation (tools/characterise), once, before the missions, in
# the same sim session: scripted manoeuvres checked against settings.json.
# sim_check.json's `characterisation.enabled` switches it; sim_report.py judges it.
characterise=""
if python3 -c 'import json, sys; sys.exit(0 if (json.load(open(sys.argv[1])).get("characterisation") or {}).get("enabled") else 1)' \
    "$ROOT/tools/ci/sim_check.json"; then
  characterise="--characterise"
fi
echo "car characterisation: ${characterise:-off}"

# --poll-s 0.2: the car's path is sampled 5 times a second (the maps need it).
# shellcheck disable=SC2086  # $characterise is one flag or nothing
python3 -m headless_run run --sim "$SIM" --settings-dir "$ROOT" --mc-up \
  --results-root "$RESULTS" --max-run-s 2400 $characterise \
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
