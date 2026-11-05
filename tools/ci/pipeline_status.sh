#!/usr/bin/env bash
# Posts the sim check's state on an IFS09-DV-PIPELINE commit, as the
# `sim check` commit status, linking to this run. Used by sim-pr.yml when a
# pipeline PR started it.
#
#   tools/ci/pipeline_status.sh <pending|success|failure|error> <description>
#
# Env: PIPELINE_SHA (the commit), TOKEN (Commit statuses: write on the
# pipeline repo; without it this only warns), GITHUB_SERVER_URL,
# GITHUB_REPOSITORY, GITHUB_RUN_ID (set by Actions).
set -euo pipefail
state="$1"
desc="$2"
: "${PIPELINE_SHA:?set PIPELINE_SHA}"
[[ "$PIPELINE_SHA" =~ ^[0-9a-f]{40}$ ]] || { echo "::error::not a commit SHA: $PIPELINE_SHA"; exit 1; }
if [ -z "${TOKEN:-}" ]; then
  echo "::warning::SIM_CHECK_TOKEN isn't set, so the result isn't posted on the pipeline PR"
  exit 0
fi
url="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-isc-fs/IFSSIM}/actions/runs/${GITHUB_RUN_ID:-}"
body="$(jq -n --arg s "$state" --arg d "$desc" --arg u "$url" \
  '{state: $s, description: $d, target_url: $u, context: "sim check"}')"
curl -fsS -o /dev/null -X POST \
  -H "Authorization: Bearer $TOKEN" \
  -H "Accept: application/vnd.github+json" \
  -H "X-GitHub-Api-Version: 2022-11-28" \
  "https://api.github.com/repos/isc-fs/IFS09-DV-PIPELINE/statuses/$PIPELINE_SHA" \
  -d "$body"
echo "posted '$state' on isc-fs/IFS09-DV-PIPELINE@${PIPELINE_SHA:0:7}"
