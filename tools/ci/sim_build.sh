#!/usr/bin/env bash
# Picks the sim build the sim check drives (.github/workflows/sim-pr.yml) and
# writes its IFSSIM.sh to $GITHUB_ENV as IFSSIM_BUILD.
#
# The box keeps the last build of dev in $SHARED/dev. When none of the sim's
# build inputs (Source/, Plugins/, Config/, Content/ minus tracks,
# IFSSIM.uproject) differ between the commit it was built from and this
# checkout, it is reused as is. Otherwise the sim is packaged into $STAGE,
# and on dev that build replaces the shared one.
#
# The sim reads track CSVs from its working directory (this checkout's
# Content/tracks), so track edits never need a new build.
#
# Env:
#   UE5_ROOT  the engine (required to package)
#   STAGE     where a new build is staged (required)
#   CI_HOME   the box's CI folder (default /build/ifssim-ci)
#   SHARED    the shared builds (default $CI_HOME/builds)
#   PUBLISH   true: a new build becomes the shared one (set on dev)
#   FORCE_PACKAGE  true: package even if the shared build would do
set -euo pipefail

SHARED="${SHARED:-${CI_HOME:-/build/ifssim-ci}/builds}"
: "${STAGE:?set STAGE}"
BUILD_INPUTS='^(Source|Plugins|Config|Content)/|^IFSSIM\.uproject$'
NOT_BUILD_INPUTS='^Content/tracks/'

# Prints why the shared build won't do; nothing (and status 1) when it will.
needs_package() {
  if [ "${FORCE_PACKAGE:-false}" = "true" ]; then echo "packaging was asked for"; return 0; fi
  if [ ! -x "$SHARED/dev/IFSSIM.sh" ] || [ ! -f "$SHARED/dev/.commit" ]; then
    echo "there is no shared dev build yet"; return 0
  fi
  local built changed
  built="$(cat "$SHARED/dev/.commit")"
  if ! git cat-file -e "$built^{commit}" 2>/dev/null; then
    git fetch -q --no-tags --depth=1 origin "$built" 2>/dev/null \
      || { echo "can't fetch $built, the shared build's commit"; return 0; }
  fi
  changed="$(git diff --name-only "$built" HEAD | grep -E "$BUILD_INPUTS" | grep -Ev "$NOT_BUILD_INPUTS" || true)"
  if [ -n "$changed" ]; then
    echo "sim build inputs differ from the shared build (${built:0:7}):"
    printf '%s\n' "$changed" | head -20 | sed 's/^/  /'
    return 0
  fi
  return 1
}

package() {
  # Lowest CPU priority: the build shares the box with Flamenco renders.
  nice -n 19 "$UE5_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildCookRun \
    -project="$PWD/IFSSIM.uproject" -platform=Linux \
    -configuration=Development -clientconfig=Development \
    -build -cook -iterate -stage -pak -iostore -compressed -noeditor \
    -unattended -nop4 -utf8output -stagingdirectory="$STAGE"
}

# Copies the new build to $SHARED/dev-<commit> and points $SHARED/dev at it in
# one rename, keeping this build and the one before.
publish() {
  local sha dest
  sha="$(git rev-parse HEAD)"
  dest="$SHARED/dev-$sha"
  mkdir -p "$SHARED"
  rm -rf "$SHARED"/*.tmp
  cp -a "$STAGE/Linux" "$dest.tmp"
  echo "$sha" > "$dest.tmp/.commit"
  rm -rf "$dest"
  mv "$dest.tmp" "$dest"
  ln -sfn "$dest" "$SHARED/dev.new"
  mv -T "$SHARED/dev.new" "$SHARED/dev"
  # shellcheck disable=SC2012  # names are dev-<sha>, no odd characters
  ls -1dt "$SHARED"/dev-* | tail -n +3 | xargs -r rm -rf
  echo "published as the shared dev build: $dest"
}

if reason="$(needs_package)"; then
  echo "packaging the sim: $reason"
  package
  build="$STAGE/Linux/IFSSIM.sh"
  if [ "${PUBLISH:-false}" = "true" ]; then publish; fi
  summary="packaged: $reason"
else
  build="$(readlink -f "$SHARED/dev")/IFSSIM.sh"
  summary="reused the shared dev build ($(cut -c1-7 "$SHARED/dev/.commit")): no sim build inputs changed"
  echo "$summary"
fi

echo "IFSSIM_BUILD=$build" >> "${GITHUB_ENV:-/dev/null}"
printf '### Sim build\n\n%s\n\n' "$(printf '%s\n' "$summary" | head -1)" >> "${GITHUB_STEP_SUMMARY:-/dev/null}"
