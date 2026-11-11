#!/usr/bin/env bash
# Publish this OS's build of the sim to GitHub Releases, where the sim's
# updater (Settings, Updates) finds it. Anyone with write access and UE 5.8
# runs it, and so does the package workflow on a runner: on each release,
# whoever has time builds their OS and adds it.
#
#   tools/publish_build.sh stable   # adds this OS's zip to release v<ProjectVersion>
#   tools/publish_build.sh latest   # this OS's zip on the rolling `latest-dev` pre-release
#
# stable: the first person creates the release (and its tag, on this commit);
#         the next ones must be on that same commit and add their OS's zip.
#         Bump ProjectVersion in Config/DefaultGame.ini for a new release.
# latest: moves `latest-dev` to this commit if it's elsewhere (dropping the
#         older builds on it) and adds this OS's zip,
#         IFSSIM-v<version>-<commit>-<os>.zip (the commit tells the updater
#         it's another build).
#
# Zips: IFSSIM-v<version>-Windows-x64.zip, -macOS-arm64.zip, -Linux-x64.zip,
# each with a .sha256 the updater checks.
# Needs: a clean tree (the build says which commit it is), gh logged in with
# write access, and package_<os>.sh's own needs (UE 5.8). Pass --skip-build
# to publish the build already made.
set -euo pipefail

CHANNEL="${1:-}"
SKIP_BUILD=0
[ "${2:-}" = "--skip-build" ] && SKIP_BUILD=1
if [ "$CHANNEL" != "stable" ] && [ "$CHANNEL" != "latest" ]; then
    echo "usage: $0 stable|latest [--skip-build]" >&2
    exit 2
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) OS=windows; SUFFIX="Windows-x64" ;;
    Darwin)               OS=mac;     SUFFIX="macOS-arm64" ;;
    Linux)                OS=linux;   SUFFIX="Linux-x64" ;;
    *) echo "ERROR: unknown OS $(uname -s)." >&2; exit 1 ;;
esac

if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    echo "ERROR: uncommitted changes. Publish from a clean tree, so the build is the commit it says." >&2
    exit 1
fi
gh auth status >/dev/null 2>&1 || { echo "ERROR: gh isn't logged in (gh auth login)." >&2; exit 1; }

VERSION="$(grep -E '^ProjectVersion=' Config/DefaultGame.ini | head -1 | cut -d= -f2 | tr -d '\r\n ')"
HEAD_SHA="$(git rev-parse HEAD)"
COMMIT="$(git rev-parse --short HEAD)"
TAG="v$VERSION"
[ "$CHANNEL" = "latest" ] && TAG="latest-dev"

# The commit a remote tag points at, or nothing.
tag_commit() {
    git ls-remote origin "refs/tags/$1^{}" "refs/tags/$1" | awk '{print $1}' | tail -1
}
# Immutable releases (a repo setting) take no assets once published.
is_immutable() {
    [ "$(gh release view "$1" --json isImmutable --jq .isImmutable 2>/dev/null || echo false)" = "true" ]
}
has_asset() {
    gh release view "$TAG" --json assets --jq '.assets[].name' 2>/dev/null | grep -qx "$1"
}

if [ "$CHANNEL" = "stable" ]; then
    NAME="IFSSIM-v${VERSION}-${SUFFIX}.zip"
    AT="$(tag_commit "$TAG")"
    if [ -n "$AT" ] && [ "$AT" != "$HEAD_SHA" ]; then
        echo "ERROR: $TAG is commit ${AT:0:7}, this is $COMMIT. Build that commit (git checkout $TAG)," >&2
        echo "       or bump ProjectVersion in Config/DefaultGame.ini for a new release." >&2
        exit 1
    fi
    if has_asset "$NAME"; then
        echo "Nothing to do: $TAG already has $NAME (someone published this OS)."
        exit 0
    fi
    if is_immutable "$TAG"; then
        echo "ERROR: $TAG is an immutable release: it takes no more builds. Keep" >&2
        echo "       Settings, General, 'Immutable releases' off so each OS can be added." >&2
        exit 1
    fi
else
    NAME="IFSSIM-v${VERSION}-${COMMIT}-${SUFFIX}.zip"
fi

# Build, then bring the zip to its published name.
if [ "$SKIP_BUILD" = "0" ]; then
    bash "$ROOT/package_$OS.sh"
fi
mkdir -p "$ROOT/dist"
ZIP="$ROOT/dist/$NAME"
case "$OS" in
    windows) SRC="$ROOT/dist/IFSSIM-v${VERSION}-Windows-x64.zip" ;;
    mac)     SRC="$ROOT/dist/IFSSIM-v${VERSION}-Mac.zip" ;;
    linux)   # package_linux.sh leaves the staged tree; zip it here.
             [ -d "$ROOT/Saved/StagedBuilds/Linux" ] || { echo "ERROR: no Saved/StagedBuilds/Linux." >&2; exit 1; }
             rm -f "$ZIP"
             ( cd "$ROOT/Saved/StagedBuilds/Linux" && zip -ryq "$ZIP" . ) ;;
esac
if [ "$OS" != "linux" ] && [ "$SRC" != "$ZIP" ]; then
    [ -f "$SRC" ] || { echo "ERROR: $SRC not found." >&2; exit 1; }
    cp "$SRC" "$ZIP"
fi
SIZE=$(( $(wc -c < "$ZIP") / 1048576 ))
[ "$SIZE" -lt 2000 ] || { echo "ERROR: $NAME is $SIZE MB; GitHub takes up to 2 GiB per file." >&2; exit 1; }
if command -v sha256sum >/dev/null; then
    ( cd "$ROOT/dist" && sha256sum "$NAME" > "$NAME.sha256" )
else
    ( cd "$ROOT/dist" && shasum -a 256 "$NAME" > "$NAME.sha256" )
fi
echo "Publishing $NAME ($SIZE MB, $CHANNEL) for $COMMIT"

if [ "$CHANNEL" = "stable" ]; then
    if ! gh release view "$TAG" >/dev/null 2>&1; then
        gh release create "$TAG" --target "$HEAD_SHA" --title "IFSSIM $TAG" \
            --notes "Build of $COMMIT. Members on Stable get it from Settings, Updates. Each OS's zip is added by whoever builds it (tools/publish_build.sh)."
    fi
    gh release upload "$TAG" "$ZIP" "$ZIP.sha256"
else
    if [ "$(tag_commit latest-dev)" != "$HEAD_SHA" ]; then
        # Another commit: move the tag, and only this commit's builds stay on it.
        git tag -f latest-dev HEAD
        git push -f origin refs/tags/latest-dev
        if gh release view latest-dev >/dev/null 2>&1; then
            gh release view latest-dev --json assets --jq '.assets[].name' | while read -r A; do
                gh release delete-asset latest-dev "$A" -y
            done
        fi
    fi
    NOTES="The last build of dev, $COMMIT. Members on Latest get it from Settings, Updates."
    if gh release view latest-dev >/dev/null 2>&1; then
        gh release edit latest-dev --prerelease --title "Latest dev build ($COMMIT)" --notes "$NOTES"
    else
        gh release create latest-dev --prerelease --title "Latest dev build ($COMMIT)" --notes "$NOTES"
    fi
    gh release upload latest-dev "$ZIP" "$ZIP.sha256" --clobber
fi
echo "Done: $(gh release view "$TAG" --json url --jq .url)"
