#!/usr/bin/env bash
# Publish a Windows build of the sim to GitHub Releases, where the sim's
# updater (Settings, Updates) finds it. Until a CI runner does this on every
# merge, whoever has UE 5.8 runs it from Git Bash.
#
#   tools/publish_windows_build.sh stable   # a tagged release, v<ProjectVersion>
#   tools/publish_windows_build.sh latest   # the rolling `latest-dev` pre-release
#
# stable: bump ProjectVersion in Config/DefaultGame.ini first; the tag must
#         not exist yet. Publishes IFSSIM-v<version>-Windows-x64.zip.
# latest: moves the `latest-dev` tag to this commit and replaces its build,
#         IFSSIM-v<version>-<commit>-Windows-x64.zip (the commit tells the
#         updater it's another build).
#
# Each zip goes up with a .sha256 beside it, which the updater checks.
# Needs: a clean tree (the build says which commit it is), gh logged in with
# write access, and package_windows.sh's own needs (UE 5.8). Pass
# --skip-build to publish the zip already in dist/.
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

if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    echo "ERROR: uncommitted changes. Publish from a clean tree, so the build is the commit it says." >&2
    exit 1
fi
gh auth status >/dev/null 2>&1 || { echo "ERROR: gh isn't logged in (gh auth login)." >&2; exit 1; }

VERSION="$(grep -E '^ProjectVersion=' Config/DefaultGame.ini | head -1 | cut -d= -f2 | tr -d '\r\n ')"
COMMIT="$(git rev-parse --short HEAD)"
TAG="v$VERSION"

if [ "$CHANNEL" = "stable" ] && gh release view "$TAG" >/dev/null 2>&1; then
    echo "ERROR: $TAG is already released. Bump ProjectVersion in Config/DefaultGame.ini." >&2
    exit 1
fi

if [ "$SKIP_BUILD" = "0" ]; then
    bash "$ROOT/package_windows.sh"
fi
ZIP="$ROOT/dist/IFSSIM-v${VERSION}-Windows-x64.zip"
[ -f "$ZIP" ] || { echo "ERROR: $ZIP not found." >&2; exit 1; }

if [ "$CHANNEL" = "latest" ]; then
    NAME="IFSSIM-v${VERSION}-${COMMIT}-Windows-x64.zip"
    cp "$ZIP" "$ROOT/dist/$NAME"
    ZIP="$ROOT/dist/$NAME"
fi
NAME="$(basename "$ZIP")"
( cd "$(dirname "$ZIP")" && sha256sum "$NAME" > "$NAME.sha256" )
echo "Publishing $NAME ($CHANNEL) for $COMMIT"

if [ "$CHANNEL" = "stable" ]; then
    gh release create "$TAG" "$ZIP" "$ZIP.sha256" \
        --target "$(git rev-parse HEAD)" --title "IFSSIM $TAG" \
        --notes "Windows build of $COMMIT. Members on Stable get it from Settings, Updates."
else
    git tag -f latest-dev HEAD
    git push -f origin refs/tags/latest-dev
    if gh release view latest-dev >/dev/null 2>&1; then
        # Only the newest build stays on it.
        gh release view latest-dev --json assets --jq '.assets[].name' | while read -r A; do
            gh release delete-asset latest-dev "$A" -y
        done
        gh release edit latest-dev --prerelease --title "Latest dev build ($COMMIT)" \
            --notes "The last build of dev, $COMMIT. Members on Latest get it from Settings, Updates."
    else
        gh release create latest-dev --prerelease --title "Latest dev build ($COMMIT)" \
            --notes "The last build of dev, $COMMIT. Members on Latest get it from Settings, Updates."
    fi
    gh release upload latest-dev "$ZIP" "$ZIP.sha256" --clobber
fi
echo "Done."
