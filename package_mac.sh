#!/usr/bin/env bash
# post-build: package Mac build with entitlements + track CSVs
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP="$SCRIPT_DIR/Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app"
ENTITLEMENTS="$SCRIPT_DIR/Build/Mac/Resources/Sandbox.Server.entitlements"
TRACKS_SRC="$SCRIPT_DIR/Content/tracks"
STAGED_DIR="$SCRIPT_DIR/Saved/StagedBuilds/Mac"
TRACKS_DST="$STAGED_DIR/tracks"

# Project version — read from Config/DefaultGame.ini's ProjectVersion
# field so the build artifact name tracks the in-binary version UE5
# stamps into the cooked .app's Info.plist. Bump in DefaultGame.ini;
# this script picks it up automatically.
PROJECT_VERSION="$(grep -E '^ProjectVersion=' "$SCRIPT_DIR/Config/DefaultGame.ini" \
    | head -1 | cut -d= -f2 | tr -d '\r\n ')"
if [ -z "$PROJECT_VERSION" ]; then
    echo "ERROR: ProjectVersion not found in Config/DefaultGame.ini" >&2
    exit 1
fi

echo "=== IFSSIM Mac post-build (v$PROJECT_VERSION) ==="

# 1. BuildCookRun
#
# `-pak -iostore -compressed` is the UE5.7 recipe for packaged
# shipping builds — generates one bundled .pak + .ucas/.utoc pair
# under Content/Paks/. Tried dropping `-pak` on perf/sim-tier1-cook-strip
# expecting IoStore to fully replace it; got 200 MB of loose
# .uexp/.uasset/.ubulk files instead (worse). Both flags stay.
echo "[1/3] Building..."
"/Users/Shared/Epic Games/UE_5.7/Engine/Build/BatchFiles/RunUAT.sh" \
  BuildCookRun \
  -project="$SCRIPT_DIR/IFSSIM.uproject" \
  -platform=Mac \
  -configuration=Shipping \
  -build -cook -stage -pak \
  -iostore -compressed \
  -clientconfig=Shipping \
  -noeditor

# 2. Re-sign with server entitlements (network.server for port 41451)
echo "[2/3] Re-signing with sandbox entitlements..."
codesign --sign - --force --deep \
  --entitlements "$ENTITLEMENTS" \
  "$APP"

# 3. Copy track CSVs next to the .app
echo "[3/3] Staging track CSVs..."
mkdir -p "$TRACKS_DST"
cp "$TRACKS_SRC/"*.csv "$TRACKS_DST/"
echo "  Tracks staged: $(ls "$TRACKS_DST"/*.csv | wc -l | tr -d ' ') files"

# 3b. Copy settings.json to the UE5 UserSettingsDir for this game.
# On macOS FPlatformProcess::UserSettingsDir() = ~/Library/Application Support/Epic/
# and FSDSSettings::AutoLoad() appends "IFSSIM" making the full search path:
#   ~/Library/Application Support/Epic/IFSSIM/settings.json
# LaunchDir resolves to empty and ProjectDir is relative in shipping builds,
# so this is the only search path that reliably resolves on macOS.
USER_SETTINGS_DIR="$HOME/Library/Application Support/Epic/IFSSIM"
mkdir -p "$USER_SETTINGS_DIR"
cp "$SCRIPT_DIR/settings.json" "$USER_SETTINGS_DIR/settings.json"
echo "  settings.json staged → $USER_SETTINGS_DIR/settings.json"

# 4. Force absolute -project path + windowed + 60 FPS cap in UECommandLine.txt.
#    BuildCookRun writes its own relative -project path here that's only
#    valid when the engine and project live on the same volume / under
#    the same user. With the engine at /Users/Shared/Epic Games/UE_5.7/
#    and the project at /Users/$USER/.../IFSSIM/, UE's relative-path
#    composer produces a malformed string of the form
#    "../../../../../../Shared/Epic Games/UE_5.7/../../../<user>/Documents/..."
#    which doesn't resolve to a real file — every launch from the
#    packaged .app then throws "Failed to open descriptor file" and
#    the app exits before any in-game code runs. We force-overwrite
#    with an absolute path computed from $SCRIPT_DIR to sidestep
#    UE's broken relative composition entirely.
#    -ExecCmds runs console commands at startup; t.MaxFPS is also baked
#    into DefaultEngine.ini [SystemSettings] for future cooked builds,
#    but -ExecCmds catches builds that predate that change.
#
#    FPS cap dropped 60 → 30. UE5's CPU+GPU work is proportional to
#    frame rate; for autonomy testing (sensors run at their own rates,
#    not tied to render FPS) 30 FPS is plenty smooth visually and
#    halves the host load that was competing with the SLAM optimizer.
#    Override via env: `IFSSIM_MAX_FPS=60 ./package_mac.sh` if you
#    want the higher cap back for visual demos.
IFSSIM_MAX_FPS="${IFSSIM_MAX_FPS:-30}"
CMDLINE="$SCRIPT_DIR/Saved/StagedBuilds/Mac/UECommandLine.txt"
PROJECT_ABS="$SCRIPT_DIR/IFSSIM.uproject"
echo "-project=\"$PROJECT_ABS\" -windowed -ExecCmds=\"t.MaxFPS $IFSSIM_MAX_FPS\"" > "$CMDLINE"
echo "  UECommandLine.txt: $(cat "$CMDLINE")"

echo ""
echo "=== Done — distribution in Saved/StagedBuilds/Mac/ ==="
echo "  App:    IFSSIM-Mac-Shipping.app"
echo "  Tracks: tracks/ ($(ls "$TRACKS_DST"/*.csv | wc -l | tr -d ' ') CSV files)"

# 5. Archive the staged directory into a release zip. macOS ships
#    `zip` everywhere, no extra tooling needed. Skip with
#    SKIP_ARCHIVE=1 for iterative cooking where the compress step
#    is wasted effort.
DIST_DIR="$SCRIPT_DIR/dist"
ARCHIVE_NAME="IFSSIM-v${PROJECT_VERSION}-Mac.zip"
ARCHIVE_PATH="$DIST_DIR/$ARCHIVE_NAME"

if [ "${SKIP_ARCHIVE:-0}" = "1" ]; then
    echo ""
    echo "[skip] SKIP_ARCHIVE=1 — distribution archive not created."
else
    echo ""
    echo "[4/4] Archiving → $ARCHIVE_NAME"
    mkdir -p "$DIST_DIR"
    rm -f "$ARCHIVE_PATH"
    # cd into the stage dir so the zip's top-level entries are the
    # .app + tracks/ + UECommandLine.txt, not a "Mac/" wrapper. -y
    # preserves symlinks inside the .app bundle (critical: Mach-O
    # codesign symlinks would otherwise be copied as their targets
    # and break the signature).
    ( cd "$STAGED_DIR" && zip -ryq "$ARCHIVE_PATH" . )
    if [ -f "$ARCHIVE_PATH" ]; then
        SIZE=$(stat -f %z "$ARCHIVE_PATH" 2>/dev/null || stat -c %s "$ARCHIVE_PATH" 2>/dev/null || echo "?")
        SIZE_MB=$((SIZE / 1024 / 1024))
        echo "       OK — ${SIZE_MB} MB at $ARCHIVE_PATH"
    else
        echo "       FAILED — archive not produced" >&2
        exit 1
    fi
fi
