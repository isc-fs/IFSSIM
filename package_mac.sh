#!/usr/bin/env bash
# post-build: package Mac build with entitlements + track CSVs
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP="$SCRIPT_DIR/Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app"
ENTITLEMENTS="$SCRIPT_DIR/Build/Mac/Resources/Sandbox.Server.entitlements"
TRACKS_SRC="$SCRIPT_DIR/Content/tracks"
TRACKS_DST="$SCRIPT_DIR/Saved/StagedBuilds/Mac/tracks"

echo "=== IFSSIM Mac post-build ==="

# 1. BuildCookRun
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

# 3b. Copy settings.json next to the .app (AdditionalNonUFSFiles misses it on Mac)
cp "$SCRIPT_DIR/settings.json" "$SCRIPT_DIR/Saved/StagedBuilds/Mac/settings.json"
echo "  settings.json staged"

# 4. Force windowed mode in UECommandLine.txt (BuildCookRun overwrites this file)
CMDLINE="$SCRIPT_DIR/Saved/StagedBuilds/Mac/UECommandLine.txt"
if [ -f "$CMDLINE" ]; then
  # Append -windowed if not already present
  if ! grep -q '\-windowed' "$CMDLINE"; then
    sed -i '' 's/$/ -windowed/' "$CMDLINE"
  fi
else
  echo '-project="../../../IFSSIM/IFSSIM.uproject" -windowed' > "$CMDLINE"
fi
echo "  UECommandLine.txt: $(cat "$CMDLINE")"

echo ""
echo "=== Done — distribution in Saved/StagedBuilds/Mac/ ==="
echo "  App:    IFSSIM-Mac-Shipping.app"
echo "  Tracks: tracks/ ($(ls "$TRACKS_DST"/*.csv | wc -l | tr -d ' ') CSV files)"
