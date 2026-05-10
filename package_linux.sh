#!/usr/bin/env bash
# Build the IFSSIM Linux shipping artefact.
#
# Two ways to run:
#
# 1. NATIVE Linux build, on a Linux box with UE5 5.7 installed.
#    UE5_ROOT=/path/to/Linux/UnrealEngine ./package_linux.sh
#
# 2. CROSS-COMPILE from a Mac dev box. Requires UE5 5.7 + the Linux
#    Clang Toolchain ("v22 clang-19.0.1-rockylinux8" or compatible)
#    installed and pointed to via LINUX_MULTIARCH_ROOT. Epic ships
#    the toolchain at:
#        https://dev.epicgames.com/documentation/en-us/unreal-engine/linux-development-requirements-for-unreal-engine
#    Set both UE5_ROOT (Mac install) and LINUX_MULTIARCH_ROOT before
#    running:
#        UE5_ROOT="/Users/Shared/Epic Games/UE_5.7" \
#        LINUX_MULTIARCH_ROOT="$HOME/UnrealToolchains/v22_clang-19.0.1-rockylinux8" \
#          ./package_linux.sh
#
# Output: Saved/StagedBuilds/Linux/IFSSIM/Binaries/Linux/IFSSIM-Linux-Shipping
# Plus a sibling tracks/ directory next to the binary, same shape as
# the Mac distribution (FSDSConeSpawner / loadTrack RPC search there).

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Resolve UE5 root. On Mac dev boxes the default install is under
# /Users/Shared/Epic Games/UE_5.7. On a Linux runner with a custom
# clone, the operator sets UE5_ROOT.
UE5_ROOT="${UE5_ROOT:-/Users/Shared/Epic Games/UE_5.7}"
if [ ! -f "$UE5_ROOT/Engine/Build/BatchFiles/RunUAT.sh" ]; then
    echo "ERROR: UE5 not found at '$UE5_ROOT'." >&2
    echo "Set UE5_ROOT to your UE_5.7 install directory." >&2
    exit 1
fi

# When cross-compiling on macOS, UE5 reads LINUX_MULTIARCH_ROOT to
# find the Linux Clang Toolchain. Warn early if it's missing on a
# Mac host — failing inside RunUAT 30 s in is a worse experience.
HOST_OS="$(uname -s)"
if [ "$HOST_OS" = "Darwin" ] && [ -z "${LINUX_MULTIARCH_ROOT:-}" ]; then
    echo "ERROR: cross-compile from macOS requires LINUX_MULTIARCH_ROOT." >&2
    echo "  Install Epic's Linux clang toolchain (v22+) and export the path:" >&2
    echo "    https://dev.epicgames.com/documentation/en-us/unreal-engine/linux-development-requirements-for-unreal-engine" >&2
    exit 1
fi

APP_BIN="$SCRIPT_DIR/Saved/StagedBuilds/Linux/IFSSIM/Binaries/Linux/IFSSIM-Linux-Shipping"
TRACKS_SRC="$SCRIPT_DIR/Content/tracks"
TRACKS_DST="$SCRIPT_DIR/Saved/StagedBuilds/Linux/tracks"

echo "=== IFSSIM Linux post-build ==="
echo "  UE5 root:   $UE5_ROOT"
echo "  Host OS:    $HOST_OS"
[ -n "${LINUX_MULTIARCH_ROOT:-}" ] && echo "  Toolchain:  $LINUX_MULTIARCH_ROOT"

# 1. BuildCookRun
echo "[1/3] Building..."
"$UE5_ROOT/Engine/Build/BatchFiles/RunUAT.sh" \
  BuildCookRun \
  -project="$SCRIPT_DIR/IFSSIM.uproject" \
  -platform=Linux \
  -configuration=Shipping \
  -build -cook -stage -pak \
  -iostore -compressed \
  -clientconfig=Shipping \
  -noeditor \
  -utf8output

# 2. Copy track CSVs next to the staged tree.
#    FSDSConeSpawner and the loadTrack RPC search a sibling tracks/
#    directory at runtime — same convention as the Mac distribution.
echo "[2/3] Staging track CSVs..."
mkdir -p "$TRACKS_DST"
cp "$TRACKS_SRC/"*.csv "$TRACKS_DST/"
echo "  Tracks staged: $(ls "$TRACKS_DST"/*.csv | wc -l | tr -d ' ') files"

# 3. Copy settings.json to UE5's UserSettingsDir so FSDSSettings::AutoLoad
#    finds it at runtime. On Linux UE5 reads from
#    ~/.config/Epic/IFSSIM/settings.json (per
#    FPlatformProcess::UserSettingsDir() on Linux).
USER_SETTINGS_DIR="$HOME/.config/Epic/IFSSIM"
mkdir -p "$USER_SETTINGS_DIR"
cp "$SCRIPT_DIR/settings.json" "$USER_SETTINGS_DIR/settings.json"
echo "  settings.json staged → $USER_SETTINGS_DIR/settings.json"

# 4. Force windowed mode + 60 FPS cap in UECommandLine.txt
#    (BuildCookRun overwrites this file). Same idea as package_mac.sh —
#    -ExecCmds runs console commands at startup; t.MaxFPS is also
#    baked into DefaultEngine.ini [SystemSettings] for cooked builds.
echo "[3/3] Patching UECommandLine.txt..."
CMDLINE="$SCRIPT_DIR/Saved/StagedBuilds/Linux/UECommandLine.txt"
if [ -f "$CMDLINE" ]; then
    if ! grep -q '\-windowed' "$CMDLINE"; then
        # Linux sed needs a portable in-place form. -i '' breaks on GNU
        # sed; the -E flag is consistently supported.
        sed -i.bak 's/$/ -windowed/' "$CMDLINE" && rm -f "$CMDLINE.bak"
    fi
    if ! grep -q 't.MaxFPS' "$CMDLINE"; then
        sed -i.bak 's/$/ -ExecCmds="t.MaxFPS 60"/' "$CMDLINE" && rm -f "$CMDLINE.bak"
    fi
else
    echo '-project="../../../IFSSIM/IFSSIM.uproject" -windowed -ExecCmds="t.MaxFPS 60"' > "$CMDLINE"
fi
echo "  UECommandLine.txt: $(cat "$CMDLINE")"

echo ""
echo "=== Done — distribution in Saved/StagedBuilds/Linux/ ==="
echo "  Binary:    $APP_BIN"
echo "  Tracks:    tracks/ ($(ls "$TRACKS_DST"/*.csv | wc -l | tr -d ' ') CSV files)"
echo ""
echo "Run with:"
echo "  cd Saved/StagedBuilds/Linux && IFSSIM/Binaries/Linux/IFSSIM-Linux-Shipping"
