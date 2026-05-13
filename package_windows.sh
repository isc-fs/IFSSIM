#!/usr/bin/env bash
# post-build: package Windows build with tracks + settings.
#
# Mirrors package_mac.sh — same structure, same env knobs, same end
# layout — but adapted for Win64:
#
#   - RunUAT.bat (not RunUAT.sh) under the standard Windows install
#     of UE_5.7 ("C:\Program Files\Epic Games\UE_5.7\…").
#   - Platform=Win64.
#   - Skips macOS-only codesign step (Windows has no entitlements
#     equivalent for binding to TCP/41451 — Windows Firewall prompts
#     once on first launch).
#   - settings.json lands in %LOCALAPPDATA%\IFSSIM\settings.json
#     AND alongside the .exe (FSDSSettings::AutoLoad search order
#     hits LaunchDir first on Windows, so the colocated copy is the
#     reliable one; the AppData copy is a fallback that matches the
#     Mac semantics).
#   - UECommandLine.txt force-overwrite same defensive reason as Mac:
#     UE's relative -project path breaks if the engine and project
#     live on different drives.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Git Bash on Windows mangles unix-style absolute paths inside argv
# to native executables — RunUAT.bat and its child processes need
# the host conventions intact (drive letters, backslashes) for the
# -project= and -archivedirectory= flags. Pin conversion off so what
# we type is what they see.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

UE_ROOT="${UE_ROOT:-/c/Program Files/Epic Games/UE_5.7}"
RUN_UAT="$UE_ROOT/Engine/Build/BatchFiles/RunUAT.bat"

if [ ! -f "$RUN_UAT" ]; then
    echo "ERROR: RunUAT.bat not found at $RUN_UAT" >&2
    echo "Set UE_ROOT=/path/to/UE_5.7 (drive letter via /c/, /d/, etc.) if installed elsewhere." >&2
    exit 1
fi

# UE5 writes Win64 staged builds under Saved/StagedBuilds/Windows/
# (the directory name comes from the platform's UBT name "Windows",
# not the target name "Win64").
STAGED_DIR="$SCRIPT_DIR/Saved/StagedBuilds/Windows"
TRACKS_SRC="$SCRIPT_DIR/Content/tracks"
TRACKS_DST="$STAGED_DIR/tracks"

# UE produces IFSSIM.exe as the user-facing launcher (proxy) at the
# stage root; the real binary is under IFSSIM/Binaries/Win64/. Mac's
# Sandbox.Server.entitlements doesn't have a Windows analogue.

echo "=== IFSSIM Windows post-build ==="
echo "    UE_ROOT: $UE_ROOT"

# 1. BuildCookRun -------------------------------------------------------
echo "[1/3] Building (Win64 Shipping)…"
# CMD-equivalent path for -project=, since RunUAT.bat is invoked by
# cmd.exe and won't accept /c/-style paths. pwd -W gives us C:/… form.
PROJECT_WIN="$(cygpath -w "$SCRIPT_DIR/IFSSIM.uproject" 2>/dev/null || echo "$SCRIPT_DIR/IFSSIM.uproject")"

"$RUN_UAT" \
    BuildCookRun \
    -project="$PROJECT_WIN" \
    -platform=Win64 \
    -configuration=Shipping \
    -build -cook -stage -pak \
    -iostore -compressed \
    -clientconfig=Shipping \
    -noeditor

# 2. (No codesign step — Windows has no equivalent for the macOS
#    sandbox/entitlements gate. The first launch will trigger
#    Windows Firewall once for port 41451; allow and move on.)

# 3. Copy track CSVs next to the executable
echo "[2/3] Staging track CSVs…"
mkdir -p "$TRACKS_DST"
cp "$TRACKS_SRC/"*.csv "$TRACKS_DST/"
N_TRACKS=$(ls "$TRACKS_DST"/*.csv 2>/dev/null | wc -l | tr -d ' ')
echo "    Tracks staged: $N_TRACKS files"

# 3b. settings.json — see FSDSSettings::AutoLoad() in FSDSSettings.cpp
#     for the search order:
#       1) LaunchDir (next to .exe)
#       2) ProjectDir (resolves correctly on Windows shipping builds)
#       3) FPlatformProcess::UserSettingsDir() + "IFSSIM"
#
#     On Windows, UserSettingsDir() returns %LOCALAPPDATA%\, so #3
#     resolves to %LOCALAPPDATA%\IFSSIM\settings.json. We stage both
#     LaunchDir AND UserSettingsDir copies so the build runs even if
#     a future UE refactor changes which path resolves first.
echo "[3/3] Staging settings.json (LaunchDir + UserSettingsDir)…"
cp "$SCRIPT_DIR/settings.json" "$STAGED_DIR/settings.json"
echo "    LaunchDir copy:       $STAGED_DIR/settings.json"

USER_SETTINGS_DIR="$LOCALAPPDATA/IFSSIM"
# $LOCALAPPDATA on Git Bash is usually C:\Users\<user>\AppData\Local
# in native form; mkdir -p handles either / or \.
if [ -n "$LOCALAPPDATA" ]; then
    mkdir -p "$USER_SETTINGS_DIR"
    cp "$SCRIPT_DIR/settings.json" "$USER_SETTINGS_DIR/settings.json"
    echo "    UserSettingsDir copy: $USER_SETTINGS_DIR/settings.json"

    # Pre-seed the engine's persistent GameUserSettings.ini so the first
    # launch lands in windowed mode rather than the WindowedFullscreen
    # default. Config/DefaultGameUserSettings.ini holds the source of
    # truth; UE5 normally creates the runtime copy on first launch from
    # the project defaults, but we stage it explicitly here so a wiped
    # AppData doesn't temporarily lose the override.
    if [ -f "$SCRIPT_DIR/Config/DefaultGameUserSettings.ini" ]; then
        USER_CFG_DIR="$USER_SETTINGS_DIR/Saved/Config/Windows"
        mkdir -p "$USER_CFG_DIR"
        cp "$SCRIPT_DIR/Config/DefaultGameUserSettings.ini" \
           "$USER_CFG_DIR/GameUserSettings.ini"
        echo "    GameUserSettings.ini: $USER_CFG_DIR/GameUserSettings.ini"
    fi
else
    echo "    WARNING: \$LOCALAPPDATA not set — skipping UserSettingsDir copy"
fi

# 4. Force absolute -project path + windowed + FPS cap in UECommandLine.txt.
#    Same root cause as the macOS comment: UE's BuildCookRun composes a
#    relative path that breaks the first time the engine and project
#    live on different drives. We overwrite it with an absolute path
#    computed from $SCRIPT_DIR.
#
#    FPS cap mirrored from macOS (30 default; override IFSSIM_MAX_FPS).
#
#    Windowed mode on Win64 shipping: -windowed alone is not enough.
#    UE5 still picks the desktop resolution and the resulting window
#    is fullscreen-shaped. -ResX/-ResY forces a concrete size and
#    -ForceRes makes the engine ignore any saved GameUserSettings
#    that might otherwise restore a previous fullscreen value.
#    Override the window size via IFSSIM_WIN_W / IFSSIM_WIN_H.
IFSSIM_MAX_FPS="${IFSSIM_MAX_FPS:-30}"
IFSSIM_WIN_W="${IFSSIM_WIN_W:-1280}"
IFSSIM_WIN_H="${IFSSIM_WIN_H:-720}"
CMDLINE="$STAGED_DIR/UECommandLine.txt"
PROJECT_ABS_WIN="$(cygpath -w "$SCRIPT_DIR/IFSSIM.uproject" 2>/dev/null || echo "$SCRIPT_DIR/IFSSIM.uproject")"
echo "-project=\"$PROJECT_ABS_WIN\" -windowed -ResX=$IFSSIM_WIN_W -ResY=$IFSSIM_WIN_H -ForceRes -ExecCmds=\"t.MaxFPS $IFSSIM_MAX_FPS\" -log" > "$CMDLINE"
echo "    UECommandLine.txt: $(cat "$CMDLINE")"

echo ""
echo "=== Done — distribution in $STAGED_DIR ==="
echo "    Exe:    IFSSIM.exe (root of staged dir)"
echo "    Tracks: tracks/ ($N_TRACKS CSV files)"
echo "    Run:    \"$STAGED_DIR/IFSSIM.exe\""
