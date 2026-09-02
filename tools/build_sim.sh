#!/usr/bin/env bash
#
# build_sim.sh — one command, from a fresh clone to a runnable sim.
#
# Works on macOS, Windows (inside Git Bash) and Linux. Detects the
# machine, checks every prerequisite, installs the ones it safely can,
# repairs the repo (LFS + submodules), then runs the right
# package_*.sh for the platform.
#
#   ./tools/build_sim.sh              # check, fix, build
#   ./tools/build_sim.sh --check      # check only, change nothing
#   ./tools/build_sim.sh --yes        # never ask, just do it
#   ./tools/build_sim.sh --fast       # skip the ~1 GB release zip
#
# One thing this script cannot do for you: install Unreal Engine 5.7.
# It needs an Epic Games account and a licence you have to accept in
# person. The script detects whether it is there and tells you exactly
# what to click if it is not.
#
# Exit codes:  0 ok   1 build failed   2 a prerequisite is missing
set -uo pipefail

# ---------------------------------------------------------------- setup

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSUME_YES=0
CHECK_ONLY=0
FAST=0
MISSING=0          # count of prerequisites we could not satisfy
FIXED=()           # human-readable list of things we repaired

for arg in "$@"; do
    case "$arg" in
        --yes|-y)        ASSUME_YES=1 ;;
        --check|--dry-run) CHECK_ONLY=1 ;;
        --fast)          FAST=1 ;;
        --help|-h)
            sed -n '3,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "unknown option: $arg (try --help)" >&2; exit 2 ;;
    esac
done

# Colour, but only when a human is watching a capable terminal.
if [ -t 1 ] && [ "$(tput colors 2>/dev/null || echo 0)" -ge 8 ]; then
    BOLD=$(tput bold); DIM=$(tput dim); RESET=$(tput sgr0)
    RED=$(tput setaf 1); GREEN=$(tput setaf 2)
    YELLOW=$(tput setaf 3); BLUE=$(tput setaf 4)
else
    BOLD=""; DIM=""; RESET=""; RED=""; GREEN=""; YELLOW=""; BLUE=""
fi

ok()    { printf "  ${GREEN}OK${RESET}    %s\n" "$1"; }
warn()  { printf "  ${YELLOW}WARN${RESET}  %s\n" "$1"; }
bad()   { printf "  ${RED}MISSING${RESET} %s\n" "$1"; MISSING=$((MISSING+1)); }
info()  { printf "  ${DIM}%s${RESET}\n" "$1"; }
step()  { printf "\n${BOLD}${BLUE}%s${RESET}\n" "$1"; }
fixed() { printf "  ${GREEN}FIXED${RESET} %s\n" "$1"; FIXED+=("$1"); }

# Ask a yes/no question. Returns 0 for yes. Auto-yes with --yes, and
# auto-no when nothing is attached to stdin (CI, piped installs).
confirm() {
    [ "$ASSUME_YES" = "1" ] && return 0
    [ -t 0 ] || return 1
    local reply
    printf "  ${BOLD}%s${RESET} [y/N] " "$1"
    read -r reply
    [[ "$reply" =~ ^[Yy] ]]
}

# ------------------------------------------------------- 1. the machine

step "1/6  What machine is this?"

RAW_OS="$(uname -s)"
ARCH="$(uname -m)"
case "$RAW_OS" in
    Darwin)                 OS=mac ;;
    Linux)                  OS=linux ;;
    MINGW*|MSYS*|CYGWIN*)   OS=windows ;;
    *)
        echo "Unsupported system: $RAW_OS" >&2
        echo "IFSSIM builds on macOS, Windows (Git Bash) and Linux." >&2
        exit 2 ;;
esac

case "$OS" in
    mac)     PRETTY="macOS $(sw_vers -productVersion 2>/dev/null || echo '?') ($ARCH)" ;;
    windows) PRETTY="Windows ($ARCH, Git Bash)" ;;
    linux)   PRETTY="Linux ($ARCH)" ;;
esac
ok "$PRETTY"

# The engine wants a lot of room: a cold cook writes Intermediate/,
# DerivedDataCache/ and the staged build. 40 GB is the number that
# stops people discovering the problem 40 minutes in.
DISK_FREE_GB=$(df -Pk "$REPO_ROOT" 2>/dev/null | awk 'NR==2 {print int($4/1048576)}')
if [ -n "${DISK_FREE_GB:-}" ]; then
    if [ "$DISK_FREE_GB" -ge 40 ]; then
        ok "Disk space: ${DISK_FREE_GB} GB free"
    elif [ "$DISK_FREE_GB" -ge 15 ]; then
        warn "Disk space: only ${DISK_FREE_GB} GB free"
        info "A cold build wants ~40 GB (Intermediate + DerivedDataCache + staged build)."
        info "It may still work if you have built before, but do not count on it."
    else
        bad "Disk space: ${DISK_FREE_GB} GB free — need ~40 GB for a cold build"
        info "Free some space, then run this again."
    fi
fi

# ------------------------------------------------ 2. package managers

step "2/6  Package manager"

# Every auto-install below goes through the platform's own manager, so
# find (or offer to install) it first.
PKG=""
case "$OS" in
    mac)
        if command -v brew >/dev/null 2>&1; then
            PKG=brew; ok "Homebrew — $(command -v brew)"
        else
            bad "Homebrew is not installed"
            info "Homebrew is how this script installs git-lfs on macOS."
            if [ "$CHECK_ONLY" = "0" ] && confirm "Install Homebrew now? (downloads from brew.sh)"; then
                /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" || true
                # A fresh install is not on PATH until the shell is re-inited.
                for candidate in /opt/homebrew/bin/brew /usr/local/bin/brew; do
                    [ -x "$candidate" ] && eval "$("$candidate" shellenv)" && break
                done
                if command -v brew >/dev/null 2>&1; then
                    PKG=brew; MISSING=$((MISSING-1)); fixed "Installed Homebrew"
                fi
            else
                info "Install it yourself: https://brew.sh"
            fi
        fi ;;
    windows)
        if command -v winget >/dev/null 2>&1; then
            PKG=winget; ok "winget — $(command -v winget)"
        else
            warn "winget not found (Windows 10 older than 1809, or App Installer missing)"
            info "Anything missing below has to be installed by hand. Get winget from"
            info "the Microsoft Store — search 'App Installer'."
        fi ;;
    linux)
        for m in apt-get dnf pacman zypper; do
            if command -v "$m" >/dev/null 2>&1; then PKG="$m"; break; fi
        done
        if [ -n "$PKG" ]; then ok "Package manager — $PKG"
        else warn "No known package manager found; install missing tools by hand"; fi ;;
esac

# Install one package, using whatever manager this platform has.
# Returns non-zero if it could not.
pkg_install() {
    local brew_name="$1" apt_name="$2" winget_id="$3"
    case "$PKG" in
        brew)    brew install "$brew_name" ;;
        apt-get) sudo apt-get update -qq && sudo apt-get install -y "$apt_name" ;;
        dnf)     sudo dnf install -y "$apt_name" ;;
        pacman)  sudo pacman -S --noconfirm "$apt_name" ;;
        zypper)  sudo zypper install -y "$apt_name" ;;
        winget)  winget install --id "$winget_id" -e --accept-source-agreements --accept-package-agreements ;;
        *)       return 1 ;;
    esac
}

# ------------------------------------------------------- 3. build tools

step "3/6  Build tools"

# --- git ---------------------------------------------------------------
if command -v git >/dev/null 2>&1; then
    ok "git — $(git --version | awk '{print $3}')"
else
    # You cannot actually reach this on Windows: no git means no Git
    # Bash means this script never started. Kept for macOS/Linux.
    bad "git is not installed"
    if [ "$CHECK_ONLY" = "0" ] && [ -n "$PKG" ] && confirm "Install git now?"; then
        pkg_install git git Git.Git && command -v git >/dev/null 2>&1 \
            && { MISSING=$((MISSING-1)); fixed "Installed git"; }
    fi
fi

# --- git-lfs -----------------------------------------------------------
# Not optional. Without it the cone meshes and the LiDAR encoder
# material arrive as 130-byte text pointers and the cook fails in a way
# that looks like a corrupt asset rather than a missing tool.
if command -v git-lfs >/dev/null 2>&1 || git lfs version >/dev/null 2>&1; then
    ok "git-lfs — $(git lfs version 2>/dev/null | awk '{print $1}')"
else
    bad "git-lfs is not installed (binary assets will not download)"
    if [ "$CHECK_ONLY" = "0" ] && [ -n "$PKG" ] && confirm "Install git-lfs now?"; then
        pkg_install git-lfs git-lfs GitHub.GitLFS
        if git lfs version >/dev/null 2>&1; then
            MISSING=$((MISSING-1)); fixed "Installed git-lfs"
        fi
    fi
fi

# --- the platform compiler --------------------------------------------
case "$OS" in
    mac)
        # UBT needs the full Xcode, not just the command line tools —
        # it shells out to xcodebuild and needs a signing identity.
        if xcode-select -p >/dev/null 2>&1; then
            XC_PATH="$(xcode-select -p)"
            if [ -d "$XC_PATH/../.." ] && command -v xcodebuild >/dev/null 2>&1 \
               && xcodebuild -version >/dev/null 2>&1; then
                ok "Xcode — $(xcodebuild -version | head -1)"
            else
                bad "Only the Command Line Tools are installed; UBT needs full Xcode"
                info "Install Xcode from the App Store, then run:"
                info "  sudo xcode-select -s /Applications/Xcode.app/Contents/Developer"
            fi
        else
            bad "No Xcode toolchain"
            info "Install Xcode from the App Store (free, ~10 GB), open it once to"
            info "accept the licence, then re-run this script."
            if [ "$CHECK_ONLY" = "0" ] && confirm "Install the Command Line Tools now? (not enough on its own, but a start)"; then
                xcode-select --install || true
            fi
        fi ;;
    windows)
        # UE needs the MSVC toolchain. Look for any VS 2022 install via
        # the standard locator before deciding it is absent.
        VSWHERE="/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"
        if [ -f "$VSWHERE" ] && "$VSWHERE" -latest -products '*' \
                -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 \
                -property installationPath >/dev/null 2>&1 \
             && [ -n "$("$VSWHERE" -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>/dev/null)" ]; then
            ok "Visual Studio with C++ tools — $("$VSWHERE" -latest -property displayName 2>/dev/null | tr -d '\r')"
        else
            bad "Visual Studio 2022 with the C++ workload was not found"
            info "UE cannot compile the plugin without MSVC."
            if [ "$CHECK_ONLY" = "0" ] && [ "$PKG" = "winget" ] \
               && confirm "Install VS 2022 Build Tools + C++ workload now? (~8 GB, several minutes)"; then
                winget install --id Microsoft.VisualStudio.2022.BuildTools -e \
                    --accept-source-agreements --accept-package-agreements \
                    --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --includeRecommended" \
                    && { MISSING=$((MISSING-1)); fixed "Installed VS 2022 Build Tools"; }
            else
                info "Or install Visual Studio 2022 Community by hand and tick"
                info "'Game development with C++': https://visualstudio.microsoft.com/"
            fi
        fi ;;
    linux)
        if command -v clang >/dev/null 2>&1; then
            ok "clang — $(clang --version | head -1 | awk '{print $NF}')"
        else
            warn "clang not found — the engine ships its own toolchain, so this is usually fine"
        fi ;;
esac

# ------------------------------------------------------- 4. the engine

step "4/6  Unreal Engine 5.7"

# The project pins its engine in IFSSIM.uproject. Read it rather than
# hardcoding, so this keeps working after an engine bump.
WANT_UE="$(grep -o '"EngineAssociation"[^,]*' "$REPO_ROOT/IFSSIM.uproject" 2>/dev/null \
           | sed 's/.*"\([0-9.]*\)".*/\1/')"
WANT_UE="${WANT_UE:-5.7}"

# RunUAT is the thing we actually need; its presence is the real test.
uat_for() {
    if [ "$OS" = "windows" ]; then echo "$1/Engine/Build/BatchFiles/RunUAT.bat"
    else echo "$1/Engine/Build/BatchFiles/RunUAT.sh"; fi
}

UE_FOUND=""
# Anything the user already exported wins, then the per-platform
# defaults the package_*.sh scripts use, then the usual alternatives.
CANDIDATES=()
[ -n "${UE_ROOT:-}"  ] && CANDIDATES+=("$UE_ROOT")
[ -n "${UE5_ROOT:-}" ] && CANDIDATES+=("$UE5_ROOT")
case "$OS" in
    mac)
        CANDIDATES+=("/Users/Shared/Epic Games/UE_$WANT_UE"
                     "/Applications/Epic Games/UE_$WANT_UE"
                     "$HOME/Epic Games/UE_$WANT_UE") ;;
    windows)
        CANDIDATES+=("/c/Program Files/Epic Games/UE_$WANT_UE"
                     "/d/Program Files/Epic Games/UE_$WANT_UE"
                     "/c/Epic Games/UE_$WANT_UE"
                     "/d/Epic Games/UE_$WANT_UE") ;;
    linux)
        CANDIDATES+=("/opt/UE_$WANT_UE"
                     "$HOME/UnrealEngine"
                     "$HOME/UE_$WANT_UE"
                     "/opt/UnrealEngine") ;;
esac

for c in "${CANDIDATES[@]}"; do
    if [ -f "$(uat_for "$c")" ]; then UE_FOUND="$c"; break; fi
done

if [ -n "$UE_FOUND" ]; then
    ok "Unreal Engine — $UE_FOUND"
    export UE_ROOT="$UE_FOUND"
    export UE5_ROOT="$UE_FOUND"     # package_linux.sh reads this name
else
    bad "Unreal Engine $WANT_UE not found"
    info "Looked in:"
    for c in "${CANDIDATES[@]}"; do info "    $c"; done
    echo ""
    info "${BOLD}This is the one thing the script cannot install for you.${RESET}"
    info "The engine is ~40 GB and needs an Epic Games account and a licence"
    info "you have to accept yourself."
    echo ""
    case "$OS" in
        mac|windows)
            info "  1. Get the Epic Games Launcher:  https://www.epicgames.com/store/download"
            info "  2. Sign in, open the 'Unreal Engine' tab, then 'Library'"
            info "  3. Click '+' next to ENGINE VERSIONS, pick $WANT_UE, Install"
            info "  4. Re-run this script — it will find it automatically"
            [ "$OS" = "windows" ] && info "     (If you installed it somewhere unusual: UE_ROOT=/c/path/to/UE_$WANT_UE $0)" ;;
        linux)
            info "  Epic ships no Linux binary — the engine must be built from source:"
            info "    1. Link your GitHub account to Epic:  https://www.unrealengine.com/ue-on-github"
            info "    2. git clone -b 5.7 https://github.com/EpicGames/UnrealEngine"
            info "    3. cd UnrealEngine && ./Setup.sh && ./GenerateProjectFiles.sh && make"
            info "       (about an hour on 16 cores)"
            info "    4. UE_ROOT=/path/to/UnrealEngine $0" ;;
    esac
fi

# -------------------------------------------------- 5. repo is healthy

step "5/6  Repository"

cd "$REPO_ROOT" || exit 1

if [ ! -d .git ]; then
    warn "Not a git checkout — skipping LFS and submodule repair"
else
    # LFS assets. `git lfs ls-files` lists what should be here; a file
    # that still starts with the pointer header never got fetched.
    if git lfs version >/dev/null 2>&1; then
        STUBS=0
        while IFS= read -r f; do
            [ -f "$f" ] || continue
            if head -c 40 "$f" 2>/dev/null | grep -q "version https://git-lfs"; then
                STUBS=$((STUBS+1))
            fi
        done < <(git lfs ls-files -n 2>/dev/null)

        TOTAL_LFS=$(git lfs ls-files 2>/dev/null | wc -l | tr -d ' ')
        if [ "$STUBS" -gt 0 ]; then
            warn "$STUBS of $TOTAL_LFS LFS files are unfetched pointers"
            if [ "$CHECK_ONLY" = "0" ]; then
                info "Downloading binary assets (~500 MB, one time)…"
                git lfs install --local >/dev/null 2>&1
                if git lfs pull; then fixed "Pulled $STUBS LFS assets"
                else bad "git lfs pull failed — check your network and GitHub access"; fi
            fi
        else
            ok "LFS assets — $TOTAL_LFS files present"
        fi
    fi

    # Submodules. pipeline/ is the autonomy stack; without it the sim
    # still builds, but there is no autonomy to talk to.
    if [ -f .gitmodules ]; then
        UNINIT=$(git submodule status 2>/dev/null | grep -c '^-' || true)
        if [ "$UNINIT" -gt 0 ]; then
            warn "$UNINIT submodule(s) not initialised"
            if [ "$CHECK_ONLY" = "0" ]; then
                if git submodule update --init --recursive; then
                    fixed "Initialised $UNINIT submodule(s)"
                else
                    warn "Submodule init failed — the sim will build, but there is no autonomy stack"
                    info "pipeline/ is a private repo; check your GitHub access."
                fi
            fi
        else
            ok "Submodules — $(git submodule status | wc -l | tr -d ' ') initialised"
        fi
    fi
fi

VERSION="$(grep -E '^ProjectVersion=' Config/DefaultGame.ini 2>/dev/null \
           | head -1 | cut -d= -f2 | tr -d '\r\n ')"
[ -n "$VERSION" ] && ok "Project version — $VERSION"

# ------------------------------------------------------------ 6. build

step "6/6  Build"

if [ "$MISSING" -gt 0 ]; then
    printf "\n${RED}${BOLD}Cannot build yet — %d prerequisite(s) missing.${RESET}\n" "$MISSING"
    echo "Fix the MISSING lines above and run this script again."
    exit 2
fi

if [ "$CHECK_ONLY" = "1" ]; then
    printf "\n${GREEN}${BOLD}All prerequisites satisfied.${RESET}  Run without --check to build.\n"
    exit 0
fi

case "$OS" in
    mac)     SCRIPT=./package_mac.sh;      LABEL="macOS" ;;
    windows) SCRIPT="bash package_windows.sh"; LABEL="Windows" ;;
    linux)   SCRIPT=./package_linux.sh;    LABEL="Linux" ;;
esac

echo "  Running $SCRIPT"
info "First build takes about 5 minutes; later ones about 1 minute."
info "Long silences are normal — the engine is cooking content."
echo ""

[ "$FAST" = "1" ] && export SKIP_ARCHIVE=1

START=$(date +%s)
if ! $SCRIPT; then
    printf "\n${RED}${BOLD}Build failed.${RESET}\n"
    echo "The engine's own error is above. Common causes:"
    echo "  - 'Access to the path IFSSIM.exe is denied' — the sim is still running; close it."
    echo "  - Out of disk space mid-cook — see the disk line at the top."
    echo "  - Missing LFS assets — run: git lfs pull"
    exit 1
fi
ELAPSED=$(( $(date +%s) - START ))

# ------------------------------------------------------------- results

case "$OS" in
    mac)     STAGED="Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app"; RUN="open $STAGED" ;;
    windows) STAGED="Saved/StagedBuilds/Windows/IFSSIM.exe";           RUN="Double-click $STAGED" ;;
    linux)   STAGED="Saved/StagedBuilds/Linux/IFSSIM.sh";              RUN="./$STAGED" ;;
esac

printf "\n${GREEN}${BOLD}Done in %dm %ds.${RESET}\n" $((ELAPSED/60)) $((ELAPSED%60))
if [ ${#FIXED[@]} -gt 0 ]; then
    echo ""
    echo "Repaired along the way:"
    for f in "${FIXED[@]}"; do echo "  - $f"; done
fi
echo ""
echo "  Sim:  $STAGED"
[ "$FAST" = "0" ] && [ -d dist ] && echo "  Zip:  $(ls -1 dist/*.zip 2>/dev/null | tail -1)"
echo ""
echo "${BOLD}Start it:${RESET}  $RUN"
echo ""
echo "The sim opens on an empty map — that is expected. Tracks are loaded"
echo "from Mission Control; see docs/SETUP.md step 4 to bring up Docker."
