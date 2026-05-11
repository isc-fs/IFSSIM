<#
.SYNOPSIS
    Build the IFSSIM Windows shipping artefact.

.DESCRIPTION
    Mirrors package_mac.sh / package_linux.sh for Win64. Requires:
      - UE5 5.7 installed (default lookup: C:\Program Files\Epic Games\UE_5.7)
      - Visual Studio 2022 with C++ workload
      - The project's content + plugin source in this repo
    UE5 cannot cross-compile to Windows from macOS or Linux, so this
    script genuinely needs a Windows host.

    Output:
      Saved/StagedBuilds/Windows/IFSSIM/Binaries/Win64/IFSSIM-Win64-Shipping.exe
    Plus a sibling tracks/ directory next to the .exe (FSDSConeSpawner
    / loadTrack RPC search there).

.PARAMETER Ue5Root
    Path to a UE_5.7 install. Defaults to %UE5_ROOT% env var if set,
    otherwise C:\Program Files\Epic Games\UE_5.7.

.EXAMPLE
    PS> ./package_windows.ps1
    PS> ./package_windows.ps1 -Ue5Root "D:\Epic\UE_5.7"

.NOTES
    Run from a "x64 Native Tools Command Prompt for VS 2022" shell
    if MSVC isn't on the default PATH. Otherwise PowerShell from any
    location is fine.
#>
[CmdletBinding()]
param(
    [string]$Ue5Root = $(if ($env:UE5_ROOT) { $env:UE5_ROOT } else { "C:\Program Files\Epic Games\UE_5.7" })
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RunUAT = Join-Path $Ue5Root "Engine\Build\BatchFiles\RunUAT.bat"

if (-not (Test-Path $RunUAT)) {
    Write-Error "UE5 not found at '$Ue5Root'. Install UE 5.7 or set -Ue5Root / UE5_ROOT."
    exit 1
}

$Project = Join-Path $ScriptDir "IFSSIM.uproject"
$StagedRoot = Join-Path $ScriptDir "Saved\StagedBuilds\Windows"
$AppExe = Join-Path $StagedRoot "IFSSIM\Binaries\Win64\IFSSIM-Win64-Shipping.exe"
$TracksSrc = Join-Path $ScriptDir "Content\tracks"
$TracksDst = Join-Path $StagedRoot "tracks"
$CmdLineFile = Join-Path $StagedRoot "UECommandLine.txt"

Write-Host "=== IFSSIM Windows post-build ===" -ForegroundColor Cyan
Write-Host "  UE5 root:  $Ue5Root"

# 1. BuildCookRun
Write-Host "[1/3] Building..." -ForegroundColor Yellow
& $RunUAT BuildCookRun `
    -project="$Project" `
    -platform=Win64 `
    -configuration=Shipping `
    -build -cook -stage -pak `
    -iostore -compressed `
    -clientconfig=Shipping `
    -noeditor `
    -utf8output
if ($LASTEXITCODE -ne 0) {
    Write-Error "RunUAT BuildCookRun failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}

# 2. Stage track CSVs sibling to the .exe (FSDSConeSpawner /
#    loadTrack RPC look there at runtime — same convention as Mac/Linux).
Write-Host "[2/3] Staging track CSVs..." -ForegroundColor Yellow
New-Item -ItemType Directory -Force -Path $TracksDst | Out-Null
Copy-Item -Path (Join-Path $TracksSrc "*.csv") -Destination $TracksDst -Force
$Count = (Get-ChildItem -Path $TracksDst -Filter "*.csv").Count
Write-Host "  Tracks staged: $Count files"

# 3a. Copy settings.json to the user-settings dir so FSDSSettings::AutoLoad
#     finds it at runtime. On Windows UE5 reads from
#     %LOCALAPPDATA%\Epic\IFSSIM\settings.json (per
#     FPlatformProcess::UserSettingsDir() on Windows).
$UserSettingsDir = Join-Path $env:LOCALAPPDATA "Epic\IFSSIM"
New-Item -ItemType Directory -Force -Path $UserSettingsDir | Out-Null
Copy-Item -Path (Join-Path $ScriptDir "settings.json") -Destination $UserSettingsDir -Force
Write-Host "  settings.json staged -> $UserSettingsDir\settings.json"

# 3b. Force windowed mode + 60 FPS cap in UECommandLine.txt.
#     BuildCookRun overwrites this file every cook; same logic as
#     package_mac.sh / package_linux.sh.
Write-Host "[3/3] Patching UECommandLine.txt..." -ForegroundColor Yellow
if (Test-Path $CmdLineFile) {
    $line = Get-Content $CmdLineFile -Raw
    if ($line -notmatch '-windowed') { $line = $line.TrimEnd() + " -windowed" }
    if ($line -notmatch 't\.MaxFPS')  { $line = $line.TrimEnd() + ' -ExecCmds="t.MaxFPS 60"' }
    Set-Content -Path $CmdLineFile -Value $line -NoNewline
} else {
    Set-Content -Path $CmdLineFile -Value '-project="../../../IFSSIM/IFSSIM.uproject" -windowed -ExecCmds="t.MaxFPS 60"' -NoNewline
}
Write-Host "  UECommandLine.txt: $(Get-Content $CmdLineFile -Raw)"

Write-Host ""
Write-Host "=== Done — distribution in Saved\StagedBuilds\Windows\ ===" -ForegroundColor Green
Write-Host "  Exe:     $AppExe"
Write-Host "  Tracks:  tracks\ ($Count CSV files)"
Write-Host ""
Write-Host "Run with:"
Write-Host "  cd Saved\StagedBuilds\Windows ; .\IFSSIM\Binaries\Win64\IFSSIM-Win64-Shipping.exe"
