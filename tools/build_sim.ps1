<#
.SYNOPSIS
    Windows entry point for building IFSSIM from source.

.DESCRIPTION
    The real work lives in build_sim.sh, which needs bash. On Windows
    bash arrives with Git for Windows — so if you have no git yet, you
    cannot run the shell script at all. This bootstrap solves that
    chicken-and-egg: it installs Git if missing, then hands over.

    Right-click this file and pick "Run with PowerShell", or:
        powershell -ExecutionPolicy Bypass -File tools\build_sim.ps1

.PARAMETER Check
    Check prerequisites and change nothing.

.PARAMETER Yes
    Never ask; install whatever is missing and buildable.

.PARAMETER Fast
    Skip building the ~1 GB distributable zip.
#>
param(
    [switch]$Check,
    [switch]$Yes,
    [switch]$Fast
)

$ErrorActionPreference = 'Stop'

function Say  ($m) { Write-Host "  $m" }
function Step ($m) { Write-Host "`n$m" -ForegroundColor Cyan }
function Good ($m) { Write-Host "  OK    $m" -ForegroundColor Green }
function Bad  ($m) { Write-Host "  ERROR $m" -ForegroundColor Red }

Step "Windows bootstrap"

# --- git (and therefore bash) ------------------------------------------
$git = Get-Command git -ErrorAction SilentlyContinue
if (-not $git) {
    Bad "Git for Windows is not installed."
    Say "It supplies both git and the bash shell the build script runs in."

    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        Say ""
        Say "winget is unavailable too, so this has to be done by hand:"
        Say "  Download and install: https://git-scm.com/download/win"
        Say "  Accept every default. Then re-run this script."
        exit 2
    }

    $answer = if ($Yes) { 'y' } else { Read-Host "  Install Git for Windows now? [y/N]" }
    if ($answer -notmatch '^[Yy]') {
        Say "Nothing installed. Get it from https://git-scm.com/download/win"
        exit 2
    }

    winget install --id Git.Git -e --accept-source-agreements --accept-package-agreements
    # winget updates the machine PATH, but not this already-running
    # process, so re-read it rather than making the user reopen a window.
    $env:Path = [Environment]::GetEnvironmentVariable('Path','Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path','User')

    if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
        Bad "Git still not on PATH. Close this window, open a new one, and re-run."
        exit 2
    }
    Good "Installed Git for Windows"
} else {
    Good "git — $((git --version) -replace 'git version ','')"
}

# --- locate bash -------------------------------------------------------
# Deliberately NOT `where.exe bash`: on most Windows boxes that finds
# the WSL stub at System32\bash.exe, which runs inside the Linux VM
# where the Windows UE install and drive letters do not exist.
$bash = $null
$candidates = @(
    "$env:ProgramFiles\Git\bin\bash.exe",
    "${env:ProgramFiles(x86)}\Git\bin\bash.exe",
    "$env:LOCALAPPDATA\Programs\Git\bin\bash.exe"
)
foreach ($c in $candidates) { if (Test-Path $c) { $bash = $c; break } }

if (-not $bash) {
    # Fall back to deriving it from wherever git actually landed.
    $gitDir = Split-Path (Split-Path (Get-Command git).Source -Parent) -Parent
    $guess  = Join-Path $gitDir 'bin\bash.exe'
    if (Test-Path $guess) { $bash = $guess }
}

if (-not $bash) {
    Bad "Could not find Git Bash (bash.exe)."
    Say "Reinstall Git for Windows from https://git-scm.com/download/win"
    exit 2
}
Good "bash — $bash"

# --- hand over ---------------------------------------------------------
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$shPath    = Join-Path $scriptDir 'build_sim.sh'
if (-not (Test-Path $shPath)) {
    Bad "build_sim.sh is missing from $scriptDir"
    exit 2
}

# Git Bash wants a POSIX path: C:\a\b -> /c/a/b
$unix  = $shPath -replace '\\','/'                 # C:/a/b
$posix = '/' + $unix.Substring(0,1).ToLower() + $unix.Substring(2)

$fwd = @()
if ($Check) { $fwd += '--check' }
if ($Yes)   { $fwd += '--yes' }
if ($Fast)  { $fwd += '--fast' }

Step "Handing over to build_sim.sh"
& $bash -lc "'$posix' $($fwd -join ' ')"
exit $LASTEXITCODE
