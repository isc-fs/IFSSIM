@echo off
setlocal

rem Rebuild Lichtblick from cmd.exe / PowerShell on Windows.
rem This avoids Git Bash/MSYS path rewriting of Linux paths such as
rem /entrypoint.sh, and normalizes the Dockerfile to LF before build.

cd /d "%~dp0\.." || exit /b 1

echo ==^> Normalizing docker\lichtblick\Dockerfile to LF
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$p = Join-Path (Get-Location) 'docker/lichtblick/Dockerfile';" ^
  "$text = [System.IO.File]::ReadAllText($p);" ^
  "$text = $text -replace \"`r`n\", \"`n\";" ^
  "[System.IO.File]::WriteAllText($p, $text, [System.Text.UTF8Encoding]::new($false))"
if errorlevel 1 exit /b %errorlevel%

echo ==^> Stopping Lichtblick
docker compose stop lichtblick

echo ==^> Rebuilding Lichtblick without cache
docker compose build --no-cache lichtblick
if errorlevel 1 exit /b %errorlevel%

echo ==^> Recreating Lichtblick
docker compose up -d --force-recreate lichtblick
if errorlevel 1 exit /b %errorlevel%

echo ==^> Done. Lichtblick should be available at http://localhost:8080
