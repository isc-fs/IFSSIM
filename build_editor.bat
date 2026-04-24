@echo off
REM Build and launch the IFSSIM editor.
REM
REM Project path is derived from %~dp0 (the directory this script lives
REM in), so the repo can be cloned anywhere. UE5 root defaults to the
REM standard Epic install path and can be overridden via the UE5_ROOT
REM environment variable — e.g.:
REM     set UE5_ROOT=D:\Games\Epic\UE_5.7
REM     build_editor.bat
setlocal

if not defined UE5_ROOT set "UE5_ROOT=C:\Program Files\Epic Games\UE_5.7"
set "PROJECT=%~dp0IFSSIM.uproject"

if not exist "%UE5_ROOT%\Engine\Build\BatchFiles\Build.bat" (
    echo [error] UE5 not found at "%UE5_ROOT%".
    echo         Set UE5_ROOT to your UE_5.7 install path and retry.
    exit /b 1
)

echo Building IFSSIMEditor from "%PROJECT%"...
"%UE5_ROOT%\Engine\Build\BatchFiles\Build.bat" IFSSIMEditor Win64 Development -Project="%PROJECT%" -WaitMutex -FromMsBuild
if errorlevel 1 (
    echo.
    echo [error] Build failed.
    exit /b %errorlevel%
)

echo.
echo Build complete. Launching editor...
"%UE5_ROOT%\Engine\Binaries\Win64\UnrealEditor.exe" "%PROJECT%"

endlocal
