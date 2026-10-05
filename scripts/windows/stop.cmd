@echo off
setlocal
for %%I in ("%~dp0..\..") do set "PROJECT_ROOT=%%~fI"

title Stop OnboardAutonomy Gazebo Demo

echo Stopping OnboardAutonomy Gazebo demo processes...
wsl.exe -d Ubuntu-24.04 --cd "%PROJECT_ROOT%" -- bash scripts/stop_onboard_autonomy_gazebo.sh

if errorlevel 1 (
    echo Some demo processes could not be stopped.
    echo Check the shutdown details above before restarting the demo.
    pause
    exit /b 1
)

echo Demo stopped. Windows and other WSL work remain running.
timeout /t 2 /nobreak >nul
exit /b 0
