@echo off
setlocal

title Stop OnboardAutonomy Gazebo Demo

echo Stopping OnboardAutonomy Gazebo demo processes...
wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- bash scripts/stop_onboard_autonomy_gazebo.sh

if errorlevel 1 (
    echo Some demo processes could not be stopped.
    echo If Gazebo still cannot reopen, run: wsl --shutdown
    pause
    exit /b 1
)

echo Demo stopped. Windows and other WSL work remain running.
timeout /t 2 /nobreak >nul
