@echo off
setlocal

title OnboardAutonomy Aerial Tracking Launcher

call "%~dp0StartOnboardAutonomyGazeboDemo.cmd"
if errorlevel 1 exit /b 1

echo Adding the scripted fixed-wing target...
wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- bash scripts/spawn_gazebo_aerial_target.sh
if errorlevel 1 (
    echo Could not add the scripted aerial target.
    pause
    exit /b 1
)

echo Aerial tracking scene is ready.
timeout /t 2 /nobreak >nul
