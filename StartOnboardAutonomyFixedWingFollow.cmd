@echo off
setlocal

title OnboardAutonomy Fixed-Wing Follow Lab

set "WEATHER_PROFILE=config/onboard_autonomy-gazebo-weather.parm"
if not "%~1"=="" set "WEATHER_PROFILE=%~1"

echo Preparing the pinned Skywalker X8 ArduPlane SITL...
wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- bash scripts/prepare_skywalker_x8_sitl.sh
if errorlevel 1 (
    echo Could not prepare Skywalker X8 SITL.
    pause
    exit /b 1
)

echo Stopping any previous OnboardAutonomy Gazebo demo...
wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- bash scripts/stop_onboard_autonomy_gazebo.sh
if errorlevel 1 (
    echo Could not clean up the previous demo processes.
    pause
    exit /b 1
)
powershell.exe -NoProfile -Command "Start-Sleep -Seconds 1"

start "Fixed-Wing Gazebo Server" /min wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_GAZEBO_HEADLESS=1 ONBOARD_AUTONOMY_WEATHER_PROFILE="%WEATHER_PROFILE%" bash scripts/run_gazebo_fixed_wing_follow_weather.sh
powershell.exe -NoProfile -Command "Start-Sleep -Seconds 3"
start "Fixed-Wing Follow World" wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_GAZEBO_WEATHER=1 ONBOARD_AUTONOMY_WEATHER_PROFILE="%WEATHER_PROFILE%" bash scripts/run_gazebo_fixed_wing_gui.sh
powershell.exe -NoProfile -Command "Start-Sleep -Seconds 4"
start "Skywalker X8 ArduPlane SITL" wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_WEATHER_PROFILE="%WEATHER_PROFILE%" bash scripts/run_arduplane_skywalker_x8_weather.sh

echo Fixed-wing lab started.
echo Skywalker X8 is a temporary ArduPlane integration model, not the final high-speed platform.
