@echo off
setlocal

title OnboardAutonomy Demo Launcher

set "WEATHER_PROFILE=config/onboard_autonomy-gazebo-weather.parm"
if not "%~1"=="" set "WEATHER_PROFILE=%~1"
set "GAZEBO_WORLD=simulation/worlds/camera_observation.sdf"
if not "%~2"=="" set "GAZEBO_WORLD=%~2"
set "INTERACTIVE_VALUE=1"
if "%ONBOARD_AUTONOMY_DIAGNOSTIC_AUTO%"=="1" set "INTERACTIVE_VALUE=0"

echo Weather profile: %WEATHER_PROFILE%
echo Gazebo world: %GAZEBO_WORLD%

echo Checking that the camera demo endpoints are available...
wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_DOWNWARD_CAMERA="%ONBOARD_AUTONOMY_DOWNWARD_CAMERA%" python3 scripts/check_camera_demo_environment.py
if errorlevel 1 (
    echo Close the previous demo before starting another session.
    pause
    exit /b 1
)

start "Gazebo Simulation Server" /min wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_GAZEBO_HEADLESS=1 ONBOARD_AUTONOMY_WEATHER_PROFILE="%WEATHER_PROFILE%" ONBOARD_AUTONOMY_GAZEBO_WORLD="%GAZEBO_WORLD%" bash scripts/run_gazebo_camera_weather.sh
timeout /t 3 /nobreak >nul
start "Gazebo Camera World" wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_GAZEBO_WEATHER=1 ONBOARD_AUTONOMY_WEATHER_PROFILE="%WEATHER_PROFILE%" bash scripts/run_gazebo_gui.sh
timeout /t 4 /nobreak >nul
start "ArduCopter Gazebo SITL" wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_WEATHER_PROFILE="%WEATHER_PROFILE%" bash scripts/run_arducopter_gazebo_weather.sh
timeout /t 4 /nobreak >nul
start "OnboardAutonomy Console" wsl.exe -d Ubuntu-24.04 --cd "%~dp0" -- env ONBOARD_AUTONOMY_DOWNWARD_CAMERA="%ONBOARD_AUTONOMY_DOWNWARD_CAMERA%" ONBOARD_AUTONOMY_BUILD_DIR="%ONBOARD_AUTONOMY_BUILD_DIR%" ONBOARD_AUTONOMY_INTERACTIVE=%INTERACTIVE_VALUE% ONBOARD_AUTONOMY_SNAPSHOT_MS="%ONBOARD_AUTONOMY_SNAPSHOT_MS%" ONBOARD_AUTONOMY_DIAGNOSTIC_LOG="%ONBOARD_AUTONOMY_DIAGNOSTIC_LOG%" ONBOARD_AUTONOMY_WEATHER_PROFILE="%WEATHER_PROFILE%" bash scripts/run_onboard_autonomy_gazebo_weather_vision.sh

echo Waiting for the first camera frame...
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$deadline = (Get-Date).AddSeconds(30); while ((Get-Date) -lt $deadline) { try { $response = Invoke-WebRequest -UseBasicParsing -Uri 'http://localhost:8080/api/frame' -TimeoutSec 1; if ($response.StatusCode -eq 200) { exit 0 } } catch {}; Start-Sleep -Milliseconds 500 }; exit 1"

if errorlevel 1 (
    echo Camera preview did not become available at http://localhost:8080/
    echo Check the OnboardAutonomy Console window for details.
    pause
) else (
    start "" "http://localhost:8080/"
)
