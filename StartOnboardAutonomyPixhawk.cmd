@echo off
setlocal

if exist "%~dp0OnboardAutonomyPiLocal.cmd" call "%~dp0OnboardAutonomyPiLocal.cmd"

if not defined ONBOARD_AUTONOMY_PI_HOST set "ONBOARD_AUTONOMY_PI_HOST=companionpi.local"
if not defined ONBOARD_AUTONOMY_PI_USER set "ONBOARD_AUTONOMY_PI_USER=companion"
if not defined ONBOARD_AUTONOMY_SSH_KEY set "ONBOARD_AUTONOMY_SSH_KEY=%USERPROFILE%\.ssh\onboard_autonomy_ed25519"
if not defined ONBOARD_AUTONOMY_REMOTE_ROOT set "ONBOARD_AUTONOMY_REMOTE_ROOT=/home/%ONBOARD_AUTONOMY_PI_USER%/onboard_autonomy-pi5"

start "OnboardAutonomy - Raspberry Pi 5 and Pixhawk 6C" ssh.exe ^
  -t ^
  -i "%ONBOARD_AUTONOMY_SSH_KEY%" ^
  -o StrictHostKeyChecking=accept-new ^
  %ONBOARD_AUTONOMY_PI_USER%@%ONBOARD_AUTONOMY_PI_HOST% ^
  "env ONBOARD_AUTONOMY_SERIAL='%ONBOARD_AUTONOMY_SERIAL%' '%ONBOARD_AUTONOMY_REMOTE_ROOT%/bin/run_onboard_autonomy_pi.sh'"

timeout /t 3 /nobreak >nul
start "" "http://%ONBOARD_AUTONOMY_PI_HOST%:8080/"
