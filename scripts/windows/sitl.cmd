@echo off
setlocal
for %%I in ("%~dp0..\..") do set "PROJECT_ROOT=%%~fI"

start "ArduCopter SITL" wsl.exe -d Ubuntu-24.04 --cd "%PROJECT_ROOT%" -- bash scripts/run_arducopter_sitl.sh
timeout /t 5 /nobreak >nul
start "OnboardAutonomy" wsl.exe -d Ubuntu-24.04 --cd "%PROJECT_ROOT%" -- bash scripts/run_onboard_autonomy_sitl.sh
