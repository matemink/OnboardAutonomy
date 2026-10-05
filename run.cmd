@echo off
setlocal

if "%~1"=="" goto usage
if not "%~3"=="" goto usage
if /i "%~1"=="gazebo" goto gazebo
if not "%~2"=="" goto usage
if /i "%~1"=="sitl" goto sitl
if /i "%~1"=="pi" goto pi
if /i "%~1"=="stop" goto stop
if /i "%~1"=="--help" goto help
if /i "%~1"=="-h" goto help
goto usage

:gazebo
if not "%~2"=="" if /i not "%~2"=="showcase" goto usage
call "%~dp0scripts\windows\gazebo.cmd" "%~2"
exit /b %errorlevel%

:sitl
call "%~dp0scripts\windows\sitl.cmd"
exit /b %errorlevel%

:pi
call "%~dp0scripts\windows\pi.cmd"
exit /b %errorlevel%

:stop
call "%~dp0scripts\windows\stop.cmd"
exit /b %errorlevel%

:usage
echo Unknown or missing command. Run: run.cmd --help
exit /b 2

:help
echo Usage: run.cmd gazebo [showcase] ^| sitl ^| pi ^| stop
echo Gazebo opens the simulator, telemetry console, and camera preview.
echo SITL opens telemetry without Gazebo. Pi connects to the hardware bench.
echo Stop closes only this checkout's tagged Gazebo demo processes.
exit /b 0
