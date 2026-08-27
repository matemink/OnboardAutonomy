@echo off
setlocal

title OnboardAutonomy Aerial Tracking Launcher

set "ONBOARD_AUTONOMY_GAZEBO_AERIAL_TARGET=1"
set "ONBOARD_AUTONOMY_AERIAL_OBSERVATION=1"
set "ONBOARD_AUTONOMY_SNAPSHOT_MS=100"
set "ONBOARD_AUTONOMY_DIAGNOSTIC_LOG=.local/logs/aerial-yaw.jsonl"
call "%~dp0StartOnboardAutonomyGazeboDemo.cmd"
