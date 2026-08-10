@echo off

call "%~dp0StartOnboardAutonomyGazeboDemo.cmd" ^
  config/onboard_autonomy-gazebo-weather.parm ^
  simulation/worlds/apriltag_showcase.sdf
