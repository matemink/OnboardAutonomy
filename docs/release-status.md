# Release status

OnboardAutonomy 1.0 is complete as a hardware-backed companion-computer
prototype. Autonomous motion is verified in ArduPilot SITL with Gazebo;
physical Raspberry Pi 5 and Pixhawk 6C testing remains deliberately
observation-only until a legal and safe flight test is possible.

## Delivered

- C++20 runtime with MAVLink UDP, USB, and TELEM2 UART transports.
- Readiness-gated startup, command acknowledgement, safety supervision, and
  companion-link failsafe validation.
- Gazebo camera streaming through GStreamer, AprilTag pose estimation, and
  vision-guided precision landing.
- Forward-camera ONNX detection, temporal target locking, and adaptive yaw-rate
  centering while the vehicle remains in GUIDED hold.
- Raspberry Pi 5 deployment with Camera Module 3 Wide, `systemd`, bounded JSONL
  logging, recovery after camera or serial loss, and runtime profiling.
- Linux tests, Python integration and fault-injection tests, C++ static
  analysis, and native ARM64 CI builds.
- Architecture documentation and two short demonstration videos linked from
  the project README.

## Evidence

- [Precision landing in SITL](evidence/precision-landing-sitl.md)
- [Independent companion-link failsafe](evidence/companion-link-failsafe-sitl.md)
- [Physical AprilTag scale](evidence/physical-apriltag-scale.md)
- [Raspberry Pi runtime profile](evidence/raspberry-pi-runtime-profile.md)
- [Serial recovery](evidence/serial-recovery.md)
- [Pixhawk TELEM2 UART](evidence/uart-hardware.md)

## Current scope

- ArduPilot owns stabilization, state estimation, arming, and low-level flight
  control.
- The forward-object demo receives only camera frames. It does not receive the
  simulated target pose, infer range, pursue a target, or command a standoff
  distance.
- GPS supplies the current navigation estimate. Camera-based target tracking
  does not replace vehicle localization.
- AprilTag precision landing and forward-object yaw tracking are verified in
  simulation; the physical bench verifies the compute, camera, flight
  controller, and transport paths without propellers.

There is no committed future-feature backlog. New work should start from a
measured limitation and a dedicated GitHub issue with explicit acceptance
criteria.
