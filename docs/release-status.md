# Release status

OnboardAutonomy is a companion-computer prototype with a Raspberry Pi 5 /
Pixhawk 6C observation bench and ArduPilot SITL integration. Physical testing
uses no automated motion. The build version is recorded in `CMakeLists.txt`.

## Current scope

- C++20 runtime, MAVLink UDP and Linux USB/UART transports.
- Controller identity filtering, freshness-aware telemetry, acknowledged
  commands, and automatic reconnect.
- Independent camera capture, process recovery, browser preview, and JSONL logs.
- Forward-camera OpenCV DNN observations and the existing simulation runtime.
- Explicit SITL motion gate and ArduPilot-owned companion-link failsafe checks.
- Linux C++ / Python tests, static analysis, and native ARM64 CI.

Marker detection and marker-guided landing have been removed, including their
CLI, protocol output, dependencies, assets, and acceptance harness. Ordinary
ArduPilot LAND and RTL remain flight-controller operations.

## Evidence

- [Companion-link failsafe](evidence/companion-link-failsafe-sitl.md)
- [Serial recovery](evidence/serial-recovery.md)
- [Pixhawk TELEM2 UART](evidence/uart-hardware.md)
- [Historical Raspberry Pi runtime profile](evidence/raspberry-pi-runtime-profile.md)

Evidence documents record the setup they actually measured. The historical
runtime profile includes a detector removed from the current workload; it
requires a new measurement before being used as a current performance claim.
Uncommitted development work and unrun flight experiments are not release evidence.
