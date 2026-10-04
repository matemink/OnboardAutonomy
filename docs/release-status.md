# Release status

OnboardAutonomy is a C++20 telemetry and camera observation prototype for a
Raspberry Pi 5 / Pixhawk 6C bench and ArduPilot SITL. The build version is
recorded in `CMakeLists.txt`.

## Current scope

- MAVLink UDP and Linux USB/UART, controller identity filtering, telemetry
  freshness, bounded stream setup, and automatic reconnect.
- Independent camera capture, process recovery, browser preview, and JSONL logs.
- Read-only companion-link failsafe validation and controller metadata.
- Generic image-processing and calibration utilities for independent experiments.
- Linux C++ / Python tests, static analysis, and native ARM64 CI.

The companion sends heartbeat, telemetry configuration, and read-only information
requests. Specialized target aircraft, the two-aircraft pursuit scenario,
flight-start and tracking controllers, and flight-command encoders have been
removed. Marker-guided landing was removed earlier. Interactive input only exits
the application. The camera demo does not load a detector or start a flight.

## Evidence

- [Serial recovery](evidence/serial-recovery.md)
- [Pixhawk TELEM2 UART](evidence/uart-hardware.md)
- [Historical Raspberry Pi runtime profile](evidence/raspberry-pi-runtime-profile.md)

Evidence records the binary and workload actually measured. Historical profiles
and logs may contain fields and detectors removed from the current revision;
they are not current performance claims. Current snapshots omit `flight_startup`,
`autonomy`, `motion_commands_allowed`, and `aerial_tracking_available`.
