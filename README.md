# OnboardAutonomy

[![CI](https://github.com/matemink/OnboardAutonomy/actions/workflows/ci.yml/badge.svg)](https://github.com/matemink/OnboardAutonomy/actions/workflows/ci.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20x86__64%20%7C%20ARM64-FCC624.svg)](https://www.raspberrypi.com/)

OnboardAutonomy is a C++20 companion-computer runtime for ArduPilot UAVs.
It brings together MAVLink telemetry, camera capture, operator diagnostics,
and fault recovery. The physical Raspberry Pi 5 / Pixhawk 6C bench runs in
observation mode. Automated motion is restricted to explicitly configured SITL.

## Runtime

- MAVLink 2 over UDP or Linux USB/UART, with controller identity filtering,
  telemetry freshness, command acknowledgements, and bounded retries.
- Independent camera streams through GStreamer or `rpicam-vid`, with automatic
  recovery after producer failure or stalled frames.
- Forward-camera OpenCV DNN / YOLOX processing and temporal observation tracking
  in simulation.
- Console status, JSONL diagnostics, and a browser camera preview.
- ArduPilot-owned companion-link failsafe validation and explicit separation
  between simulation and physical endpoints.

ArduPilot owns stabilization, navigation, and flight control. The companion
runtime observes its telemetry and runs the configured simulation workflow.

```mermaid
flowchart LR
    Cameras --> Runtime[OnboardAutonomy]
    Pixhawk[Pixhawk / ArduPilot] <--> Runtime
    SITL[ArduPilot SITL] <--> Runtime
    Gazebo --> Cameras
    Gazebo <--> SITL
    Runtime --> Console
    Runtime --> Diagnostics[JSONL / camera preview]
```

## Build and check

The verified development environment is Ubuntu 24.04 / WSL2; CI also builds
natively on ARM64. Follow the [development runbook](docs/development.md) for
system dependencies, then:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure
PYTHONPATH=python python3 -m unittest discover -s python/tests -v
ruff check python scripts
```

CMake fetches pinned dependencies. ONNX model weights are downloaded separately
with `scripts/download_yolox_model.sh` and are not committed to the repository.

## Explore

- [Architecture and package responsibilities](docs/architecture.md)
- [Gazebo camera simulation](docs/simulation.md)
- [Raspberry Pi 5 hardware bench](docs/raspberry-pi-5-bench.md)
- [Release status and evidence boundaries](docs/release-status.md)

Historical hardware measurements identify their tested binary and workload;
they do not automatically describe the current revision. CI covers C++ tests,
Python integration and fault injection, static analysis, and native ARM64 builds.
