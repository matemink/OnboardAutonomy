# OnboardAutonomy

[![CI](https://github.com/matemink/OnboardAutonomy/actions/workflows/ci.yml/badge.svg)](https://github.com/matemink/OnboardAutonomy/actions/workflows/ci.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20x86__64%20%7C%20ARM64-FCC624.svg)](https://www.raspberrypi.com/)

OnboardAutonomy is a C++20 companion-computer runtime for ArduPilot UAVs. It
combines MAVLink, onboard vision, safety supervision, and precision landing.
The same application runs in Gazebo or on a Raspberry Pi 5 with a Pixhawk 6C.

## Demo

- **Vision-guided autonomous landing** (`39 s`) - takeoff, target tracking,
  guidance, and precision landing.
  [![Watch on YouTube](https://img.shields.io/badge/YouTube-Watch-FF0000?logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=rsuRYYDfZZI)
- **Hurricane-force wind stress test** (`36 s`) - the same autonomy flow under
  severe simulated gusts.
  [![Watch on YouTube](https://img.shields.io/badge/YouTube-Watch-FF0000?logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=eqdRw3oofTI)

## System overview

```mermaid
flowchart LR
    Camera["Camera Module 3"]

    subgraph Pi["Raspberry Pi 5"]
        Runtime["OnboardAutonomy"]
    end

    subgraph FC["Pixhawk 6C"]
        Firmware["ArduPilot firmware"]
    end

    Camera --> Runtime
    Runtime --> Firmware
    Runtime --> SITL["ArduPilot SITL"]
    SITL <--> Gazebo["Gazebo Harmonic"]

    classDef hardware fill:#FFF3C4,stroke:#B7791F,color:#3D2C00,stroke-width:2px
    classDef software fill:#DCEBFF,stroke:#2563EB,color:#0F2A52,stroke-width:2px
    class Camera hardware
    class Runtime,Firmware,SITL,Gazebo software
    style Pi fill:#FFF8DE,stroke:#B7791F,stroke-width:2px,color:#3D2C00
    style FC fill:#FFF8DE,stroke:#B7791F,stroke-width:2px,color:#3D2C00
```

ArduPilot owns stabilization and flight control. OnboardAutonomy turns camera
and telemetry data into supervised guidance for either SITL or a real Pixhawk.

## Highlights

- MAVLink 2 telemetry and acknowledged commands over SITL UDP or Linux
  USB/UART serial transport.
- Readiness-gated takeoff and AprilTag landing with target-loss and link-loss
  fallbacks.
- Dual-camera YUV420/OpenCV processing for AprilTag landing and forward ONNX
  aerial-object detection.
- SITL-only forward-object lock confirms spatial and temporal continuity,
  then applies bounded yaw centering while the vehicle remains in GUIDED hold.
- Automatic serial and camera recovery after disconnects or process stalls.
- Motion is SITL-gated; physical endpoints remain observation-only. CI covers
  C++ tests, Python integration and fault injection, plus a native ARM64 build.

## Built and tested with

- **Simulation:** ArduCopter SITL, Gazebo Harmonic, OpenCV, and GStreamer.
- **Hardware:** Raspberry Pi 5, Camera Module 3 Wide, and Pixhawk 6C over USB
  and TELEM2 UART.
- **Evidence:** repeatable precision-landing runs, companion-link failsafe
  injection, camera and serial recovery, and ARM runtime profiling.

## Explore

- [Architecture](docs/architecture.md)
- [Run the Gazebo demo](docs/simulation.md)
- [Hardware bench and evidence](docs/raspberry-pi-5-bench.md)
