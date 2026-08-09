# Roadmap

The roadmap is ordered to keep each milestone demonstrable and testable.

## Project direction

The goal is a production-shaped companion-computer prototype combining
C++ and Python, MAVLink, Embedded Linux, ARM deployment, video
processing, hardware interfaces, automated tests, and measurable
performance.

The terminal dashboard, one-command launchers, camera preview, activity
animation, readable reports, and other operator conveniences are
intentional engineering deliverables. They make command flow observable,
experiments repeatable, and hardware failures diagnosable. They are not
considered a deviation from the autonomy work.

The next feature priority is the complete vision-to-guidance vertical
slice. Further interface work should solve a concrete usability or
diagnostic problem rather than delay that slice.

## Repository hygiene

This work can proceed alongside the technical milestones.

- [x] Audit ignored/generated files and remove machine-specific or
  sensitive data from the publishable project.
- [x] Create one honest initial Git snapshot; do not manufacture past
  history.
- [x] Establish a focused-commit workflow for subsequent features and
  tests.
- [x] Publish the repository after a clean build and secret scan.
- [x] Store C++ and Python test reports as CI artifacts and keep
  hardware benchmark evidence in versioned documentation.

## Milestone 1: Telemetry foundation

Status: implemented and verified on Ubuntu 24.04 under WSL2.

- [x] C++20 project and CMake build.
- [x] MAVLink UDP and Linux serial transports.
- [x] Minimum health-message decoder.
- [x] Freshness-aware vehicle state.
- [x] Native tests and Python scenario generator.
- [x] End-to-end Python-to-C++ UDP integration check.
- [x] ARM64 cross-build workflow.

Engineering focus: C++, MAVLink integration, architecture, testing, Git.

## Milestone 2: ArduPilot SITL integration

Status: implemented and verified locally against ArduCopter 4.6.3.

- [x] Install Ubuntu with WSL2.
- [x] Build and run ArduCopter SITL.
- [x] Route MAVLink telemetry to OnboardAutonomy.
- [x] Let OnboardAutonomy configure its required message rates and verify
  `COMMAND_ACK` responses.
- [x] Start and stop the complete SITL stack from Python.
- [x] Assert healthy telemetry, command acknowledgements, and process
  cleanup in a repeatable smoke test.
- [x] Inject heartbeat loss and assert stale connection handling.
- [x] Inject simulated GPS loss and assert readiness handling.
- [x] Drain a simulated battery and assert the 20% readiness threshold.
- [x] Trigger a real ArduPilot PreArm failure and assert its full
  MAVLink-to-domain path.
- [x] Visualize outbound commands, inbound acknowledgements, and
  telemetry confirmations in a bounded live terminal view.
- [x] Exercise guarded command sequences and manual LAND from the live
  terminal while keeping serial hardware blocked.
- [x] Verify local-NED routing, RTL, acknowledgements, state confirmation,
  and MAVLink `LANDING_TARGET` encoding before connecting vision.
- [x] Replace scripted product demos with a startup controller and
  continuous autonomy runtime driven by current world state.
- [x] Validate the ArduPilot companion-heartbeat LAND policy before
  autonomous startup and prove independent link-loss recovery in SITL.

Engineering focus: Python, Embedded Linux, ArduPilot, integration
tests, observability, and developer tooling.

## Milestone 3: Video pipeline

- [x] Pin the official ArduPilot Gazebo plugin and add reproducible
  install and launch scripts.
- [x] Install Gazebo Harmonic and verify WSLg/Ogre2 rendering.
- [x] Run the official Iris world with ArduCopter and OnboardAutonomy.
- [x] Execute and verify an automated GUIDED takeoff, hold, and landing
  from OnboardAutonomy.
- [x] Stream the simulated camera through GStreamer and verify decoded
  frames with a bounded smoke test.
- [x] Add a project-owned Gazebo landing camera and AprilTag pad, analytic
  calibration, RTP/H.264 ingestion, and geometry/texture regression tests.
- [x] Receive Raspberry Pi Camera Module 3 Wide frames through
  `rpicam`/libcamera on Raspberry Pi 5.
- [x] Benchmark raw YUV420 frame cadence, estimated drops, process CPU,
  RSS, and sensor metadata with machine-readable reports.
- [x] Measure sensor-to-application frame latency inside the C++ runtime
  receiver using per-frame `FrameWallClock` metadata.
- [x] Run Camera Module 3 and Pixhawk telemetry concurrently at 30 FPS
  without blocking the application loop.
- [x] Reconnect both camera adapters after process or stream loss and prove
  GStreamer recovery without restarting the companion process.

Engineering focus: GStreamer, video streaming, profiling, ARM.

Experiment note: a project-owned decorative airfield was implemented,
tested, and rolled back because the Gazebo GUI displayed only its
background instead of the scene entities. The server-side world,
camera, physics, and automated flight continued to run, but the
user-visible result did not satisfy the acceptance criterion. See
`docs/learning/14-gazebo-airfield-failed-experiment.uk.md`.

Recovery acceptance evidence: the same process consumed 11 frames, exposed a
2-second stream stall and reconnect state, then consumed 22 frames after the
Gazebo producer restarted. See `docs/evidence/camera-recovery.md`.

## Milestone 4: Vision and precision landing

- [x] Integrate the official AprilTag 3 detector, typed pixel-space
  observations, processing metrics, and a generated-marker unit test.
- [x] Validate a physical `tagStandard41h12` marker through the Camera
  Module 3 pipeline at 301/301 detected frames over a 10-second window.
- [x] Calibrate Camera Module 3 Wide and store reproducible camera
  intrinsics and distortion coefficients. The physical 640x480 capture
  accepted 39/40 views with 0.6728 px RMS reprojection error; the full
  quality-gated result is stored in
  `config/camera-module-3-wide-640x480.json`.
- [x] Estimate the AprilTag 3D pose and expose position, orientation,
  confidence, and observation freshness through typed state models.
- [ ] Validate metric pose scale against a physically measured printed
  target and camera-to-target distance.
- [x] Transform camera coordinates into the MAVLink/body coordinate
  frame and validate axes, signs, units, and timestamps with tests.
- [x] Reject invalid, low-confidence, and stale measurements without adding
  moving-frame translation lag to the default guidance path.
- [x] Replace the synthetic SITL target provider with real AprilTag
  `LANDING_TARGET` observations.
- [x] Verify simulated AprilTag acquisition and metric pose after takeoff
  over the project landing pad.
- [x] Validate the complete precision-landing sequence in Gazebo.
- [x] Test lost-target, reacquisition, outlier, and noisy-observation
  behavior.
- [x] Separate world state, decision intent, safety supervision, flight
  startup, and continuous runtime into tested application components.
- [x] Stop stale target guidance immediately and request a bounded fallback
  LAND when vision remains unavailable.
- [x] Latch vision guidance off for terminal descent only after stable
  low-altitude alignment, and verify the handoff in runtime telemetry.

Production simulation acceptance evidence: ten consecutive 3 m offset
flights, 174-177 valid body-FRD `LANDING_TARGET` messages per run, accepted
GUIDED/ARM/TAKEOFF/LAND, automatic DISARMED, zero large center crossings,
and 0.000 m median/worst/population-standard-deviation horizontal error at
MAVLink telemetry resolution. See
`docs/evidence/precision-landing-sitl.md`.

Independent safety acceptance evidence: ArduPilot entered LAND 3.237 seconds
after a controlled companion-link cut, emitted `GCS Failsafe`, and disarmed;
the companion tlog contained no LAND or RTL command. See
`docs/evidence/companion-link-failsafe-sitl.md`.

Engineering focus: computer vision, guidance integration, algorithms.

## Milestone 5: Raspberry Pi deployment

- [x] Add a reproducible ARM64 Release package candidate.
- [x] Add read-only hardware diagnostics, deterministic Pixhawk serial
  discovery, and JSONL telemetry logging.
- [x] Provision Raspberry Pi OS Lite 64-bit and verify headless
  public-key SSH access.
- [x] Deploy and run the cross-built ARM64 package on Raspberry Pi 5.
- [x] Connect Pixhawk 6C over USB and capture real MAVLink telemetry.
- [x] Read ArduPilot `BATT_ARM_VOLT` and verify battery readiness
  against a real pre-arm failure.
- [x] Expose the six acknowledged telemetry-rate requests and verify
  documented `AUTOPILOT_VERSION` metadata on the real Pixhawk 6C.
- [x] Resolve every pinned ArduPilot bootloader board ID through the
  full official table while preserving ambiguous aliases.
- [x] Visualize actual complete TX/RX MAVLink frames with bounded
  freshness and a tested live activity pulse.
- [ ] Build natively on Raspberry Pi 5 as a toolchain comparison.
- [x] Connect and identify Camera Module 3 Wide (Sony IMX708).
- [x] Deploy the C++ camera receiver and verify 30.013 FPS, zero
  processing drops, and approximately 10 ms latency on Raspberry Pi 5.
- [x] Add a read-only browser camera preview from the same Y plane used
  by AprilTag, with target ID and corner overlay.
- [x] Add a hardened non-root `systemd` service and bounded streaming
  JSONL log rotation while preserving live output in `journald`.
- [x] Add a bounded whole-process-group ARM profiler with CPU, RSS,
  temperature, throttling, and machine-readable reports.
- [ ] Run the profiler on Raspberry Pi 5, establish an evidence-backed
  baseline, and optimize only measured bottlenecks.
- [x] Recover camera processes/streams and Linux serial sessions without
  restarting the companion process; validate serial recovery with a real PTY
  replacement and repeat telemetry setup after heartbeat recovery.
- [ ] Repeat the established-link USB unplug/replug acceptance test with the
  physical Raspberry Pi 5 and Pixhawk 6C.

Engineering focus: Embedded Linux, ARM Cortex, target deployment.

## Milestone 6: Hardware interfaces and delivery

- [ ] Move from USB to the documented Pixhawk TELEM/UART connection.
- [ ] Add one documented I2C or GPIO peripheral if useful.
- [ ] Add a reproducible deployment image or Yocto proof of concept.
- [ ] Record a short architecture and demonstration video.
- [ ] Publish diagrams, test evidence, performance numbers, and
  limitations.

Engineering focus: UART, I2C/GPIO, deployment, technical communication.
