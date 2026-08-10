# Raspberry Pi 5 complete-runtime profile

Verified on 2026-08-10 using the ARM64 binary with SHA-256
`426f943e786aa142f92c28e89656076153f7752dd47a3adeb483164568b31c0e`
and GNU build ID `a8b8406efac2fd7dbbef33beb9ba7e358f013c66`. The final package
containing the identical binary has SHA-256
`cb7a40bc405272c8e75cee71b2b72432734d7485ed4408815a046c0a554cf4a4`.

## Workload

The 60-second process-group profile ran the complete hardware runtime on a
Raspberry Pi 5:

- real Pixhawk 6C telemetry through `/dev/ttyAMA0` at 57600 baud;
- Camera Module 3 Wide at 640x480 and 30 FPS;
- AprilTag 3 detection with the physically measured 86 mm target size;
- JSON snapshot generation and bounded JSONL log rotation;
- camera preview disabled so the baseline measures onboard autonomy rather
  than an optional browser convenience.

Serial hardware stayed observe-only for the entire run.

## Result

Result: **PASS**

| Metric | Value |
| --- | ---: |
| Architecture | `aarch64` |
| Kernel | `6.18.34+rpt-rpi-2712` |
| Sampled duration | 60.00 / 60.00 s |
| Average process-group CPU | 109.76% |
| p95 process-group CPU | 120.35% |
| Peak process-group RSS | 224.19 MiB |
| Peak process count | 4 |
| Maximum SoC temperature | 63.90 C |
| Throttling bit union | `0x0` |

Linux reports 100% as one fully occupied CPU core, so 109.76% is aggregate
use of about 1.10 cores rather than 109.76% of the entire four-core Pi. The
runtime completed all acceptance checks: ARM64 architecture, full sample
window, process samples present, expected shutdown, throttling telemetry
available, and no throttling observed.

The final runtime snapshot also reported `telemetry_setup=active (6/6)`,
camera streaming at 30.013 FPS, 1762 processed frames, zero processing drops,
10.084 ms average frame latency, 11.743 ms maximum latency, and
`motion_commands_allowed=false`.

## Claim boundary

This is a one-minute, propeller-free bench baseline, not a thermal-soak or
flight-duration endurance test. It identifies no measured bottleneck that
blocks the 30 FPS workload, but longer runs and the final airframe enclosure
could produce different thermal behavior.

The profiler generates `report.json`, `report.md`, process samples, and
runtime logs under
`~/.local/state/onboard_autonomy/profiles/<run-id>/`. Generated copies stay
under the ignored local `artifacts/` directory.
