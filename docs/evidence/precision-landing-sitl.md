# Precision landing SITL evidence

Verified on 2026-08-09 with ArduCopter 4.6.3, Gazebo Sim 8.14.0, and the
project-owned `apriltag_landing` world.

## Acceptance path

```text
Gazebo landing camera
  -> RTP/H.264
  -> GStreamer I420 frame
  -> AprilTag pose in camera-optical coordinates
  -> freshness-aware confirmed track
  -> configured camera-to-body-FRD transform
  -> MAVLink LANDING_TARGET at 10 Hz
  -> ArduCopter precision-land backend
  -> touchdown and DISARMED
```

The production runtime waited for readiness, entered GUIDED, armed, took off
to 8.04 m, and handed control from `FlightStartupController` to
`AutonomyRuntime`. It acquired tag ID 0 directly from current camera state,
requested LAND only after the target warmup, and exited with code zero after
telemetry-confirmed automatic disarm. No numbered scenario, fixed route, or
Python-issued flight command participated in the run.

## Protocol evidence

The MAVProxy tlog was inspected with `python/inspect_tlog.py`:

| Check | Observed |
| --- | --- |
| Arm state | `DISARMED -> ARMED -> DISARMED` |
| Modes | `STABILIZE (0), GUIDED (4), LAND (9)` |
| Flight command ACKs | GUIDED, ARM, TAKEOFF, and LAND accepted |
| Consecutive flights | 10/10 |
| `LANDING_TARGET` count | 174-177 per flight |
| MAVLink frame | 12, `MAV_FRAME_BODY_FRD` |
| `position_valid` | 1 |
| Maximum relative altitude | `8.04 m` |
| Final local N/E/D | `0.000 / 0.000 / -0.200 m` |
| Final horizontal error | median `0.000 m`, worst `0.000 m` |
| Population standard deviation | `0.000 m` |
| Large center crossings | 0 in every flight |
| Terminal handoff | Confirmed in every flight |

## Target loss behavior

Every live run exercised the expected close-range target loss.
OnboardAutonomy first observed at least 0.5 seconds of alignment within 0.25 m
below 1.5 m. When the target disappeared, it latched terminal descent, stopped
all further vision corrections, and ignored any close-range reacquisition.
Unit tests separately verify interrupted warmup, reacquisition, optional
smoothing, expiry, confidence rejection, corrected-bit rejection, protection
against switching between tag IDs, and unsafe target loss without alignment.
A production-runtime test also verifies the five-second fallback LAND when
vision is unavailable before LAND starts.

The two-metre marker no longer fits fully in the 640x480 image below roughly
2 m. With the simulation profile's `PLND_STRICT=0`, ArduPilot completed the
remaining vertical descent as a normal landing. The profile also selects the
documented RawSensor estimator with `PLND_EST_TYPE=0`: the synthetic pinhole
camera is deterministic, while the previous companion EMA plus ArduPilot
Kalman estimator introduced moving-frame lag and repeated overshoot. Neither
parameter is evidence that the physical camera geometry, latency, or noise has
been validated.

## Remaining hardware gate

Before real-aircraft guidance is enabled, repeat the metric scale check with
the printed marker at measured distances, record the physical camera mount
extrinsics, and review the ArduPilot precision-landing parameters. Serial
hardware motion remains blocked by the application safety policy.

## Reproduction

The complete run, cleanup, final JSON assertion, and independent tlog checks
are automated by:

```bash
source ~/venv-ardupilot/bin/activate
python python/autonomy_sitl_acceptance.py
python python/autonomy_sitl_acceptance.py --runs 10
```

The recorded acceptance summary was:

```text
PASSED
Path: readiness -> GUIDED -> ARM -> TAKEOFF -> vision LAND
Evidence: 177 LANDING_TARGET, 8.04 m max, 0.000 m error
Accuracy: median 0.000 m, worst 0.000 m, stddev 0.000 m
```
