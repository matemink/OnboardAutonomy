# Physical AprilTag scale evidence

Verified on 2026-08-10 with Raspberry Pi 5, Raspberry Pi Camera Module 3
Wide, and the calibrated 640x480 camera profile.

## Setup

- Detector: AprilTag 3, `tagStandard41h12`, tag ID 0.
- Printed tag black-square width: physically measured as `86 mm`.
- Runtime configuration: `ONBOARD_AUTONOMY_APRILTAG_SIZE_MM=86`.
- Camera-to-target separations: physically set to `500 mm` and `300 mm`.
- Sampling: the final ten one-second JSON snapshots after the target was
  stationary at each distance.
- Safety: serial hardware remained observe-only and
  `motion_commands_allowed=false`.

The measured 86 mm black square is intentionally used instead of the nominal
90 mm print setting. AprilTag pose scale is proportional to the supplied tag
size, so using the intended print dimension would have introduced a known
scale error before testing the estimator.

## Result

| Physical separation | Samples | Median forward | Mean forward | Population stddev | Median error | Relative error |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 500 mm | 10 | 500.75 mm | 500.91 mm | 1.72 mm | +0.75 mm | +0.15% |
| 300 mm | 10 | 304.85 mm | 303.31 mm | 5.56 mm | +4.85 mm | +1.62% |

The minimum AprilTag decision margins were 134.88 and 127.55 respectively,
with no corrected bits and a reported object-space error of zero. The worst
median scale error across the two measured distances was 1.62%.

## Claim boundary

This closes the physical metric-scale bench check for the printed target,
camera calibration, detector, and pose estimator. It does not validate
airframe camera extrinsics, vibration, outdoor lighting, motion blur, or
closed-loop physical flight. Those remain outside the propeller-free bench
and must not be inferred from this result.

The source captures are generated as `artifacts/pose-scale-500mm.jsonl` and
`artifacts/pose-scale-300mm.jsonl`; the repository keeps this compact,
reviewable evidence rather than committing large runtime logs.
