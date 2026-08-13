#!/usr/bin/env python3
"""Launch and verify the complete OnboardAutonomy ArduCopter SITL stack."""

from __future__ import annotations

import argparse
import os
from datetime import datetime, timezone
from pathlib import Path

from sitl_harness import (
    HarnessPaths,
    log_tails,
    run_smoke_test,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    home = Path.home()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ardupilot-dir",
        type=Path,
        default=home / "src/ardupilot-Copter-4.6.3",
    )
    parser.add_argument(
        "--mavproxy",
        type=Path,
        default=home / "venv-ardupilot/bin/mavproxy.py",
    )
    parser.add_argument(
        "--companion",
        type=Path,
        default=home / "build/onboard_autonomy/onboard_autonomy",
    )
    parser.add_argument(
        "--artifacts-root",
        type=Path,
        default=PROJECT_ROOT / "artifacts/sitl-smoke",
    )
    parser.add_argument(
        "--scenario",
        choices=(
            "healthy",
            "heartbeat-loss",
            "gps-loss",
            "low-battery",
            "prearm",
        ),
        default="healthy",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    artifacts = args.artifacts_root / f"{run_id}-{os.getpid()}"
    paths = HarnessPaths(
        ardupilot_dir=args.ardupilot_dir.expanduser().resolve(),
        mavproxy=args.mavproxy.expanduser().resolve(),
        companion=args.companion.expanduser().resolve(),
    )

    print("OnboardAutonomy SITL smoke test")
    print(f"Scenario: {args.scenario}")
    print(f"Artifacts: {artifacts}")

    try:
        result = run_smoke_test(
            paths=paths,
            artifacts=artifacts,
            timeout=args.timeout,
            scenario=args.scenario,
        )
    except Exception as error:  # noqa: BLE001 - CLI reports scenario failures.
        print(f"\nFAILED: {error}")
        if artifacts.exists():
            print(log_tails(artifacts))
        return 1

    print("\nPASSED")
    print("Readiness: healthy GPS, battery, and system baseline reached")
    print(
        "Protocol: "
        f"{len(result.evidence.interval_requests)} interval requests, "
        f"{result.evidence.accepted_ack_count} accepted ACKs, "
        f"{result.evidence.companion_heartbeat_count} companion heartbeats"
    )
    if result.failure_snapshot is not None:
        failure_names = {
            "heartbeat-loss": "heartbeat loss detected",
            "gps-loss": "GPS loss detected",
            "low-battery": "low battery detected",
            "prearm": "ArduPilot PreArm warning detected",
        }
        print(f"Failure injection: {failure_names[args.scenario]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
