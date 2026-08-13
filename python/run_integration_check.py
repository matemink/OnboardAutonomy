"""Run a real UDP message path through the OnboardAutonomy C++ process."""

from __future__ import annotations

import argparse
import json
import selectors
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil
from scenario_runner import SCENARIOS, send_observation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--companion",
        type=Path,
        required=True,
        help="Path to the onboard_autonomy executable",
    )
    parser.add_argument("--port", type=int, default=14650)
    return parser.parse_args()


def wait_for_ready_snapshot(
    process: subprocess.Popen[str],
    timeout: float,
) -> dict[str, object]:
    assert process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if not selector.select(remaining):
            break

        line = process.stdout.readline()
        if not line:
            break

        snapshot = json.loads(line)
        print(json.dumps(snapshot, indent=2))
        if snapshot.get("armable") is True:
            return snapshot

    raise RuntimeError("Companion did not produce an armable snapshot")


def main() -> int:
    args = parse_args()
    process = subprocess.Popen(
        [
            str(args.companion),
            "--udp-port",
            str(args.port),
            "--snapshot-ms",
            "100",
            "--json",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    try:
        connection = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{args.port}",
            source_system=1,
            source_component=1,
        )
        scenario = SCENARIOS["healthy"]
        send_deadline = time.monotonic() + 1.0
        while time.monotonic() < send_deadline:
            send_observation(connection, scenario)
            time.sleep(0.1)

        snapshot = wait_for_ready_snapshot(process, timeout=3.0)
        if snapshot["gps_fix_type"] != 3:
            raise RuntimeError("Expected a 3D GPS fix")
        if snapshot["battery_remaining_pct"] != 88:
            raise RuntimeError("Expected 88% battery remaining")

        print("OnboardAutonomy UDP integration check passed")
        return 0
    finally:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
