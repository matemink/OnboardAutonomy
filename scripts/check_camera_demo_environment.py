#!/usr/bin/env python3
"""Refuse a second camera demo without stopping unrelated processes."""

import socket
import subprocess
import sys

# Ports owned by the default camera observation world and its runtime.
ENDPOINTS = (
    (socket.SOCK_STREAM, 5760, "ArduCopter SITL"),
    (socket.SOCK_STREAM, 8080, "camera preview"),
    (socket.SOCK_DGRAM, 9002, "Gazebo flight dynamics"),
    (socket.SOCK_DGRAM, 14550, "companion telemetry"),
    (socket.SOCK_DGRAM, 5601, "downward camera"),
    (socket.SOCK_DGRAM, 5602, "forward camera"),
)


def check_environment() -> list[str]:
    conflicts = []
    result = subprocess.run(
        ["pgrep", "-f", "^gz sim "], capture_output=True, check=False
    )
    if result.returncode == 0:
        conflicts.append("a Gazebo simulation is already running")
    elif result.returncode != 1:
        conflicts.append("could not check running Gazebo simulations")
    for socket_type, port, label in ENDPOINTS:
        with socket.socket(socket.AF_INET, socket_type) as probe:
            try:
                probe.bind(("0.0.0.0", port))
            except OSError:
                conflicts.append(f"{label}: port {port} is unavailable")
    return conflicts


def main() -> int:
    conflicts = check_environment()
    for conflict in conflicts:
        print(f"Camera demo cannot start: {conflict}", file=sys.stderr)
    return int(bool(conflicts))


if __name__ == "__main__":
    sys.exit(main())
