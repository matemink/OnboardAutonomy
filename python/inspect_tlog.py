#!/usr/bin/env python3
"""Print health and automated-flight evidence from a MAVLink telemetry log."""

from __future__ import annotations

import argparse
from pathlib import Path

from pymavlink import mavutil


def sensor_names(mask: int) -> list[str]:
    names: list[str] = []
    prefixes = (
        "MAV_SYS_STATUS_SENSOR_",
        "MAV_SYS_STATUS_",
    )

    for name, value in vars(mavutil.mavlink).items():
        matching_prefix = next(
            (prefix for prefix in prefixes if name.startswith(prefix)),
            None,
        )
        if (
            matching_prefix is not None
            and isinstance(value, int)
            and 0 < value <= 0xFFFFFFFF
            and value & (value - 1) == 0
            and mask & value
        ):
            names.append(name.removeprefix(matching_prefix))

    return sorted(set(names))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tlog", type=Path)
    args = parser.parse_args()

    connection = mavutil.mavlink_connection(str(args.tlog))
    latest = None
    count = 0
    prearm_messages: list[tuple[int, int, str]] = []
    flight_command_names = {
        mavutil.mavlink.MAV_CMD_DO_SET_MODE: "SET_GUIDED",
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "ARM",
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
        mavutil.mavlink.MAV_CMD_NAV_LAND: "LAND",
    }
    flight_commands: list[tuple[int, int, str, float]] = []
    flight_acks: list[tuple[int, int, str, int]] = []
    maximum_relative_altitude_m: float | None = None
    armed_transitions: list[bool] = []
    last_armed: bool | None = None
    observed_modes: set[int] = set()
    final_local_position: tuple[float, float, float] | None = None
    autopilot_statuses: list[str] = []

    while message := connection.recv_match():
        message_type = message.get_type()
        if message_type == "SYS_STATUS":
            latest = message
            count += 1
        elif message_type == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            text = str(text).rstrip("\0")
            if text.startswith("PreArm:"):
                prearm_messages.append(
                    (
                        message.get_srcSystem(),
                        message.get_srcComponent(),
                        text,
                    )
                )
            elif text.startswith("PrecLand:"):
                autopilot_statuses.append(text)
        elif message_type == "COMMAND_LONG":
            command = int(message.command)
            if command in flight_command_names:
                flight_commands.append(
                    (
                        message.get_srcSystem(),
                        message.get_srcComponent(),
                        flight_command_names[command],
                        float(message.param7),
                    )
                )
        elif message_type == "COMMAND_ACK":
            command = int(message.command)
            if command in flight_command_names:
                flight_acks.append(
                    (
                        message.get_srcSystem(),
                        message.get_srcComponent(),
                        flight_command_names[command],
                        int(message.result),
                    )
                )
        elif message_type == "GLOBAL_POSITION_INT":
            altitude_m = float(message.relative_alt) / 1000.0
            if (
                maximum_relative_altitude_m is None
                or altitude_m > maximum_relative_altitude_m
            ):
                maximum_relative_altitude_m = altitude_m
        elif message_type == "LOCAL_POSITION_NED":
            final_local_position = (
                float(message.x),
                float(message.y),
                float(message.z),
            )
        elif (
            message_type == "HEARTBEAT"
            and int(message.autopilot)
            != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            observed_modes.add(int(message.custom_mode))
            armed = bool(
                int(message.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            if armed != last_armed:
                armed_transitions.append(armed)
                last_armed = armed

    if latest is None:
        print("No SYS_STATUS messages found")
        return 1

    present = int(latest.onboard_control_sensors_present)
    enabled = int(latest.onboard_control_sensors_enabled)
    healthy = int(latest.onboard_control_sensors_health)
    unhealthy = enabled & ~healthy

    print(f"SYS_STATUS messages: {count}")
    print(f"present:   0x{present:08X}")
    print(f"enabled:   0x{enabled:08X}")
    print(f"healthy:   0x{healthy:08X}")
    print(f"unhealthy: 0x{unhealthy:08X}")
    print("unhealthy sensors:")
    for name in sensor_names(unhealthy):
        print(f"  - {name}")

    print(f"PreArm messages: {len(prearm_messages)}")
    for system_id, component_id, text in prearm_messages:
        print(f"  - sysid={system_id} compid={component_id}: {text}")

    print("Automated flight evidence:")
    print(
        "  maximum relative altitude: "
        + (
            f"{maximum_relative_altitude_m:.2f} m"
            if maximum_relative_altitude_m is not None
            else "not observed"
        )
    )
    print(
        "  armed transitions: "
        + (
            " -> ".join(
                "ARMED" if armed else "DISARMED"
                for armed in armed_transitions
            )
            if armed_transitions
            else "not observed"
        )
    )
    print(
        "  custom modes: "
        + (
            ", ".join(str(mode) for mode in sorted(observed_modes))
            if observed_modes
            else "not observed"
        )
    )
    print("  commands:")
    for system_id, component_id, name, param7 in flight_commands:
        suffix = f" altitude={param7:.1f}m" if name == "TAKEOFF" else ""
        print(
            f"    - sysid={system_id} compid={component_id}: "
            f"{name}{suffix}"
        )
    print("  acknowledgements:")
    for system_id, component_id, name, result in flight_acks:
        print(
            f"    - sysid={system_id} compid={component_id}: "
            f"{name} result={result}"
        )
    print("Position evidence:")
    if final_local_position is not None:
        horizontal_error = (
            final_local_position[0] ** 2
            + final_local_position[1] ** 2
        ) ** 0.5
        print(
            "  final local N/E/D: "
            f"{final_local_position[0]:.3f} / "
            f"{final_local_position[1]:.3f} / "
            f"{final_local_position[2]:.3f} m"
        )
        print(f"  final horizontal error: {horizontal_error:.3f} m")
    print("  ArduPilot statuses:")
    for status in dict.fromkeys(autopilot_statuses):
        print(f"    - {status}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
