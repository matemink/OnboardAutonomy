"""Generate deterministic MAVLink health scenarios for OnboardAutonomy."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Scenario:
    gps_fix: int
    satellites: int
    battery_percent: int
    prearm_warning: str | None = None


SCENARIOS = {
    "healthy": Scenario(gps_fix=3, satellites=14, battery_percent=88),
    "no-gps": Scenario(gps_fix=1, satellites=2, battery_percent=88),
    "low-battery": Scenario(gps_fix=3, satellites=14, battery_percent=12),
    "prearm": Scenario(
        gps_fix=3,
        satellites=14,
        battery_percent=88,
        prearm_warning="PreArm: Compass not calibrated",
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--target",
        default="udpout:127.0.0.1:14550",
        help="pymavlink connection string",
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS),
        default="healthy",
    )
    parser.add_argument("--duration", type=float, default=10.0)
    return parser.parse_args()


def send_observation(connection, scenario: Scenario) -> None:
    from pymavlink import mavutil

    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        mavutil.mavlink.MAV_STATE_STANDBY,
    )
    connection.mav.gps_raw_int_send(
        time_usec=int(time.time_ns() / 1000),
        fix_type=scenario.gps_fix,
        lat=0,
        lon=0,
        alt=0,
        eph=100,
        epv=100,
        vel=0,
        cog=0,
        satellites_visible=scenario.satellites,
    )
    healthy_sensors = (
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
        | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY
    )
    connection.mav.param_value_send(
        b"BATT_ARM_VOLT",
        10.5,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        1,
        0,
    )
    connection.mav.sys_status_send(
        onboard_control_sensors_present=healthy_sensors,
        onboard_control_sensors_enabled=healthy_sensors,
        onboard_control_sensors_health=healthy_sensors,
        load=100,
        voltage_battery=15200,
        current_battery=40,
        battery_remaining=scenario.battery_percent,
        drop_rate_comm=0,
        errors_comm=0,
        errors_count1=0,
        errors_count2=0,
        errors_count3=0,
        errors_count4=0,
    )
    if scenario.prearm_warning:
        connection.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            scenario.prearm_warning.encode("ascii"),
        )


def main() -> int:
    from pymavlink import mavutil

    args = parse_args()
    connection = mavutil.mavlink_connection(
        args.target,
        source_system=1,
        source_component=1,
    )
    scenario = SCENARIOS[args.scenario]
    deadline = time.monotonic() + args.duration

    while time.monotonic() < deadline:
        send_observation(connection, scenario)
        time.sleep(1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
