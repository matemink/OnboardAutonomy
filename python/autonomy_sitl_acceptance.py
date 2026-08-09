#!/usr/bin/env python3
"""Run and independently verify the production autonomy path in Gazebo."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import selectors
import socket
import subprocess
import time
from typing import TextIO
import xml.etree.ElementTree as element_tree

from sitl_harness import ProcessSupervisor, require_available_port


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LANDING_POSITION_TOLERANCE_M = 0.75
GAZEBO_WORLD = (
    PROJECT_ROOT / "simulation" / "worlds" / "apriltag_landing.sdf"
)
CAMERA_ENABLE_TOPIC = (
    "/world/apriltag_landing/model/Holybro_S500/link/"
    "Raspberry_Pi_Camera_Module_3_Wide/sensor/"
    "Raspberry_Pi_Camera_Module_3_Wide/image/enable_streaming"
)


@dataclass(frozen=True)
class AutonomyFlightEvidence:
    commands: tuple[str, ...]
    accepted_acknowledgements: tuple[str, ...]
    maximum_relative_altitude_m: float | None
    armed_transitions: tuple[bool, ...]
    modes: tuple[int, ...]
    landing_target_count: int
    landing_target_frames: tuple[int, ...]
    position_valid_values: tuple[int, ...]
    final_local_position_ne_m: tuple[float, float] | None
    final_horizontal_error_m: float | None
    precision_statuses: tuple[str, ...]


@dataclass(frozen=True)
class AutonomyAcceptanceResult:
    final_snapshot: dict[str, object]
    evidence: AutonomyFlightEvidence
    artifacts: Path


def snapshot_completed_autonomy(snapshot: dict[str, object]) -> bool:
    startup = snapshot.get("flight_startup")
    autonomy = snapshot.get("autonomy")
    return (
        isinstance(startup, dict)
        and startup.get("phase") == "completed"
        and isinstance(autonomy, dict)
        and autonomy.get("phase") == "completed"
        and snapshot.get("armed") is False
    )


def snapshot_failed_autonomy(snapshot: dict[str, object]) -> bool:
    autonomy = snapshot.get("autonomy")
    return (
        isinstance(autonomy, dict)
        and autonomy.get("phase") == "failed"
    )


def expected_landing_position_ne_m() -> tuple[float, float]:
    world = element_tree.parse(GAZEBO_WORLD).getroot()
    poses = {
        include.findtext("uri"): tuple(
            float(value)
            for value in include.findtext("pose").split()[:2]
        )
        for include in world.findall(".//world/include")
    }
    pad_east_m, pad_north_m = poses["model://apriltag_landing_pad"]
    vehicle_east_m, vehicle_north_m = poses[
        "model://iris_with_landing_camera"
    ]

    # Gazebo world XY is East/North; MAVLink LOCAL_POSITION_NED is North/East.
    return (
        pad_north_m - vehicle_north_m,
        pad_east_m - vehicle_east_m,
    )


def inspect_autonomy_tlog(path: Path) -> AutonomyFlightEvidence:
    from pymavlink import mavutil

    if not path.is_file():
        raise RuntimeError(f"MAVProxy tlog was not created: {path}")

    command_names = {
        mavutil.mavlink.MAV_CMD_DO_SET_MODE: "SET_GUIDED",
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "ARM",
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
        mavutil.mavlink.MAV_CMD_NAV_LAND: "LAND",
    }
    commands: set[str] = set()
    accepted_acknowledgements: set[str] = set()
    maximum_altitude: float | None = None
    armed_transitions: list[bool] = []
    last_armed: bool | None = None
    modes: set[int] = set()
    landing_target_count = 0
    landing_target_frames: set[int] = set()
    position_valid_values: set[int] = set()
    final_local_position: tuple[float, float] | None = None
    precision_statuses: list[str] = []

    connection = mavutil.mavlink_connection(str(path))
    try:
        while message := connection.recv_match():
            message_type = message.get_type()
            if message_type == "COMMAND_LONG":
                command = int(message.command)
                if (
                    command in command_names
                    and message.get_srcComponent() == 191
                ):
                    commands.add(command_names[command])
            elif message_type == "COMMAND_ACK":
                command = int(message.command)
                if command in command_names and int(message.result) == 0:
                    accepted_acknowledgements.add(command_names[command])
            elif message_type == "GLOBAL_POSITION_INT":
                altitude = float(message.relative_alt) / 1000.0
                maximum_altitude = (
                    altitude
                    if maximum_altitude is None
                    else max(maximum_altitude, altitude)
                )
            elif message_type == "LOCAL_POSITION_NED":
                final_local_position = (
                    float(message.x),
                    float(message.y),
                )
            elif message_type == "LANDING_TARGET":
                landing_target_count += 1
                landing_target_frames.add(int(message.frame))
                position_valid_values.add(int(message.position_valid))
            elif (
                message_type == "HEARTBEAT"
                and int(message.autopilot)
                != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                modes.add(int(message.custom_mode))
                armed = bool(
                    int(message.base_mode)
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                if armed != last_armed:
                    armed_transitions.append(armed)
                    last_armed = armed
            elif message_type == "STATUSTEXT":
                status = message.text
                if isinstance(status, bytes):
                    status = status.decode("utf-8", errors="replace")
                status = str(status).rstrip("\0")
                if status.startswith("PrecLand:"):
                    precision_statuses.append(status)
    finally:
        connection.close()

    horizontal_error = None
    if final_local_position is not None:
        expected_north_m, expected_east_m = (
            expected_landing_position_ne_m()
        )
        horizontal_error = (
            (final_local_position[0] - expected_north_m) ** 2
            + (final_local_position[1] - expected_east_m) ** 2
        ) ** 0.5

    return AutonomyFlightEvidence(
        commands=tuple(sorted(commands)),
        accepted_acknowledgements=tuple(
            sorted(accepted_acknowledgements)
        ),
        maximum_relative_altitude_m=maximum_altitude,
        armed_transitions=tuple(armed_transitions),
        modes=tuple(sorted(modes)),
        landing_target_count=landing_target_count,
        landing_target_frames=tuple(sorted(landing_target_frames)),
        position_valid_values=tuple(sorted(position_valid_values)),
        final_local_position_ne_m=final_local_position,
        final_horizontal_error_m=horizontal_error,
        precision_statuses=tuple(dict.fromkeys(precision_statuses)),
    )


def validate_autonomy_evidence(evidence: AutonomyFlightEvidence) -> None:
    expected_actions = {"SET_GUIDED", "ARM", "TAKEOFF", "LAND"}
    if set(evidence.commands) != expected_actions:
        raise RuntimeError(
            f"Unexpected production commands: {evidence.commands}"
        )
    if set(evidence.accepted_acknowledgements) != expected_actions:
        raise RuntimeError(
            "Missing accepted production ACKs: "
            f"{evidence.accepted_acknowledgements}"
        )
    if (
        evidence.maximum_relative_altitude_m is None
        or evidence.maximum_relative_altitude_m < 7.5
    ):
        raise RuntimeError(
            "Vehicle did not reach the verified takeoff altitude"
        )
    if (
        not evidence.armed_transitions
        or evidence.armed_transitions[0] is not False
        or True not in evidence.armed_transitions
        or evidence.armed_transitions[-1] is not False
    ):
        raise RuntimeError(
            f"Invalid arm lifecycle: {evidence.armed_transitions}"
        )
    if not {0, 4, 9}.issubset(evidence.modes):
        raise RuntimeError(f"Missing flight modes: {evidence.modes}")
    if evidence.landing_target_count < 5:
        raise RuntimeError("No sustained LANDING_TARGET stream observed")
    if evidence.landing_target_frames != (12,):
        raise RuntimeError(
            "LANDING_TARGET must use MAV_FRAME_BODY_FRD (12)"
        )
    if evidence.position_valid_values != (1,):
        raise RuntimeError("LANDING_TARGET position must be valid")
    if (
        evidence.final_horizontal_error_m is None
        or evidence.final_horizontal_error_m > LANDING_POSITION_TOLERANCE_M
    ):
        raise RuntimeError(
            "Final horizontal error from the offset landing pad exceeds "
            f"{LANDING_POSITION_TOLERANCE_M} m: "
            f"{evidence.final_horizontal_error_m}"
        )
    if "PrecLand: Target Found" not in evidence.precision_statuses:
        raise RuntimeError("ArduPilot did not report precision target lock")


def wait_for_gazebo_camera(
    gazebo: subprocess.Popen[str],
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    environment = os.environ.copy()
    environment["GZ_VERSION"] = "harmonic"

    while time.monotonic() < deadline:
        if gazebo.poll() is not None:
            raise RuntimeError(
                f"Gazebo exited early with code {gazebo.returncode}"
            )
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=3.0,
            env=environment,
            check=False,
        )
        if CAMERA_ENABLE_TOPIC in result.stdout.splitlines():
            return
        time.sleep(0.5)

    raise RuntimeError("Gazebo landing camera did not become available")


def wait_for_terminal_snapshot(
    process: subprocess.Popen[str],
    dependencies: tuple[tuple[str, subprocess.Popen[str]], ...],
    output_log: TextIO,
    timeout: float,
) -> dict[str, object]:
    if process.stdout is None:
        raise RuntimeError("OnboardAutonomy stdout pipe is unavailable")

    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + timeout
    last_snapshot: dict[str, object] | None = None

    try:
        while time.monotonic() < deadline:
            for name, dependency in dependencies:
                if dependency.poll() is not None:
                    raise RuntimeError(
                        f"{name} exited early with code "
                        f"{dependency.returncode}"
                    )

            events = selector.select(min(deadline - time.monotonic(), 0.5))
            if events:
                line = process.stdout.readline()
                if line:
                    output_log.write(line)
                    output_log.flush()
                    try:
                        candidate = json.loads(line)
                    except json.JSONDecodeError:
                        candidate = None
                    if isinstance(candidate, dict):
                        last_snapshot = candidate
                        if snapshot_failed_autonomy(candidate):
                            detail = candidate.get("autonomy")
                            raise RuntimeError(
                                f"Production autonomy failed: {detail}"
                            )
                        if snapshot_completed_autonomy(candidate):
                            return candidate

            if process.poll() is not None:
                for line in process.stdout:
                    output_log.write(line)
                    try:
                        candidate = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if isinstance(candidate, dict):
                        last_snapshot = candidate
                        if snapshot_completed_autonomy(candidate):
                            return candidate
                break
    finally:
        selector.close()

    raise RuntimeError(
        "Production autonomy did not complete: "
        + (
            json.dumps(last_snapshot, sort_keys=True)
            if last_snapshot is not None
            else "no JSON snapshot received"
        )
    )


def run_acceptance(
    companion: Path,
    artifacts: Path,
    timeout: float,
) -> AutonomyAcceptanceResult:
    if not companion.is_file() or not os.access(companion, os.X_OK):
        raise RuntimeError(f"OnboardAutonomy is not built: {companion}")

    require_available_port(14550, socket.SOCK_DGRAM)
    require_available_port(5601, socket.SOCK_DGRAM)
    require_available_port(8080, socket.SOCK_STREAM)
    artifacts.mkdir(parents=True, exist_ok=False)
    tlog = artifacts / "autonomy.tlog"
    supervisor = ProcessSupervisor()

    base_environment = os.environ.copy()
    try:
        gazebo_environment = base_environment | {
            "ONBOARD_AUTONOMY_GAZEBO_HEADLESS": "1",
        }
        with (artifacts / "gazebo.log").open(
            "w", encoding="utf-8"
        ) as gazebo_log:
            gazebo = subprocess.Popen(
                ["bash", "scripts/run_gazebo_apriltag.sh"],
                cwd=PROJECT_ROOT,
                stdout=gazebo_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
                env=gazebo_environment,
            )
        supervisor.track("Gazebo", gazebo)
        wait_for_gazebo_camera(gazebo, timeout=30.0)

        sitl_environment = base_environment | {
            "ONBOARD_AUTONOMY_TLOG": str(tlog),
            "ONBOARD_AUTONOMY_MAVPROXY_STATE_DIR": str(
                artifacts / "mavproxy-state"
            ),
        }
        with (artifacts / "sitl.log").open(
            "w", encoding="utf-8"
        ) as sitl_log:
            sitl = subprocess.Popen(
                ["bash", "scripts/run_arducopter_gazebo.sh"],
                cwd=PROJECT_ROOT,
                stdout=sitl_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
                env=sitl_environment,
            )
        supervisor.track("ArduCopter/MAVProxy", sitl)

        app_environment = base_environment | {
            "ONBOARD_AUTONOMY_BUILD_DIR": str(companion.parent),
            "ONBOARD_AUTONOMY_AUTONOMOUS": "1",
            "ONBOARD_AUTONOMY_EXIT_AFTER_AUTONOMY": "1",
            "ONBOARD_AUTONOMY_JSON": "1",
        }
        with (artifacts / "companion.stderr.log").open(
            "w", encoding="utf-8"
        ) as companion_stderr:
            runtime = subprocess.Popen(
                ["bash", "scripts/run_onboard_autonomy_gazebo_vision.sh"],
                cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE,
                stderr=companion_stderr,
                text=True,
                start_new_session=True,
                env=app_environment,
            )
        supervisor.track("OnboardAutonomy", runtime)

        with (artifacts / "companion.snapshots.jsonl").open(
            "w", encoding="utf-8"
        ) as snapshot_log:
            final_snapshot = wait_for_terminal_snapshot(
                runtime,
                (("Gazebo", gazebo), ("ArduCopter/MAVProxy", sitl)),
                snapshot_log,
                timeout,
            )
        return_code = runtime.wait(timeout=5.0)
        if return_code != 0:
            raise RuntimeError(
                f"OnboardAutonomy exited with code {return_code}"
            )
        # The runtime receives DISARMED through MAVProxy before MAVProxy's
        # buffered tlog writer is guaranteed to persist that last frame.
        time.sleep(1.0)
    finally:
        supervisor.stop_all(timeout=5.0)

    evidence = inspect_autonomy_tlog(tlog)
    validate_autonomy_evidence(evidence)
    result = AutonomyAcceptanceResult(
        final_snapshot=final_snapshot,
        evidence=evidence,
        artifacts=artifacts,
    )
    (artifacts / "summary.json").write_text(
        json.dumps(
            {
                "final_snapshot": result.final_snapshot,
                "evidence": asdict(result.evidence),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--companion",
        type=Path,
        default=Path.home() / "build/onboard_autonomy/onboard_autonomy",
    )
    parser.add_argument(
        "--artifacts-root",
        type=Path,
        default=PROJECT_ROOT / "artifacts/autonomy-sitl",
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_id = datetime.now().strftime("%Y%m%d-%H%M%S")
    artifacts = args.artifacts_root / f"{run_id}-{os.getpid()}"

    print("OnboardAutonomy production SITL acceptance")
    print(f"Artifacts: {artifacts}")
    try:
        result = run_acceptance(
            args.companion.expanduser().resolve(),
            artifacts,
            args.timeout,
        )
    except Exception as error:
        print(f"FAILED: {error}")
        return 1

    print("PASSED")
    print("Path: readiness -> GUIDED -> ARM -> TAKEOFF -> vision LAND")
    print(
        "Evidence: "
        f"{result.evidence.landing_target_count} LANDING_TARGET, "
        f"{result.evidence.maximum_relative_altitude_m:.2f} m max, "
        f"{result.evidence.final_horizontal_error_m:.3f} m error"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
