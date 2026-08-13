#!/usr/bin/env python3
"""Prove that ArduPilot, not the companion process, owns link-loss LAND."""

from __future__ import annotations

import argparse
import json
import os
import selectors
import socket
import subprocess
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import TextIO

from autonomy_sitl_acceptance import wait_for_gazebo_camera
from sitl_harness import ProcessSupervisor, require_available_port

PROJECT_ROOT = Path(__file__).resolve().parents[1]
APP_PORT = 14550
RELAY_PORT = 14551
MONITOR_PORT = 14552
CAMERA_PORT = 5601


@dataclass(frozen=True)
class LinkFailsafeEvidence:
    flight_commands: tuple[str, ...]
    companion_heartbeat_count: int
    modes: tuple[int, ...]
    armed_transitions: tuple[bool, ...]
    failover_latency_s: float | None
    failsafe_statuses: tuple[str, ...]


@dataclass(frozen=True)
class IndependentMonitorEvidence:
    modes: tuple[int, ...]
    armed_transitions: tuple[bool, ...]


@dataclass(frozen=True)
class LinkFailsafeAcceptanceResult:
    armed_snapshot: dict[str, object]
    link_loss_snapshot: dict[str, object]
    evidence: LinkFailsafeEvidence
    independent_monitor: IndependentMonitorEvidence
    artifacts: Path


class UdpCutRelay:
    """Bidirectional MAVLink relay whose forwarding can be cut atomically."""

    def __init__(self, listen_port: int, application_port: int) -> None:
        self._application = ("127.0.0.1", application_port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", listen_port))
        self._socket.settimeout(0.1)
        self._running = threading.Event()
        self._running.set()
        self._forwarding = threading.Event()
        self._forwarding.set()
        self._mavproxy_peer: tuple[str, int] | None = None
        self._thread = threading.Thread(
            target=self._run,
            name="mavlink-cut-relay",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def cut(self) -> None:
        self._forwarding.clear()

    def close(self) -> None:
        self._running.clear()
        self._thread.join(timeout=2.0)
        self._socket.close()

    def _run(self) -> None:
        while self._running.is_set():
            try:
                payload, sender = self._socket.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                return

            if sender[1] == self._application[1]:
                destination = self._mavproxy_peer
            else:
                self._mavproxy_peer = sender
                destination = self._application

            if self._forwarding.is_set() and destination is not None:
                self._socket.sendto(payload, destination)


def snapshot_ready_for_link_cut(snapshot: dict[str, object]) -> bool:
    startup = snapshot.get("flight_startup")
    failsafe = snapshot.get("companion_link_failsafe")
    altitude = snapshot.get("relative_altitude_m")
    return (
        isinstance(startup, dict)
        and startup.get("phase") == "completed"
        and isinstance(failsafe, dict)
        and failsafe.get("phase") == "accepted"
        and failsafe.get("action") == "land"
        and snapshot.get("armed") is True
        and isinstance(altitude, (int, float))
        and float(altitude) >= 7.5
    )


def snapshot_records_link_loss(snapshot: dict[str, object]) -> bool:
    autonomy = snapshot.get("autonomy")
    return (
        snapshot.get("connected") is False
        and isinstance(autonomy, dict)
        and autonomy.get("phase") == "failed"
        and "heartbeat was lost" in str(autonomy.get("detail", ""))
    )


def validate_link_failsafe_evidence(
    evidence: LinkFailsafeEvidence,
    independent_monitor: IndependentMonitorEvidence,
    expected_timeout_s: float,
) -> None:
    expected_commands = {"SET_GUIDED", "ARM", "TAKEOFF"}
    if set(evidence.flight_commands) != expected_commands:
        raise RuntimeError(
            "Companion emitted unexpected flight commands after link loss: "
            f"{evidence.flight_commands}"
        )
    if evidence.companion_heartbeat_count < 3:
        raise RuntimeError("Companion heartbeat was not established")
    if not {4, 9}.issubset(evidence.modes):
        raise RuntimeError(
            f"ArduPilot did not transition GUIDED -> LAND: {evidence.modes}"
        )
    if 9 not in independent_monitor.modes:
        raise RuntimeError(
            "Independent monitor did not observe ArduPilot LAND: "
            f"{independent_monitor.modes}"
        )
    if (
        not independent_monitor.armed_transitions
        or True not in independent_monitor.armed_transitions
        or independent_monitor.armed_transitions[-1] is not False
    ):
        raise RuntimeError(
            "ArduPilot did not complete the independent LAND/disarm: "
            f"{independent_monitor.armed_transitions}"
        )
    if evidence.failover_latency_s is None:
        raise RuntimeError("Could not measure the GCS failsafe latency")
    if not (
        expected_timeout_s - 0.75
        <= evidence.failover_latency_s
        <= expected_timeout_s + 2.5
    ):
        raise RuntimeError(
            "ArduPilot LAND latency is inconsistent with FS_GCS_TIMEOUT: "
            f"{evidence.failover_latency_s:.3f} s"
        )


def inspect_link_failsafe_tlog(path: Path) -> LinkFailsafeEvidence:
    from pymavlink import mavutil

    if not path.is_file():
        raise RuntimeError(f"MAVProxy tlog was not created: {path}")

    command_names = {
        mavutil.mavlink.MAV_CMD_DO_SET_MODE: "SET_GUIDED",
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "ARM",
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
        mavutil.mavlink.MAV_CMD_NAV_LAND: "LAND",
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH: "RTL",
    }
    commands: set[str] = set()
    companion_heartbeats: list[float] = []
    autopilot_heartbeats: list[tuple[float, int, bool]] = []
    modes: set[int] = set()
    armed_transitions: list[bool] = []
    last_armed: bool | None = None
    failsafe_statuses: list[str] = []

    connection = mavutil.mavlink_connection(str(path))
    try:
        while message := connection.recv_match():
            message_type = message.get_type()
            timestamp = float(getattr(message, "_timestamp", 0.0))
            if (
                message_type == "HEARTBEAT"
                and message.get_srcComponent() == 191
                and int(message.autopilot)
                == mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                companion_heartbeats.append(timestamp)
            elif (
                message_type == "HEARTBEAT"
                and int(message.autopilot)
                != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                mode = int(message.custom_mode)
                armed = bool(
                    int(message.base_mode)
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                modes.add(mode)
                autopilot_heartbeats.append((timestamp, mode, armed))
                if armed != last_armed:
                    armed_transitions.append(armed)
                    last_armed = armed
            elif (
                message_type == "COMMAND_LONG"
                and message.get_srcComponent() == 191
            ):
                command = command_names.get(int(message.command))
                if command is not None:
                    commands.add(command)
            elif message_type == "STATUSTEXT":
                status = message.text
                if isinstance(status, bytes):
                    status = status.decode("utf-8", errors="replace")
                status = str(status).rstrip("\0")
                lowered = status.lower()
                if "gcs" in lowered and "failsafe" in lowered:
                    failsafe_statuses.append(status)
    finally:
        connection.close()

    failover_latency = None
    if companion_heartbeats:
        last_companion = companion_heartbeats[-1]
        first_land = next(
            (
                timestamp
                for timestamp, mode, _ in autopilot_heartbeats
                if timestamp >= last_companion and mode == 9
            ),
            None,
        )
        if first_land is not None:
            failover_latency = first_land - last_companion

    return LinkFailsafeEvidence(
        flight_commands=tuple(sorted(commands)),
        companion_heartbeat_count=len(companion_heartbeats),
        modes=tuple(sorted(modes)),
        armed_transitions=tuple(armed_transitions),
        failover_latency_s=failover_latency,
        failsafe_statuses=tuple(dict.fromkeys(failsafe_statuses)),
    )


def _read_snapshot_line(
    process: subprocess.Popen[str],
    output_log: TextIO,
) -> dict[str, object] | None:
    if process.stdout is None:
        return None
    line = process.stdout.readline()
    if not line:
        return None
    output_log.write(line)
    output_log.flush()
    try:
        value = json.loads(line)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def wait_for_snapshot(
    process: subprocess.Popen[str],
    dependencies: tuple[tuple[str, subprocess.Popen[str]], ...],
    output_log: TextIO,
    predicate,
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
            if process.poll() is not None:
                break
            for _key, _mask in selector.select(0.25):
                snapshot = _read_snapshot_line(process, output_log)
                if snapshot is not None:
                    last_snapshot = snapshot
                    if predicate(snapshot):
                        return snapshot
    finally:
        selector.close()

    raise RuntimeError(
        "Expected OnboardAutonomy state was not observed: "
        + (
            json.dumps(last_snapshot, sort_keys=True)
            if last_snapshot is not None
            else "no JSON snapshot"
        )
    )


def wait_for_ardupilot_land_and_disarm(
    monitor,
    dependencies: tuple[tuple[str, subprocess.Popen[str]], ...],
    timeout: float,
) -> IndependentMonitorEvidence:
    from pymavlink import mavutil

    deadline = time.monotonic() + timeout
    armed_seen = False
    land_seen = False
    modes: list[int] = []
    armed_transitions: list[bool] = []
    last_armed: bool | None = None
    while time.monotonic() < deadline:
        for name, dependency in dependencies:
            if dependency.poll() is not None:
                raise RuntimeError(
                    f"{name} exited early with code {dependency.returncode}"
                )
        message = monitor.recv_match(type="HEARTBEAT", blocking=False)
        if message is None:
            time.sleep(0.02)
            continue
        if (
            int(message.autopilot)
            == mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            continue
        armed = bool(
            int(message.base_mode)
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        mode = int(message.custom_mode)
        if not modes or modes[-1] != mode:
            modes.append(mode)
        if last_armed is None or last_armed != armed:
            armed_transitions.append(armed)
            last_armed = armed
        armed_seen = armed_seen or armed
        land_seen = land_seen or mode == 9
        if armed_seen and land_seen and not armed:
            return IndependentMonitorEvidence(
                modes=tuple(modes),
                armed_transitions=tuple(armed_transitions),
            )
    raise RuntimeError("ArduPilot did not LAND and disarm after link cut")


def run_acceptance(
    companion: Path,
    artifacts: Path,
    timeout: float,
) -> LinkFailsafeAcceptanceResult:
    from pymavlink import mavutil

    if not companion.is_file() or not os.access(companion, os.X_OK):
        raise RuntimeError(f"OnboardAutonomy is not built: {companion}")

    for port, kind in (
        (APP_PORT, socket.SOCK_DGRAM),
        (RELAY_PORT, socket.SOCK_DGRAM),
        (MONITOR_PORT, socket.SOCK_DGRAM),
        (CAMERA_PORT, socket.SOCK_DGRAM),
    ):
        require_available_port(port, kind)

    artifacts.mkdir(parents=True, exist_ok=False)
    tlog = artifacts / "link-failsafe.tlog"
    supervisor = ProcessSupervisor()
    relay = UdpCutRelay(RELAY_PORT, APP_PORT)
    monitor = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{MONITOR_PORT}"
    )
    relay.start()
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
            "ONBOARD_AUTONOMY_MAVLINK_OUT_PORT": str(RELAY_PORT),
            "ONBOARD_AUTONOMY_MAVLINK_MONITOR_PORT": str(MONITOR_PORT),
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

        app_arguments = [
            str(companion),
            "--transport", "udp",
            "--sitl",
            "--udp-bind", "127.0.0.1",
            "--udp-port", str(APP_PORT),
            "--snapshot-ms", "100",
            "--camera",
            "--camera-source", "gstreamer",
            "--camera-udp-port", str(CAMERA_PORT),
            "--camera-width", "640",
            "--camera-height", "480",
            "--apriltag",
            "--camera-calibration",
            str(PROJECT_ROOT / "config/gazebo-landing-camera-640x480.json"),
            "--apriltag-size-mm", "2000",
            "--camera-extrinsics",
            str(PROJECT_ROOT / "config/gazebo-landing-camera-extrinsics.json"),
            "--autonomous",
            "--exit-after-autonomy",
            "--json",
        ]
        with (artifacts / "companion.stderr.log").open(
            "w", encoding="utf-8"
        ) as companion_stderr:
            runtime = subprocess.Popen(
                app_arguments,
                cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE,
                stderr=companion_stderr,
                text=True,
                start_new_session=True,
                env=base_environment,
            )
        supervisor.track("OnboardAutonomy", runtime)
        dependencies = (("Gazebo", gazebo), ("ArduCopter/MAVProxy", sitl))

        with (artifacts / "companion.snapshots.jsonl").open(
            "w", encoding="utf-8"
        ) as snapshot_log:
            armed_snapshot = wait_for_snapshot(
                runtime,
                dependencies,
                snapshot_log,
                snapshot_ready_for_link_cut,
                timeout,
            )
            relay.cut()
            link_loss_snapshot = wait_for_snapshot(
                runtime,
                dependencies,
                snapshot_log,
                snapshot_records_link_loss,
                timeout=10.0,
            )

        return_code = runtime.wait(timeout=5.0)
        if return_code != 2:
            raise RuntimeError(
                "OnboardAutonomy must exit with autonomy-failure code 2, "
                f"got {return_code}"
            )
        independent_monitor = wait_for_ardupilot_land_and_disarm(
            monitor,
            dependencies,
            timeout=60.0,
        )
    finally:
        relay.close()
        monitor.close()
        supervisor.stop_all(timeout=5.0)

    evidence = inspect_link_failsafe_tlog(tlog)
    failsafe = armed_snapshot["companion_link_failsafe"]
    expected_timeout_s = float(failsafe["timeout_s"])
    validate_link_failsafe_evidence(
        evidence,
        independent_monitor,
        expected_timeout_s,
    )

    result = LinkFailsafeAcceptanceResult(
        armed_snapshot=armed_snapshot,
        link_loss_snapshot=link_loss_snapshot,
        evidence=evidence,
        independent_monitor=independent_monitor,
        artifacts=artifacts,
    )
    (artifacts / "summary.json").write_text(
        json.dumps(
            {
                "armed_snapshot": result.armed_snapshot,
                "link_loss_snapshot": result.link_loss_snapshot,
                "evidence": asdict(result.evidence),
                "independent_monitor": asdict(
                    result.independent_monitor
                ),
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
        default=PROJECT_ROOT / "artifacts/link-failsafe-sitl",
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    artifacts = args.artifacts_root / f"{run_id}-{os.getpid()}"

    print("OnboardAutonomy companion-link failsafe acceptance")
    print(f"Artifacts: {artifacts}")
    try:
        result = run_acceptance(
            args.companion.expanduser().resolve(),
            artifacts,
            args.timeout,
        )
    except Exception as error:  # noqa: BLE001 - CLI reports scenario failures.
        print(f"FAILED: {error}")
        return 1

    print("PASSED")
    print("Path: GUIDED -> ARM -> TAKEOFF -> link cut -> ArduPilot LAND")
    print(
        "Evidence: "
        f"{result.evidence.companion_heartbeat_count} heartbeats, "
        f"{result.evidence.failover_latency_s:.3f} s failover, "
        "no companion LAND command"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
