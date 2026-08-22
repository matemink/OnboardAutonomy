"""Process harness for a complete OnboardAutonomy ArduCopter SITL session."""

from __future__ import annotations

import json
import math
import os
import selectors
import signal
import socket
import subprocess
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TextIO

EXPECTED_INTERVALS = {
    (1, 1_000_000),
    (24, 500_000),
    (147, 1_000_000),
}
SUPPORTED_SCENARIOS = {
    "healthy",
    "heartbeat-loss",
    "gps-loss",
    "low-battery",
    "prearm",
}
ARDUCOPTER_DEFAULTS = Path("Tools/autotest/default_params/copter.parm")
PREARM_FAILURE_FRAGMENT = "MOT_SPIN_ARM > MOT_SPIN_MIN"


@dataclass(frozen=True)
class HarnessPaths:
    ardupilot_dir: Path
    mavproxy: Path
    companion: Path


@dataclass(frozen=True)
class TelemetryEvidence:
    interval_requests: tuple[tuple[int, int], ...]
    accepted_ack_count: int
    companion_heartbeat_count: int


@dataclass(frozen=True)
class SmokeTestResult:
    snapshot: dict[str, object]
    failure_snapshot: dict[str, object] | None
    evidence: TelemetryEvidence
    artifacts: Path


class ProcessSupervisor:
    def __init__(self) -> None:
        self._processes: list[tuple[str, subprocess.Popen[str]]] = []
        self._expected_stops: set[int] = set()

    def start_logged(
        self,
        name: str,
        command: list[str],
        working_directory: Path,
        log_path: Path,
    ) -> subprocess.Popen[str]:
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                command,
                cwd=working_directory,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        self._processes.append((name, process))
        return process

    def track(
        self,
        name: str,
        process: subprocess.Popen[str],
    ) -> None:
        self._processes.append((name, process))

    def assert_running(self) -> None:
        for name, process in self._processes:
            if process.pid in self._expected_stops:
                continue
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"{name} exited early with code {return_code}"
                )

    def stop(self, name: str, timeout: float = 3.0) -> None:
        for process_name, process in self._processes:
            if process_name == name:
                self._expected_stops.add(process.pid)
                terminate_process_group(process, timeout)
                return
        raise RuntimeError(f"Managed process not found: {name}")

    def stop_all(self, timeout: float = 3.0) -> None:
        for _, process in reversed(self._processes):
            terminate_process_group(process, timeout)


def terminate_process_group(
    process: subprocess.Popen[str],
    timeout: float,
) -> None:
    if process.poll() is not None:
        return

    for shutdown_signal in (signal.SIGINT, signal.SIGTERM):
        try:
            os.killpg(process.pid, shutdown_signal)
        except ProcessLookupError:
            return

        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=timeout)


def require_available_port(port: int, socket_type: int) -> None:
    with socket.socket(socket.AF_INET, socket_type) as probe:
        if socket_type == socket.SOCK_STREAM:
            probe.settimeout(0.2)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                raise RuntimeError(f"TCP port {port} is already in use")
            return

        try:
            probe.bind(("127.0.0.1", port))
        except OSError as error:
            raise RuntimeError(
                f"UDP port {port} is already in use"
            ) from error


def snapshot_has_required_telemetry(
    snapshot: dict[str, object],
) -> bool:
    return (
        snapshot.get("connected") is True
        and snapshot.get("battery_voltage_v") is not None
        and snapshot.get("system_health_known") is True
    )


def snapshot_is_ready(snapshot: dict[str, object]) -> bool:
    return (
        snapshot_has_required_telemetry(snapshot)
        and snapshot.get("navigation_ready") is True
        and snapshot.get("battery_ready") is True
        and snapshot.get("system_health_ok") is True
        and snapshot.get("armable") is True
    )


def snapshot_is_disconnected(snapshot: dict[str, object]) -> bool:
    return snapshot.get("connected") is False


def snapshot_has_gps_failure(snapshot: dict[str, object]) -> bool:
    fix_type = snapshot.get("gps_fix_type")
    fix_is_unusable = (
        fix_type is None
        or (
            isinstance(fix_type, (int, float))
            and not isinstance(fix_type, bool)
            and fix_type < 3
        )
    )
    return (
        snapshot.get("connected") is True
        and fix_is_unusable
        and snapshot.get("gps_ready") is False
        and snapshot.get("battery_ready") is True
        and snapshot.get("system_health_known") is True
        and snapshot.get("armable") is False
    )


def snapshot_has_low_battery(snapshot: dict[str, object]) -> bool:
    remaining = snapshot.get("battery_remaining_pct")
    remaining_is_low = (
        isinstance(remaining, (int, float))
        and not isinstance(remaining, bool)
        and 0 <= remaining < 20
    )
    return (
        snapshot.get("connected") is True
        and snapshot.get("gps_ready") is True
        and remaining_is_low
        and snapshot.get("battery_ready") is False
        and snapshot.get("system_health_known") is True
        and snapshot.get("armable") is False
    )


def snapshot_has_prearm_failure(snapshot: dict[str, object]) -> bool:
    warnings = snapshot.get("warnings")
    has_expected_warning = (
        isinstance(warnings, list)
        and any(
            isinstance(warning, str)
            and warning.startswith("PreArm:")
            and PREARM_FAILURE_FRAGMENT in warning
            for warning in warnings
        )
    )
    return (
        snapshot.get("connected") is True
        and snapshot.get("gps_ready") is True
        and snapshot.get("battery_ready") is True
        and snapshot.get("system_health_known") is True
        and snapshot.get("system_health_ok") is False
        and has_expected_warning
        and snapshot.get("armable") is False
    )


def wait_for_snapshot(
    process: subprocess.Popen[str],
    supervisor: ProcessSupervisor,
    output_log: TextIO,
    timeout: float,
    predicate: Callable[[dict[str, object]], bool],
    expectation: str,
) -> dict[str, object]:
    if process.stdout is None:
        raise RuntimeError("Companion stdout pipe is unavailable")

    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + timeout
    last_snapshot: dict[str, object] | None = None

    try:
        while time.monotonic() < deadline:
            supervisor.assert_running()
            remaining = min(deadline - time.monotonic(), 0.5)
            if not selector.select(remaining):
                continue

            line = process.stdout.readline()
            if not line:
                break
            output_log.write(line)
            output_log.flush()

            try:
                snapshot = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(snapshot, dict):
                continue

            last_snapshot = snapshot
            if predicate(snapshot):
                return snapshot
    finally:
        selector.close()

    detail = (
        json.dumps(last_snapshot, sort_keys=True)
        if last_snapshot is not None
        else "no JSON snapshot received"
    )
    raise RuntimeError(
        f"{expectation} did not occur within {timeout:.1f}s: {detail}"
    )


def wait_for_ready_snapshot(
    process: subprocess.Popen[str],
    supervisor: ProcessSupervisor,
    output_log: TextIO,
    timeout: float,
) -> dict[str, object]:
    return wait_for_snapshot(
        process,
        supervisor,
        output_log,
        timeout,
        snapshot_is_ready,
        "Healthy readiness baseline",
    )


def wait_for_disconnected_snapshot(
    process: subprocess.Popen[str],
    supervisor: ProcessSupervisor,
    output_log: TextIO,
    timeout: float,
) -> dict[str, object]:
    return wait_for_snapshot(
        process,
        supervisor,
        output_log,
        timeout,
        snapshot_is_disconnected,
        "Heartbeat-loss detection",
    )


def wait_for_gps_failure_snapshot(
    process: subprocess.Popen[str],
    supervisor: ProcessSupervisor,
    output_log: TextIO,
    timeout: float,
) -> dict[str, object]:
    return wait_for_snapshot(
        process,
        supervisor,
        output_log,
        timeout,
        snapshot_has_gps_failure,
        "GPS-loss detection",
    )


def wait_for_low_battery_snapshot(
    process: subprocess.Popen[str],
    supervisor: ProcessSupervisor,
    output_log: TextIO,
    timeout: float,
) -> dict[str, object]:
    return wait_for_snapshot(
        process,
        supervisor,
        output_log,
        timeout,
        snapshot_has_low_battery,
        "Low-battery detection",
    )


def wait_for_prearm_failure_snapshot(
    process: subprocess.Popen[str],
    supervisor: ProcessSupervisor,
    output_log: TextIO,
    timeout: float,
) -> dict[str, object]:
    return wait_for_snapshot(
        process,
        supervisor,
        output_log,
        timeout,
        snapshot_has_prearm_failure,
        "ArduPilot PreArm detection",
    )


def set_sitl_parameter(
    endpoint: str,
    name: str,
    value: float,
    timeout: float = 5.0,
) -> float:
    from pymavlink import mavutil

    connection = mavutil.mavlink_connection(
        endpoint,
        source_system=250,
        source_component=190,
    )
    deadline = time.monotonic() + timeout

    try:
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=min(deadline - time.monotonic(), 0.5),
            )
            if (
                candidate is not None
                and int(candidate.autopilot)
                != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                heartbeat = candidate
                break

        if heartbeat is None:
            raise RuntimeError(
                "Fault injector did not receive an autopilot heartbeat"
            )

        encoded_name = name.encode("ascii")
        while time.monotonic() < deadline:
            connection.mav.param_set_send(
                heartbeat.get_srcSystem(),
                heartbeat.get_srcComponent(),
                encoded_name,
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            response_deadline = min(deadline, time.monotonic() + 1.0)

            while time.monotonic() < response_deadline:
                response = connection.recv_match(
                    type="PARAM_VALUE",
                    blocking=True,
                    timeout=response_deadline - time.monotonic(),
                )
                if response is None:
                    break

                response_name = response.param_id
                if isinstance(response_name, bytes):
                    response_name = response_name.decode(
                        "ascii",
                        errors="replace",
                    )
                response_name = str(response_name).rstrip("\0")
                if response_name == name:
                    return float(response.param_value)
    finally:
        connection.close()

    raise RuntimeError(
        f"ArduPilot did not confirm parameter {name}={value}"
    )


def inspect_tlog(path: Path) -> TelemetryEvidence:
    from pymavlink import mavutil

    if not path.is_file():
        raise RuntimeError(f"MAVProxy tlog was not created: {path}")

    connection = mavutil.mavlink_connection(str(path))
    interval_requests: list[tuple[int, int]] = []
    accepted_ack_count = 0
    companion_heartbeat_count = 0

    while message := connection.recv_match():
        message_type = message.get_type()
        if (
            message_type == "COMMAND_LONG"
            and message.get_srcComponent() == 191
            and int(message.command) == 511
        ):
            interval_requests.append(
                (int(message.param1), int(message.param2))
            )
        elif (
            message_type == "COMMAND_ACK"
            and int(message.command) == 511
            and int(message.result) == 0
            and int(getattr(message, "target_component", 0)) in (0, 191)
        ):
            accepted_ack_count += 1
        elif (
            message_type == "HEARTBEAT"
            and message.get_srcComponent() == 191
        ):
            companion_heartbeat_count += 1

    return TelemetryEvidence(
        interval_requests=tuple(interval_requests),
        accepted_ack_count=accepted_ack_count,
        companion_heartbeat_count=companion_heartbeat_count,
    )


def validate_evidence(evidence: TelemetryEvidence) -> None:
    missing = EXPECTED_INTERVALS - set(evidence.interval_requests)
    if missing:
        raise RuntimeError(
            f"Missing telemetry interval requests: {sorted(missing)}"
        )
    if evidence.accepted_ack_count < len(EXPECTED_INTERVALS):
        raise RuntimeError(
            "Expected three accepted telemetry COMMAND_ACK messages, "
            f"received {evidence.accepted_ack_count}"
        )
    if evidence.companion_heartbeat_count == 0:
        raise RuntimeError("Companion heartbeat was not recorded")


def validate_paths(paths: HarnessPaths) -> None:
    arducopter = paths.ardupilot_dir / "build/sitl/bin/arducopter"
    required = {
        "ArduCopter": arducopter,
        "MAVProxy": paths.mavproxy,
        "OnboardAutonomy": paths.companion,
    }
    missing = [
        f"{name}: {path}"
        for name, path in required.items()
        if not path.is_file() or not os.access(path, os.X_OK)
    ]
    if missing:
        raise RuntimeError(
            "Required executable is missing:\n" + "\n".join(missing)
        )
    defaults = paths.ardupilot_dir / ARDUCOPTER_DEFAULTS
    if not defaults.is_file():
        raise RuntimeError(
            f"ArduCopter defaults file is missing: {defaults}"
        )


def run_smoke_test(
    paths: HarnessPaths,
    artifacts: Path,
    timeout: float,
    scenario: str = "healthy",
    tcp_port: int = 5760,
    udp_port: int = 14550,
    injector_tcp_port: int = 5762,
) -> SmokeTestResult:
    if scenario not in SUPPORTED_SCENARIOS:
        raise ValueError(f"Unsupported SITL scenario: {scenario}")

    validate_paths(paths)
    require_available_port(tcp_port, socket.SOCK_STREAM)
    require_available_port(udp_port, socket.SOCK_DGRAM)
    if scenario in {"gps-loss", "low-battery", "prearm"}:
        require_available_port(injector_tcp_port, socket.SOCK_STREAM)

    artifacts.mkdir(parents=True, exist_ok=False)
    state_directory = artifacts / "mavproxy-state"
    state_directory.mkdir()
    tlog_path = artifacts / "mav.tlog"
    supervisor = ProcessSupervisor()
    companion_stderr_path = artifacts / "companion.stderr.log"
    snapshot: dict[str, object] | None = None
    failure_snapshot: dict[str, object] | None = None

    try:
        arducopter = paths.ardupilot_dir / "build/sitl/bin/arducopter"
        defaults = paths.ardupilot_dir / ARDUCOPTER_DEFAULTS
        supervisor.start_logged(
            "ArduCopter",
            [
                str(arducopter),
                "-S",
                "--wipe",
                "--model",
                "+",
                "--speedup",
                "1",
                "--slave",
                "0",
                "--defaults",
                str(defaults),
                "--sim-address=127.0.0.1",
                "-I0",
            ],
            artifacts,
            artifacts / "arducopter.log",
        )

        mavproxy_command = [
            str(paths.mavproxy),
            f"--master=tcp:127.0.0.1:{tcp_port}",
            "--sitl=127.0.0.1:5501",
            f"--out=udp:127.0.0.1:{udp_port}",
            "--streamrate=-1",
            "--non-interactive",
            f"--logfile={tlog_path}",
            f"--state-basedir={state_directory}",
        ]
        supervisor.start_logged(
            "MAVProxy",
            mavproxy_command,
            paths.ardupilot_dir,
            artifacts / "mavproxy.log",
        )

        with companion_stderr_path.open(
            "w",
            encoding="utf-8",
        ) as companion_stderr:
            companion = subprocess.Popen(
                [
                    str(paths.companion),
                    "--udp-bind",
                    "127.0.0.1",
                    "--udp-port",
                    str(udp_port),
                    "--snapshot-ms",
                    "200",
                    "--json",
                ],
                stdout=subprocess.PIPE,
                stderr=companion_stderr,
                text=True,
                start_new_session=True,
            )
        supervisor.track("OnboardAutonomy", companion)

        with (artifacts / "companion.snapshots.jsonl").open(
            "w",
            encoding="utf-8",
        ) as output_log:
            snapshot = wait_for_ready_snapshot(
                companion,
                supervisor,
                output_log,
                timeout,
            )
            time.sleep(1.0)
            supervisor.assert_running()

            if scenario == "heartbeat-loss":
                supervisor.stop("MAVProxy")
                failure_snapshot = wait_for_disconnected_snapshot(
                    companion,
                    supervisor,
                    output_log,
                    timeout=8.0,
                )
            elif scenario == "gps-loss":
                confirmed_value = set_sitl_parameter(
                    f"tcp:127.0.0.1:{injector_tcp_port}",
                    "SIM_GPS_DISABLE",
                    1.0,
                )
                if confirmed_value != 1.0:
                    raise RuntimeError(
                        "ArduPilot confirmed unexpected "
                        "SIM_GPS_DISABLE value: "
                        f"{confirmed_value}"
                    )
                failure_snapshot = wait_for_gps_failure_snapshot(
                    companion,
                    supervisor,
                    output_log,
                    timeout=10.0,
                )
                time.sleep(1.0)
                supervisor.assert_running()
            elif scenario == "low-battery":
                endpoint = f"tcp:127.0.0.1:{injector_tcp_port}"
                confirmed_capacity = set_sitl_parameter(
                    endpoint,
                    "BATT_CAPACITY",
                    20.0,
                )
                confirmed_offset = set_sitl_parameter(
                    endpoint,
                    "BATT_AMP_OFFSET",
                    -1.0,
                )
                if (
                    confirmed_capacity != 20.0
                    or confirmed_offset != -1.0
                ):
                    raise RuntimeError(
                        "ArduPilot confirmed unexpected battery "
                        "injection parameters: "
                        f"capacity={confirmed_capacity}, "
                        f"offset={confirmed_offset}"
                    )
                failure_snapshot = wait_for_low_battery_snapshot(
                    companion,
                    supervisor,
                    output_log,
                    timeout=15.0,
                )
                time.sleep(1.0)
                supervisor.assert_running()
            elif scenario == "prearm":
                endpoint = f"tcp:127.0.0.1:{injector_tcp_port}"
                confirmed_spin_arm = set_sitl_parameter(
                    endpoint,
                    "MOT_SPIN_ARM",
                    0.20,
                )
                if not math.isclose(
                    confirmed_spin_arm,
                    0.20,
                    rel_tol=0.0,
                    abs_tol=1e-5,
                ):
                    raise RuntimeError(
                        "ArduPilot confirmed unexpected MOT_SPIN_ARM "
                        f"value: {confirmed_spin_arm}"
                    )
                failure_snapshot = wait_for_prearm_failure_snapshot(
                    companion,
                    supervisor,
                    output_log,
                    timeout=12.0,
                )
                time.sleep(1.0)
                supervisor.assert_running()
    finally:
        supervisor.stop_all()

    if snapshot is None:
        raise RuntimeError("Smoke test ended without a telemetry snapshot")

    evidence = inspect_tlog(tlog_path)
    validate_evidence(evidence)
    result = SmokeTestResult(
        snapshot=snapshot,
        failure_snapshot=failure_snapshot,
        evidence=evidence,
        artifacts=artifacts,
    )
    (artifacts / "summary.json").write_text(
        json.dumps(
            {
                "snapshot": result.snapshot,
                "failure_snapshot": result.failure_snapshot,
                "evidence": asdict(result.evidence),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return result


def log_tails(artifacts: Path, line_count: int = 20) -> str:
    sections: list[str] = []
    for path in sorted(artifacts.glob("*.log")):
        lines = path.read_text(
            encoding="utf-8",
            errors="replace",
        ).splitlines()
        sections.append(
            f"\n--- {path.name} ---\n" +
            "\n".join(lines[-line_count:])
        )
    return "".join(sections)
