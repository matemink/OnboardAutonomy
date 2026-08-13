#!/usr/bin/env python3
"""Prove that a running companion recovers after camera-stream loss."""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from autonomy_sitl_acceptance import (
    CAMERA_ENABLE_TOPIC,
    wait_for_gazebo_camera,
)
from link_failsafe_sitl_acceptance import wait_for_snapshot
from sitl_harness import ProcessSupervisor, require_available_port

PROJECT_ROOT = Path(__file__).resolve().parents[1]
APP_PORT = 14559
CAMERA_PORT = 5601


@dataclass(frozen=True)
class CameraRecoveryResult:
    initial_snapshot: dict[str, object]
    outage_snapshot: dict[str, object]
    recovered_snapshot: dict[str, object]
    artifacts: Path


def _camera(snapshot: dict[str, object]) -> dict[str, object] | None:
    camera = snapshot.get("camera")
    return camera if isinstance(camera, dict) else None


def camera_is_streaming(
    snapshot: dict[str, object],
    minimum_frames: int,
    minimum_restarts: int = 0,
) -> bool:
    camera = _camera(snapshot)
    if camera is None:
        return False
    frames = camera.get("received_frames")
    restarts = camera.get("camera_restarts")
    return (
        camera.get("phase") == "streaming"
        and isinstance(frames, int)
        and frames >= minimum_frames
        and isinstance(restarts, int)
        and restarts >= minimum_restarts
    )


def camera_is_reconnecting(
    snapshot: dict[str, object],
    minimum_restarts: int = 1,
) -> bool:
    camera = _camera(snapshot)
    if camera is None:
        return False
    restarts = camera.get("camera_restarts")
    return (
        camera.get("phase") == "reconnecting"
        and isinstance(restarts, int)
        and restarts >= minimum_restarts
        and bool(camera.get("error"))
    )


def _start_gazebo(
    supervisor: ProcessSupervisor,
    name: str,
    log_path: Path,
) -> subprocess.Popen[str]:
    environment = os.environ.copy()
    environment["ONBOARD_AUTONOMY_GAZEBO_HEADLESS"] = "1"
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            ["bash", "scripts/run_gazebo_apriltag.sh"],
            cwd=PROJECT_ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            env=environment,
        )
    supervisor.track(name, process)
    wait_for_gazebo_camera(process, timeout=30.0)
    _set_camera_streaming(True)
    return process


def _set_camera_streaming(enabled: bool) -> None:
    environment = os.environ.copy()
    environment["GZ_VERSION"] = "harmonic"
    result = subprocess.run(
        [
            "gz",
            "topic",
            "-t",
            CAMERA_ENABLE_TOPIC,
            "-m",
            "gz.msgs.Boolean",
            "-p",
            f"data: {'true' if enabled else 'false'}",
        ],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=5.0,
        env=environment,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "Could not change Gazebo camera streaming state: "
            + (result.stderr.strip() or result.stdout.strip())
        )


def run_acceptance(
    companion: Path,
    artifacts: Path,
    timeout: float,
) -> CameraRecoveryResult:
    if not companion.is_file() or not os.access(companion, os.X_OK):
        raise RuntimeError(f"OnboardAutonomy is not built: {companion}")

    require_available_port(APP_PORT, socket.SOCK_DGRAM)
    require_available_port(CAMERA_PORT, socket.SOCK_DGRAM)
    artifacts.mkdir(parents=True, exist_ok=False)
    supervisor = ProcessSupervisor()

    try:
        gazebo = _start_gazebo(
            supervisor,
            "Gazebo initial",
            artifacts / "gazebo-initial.log",
        )
        arguments = [
            str(companion),
            "--transport", "udp",
            "--udp-bind", "127.0.0.1",
            "--udp-port", str(APP_PORT),
            "--snapshot-ms", "100",
            "--camera",
            "--camera-source", "gstreamer",
            "--camera-udp-port", str(CAMERA_PORT),
            "--camera-width", "640",
            "--camera-height", "480",
            "--json",
        ]
        with (artifacts / "companion.stderr.log").open(
            "w", encoding="utf-8"
        ) as stderr_log:
            runtime = subprocess.Popen(
                arguments,
                cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE,
                stderr=stderr_log,
                text=True,
                start_new_session=True,
            )
        supervisor.track("OnboardAutonomy", runtime)

        with (artifacts / "companion.snapshots.jsonl").open(
            "w", encoding="utf-8"
        ) as snapshot_log:
            initial = wait_for_snapshot(
                runtime,
                (("Gazebo initial", gazebo),),
                snapshot_log,
                lambda snapshot: camera_is_streaming(snapshot, 10),
                timeout,
            )
            initial_camera = _camera(initial)
            if initial_camera is None:
                raise RuntimeError("Initial camera snapshot disappeared")
            initial_frames = int(initial_camera["received_frames"])

            supervisor.stop("Gazebo initial", timeout=5.0)
            outage = wait_for_snapshot(
                runtime,
                (),
                snapshot_log,
                camera_is_reconnecting,
                timeout,
            )
            outage_camera = _camera(outage)
            if outage_camera is None:
                raise RuntimeError("Outage camera snapshot disappeared")
            restart_count = int(outage_camera["camera_restarts"])

            restarted_gazebo = _start_gazebo(
                supervisor,
                "Gazebo restarted",
                artifacts / "gazebo-restarted.log",
            )
            recovered = wait_for_snapshot(
                runtime,
                (("Gazebo restarted", restarted_gazebo),),
                snapshot_log,
                lambda snapshot: camera_is_streaming(
                    snapshot,
                    initial_frames + 10,
                    restart_count,
                ),
                timeout,
            )
    finally:
        supervisor.stop_all(timeout=5.0)

    result = CameraRecoveryResult(
        initial_snapshot=initial,
        outage_snapshot=outage,
        recovered_snapshot=recovered,
        artifacts=artifacts,
    )
    (artifacts / "summary.json").write_text(
        json.dumps(
            {
                "initial_snapshot": result.initial_snapshot,
                "outage_snapshot": result.outage_snapshot,
                "recovered_snapshot": result.recovered_snapshot,
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
        default=PROJECT_ROOT / "artifacts/camera-recovery",
    )
    parser.add_argument("--timeout", type=float, default=45.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    artifacts = args.artifacts_root / f"{run_id}-{os.getpid()}"

    print("OnboardAutonomy camera recovery acceptance")
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

    initial = _camera(result.initial_snapshot)
    recovered = _camera(result.recovered_snapshot)
    print("PASSED")
    print("Path: streaming -> producer lost -> reconnecting -> streaming")
    print(
        "Evidence: "
        f"{initial['received_frames']} -> "
        f"{recovered['received_frames']} frames, "
        f"{recovered['camera_restarts']} camera restarts"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
