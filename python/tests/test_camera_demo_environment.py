"""Check that launching a camera demo never kills an existing session."""

import importlib.util
import socket
import subprocess
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location(
    "camera_demo_environment", PROJECT_ROOT / "scripts/check_camera_demo_environment.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CameraDemoEnvironmentTests(unittest.TestCase):
    def test_existing_gazebo_is_reported_without_signals(self) -> None:
        with (
            patch.object(
                MODULE.subprocess,
                "run",
                return_value=subprocess.CompletedProcess([], 0),
            ) as run,
            patch.object(MODULE, "ENDPOINTS", ()),
        ):
            self.assertIn(
                "a Gazebo simulation is already running", MODULE.check_environment()
            )
        run.assert_called_once_with(
            ["pgrep", "-f", "^gz sim "], capture_output=True, check=False
        )

    def test_busy_endpoint_is_not_reused(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(("127.0.0.1", 0))
            port = occupied.getsockname()[1]
            with (
                patch.object(
                    MODULE, "ENDPOINTS", ((socket.SOCK_DGRAM, port, "test telemetry"),)
                ),
                patch.object(
                    MODULE.subprocess,
                    "run",
                    return_value=subprocess.CompletedProcess([], 1),
                ),
            ):
                self.assertEqual(
                    MODULE.check_environment(),
                    [f"test telemetry: port {port} is unavailable"],
                )
            occupied.sendto(b"still alive", ("127.0.0.1", port))
            self.assertEqual(occupied.recv(32), b"still alive")

    def test_free_endpoint_can_start(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as available:
            available.bind(("127.0.0.1", 0))
            port = available.getsockname()[1]
        with (
            patch.object(
                MODULE, "ENDPOINTS", ((socket.SOCK_DGRAM, port, "test telemetry"),)
            ),
            patch.object(
                MODULE.subprocess,
                "run",
                return_value=subprocess.CompletedProcess([], 1),
            ),
        ):
            self.assertEqual(MODULE.check_environment(), [])

    def test_launcher_checks_environment_before_starting(self) -> None:
        launcher = (PROJECT_ROOT / "StartOnboardAutonomyGazeboDemo.cmd").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("stop_onboard_autonomy_gazebo.sh", launcher)
        self.assertLess(
            launcher.index("check_camera_demo_environment.py"),
            launcher.index('start "Gazebo'),
        )
        self.assertIn(
            'ONBOARD_AUTONOMY_BUILD_DIR="%ONBOARD_AUTONOMY_BUILD_DIR%"', launcher
        )
