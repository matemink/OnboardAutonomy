"""Check that launching a camera demo never kills an existing session."""

import importlib.util
import socket
import subprocess
import sys
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

    def test_active_tcp_listener_is_not_reused(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            port = listener.getsockname()[1]
            with (
                patch.object(
                    MODULE, "ENDPOINTS", ((socket.SOCK_STREAM, port, "test preview"),)
                ),
                patch.object(
                    MODULE.subprocess,
                    "run",
                    return_value=subprocess.CompletedProcess([], 1),
                ),
            ):
                self.assertEqual(
                    MODULE.check_environment(),
                    [f"test preview: port {port} is unavailable"],
                )
            self.assertEqual(listener.getsockname()[1], port)

    @unittest.skipUnless(sys.platform == "linux", "Linux TCP reuse behavior")
    def test_recently_closed_tcp_connection_does_not_block_restart(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            port = listener.getsockname()[1]
            with socket.create_connection(("127.0.0.1", port), timeout=2) as client:
                connection, _ = listener.accept()
                connection.close()
                self.assertEqual(client.recv(1), b"")
        # Prove this fixture has the condition that fooled the previous probe.
        with (
            socket.socket(socket.AF_INET, socket.SOCK_STREAM) as plain_probe,
            self.assertRaises(OSError),
        ):
            plain_probe.bind(("0.0.0.0", port))
        with (
            patch.object(
                MODULE, "ENDPOINTS", ((socket.SOCK_STREAM, port, "test preview"),)
            ),
            patch.object(
                MODULE.subprocess,
                "run",
                return_value=subprocess.CompletedProcess([], 1),
            ),
        ):
            self.assertEqual(MODULE.check_environment(), [])


    def test_disabled_downward_port_does_not_block_forward_demo(self) -> None:
        with (
            patch.object(MODULE, "ENDPOINTS", ()),
            patch.object(MODULE.socket, "socket") as socket_factory,
            patch.object(MODULE.subprocess, "run",
                         return_value=subprocess.CompletedProcess([], 1)),
        ):
            probe = socket_factory.return_value.__enter__.return_value
            probe.bind.side_effect = OSError("occupied")
            self.assertEqual(MODULE.check_environment(), [])
            socket_factory.assert_not_called()
            self.assertEqual(
                MODULE.check_environment(downward_enabled=True),
                ["downward camera: port 5601 is unavailable"],
            )
