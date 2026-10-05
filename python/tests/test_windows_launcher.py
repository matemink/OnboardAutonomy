"""Exercise Windows command dispatch without launching a simulator or SSH."""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).parents[2]


@unittest.skipUnless(os.name == "nt", "Requires the Windows command interpreter")
class WindowsLauncherTests(unittest.TestCase):
    def run_launcher(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory(prefix="onboard launch ") as temporary:
            root = Path(temporary) / "checkout with spaces"
            helpers = root / "scripts/windows"
            helpers.mkdir(parents=True)
            launcher = root / "run.cmd"
            shutil.copyfile(PROJECT_ROOT / "run.cmd", launcher)
            for name in ("gazebo", "sitl", "pi", "stop"):
                (helpers / f"{name}.cmd").write_bytes(
                    f'@echo off\r\necho ROUTE={name} ARG=[%~1]\r\nexit /b 17\r\n'.encode()
                )
            return subprocess.run(
                ["cmd.exe", "/d", "/c", "call", str(launcher), *arguments],
                cwd=temporary, capture_output=True, text=True, check=False, timeout=5,
            )

    def test_dispatch_from_other_directory_preserves_arguments_and_exit_status(self) -> None:
        for name in ("gazebo", "sitl", "pi", "stop"):
            with self.subTest(command=name):
                result = self.run_launcher(name)
                self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
                self.assertIn(f"ROUTE={name} ARG=[]", result.stdout)
        result = self.run_launcher("GAZEBO", "showcase")
        self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
        self.assertIn("ROUTE=gazebo ARG=[showcase]", result.stdout)

    def test_invalid_commands_do_not_launch_a_helper(self) -> None:
        for arguments in ((), ("unknown",), ("sitl", "showcase"),
                          ("gazebo", "unknown"), ("gazebo", "showcase", "extra")):
            with self.subTest(arguments=arguments):
                result = self.run_launcher(*arguments)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertNotIn("ROUTE=", result.stdout)

    def test_help_does_not_launch_a_helper(self) -> None:
        result = self.run_launcher("--help")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Usage:", result.stdout)
        self.assertNotIn("ROUTE=", result.stdout)
