"""Run the installer against a temporary package and fake systemctl."""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("bash"), "Requires Bash")
class ServiceUpgradeTests(unittest.TestCase):
    def run_installer(self, active: bool, copy_fails: bool = False,
                      state: str | None = None) -> tuple[int, list[str]]:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            package = root / "package"
            commands = root / "commands"
            (package / "bin").mkdir(parents=True)
            assets = package / "share/onboard_autonomy/systemd"
            assets.mkdir(parents=True)
            commands.mkdir()
            units = root / "units"
            units.mkdir()
            (package / "bin/onboard_autonomy").write_text("new binary")
            (package / "bin/onboard_autonomy").chmod(0o755)
            for name in ("onboard-autonomy@.service", "onboard-autonomy.env.example"):
                shutil.copy(PROJECT_ROOT / "deployment/systemd" / name, assets / name)
            installer = (PROJECT_ROOT / "scripts/install_onboard_autonomy_service.sh").read_text()
            # Only the root permission gate and destination roots are substituted.
            # The account lookup is stubbed; installer control flow and ordering are unchanged.
            installer = installer.replace('if [[ "${EUID}" -ne 0 ]]; then', "if false; then")
            for original, destination in (
                ("/opt/onboard-autonomy", root / "installation"),
                ("/etc/onboard-autonomy", root / "configuration"),
                ("/etc/systemd/system/", str(units) + "/"),
            ):
                installer = installer.replace(original, str(destination))
            path = package / "bin/install_onboard_autonomy_service.sh"
            path.write_text(installer)
            (commands / "systemctl").write_text(
                "#!/usr/bin/env bash\n"
                "printf '%s\\n' \"$*\" >> \"$TEST_LOG\"\n"
                "case \"$1\" in\n"
                "is-active) [[ \"$TEST_ACTIVE\" == 1 ]];;\n"
                "show) if [[ \"$*\" == *ActiveState* ]]; then echo \"$TEST_STATE\"; "
                "elif [[ \"$TEST_ACTIVE\" == 1 ]]; then echo loaded; else echo not-found; fi;;\n"
                "stop) touch \"$TEST_STOPPED\";;\n"
                "esac\n"
            )
            (commands / "id").write_text(
                '#!/usr/bin/env bash\n[[ "$1" == "fixture-user" ]]\n'
            )
            (commands / "cp").write_text(
                "#!/usr/bin/env bash\n"
                "echo copy >> \"$TEST_LOG\"\n"
                "if [[ \"$TEST_ACTIVE\" == 1 && ! -e \"$TEST_STOPPED\" ]]; then exit 26; fi\n"
                "if [[ \"$TEST_COPY_FAILS\" == 1 ]]; then exit 27; fi\n"
                "exec /bin/cp \"$@\"\n"
            )
            for command in commands.iterdir():
                command.chmod(0o755)
            log = root / "calls"
            result = subprocess.run(
                ["bash", str(path)], capture_output=True, text=True, timeout=5,
                env=os.environ | {
                    "PATH": str(commands) + os.pathsep + os.environ["PATH"],
                    "ONBOARD_AUTONOMY_SERVICE_USER": "fixture-user",
                    "TEST_LOG": str(log), "TEST_STOPPED": str(root / "stopped"),
                    "TEST_ACTIVE": "1" if active else "0",
                    "TEST_STATE": state or ("active" if active else "inactive"),
                    "TEST_COPY_FAILS": "1" if copy_fails else "0",
                },
            )
            return result.returncode, log.read_text().splitlines()

    def test_active_upgrade_stops_before_copy_and_restarts_after_reload(self) -> None:
        status, calls = self.run_installer(active=True)
        self.assertEqual(status, 0)
        self.assertLess(calls.index("stop onboard-autonomy@" + "fixture-user" + ".service"), calls.index("copy"))
        self.assertLess(calls.index("copy"), calls.index("daemon-reload"))
        self.assertTrue(calls[-1].startswith("start onboard-autonomy@"))

    def test_first_install_does_not_stop_or_start_an_absent_unit(self) -> None:
        status, calls = self.run_installer(active=False)
        self.assertEqual(status, 0)
        self.assertFalse(any(call.startswith(("stop ", "start ")) for call in calls))

    def test_upgrade_preserves_the_activating_retry_loop(self) -> None:
        status, calls = self.run_installer(active=True, state="activating")
        self.assertEqual(status, 0)
        self.assertTrue(calls[-1].startswith("start onboard-autonomy@"))

    def test_failed_copy_keeps_the_unit_stopped(self) -> None:
        status, calls = self.run_installer(active=True, copy_fails=True)
        self.assertEqual(status, 27)
        self.assertFalse(any(call.startswith("start ") for call in calls))
