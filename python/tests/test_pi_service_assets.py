import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]


class PiServiceAssetTests(unittest.TestCase):
    def test_service_runs_unprivileged_and_retries_startup(self) -> None:
        unit = (
            PROJECT_ROOT
            / "deployment/systemd/onboard-autonomy@.service"
        ).read_text(encoding="utf-8")
        for contract in (
            "User=%i",
            "StartLimitIntervalSec=0",
            "Restart=on-failure",
            "RestartSec=3s",
            "TimeoutStopSec=15s",
            "NoNewPrivileges=true",
            "ProtectSystem=strict",
            "ReadWritePaths=/home/%i/.local/state/onboard_autonomy",
        ):
            self.assertIn(contract, unit)
        self.assertNotIn("User=root", unit)

    def test_package_contains_service_and_log_rotation_assets(self) -> None:
        package_script = (
            PROJECT_ROOT / "scripts/package_pi5_release.sh"
        ).read_text(encoding="utf-8")
        launcher = (
            PROJECT_ROOT / "scripts/run_onboard_autonomy_pi.sh"
        ).read_text(encoding="utf-8")
        for asset in (
            "rotate_jsonl_logs.py",
            "runtime_profile.py",
            "install_onboard_autonomy_service.sh",
            "profile_onboard_autonomy_pi.sh",
            "onboard-autonomy@.service",
            "onboard-autonomy.env.example",
        ):
            self.assertIn(asset, package_script)
        self.assertIn("rotate_jsonl_logs.py", launcher)
        self.assertIn("ONBOARD_AUTONOMY_LOG_MAX_FILES", launcher)
        self.assertIn("ONBOARD_AUTONOMY_LOG_MAX_FILE_BYTES", launcher)
        self.assertIn("--stream", launcher)


if __name__ == "__main__":
    unittest.main()
