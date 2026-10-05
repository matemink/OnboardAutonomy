"""Exercise source and extracted-package capture entrypoints without hardware."""
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("bash"), "Requires Bash")
class CalibrationCaptureTests(unittest.TestCase):
    def run_capture(self, packaged: bool) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            commands = root / "commands"
            commands.mkdir()
            for name, contents in {
                "rpicam-hello": "echo '0 : imx708_wide [4608x2592]'",
                "rpicam-vid": "printf '%s\\n' \"$@\" > \"$CAPTURE_ARGUMENTS\"",
                "sleep": "exit 0",
                "python-fixture": "if [[ \"$1\" != -c ]]; then printf '%s\\n' \"$@\" > \"$ANALYZER_ARGUMENTS\"; fi",
            }.items():
                path = commands / name
                path.write_text("#!/usr/bin/env bash\n" + contents + "\n")
                path.chmod(0o755)
            package = root / "package"
            if packaged:
                (package / "bin").mkdir(parents=True)
                script = package / "bin/capture_camera_calibration.sh"
                shutil.copy(ROOT / "scripts/capture_camera_calibration.sh", script)
                shutil.copy(ROOT / "python/calibrate_camera.py", package / "bin/calibrate_camera.py")
                shutil.copy(ROOT / "python/requirements.txt", package / "requirements.txt")
            else:
                script = ROOT / "scripts/capture_camera_calibration.sh"
            mode = "4608:2592:10:U"
            capture = root / "capture"
            analyzer = root / "analyzer"
            result = subprocess.run(["bash", str(script)], capture_output=True, text=True,
                timeout=5, env=os.environ | {
                    "PATH": str(commands) + os.pathsep + os.environ["PATH"],
                    "ONBOARD_AUTONOMY_PYTHON": str(commands / "python-fixture"),
                    "ONBOARD_AUTONOMY_CAMERA_SENSOR_MODE": mode,
                    "ONBOARD_AUTONOMY_CALIBRATION_STATE_DIR": str(root / "state"),
                    "CAPTURE_ARGUMENTS": str(capture), "ANALYZER_ARGUMENTS": str(analyzer),
                })
            self.assertEqual(result.returncode, 0, result.stderr)
            capture_args = capture.read_text().splitlines()
            analyzer_args = analyzer.read_text().splitlines()
            self.assertEqual(capture_args[capture_args.index("--mode") + 1], mode)
            self.assertEqual(analyzer_args[analyzer_args.index("--sensor-mode") + 1], mode)
            self.assertTrue(Path(analyzer_args[0]).is_file())

    def test_source_capture_preserves_mode(self) -> None:
        self.run_capture(packaged=False)

    def test_extracted_package_capture_preserves_mode(self) -> None:
        self.run_capture(packaged=True)
