"""Exercise the camera launcher with fake Gazebo and companion processes."""

import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).parents[2]
FORWARD = "/world/camera_observation/model/Holybro_S500/link/Raspberry_Pi_Camera_Module_3_Wide_Forward/sensor/Raspberry_Pi_Camera_Module_3_Wide_Forward/image/enable_streaming"
DOWNWARD = "/world/camera_observation/model/Holybro_S500/link/Raspberry_Pi_Camera_Module_3_Wide/sensor/Raspberry_Pi_Camera_Module_3_Wide/image/enable_streaming"


@unittest.skipUnless(shutil.which("bash"), "Bash launchers require Linux/WSL")
class CameraLauncherTests(unittest.TestCase):
    def run_launcher(self, downward: bool) -> tuple[list[str], list[list[str]]]:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            gazebo = directory / "gz"
            gazebo.write_text(
                "#!/usr/bin/env python3\n"
                "import json, os, sys\n"
                "from pathlib import Path\n"
                "with open(os.environ['GAZEBO_LOG'], 'a') as log:\n"
                "    log.write(json.dumps(sys.argv[1:]) + '\\n')\n"
                "if sys.argv[1:] == ['topic', '-l']:\n"
                "    print(os.environ['TEST_FORWARD_TOPIC'])\n"
                "    if os.environ['ONBOARD_AUTONOMY_DOWNWARD_CAMERA'] == '1':\n"
                "        print(os.environ['TEST_DOWNWARD_TOPIC'])\n",
                encoding="utf-8",
            )
            companion = directory / "onboard_autonomy"
            companion.write_text(
                "#!/usr/bin/env python3\n"
                "import json, os, sys\n"
                "with open(os.environ['COMPANION_ARGS'], 'w') as output:\n"
                "    json.dump(sys.argv[1:], output)\n",
                encoding="utf-8",
            )
            gazebo.chmod(0o755)
            companion.chmod(0o755)
            environment = os.environ | {
                "PATH": str(directory) + os.pathsep + os.environ["PATH"],
                "ONBOARD_AUTONOMY_BUILD_DIR": str(directory),
                "ONBOARD_AUTONOMY_DOWNWARD_CAMERA": "1" if downward else "0",
                "ONBOARD_AUTONOMY_CAMERA_ENABLE_TOPIC": DOWNWARD,
                "ONBOARD_AUTONOMY_FORWARD_CAMERA_ENABLE_TOPIC": FORWARD,
                "GAZEBO_LOG": str(directory / "gazebo.jsonl"),
                "COMPANION_ARGS": str(directory / "arguments.json"),
                "TEST_FORWARD_TOPIC": FORWARD,
                "TEST_DOWNWARD_TOPIC": DOWNWARD,
            }
            for name in ("ONBOARD_AUTONOMY_DIAGNOSTIC_LOG",
                         "ONBOARD_AUTONOMY_SIM_WIND_PROFILE",
                         "ONBOARD_AUTONOMY_FORWARD_DETECTOR_MODEL"):
                environment.pop(name, None)
            subprocess.run(
                ["bash", str(PROJECT_ROOT / "scripts/run_onboard_autonomy_gazebo_vision.sh")],
                env=environment, capture_output=True, text=True, check=True, timeout=5,
            )
            return (
                json.loads((directory / "arguments.json").read_text()),
                [json.loads(line) for line in
                 (directory / "gazebo.jsonl").read_text().splitlines()],
            )

    def test_default_needs_only_forward_topic_and_receiver(self) -> None:
        arguments, gazebo_calls = self.run_launcher(downward=False)
        self.assertNotIn("--camera", arguments)
        self.assertNotIn("--camera-udp-port", arguments)
        self.assertEqual(arguments[arguments.index("--forward-camera-udp-port") + 1], "5602")
        self.assertIn("--camera-preview", arguments)
        self.assertFalse(any(DOWNWARD in call for call in gazebo_calls))
        self.assertTrue(any(FORWARD in call and "data: true" in call for call in gazebo_calls))
        self.assertTrue(any(FORWARD in call and "data: false" in call for call in gazebo_calls))

    def test_opt_in_enables_and_cleans_up_both_streams(self) -> None:
        arguments, gazebo_calls = self.run_launcher(downward=True)
        self.assertIn("--camera", arguments)
        self.assertEqual(arguments[arguments.index("--camera-udp-port") + 1], "5601")
        for topic in (FORWARD, DOWNWARD):
            self.assertTrue(any(topic in call and "data: true" in call for call in gazebo_calls))
            self.assertTrue(any(topic in call and "data: false" in call for call in gazebo_calls))
