"""Exercise ownership-based cleanup without starting Gazebo."""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MANAGER = PROJECT_ROOT / "scripts/demo_process.py"


@unittest.skipUnless(sys.platform == "linux", "Uses Linux /proc and pidfds")
class DemoProcessTests(unittest.TestCase):
    def test_stops_tagged_descendants_and_preserves_unrelated_process(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            ready = Path(temporary) / "ready.json"
            code = (
                "import json, subprocess, sys, time; from pathlib import Path; "
                "child=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)']); "
                "Path(sys.argv[1]).write_text(json.dumps(child.pid)); time.sleep(30)"
            )
            environment = os.environ.copy()
            environment.pop("ONBOARD_AUTONOMY_DEMO_ROOT", None)
            with subprocess.Popen(
                [sys.executable, "-c", "import time; time.sleep(30)"],
                env=environment,
            ) as unrelated, subprocess.Popen(
                [sys.executable, str(MANAGER), "run", "--",
                 sys.executable, "-c", code, str(ready)], env=environment,
            ) as owned:
                try:
                    deadline = time.monotonic() + 5
                    while not ready.exists() and time.monotonic() < deadline:
                        time.sleep(0.02)
                    self.assertTrue(ready.exists(), "tagged process did not start")
                    child_pid = json.loads(ready.read_text())
                    subprocess.run(
                        ["bash", str(PROJECT_ROOT / "scripts/stop_onboard_autonomy_gazebo.sh")],
                        check=True, capture_output=True, text=True, timeout=6,
                        env=environment,
                    )
                    owned.wait(timeout=2)
                    self.assertIsNone(unrelated.poll())
                    stat = Path(f"/proc/{child_pid}/stat")
                    self.assertTrue(
                        not stat.exists() or stat.read_text().rpartition(") ")[2].startswith("Z "),
                        "tagged descendant survived cleanup",
                    )
                finally:
                    for process in (owned, unrelated):
                        if process.poll() is None:
                            process.kill()
                        process.wait(timeout=2)

    def test_stale_start_time_cannot_signal_a_live_pid(self) -> None:
        sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
        import demo_process

        with subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"]) as process:
            try:
                demo_process.signal_process(process.pid, -1, signal.SIGTERM)
                self.assertIsNone(process.poll())
            finally:
                process.kill()
                process.wait(timeout=2)
