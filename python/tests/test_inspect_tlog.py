import io
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace
from unittest.mock import patch

from inspect_tlog import main


class Message(SimpleNamespace):
    def get_type(self) -> str:
        return self.kind

    def get_srcSystem(self) -> int:
        return 1

    def get_srcComponent(self) -> int:
        return 1


class InspectTlogTests(unittest.TestCase):
    def test_reports_current_statuses_separately_from_prearm(self) -> None:
        messages = iter([
            Message(kind="SYS_STATUS", onboard_control_sensors_present=0,
                    onboard_control_sensors_enabled=0,
                    onboard_control_sensors_health=0),
            Message(kind="STATUSTEXT", text=b"PreArm: GPS unavailable\0"),
            Message(kind="STATUSTEXT", text=b"GCS Failsafe\0"),
            Message(kind="STATUSTEXT", text="Land complete"),
            Message(kind="STATUSTEXT", text="Land complete"),
            Message(kind="STATUSTEXT", text=b"\0"),
        ])
        connection = SimpleNamespace(recv_match=lambda: next(messages, None))
        output = io.StringIO()
        with patch("sys.argv", ["inspect_tlog.py", "example.tlog"]), patch(
            "inspect_tlog.mavutil.mavlink_connection", return_value=connection
        ), redirect_stdout(output):
            self.assertEqual(main(), 0)

        prearm, statuses = output.getvalue().split("  ArduPilot statuses:")
        self.assertIn("PreArm: GPS unavailable", prearm)
        self.assertNotIn("PreArm:", statuses)
        self.assertIn("GCS Failsafe", statuses)
        self.assertEqual(statuses.count("Land complete"), 1)
        self.assertNotIn("\0", output.getvalue())


if __name__ == "__main__":
    unittest.main()
