import importlib.util
import sys
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).parents[1] / "sitl_harness.py"
SPEC = importlib.util.spec_from_file_location("sitl_harness", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
SITL_HARNESS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SITL_HARNESS
SPEC.loader.exec_module(SITL_HARNESS)


class SitlHarnessTests(unittest.TestCase):
    def test_complete_snapshot_requires_every_health_source(self) -> None:
        complete = {
            "connected": True,
            "battery_voltage_v": 15.2,
            "system_health_known": True,
        }
        self.assertTrue(
            SITL_HARNESS.snapshot_has_required_telemetry(complete)
        )

        for missing_key in complete:
            incomplete = dict(complete)
            incomplete[missing_key] = None
            self.assertFalse(
                SITL_HARNESS.snapshot_has_required_telemetry(incomplete),
                missing_key,
            )

    def test_ready_snapshot_requires_all_readiness_checks(self) -> None:
        ready = {
            "connected": True,
            "battery_voltage_v": 12.6,
            "system_health_known": True,
            "gps_ready": False,
            "navigation_ready": True,
            "battery_ready": True,
            "system_health_ok": True,
            "armable": True,
        }
        self.assertTrue(SITL_HARNESS.snapshot_is_ready(ready))

        for readiness_key in (
            "navigation_ready",
            "battery_ready",
            "system_health_ok",
            "armable",
        ):
            not_ready = dict(ready)
            not_ready[readiness_key] = False
            self.assertFalse(
                SITL_HARNESS.snapshot_is_ready(not_ready),
                readiness_key,
            )

    def test_disconnected_snapshot_requires_false_connection(self) -> None:
        self.assertTrue(
            SITL_HARNESS.snapshot_is_disconnected(
                {"connected": False}
            )
        )
        self.assertFalse(
            SITL_HARNESS.snapshot_is_disconnected(
                {"connected": True}
            )
        )
        self.assertFalse(
            SITL_HARNESS.snapshot_is_disconnected({})
        )

    def test_gps_failure_keeps_link_and_other_health_data(self) -> None:
        gps_failure = {
            "connected": True,
            "gps_fix_type": 1,
            "gps_ready": False,
            "battery_ready": True,
            "system_health_known": True,
            "armable": False,
        }
        self.assertTrue(
            SITL_HARNESS.snapshot_has_gps_failure(gps_failure)
        )

        for key, healthy_value in (
            ("connected", False),
            ("gps_fix_type", 3),
            ("gps_ready", True),
            ("battery_ready", False),
            ("system_health_known", False),
            ("armable", True),
        ):
            invalid = dict(gps_failure)
            invalid[key] = healthy_value
            self.assertFalse(
                SITL_HARNESS.snapshot_has_gps_failure(invalid),
                key,
            )

    def test_low_battery_keeps_link_and_gps_ready(self) -> None:
        low_battery = {
            "connected": True,
            "gps_ready": True,
            "battery_remaining_pct": 19,
            "battery_ready": False,
            "system_health_known": True,
            "armable": False,
        }
        self.assertTrue(
            SITL_HARNESS.snapshot_has_low_battery(low_battery)
        )

        for key, non_failure_value in (
            ("connected", False),
            ("gps_ready", False),
            ("battery_remaining_pct", 20),
            ("battery_ready", True),
            ("system_health_known", False),
            ("armable", True),
        ):
            invalid = dict(low_battery)
            invalid[key] = non_failure_value
            self.assertFalse(
                SITL_HARNESS.snapshot_has_low_battery(invalid),
                key,
            )

    def test_prearm_failure_keeps_link_gps_and_battery_ready(
        self,
    ) -> None:
        prearm_failure = {
            "connected": True,
            "gps_ready": True,
            "battery_ready": True,
            "system_health_known": True,
            "system_health_ok": False,
            "warnings": [
                "PreArm: Motors: MOT_SPIN_ARM > MOT_SPIN_MIN",
            ],
            "armable": False,
        }
        self.assertTrue(
            SITL_HARNESS.snapshot_has_prearm_failure(prearm_failure)
        )

        invalid_cases = {
            "no warning": [],
            "wrong prefix": [
                "Arm: Motors: MOT_SPIN_ARM > MOT_SPIN_MIN",
            ],
            "wrong failure": ["PreArm: GPS 1: Bad fix"],
        }
        for case, warnings in invalid_cases.items():
            invalid = dict(prearm_failure)
            invalid["warnings"] = warnings
            self.assertFalse(
                SITL_HARNESS.snapshot_has_prearm_failure(invalid),
                case,
            )

        for key, non_failure_value in (
            ("connected", False),
            ("gps_ready", False),
            ("battery_ready", False),
            ("system_health_known", False),
            ("system_health_ok", True),
            ("armable", True),
        ):
            invalid = dict(prearm_failure)
            invalid[key] = non_failure_value
            self.assertFalse(
                SITL_HARNESS.snapshot_has_prearm_failure(invalid),
                key,
            )

    def test_protocol_evidence_requires_commands_acks_and_heartbeat(
        self,
    ) -> None:
        valid = SITL_HARNESS.TelemetryEvidence(
            interval_requests=tuple(SITL_HARNESS.EXPECTED_INTERVALS),
            accepted_ack_count=3,
            companion_heartbeat_count=1,
        )
        SITL_HARNESS.validate_evidence(valid)

        missing_ack = SITL_HARNESS.TelemetryEvidence(
            interval_requests=valid.interval_requests,
            accepted_ack_count=2,
            companion_heartbeat_count=1,
        )
        with self.assertRaisesRegex(RuntimeError, "three accepted"):
            SITL_HARNESS.validate_evidence(missing_ack)

        missing_heartbeat = SITL_HARNESS.TelemetryEvidence(
            interval_requests=valid.interval_requests,
            accepted_ack_count=3,
            companion_heartbeat_count=0,
        )
        with self.assertRaisesRegex(RuntimeError, "heartbeat"):
            SITL_HARNESS.validate_evidence(missing_heartbeat)

if __name__ == "__main__":
    unittest.main()
