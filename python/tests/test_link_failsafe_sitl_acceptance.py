import unittest
from dataclasses import replace

from link_failsafe_sitl_acceptance import (
    IndependentMonitorEvidence,
    LinkFailsafeEvidence,
    snapshot_ready_for_link_cut,
    snapshot_records_link_loss,
    validate_link_failsafe_evidence,
)


class LinkFailsafeSitlAcceptanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.evidence = LinkFailsafeEvidence(
            flight_commands=("ARM", "SET_GUIDED", "TAKEOFF"),
            companion_heartbeat_count=8,
            modes=(0, 4, 9),
            armed_transitions=(False, True, False),
            failover_latency_s=3.2,
            failsafe_statuses=("GCS Failsafe",),
        )
        self.independent_monitor = IndependentMonitorEvidence(
            modes=(4, 9),
            armed_transitions=(True, False),
        )

    def test_independent_land_evidence_is_accepted(self) -> None:
        validate_link_failsafe_evidence(
            self.evidence,
            self.independent_monitor,
            3.0,
        )

    def test_companion_land_command_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "unexpected flight"):
            validate_link_failsafe_evidence(
                replace(
                    self.evidence,
                    flight_commands=(
                        "ARM",
                        "LAND",
                        "SET_GUIDED",
                        "TAKEOFF",
                    ),
                ),
                self.independent_monitor,
                3.0,
            )

    def test_timeout_mismatch_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "latency"):
            validate_link_failsafe_evidence(
                replace(self.evidence, failover_latency_s=8.0),
                self.independent_monitor,
                3.0,
            )

    def test_independent_monitor_must_observe_disarm(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "LAND/disarm"):
            validate_link_failsafe_evidence(
                self.evidence,
                replace(
                    self.independent_monitor,
                    armed_transitions=(True,),
                ),
                3.0,
            )

    def test_snapshots_prove_cut_and_application_loss(self) -> None:
        armed = {
            "armed": True,
            "relative_altitude_m": 8.0,
            "flight_startup": {"phase": "completed"},
            "companion_link_failsafe": {
                "phase": "accepted",
                "action": "land",
            },
        }
        lost = {
            "connected": False,
            "autonomy": {
                "phase": "failed",
                "detail": (
                    "Flight-controller heartbeat was lost during autonomy"
                ),
            },
        }
        self.assertTrue(snapshot_ready_for_link_cut(armed))
        self.assertTrue(snapshot_records_link_loss(lost))


if __name__ == "__main__":
    unittest.main()
