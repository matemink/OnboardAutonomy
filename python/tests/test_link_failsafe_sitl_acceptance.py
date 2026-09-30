import sys
import tempfile
import unittest
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

from link_failsafe_sitl_acceptance import (
    IndependentMonitorEvidence,
    LinkFailsafeEvidence,
    run_acceptance,
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

    def test_stops_live_companion_before_independent_validation(self) -> None:
        runtime = Mock(returncode=None)
        runtime.wait.side_effect = TimeoutError("runtime does not exit itself")
        supervisor = Mock()
        events: list[str] = []

        def stop(name: str, timeout: float) -> None:
            self.assertEqual(name, "OnboardAutonomy")
            self.assertGreater(timeout, 0)
            runtime.returncode = 0
            events.append("companion stopped")

        def observe(*args: object, **kwargs: object) -> IndependentMonitorEvidence:
            self.assertEqual(events, ["companion stopped"])
            events.append("independent LAND/disarm")
            return self.independent_monitor

        supervisor.stop.side_effect = stop
        armed = {"companion_link_failsafe": {"timeout_s": 3.0}}
        lost = {"connected": False, "autonomy": {"phase": "failed"}}
        module = "link_failsafe_sitl_acceptance."
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            for name, value in {
                "require_available_port": None,
                "wait_for_gazebo_camera": None,
                "ProcessSupervisor": supervisor,
                "UdpCutRelay": Mock(),
                "inspect_link_failsafe_tlog": self.evidence,
            }.items():
                stack.enter_context(patch(module + name, return_value=value))
            stack.enter_context(patch(
                "pymavlink.mavutil.mavlink_connection", return_value=Mock()
            ))
            stack.enter_context(patch(
                module + "subprocess.Popen",
                side_effect=[Mock(), Mock(), runtime],
            ))
            stack.enter_context(patch(
                module + "wait_for_snapshot", side_effect=[armed, lost]
            ))
            stack.enter_context(patch(
                module + "wait_for_ardupilot_land_and_disarm",
                side_effect=observe,
            ))
            result = run_acceptance(
                Path(sys.executable), Path(directory) / "evidence", 1.0
            )

        self.assertEqual(events, [
            "companion stopped", "independent LAND/disarm"
        ])
        self.assertEqual(result.link_loss_snapshot, lost)
        self.assertEqual(result.evidence, self.evidence)
        runtime.wait.assert_not_called()
        supervisor.stop_all.assert_called_once()

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
