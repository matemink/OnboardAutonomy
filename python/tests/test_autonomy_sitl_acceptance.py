import importlib.util
from pathlib import Path
import sys
import unittest


PYTHON_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PYTHON_ROOT))
MODULE_PATH = PYTHON_ROOT / "autonomy_sitl_acceptance.py"
SPEC = importlib.util.spec_from_file_location(
    "autonomy_sitl_acceptance",
    MODULE_PATH,
)
assert SPEC is not None
assert SPEC.loader is not None
ACCEPTANCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ACCEPTANCE
SPEC.loader.exec_module(ACCEPTANCE)


class AutonomySitlAcceptanceTests(unittest.TestCase):
    def test_expected_landing_position_comes_from_the_world(self) -> None:
        self.assertEqual(
            ACCEPTANCE.expected_landing_position_ne_m(),
            (0.0, 3.0),
        )

    def test_terminal_snapshot_requires_complete_startup_and_disarm(
        self,
    ) -> None:
        completed = {
            "flight_startup": {"phase": "completed"},
            "autonomy": {"phase": "completed"},
            "armed": False,
        }
        self.assertTrue(
            ACCEPTANCE.snapshot_completed_autonomy(completed)
        )

        for key in ("flight_startup", "autonomy", "armed"):
            incomplete = dict(completed)
            incomplete.pop(key)
            self.assertFalse(
                ACCEPTANCE.snapshot_completed_autonomy(incomplete),
                key,
            )

    def test_evidence_accepts_the_production_flight_contract(self) -> None:
        evidence = ACCEPTANCE.AutonomyFlightEvidence(
            commands=("ARM", "LAND", "SET_GUIDED", "TAKEOFF"),
            accepted_acknowledgements=(
                "ARM",
                "LAND",
                "SET_GUIDED",
                "TAKEOFF",
            ),
            maximum_relative_altitude_m=8.0,
            armed_transitions=(False, True, False),
            modes=(0, 4, 9),
            landing_target_count=50,
            landing_target_frames=(12,),
            position_valid_values=(1,),
            final_local_position_ne_m=(0.1, 3.0),
            final_horizontal_error_m=0.1,
            precision_statuses=("PrecLand: Target Found",),
        )

        ACCEPTANCE.validate_autonomy_evidence(evidence)

    def test_evidence_rejects_stale_or_missing_guidance(self) -> None:
        evidence = ACCEPTANCE.AutonomyFlightEvidence(
            commands=("ARM", "LAND", "SET_GUIDED", "TAKEOFF"),
            accepted_acknowledgements=(
                "ARM",
                "LAND",
                "SET_GUIDED",
                "TAKEOFF",
            ),
            maximum_relative_altitude_m=8.0,
            armed_transitions=(False, True, False),
            modes=(0, 4, 9),
            landing_target_count=0,
            landing_target_frames=(),
            position_valid_values=(),
            final_local_position_ne_m=(0.1, 3.0),
            final_horizontal_error_m=0.1,
            precision_statuses=("PrecLand: Target Found",),
        )

        with self.assertRaisesRegex(RuntimeError, "LANDING_TARGET"):
            ACCEPTANCE.validate_autonomy_evidence(evidence)

    def test_evidence_rejects_landing_at_the_takeoff_point(self) -> None:
        evidence = ACCEPTANCE.AutonomyFlightEvidence(
            commands=("ARM", "LAND", "SET_GUIDED", "TAKEOFF"),
            accepted_acknowledgements=(
                "ARM",
                "LAND",
                "SET_GUIDED",
                "TAKEOFF",
            ),
            maximum_relative_altitude_m=8.0,
            armed_transitions=(False, True, False),
            modes=(0, 4, 9),
            landing_target_count=50,
            landing_target_frames=(12,),
            position_valid_values=(1,),
            final_local_position_ne_m=(0.0, 0.1),
            final_horizontal_error_m=2.9,
            precision_statuses=("PrecLand: Target Found",),
        )

        with self.assertRaisesRegex(RuntimeError, "offset landing pad"):
            ACCEPTANCE.validate_autonomy_evidence(evidence)


if __name__ == "__main__":
    unittest.main()
