import importlib.util
import sys
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).parents[1] / "scenario_runner.py"
SPEC = importlib.util.spec_from_file_location("scenario_runner", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
SCENARIO_RUNNER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SCENARIO_RUNNER
SPEC.loader.exec_module(SCENARIO_RUNNER)


class ScenarioTests(unittest.TestCase):
    def test_healthy_scenario_has_3d_fix(self) -> None:
        healthy = SCENARIO_RUNNER.SCENARIOS["healthy"]
        self.assertGreaterEqual(healthy.gps_fix, 3)
        self.assertGreaterEqual(healthy.battery_percent, 20)
        self.assertIsNone(healthy.prearm_warning)

    def test_each_failure_changes_one_readiness_signal(self) -> None:
        self.assertLess(SCENARIO_RUNNER.SCENARIOS["no-gps"].gps_fix, 3)
        self.assertLess(
            SCENARIO_RUNNER.SCENARIOS["low-battery"].battery_percent,
            20,
        )
        self.assertIsNotNone(
            SCENARIO_RUNNER.SCENARIOS["prearm"].prearm_warning
        )


if __name__ == "__main__":
    unittest.main()
