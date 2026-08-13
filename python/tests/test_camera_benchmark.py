import importlib.util
import sys
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).parents[1] / "camera_benchmark.py"
SPEC = importlib.util.spec_from_file_location("camera_benchmark", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
CAMERA_BENCHMARK = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CAMERA_BENCHMARK
SPEC.loader.exec_module(CAMERA_BENCHMARK)


class CameraBenchmarkTests(unittest.TestCase):
    def test_nominal_capture_passes(self) -> None:
        timestamps = [0.0, 33.333, 66.666, 99.999, 133.332]
        metadata = [
            {
                "SensorTemperature": 28.0,
                "ExposureTime": 10000,
                "AnalogueGain": 2.0,
                "Lux": 15.0,
                "FocusFoM": 100 + index,
                "AfState": 2,
            }
            for index in range(5)
        ]
        samples = [
            (0.0, 100, 10 * 1024),
            (500.0, 140, 12 * 1024),
            (1000.0, 180, 11 * 1024),
        ]

        summary = CAMERA_BENCHMARK.build_summary(
            camera_model="imx708_wide",
            width=1280,
            height=720,
            target_fps=30.0,
            requested_frames=5,
            capture_status=0,
            timestamps_ms=timestamps,
            metadata=metadata,
            process_samples=samples,
            clock_ticks_per_second=100,
        )

        self.assertEqual(summary["result"], "PASS")
        self.assertAlmostEqual(
            summary["capture"]["measured_fps"],
            30.0003,
            places=3,
        )
        self.assertEqual(
            summary["capture"]["estimated_dropped_frames"],
            0,
        )
        self.assertAlmostEqual(
            summary["resources"]["average_process_cpu_percent"],
            80.0,
        )
        self.assertAlmostEqual(
            summary["resources"]["peak_rss_mib"],
            12.0,
        )

    def test_timestamp_gap_estimates_missing_frames(self) -> None:
        intervals = [33.333, 99.999, 33.333]

        missing = CAMERA_BENCHMARK.estimate_missing_frames(
            intervals,
            1000.0 / 30.0,
        )

        self.assertEqual(missing, 2)

    def test_failed_capture_fails_acceptance(self) -> None:
        summary = CAMERA_BENCHMARK.build_summary(
            camera_model="imx708_wide",
            width=1280,
            height=720,
            target_fps=30.0,
            requested_frames=5,
            capture_status=1,
            timestamps_ms=[],
            metadata=[],
            process_samples=[],
            clock_ticks_per_second=100,
        )

        self.assertEqual(summary["result"], "FAIL")
        self.assertFalse(summary["checks"]["capture_exit_success"])
        self.assertFalse(
            summary["checks"]["resource_samples_available"]
        )


if __name__ == "__main__":
    unittest.main()
