import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).parents[1] / "runtime_profile.py"
SPEC = importlib.util.spec_from_file_location("runtime_profile", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
RUNTIME_PROFILE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = RUNTIME_PROFILE
SPEC.loader.exec_module(RUNTIME_PROFILE)


class RuntimeProfileTests(unittest.TestCase):
    def test_sixty_second_profile_cannot_pass_at_fifty_four_seconds(self) -> None:
        summary = RUNTIME_PROFILE.build_summary(
            samples=[RUNTIME_PROFILE.RuntimeSample(0, 0, 1024, 1, 40, 0),
                     RUNTIME_PROFILE.RuntimeSample(54000, 100, 1024, 1, 40, 0)],
            requested_duration_seconds=60, runtime_status=0,
            clock_ticks_per_second=100, architecture="aarch64", kernel="test",
        )
        self.assertFalse(summary["checks"]["sample_window_complete"])
        self.assertEqual(summary["result"], "FAIL")

    def test_completion_allows_only_bounded_sampling_jitter(self) -> None:
        for last_ms, expected in ((59900, True), (59000, False)):
            with self.subTest(last_ms=last_ms):
                summary = RUNTIME_PROFILE.build_summary(
                    samples=[RUNTIME_PROFILE.RuntimeSample(0, 0, 1024, 1, 40, 0),
                             RUNTIME_PROFILE.RuntimeSample(last_ms, 100, 1024, 1, 40, 0)],
                    requested_duration_seconds=60, runtime_status=0,
                    clock_ticks_per_second=100, architecture="aarch64", kernel="test",
                )
                self.assertEqual(summary["checks"]["sample_window_complete"], expected)

    def test_delayed_exit_observation_does_not_prove_the_process_survived_the_window(self) -> None:
        for complete in (False, True):
            summary = RUNTIME_PROFILE.build_summary(
                samples=[RUNTIME_PROFILE.RuntimeSample(0, 0, 1024, 1, 40, 0),
                         RUNTIME_PROFILE.RuntimeSample(55000, 100, 1024, 1, 40, 0)],
                requested_duration_seconds=60, runtime_status=0,
                clock_ticks_per_second=100, architecture="aarch64", kernel="test",
                sample_interval_seconds=5, observed_duration_seconds=60.1,
                window_complete_at_stop=complete,
            )
            self.assertEqual(summary["checks"]["sample_window_complete"], complete)

    def test_complete_arm_profile_reports_process_group_resources(self) -> None:
        samples = [
            RUNTIME_PROFILE.RuntimeSample(0.0, 0, 20 * 1024, 3, 45.0, 0),
            RUNTIME_PROFILE.RuntimeSample(
                5000.0,
                250,
                30 * 1024,
                4,
                50.0,
                0,
            ),
            RUNTIME_PROFILE.RuntimeSample(
                10000.0,
                500,
                25 * 1024,
                4,
                48.0,
                0,
            ),
        ]

        summary = RUNTIME_PROFILE.build_summary(
            samples=samples,
            requested_duration_seconds=10.0,
            runtime_status=130,
            clock_ticks_per_second=100,
            architecture="aarch64",
            kernel="test-kernel",
        )

        self.assertEqual(summary["result"], "PASS")
        self.assertAlmostEqual(
            summary["resources"]["average_process_group_cpu_percent"],
            50.0,
        )
        self.assertAlmostEqual(
            summary["resources"]["peak_process_group_rss_mib"],
            30.0,
        )
        self.assertEqual(summary["thermal"]["maximum_temperature_c"], 50.0)
        self.assertIn(
            "Average process-group CPU",
            RUNTIME_PROFILE.render_markdown(summary),
        )

    def test_short_non_arm_run_fails_acceptance(self) -> None:
        summary = RUNTIME_PROFILE.build_summary(
            samples=[
                RUNTIME_PROFILE.RuntimeSample(
                    1000.0,
                    10,
                    1024,
                    1,
                    None,
                    None,
                )
            ],
            requested_duration_seconds=10.0,
            runtime_status=2,
            clock_ticks_per_second=100,
            architecture="x86_64",
            kernel="test-kernel",
        )

        self.assertEqual(summary["result"], "FAIL")
        self.assertFalse(summary["checks"]["sample_window_complete"])
        self.assertFalse(summary["checks"]["architecture_is_aarch64"])
        self.assertFalse(summary["checks"]["throttling_data_available"])

    def test_tsv_parser_preserves_unknown_thermal_values(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "samples.tsv"
            path.write_text(
                "elapsed_ms\tcumulative_cpu_ticks\trss_kib\t"
                "process_count\ttemperature_millic\tthrottled_hex\n"
                "100\t2\t4096\t3\t-\t-\n"
                "200\t4\t5120\t3\t47000\t0x0\n",
                encoding="utf-8",
            )

            samples = RUNTIME_PROFILE.read_samples(path)

            self.assertEqual(len(samples), 2)
            self.assertIsNone(samples[0].temperature_c)
            self.assertIsNone(samples[0].throttled)
            self.assertEqual(samples[1].temperature_c, 47.0)
            self.assertEqual(samples[1].throttled, 0)

    def test_invalid_profile_inputs_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duration"):
            RUNTIME_PROFILE.build_summary(
                samples=[],
                requested_duration_seconds=0.0,
                runtime_status=0,
                clock_ticks_per_second=100,
                architecture="aarch64",
                kernel="test",
            )
        with self.assertRaisesRegex(ValueError, "clock_ticks"):
            RUNTIME_PROFILE.build_summary(
                samples=[],
                requested_duration_seconds=1.0,
                runtime_status=0,
                clock_ticks_per_second=0,
                architecture="aarch64",
                kernel="test",
            )


if __name__ == "__main__":
    unittest.main()
