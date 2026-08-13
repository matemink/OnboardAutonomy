import importlib.util
import os
import sys
import tempfile
import unittest
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

MODULE_PATH = Path(__file__).parents[1] / "rotate_jsonl_logs.py"
SPEC = importlib.util.spec_from_file_location("rotate_jsonl_logs", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
LOG_ROTATION = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = LOG_ROTATION
SPEC.loader.exec_module(LOG_ROTATION)


class LogRotationTests(unittest.TestCase):
    def _log(
        self,
        directory: Path,
        name: str,
        size: int,
        timestamp_ns: int,
    ) -> Path:
        path = directory / name
        path.write_bytes(b"x" * size)
        os.utime(path, ns=(timestamp_ns, timestamp_ns))
        return path

    def test_oldest_logs_are_removed_by_count(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self._log(directory, "telemetry-1.jsonl", 10, 1)
            self._log(directory, "telemetry-2.jsonl", 10, 2)
            self._log(directory, "telemetry-3.jsonl", 10, 3)

            result = LOG_ROTATION.rotate_jsonl_logs(directory, 2, 100)

            self.assertEqual(
                result.kept,
                ("telemetry-3.jsonl", "telemetry-2.jsonl"),
            )
            self.assertEqual(result.removed, ("telemetry-1.jsonl",))
            self.assertFalse((directory / "telemetry-1.jsonl").exists())

    def test_byte_limit_keeps_newest_even_when_it_is_oversized(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            newest = self._log(
                directory,
                "telemetry-new.jsonl",
                120,
                2,
            )
            older = self._log(
                directory,
                "telemetry-old.jsonl",
                20,
                1,
            )
            unrelated = directory / "notes.jsonl"
            unrelated.write_text("keep", encoding="utf-8")

            result = LOG_ROTATION.rotate_jsonl_logs(directory, 10, 100)

            self.assertEqual(result.kept, (newest.name,))
            self.assertEqual(result.removed, (older.name,))
            self.assertTrue(newest.exists())
            self.assertTrue(unrelated.exists())

    def test_invalid_limits_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with self.assertRaisesRegex(ValueError, "max_files"):
                LOG_ROTATION.rotate_jsonl_logs(directory, 0, 1)
            with self.assertRaisesRegex(ValueError, "max_total_bytes"):
                LOG_ROTATION.rotate_jsonl_logs(directory, 1, 0)

    def test_stream_is_mirrored_and_rotated_while_running(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = StringIO("one\ntwo\nthree\n")
            mirror = StringIO()

            result = LOG_ROTATION.stream_jsonl_logs(
                source,
                mirror,
                directory,
                "telemetry-test",
                max_files=2,
                max_total_bytes=10,
                max_file_bytes=5,
            )

            logs = sorted(directory.glob("telemetry-*.jsonl"))
            self.assertEqual(mirror.getvalue(), "one\ntwo\nthree\n")
            self.assertEqual(len(logs), 2)
            self.assertEqual(
                [path.read_text(encoding="utf-8") for path in logs],
                ["two\n", "three\n"],
            )
            self.assertEqual(result.retained_bytes, 10)

    def test_stream_rejects_invalid_file_limit(self) -> None:
        with (
            tempfile.TemporaryDirectory() as temporary,
            self.assertRaisesRegex(ValueError, "max_file_bytes"),
        ):
            LOG_ROTATION.stream_jsonl_logs(
                StringIO(),
                StringIO(),
                Path(temporary),
                "telemetry-test",
                max_files=1,
                max_total_bytes=1,
                max_file_bytes=0,
            )

    def test_stream_uses_next_suffix_when_stem_already_exists(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            existing = self._log(
                directory,
                "telemetry-test.jsonl",
                4,
                1,
            )
            mirror = StringIO()

            LOG_ROTATION.stream_jsonl_logs(
                StringIO("new\n"),
                mirror,
                directory,
                "telemetry-test",
                max_files=3,
                max_total_bytes=100,
                max_file_bytes=10,
            )

            self.assertTrue(existing.exists())
            self.assertEqual(
                (directory / "telemetry-test-001.jsonl").read_text(
                    encoding="utf-8",
                ),
                "new\n",
            )
            self.assertEqual(mirror.getvalue(), "new\n")

    def test_main_handles_stream_interrupt_without_traceback(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            arguments = SimpleNamespace(
                directory=Path(temporary),
                max_files=20,
                max_total_bytes=100,
                max_file_bytes=10,
                stream=True,
                stem="telemetry-test",
            )
            with mock.patch.object(
                LOG_ROTATION,
                "parse_args",
                return_value=arguments,
            ), mock.patch.object(
                LOG_ROTATION,
                "stream_jsonl_logs",
                side_effect=KeyboardInterrupt,
            ):
                self.assertEqual(LOG_ROTATION.main(), 130)


if __name__ == "__main__":
    unittest.main()
