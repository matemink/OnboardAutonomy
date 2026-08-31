import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).parents[2]
SELECTOR_PATH = PROJECT_ROOT / "scripts" / "clang_tidy_scope.py"


def load_selector():
    spec = importlib.util.spec_from_file_location("clang_tidy_scope", SELECTOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not load the clang-tidy scope selector")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ClangTidyScopeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.selector = load_selector()
        self.temp_dir = tempfile.TemporaryDirectory()
        self.project_root = Path(self.temp_dir.name)
        (self.project_root / "src").mkdir()

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_irrelevant_changes_skip_analysis(self) -> None:
        scope = self.selector.select_scope(
            self.project_root,
            ["README.md", "simulation/worlds/demo.sdf"],
        )
        self.assertEqual(scope.mode, "skip")
        self.assertEqual(scope.files, ())

    def test_changed_implementations_use_partial_analysis(self) -> None:
        (self.project_root / "src" / "First.cpp").touch()
        (self.project_root / "src" / "Second.cpp").touch()

        scope = self.selector.select_scope(
            self.project_root,
            ["src/Second.cpp", "src/First.cpp", "src/First.cpp"],
        )

        self.assertEqual(scope.mode, "partial")
        self.assertEqual(scope.files, ("src/First.cpp", "src/Second.cpp"))

    def test_header_change_requires_full_analysis(self) -> None:
        scope = self.selector.select_scope(
            self.project_root,
            ["include/onboard_autonomy/Telemetry.hpp"],
        )
        self.assertEqual(scope.mode, "full")

    def test_build_or_analyzer_change_requires_full_analysis(self) -> None:
        for path in (
            "CMakeLists.txt",
            "cmake/toolchains/aarch64.cmake",
            ".clang-tidy",
            ".github/workflows/ci.yml",
            "scripts/run_static_analysis.sh",
        ):
            with self.subTest(path=path):
                scope = self.selector.select_scope(self.project_root, [path])
                self.assertEqual(scope.mode, "full")

    def test_deleted_implementation_falls_back_to_full_analysis(self) -> None:
        scope = self.selector.select_scope(
            self.project_root,
            ["src/Removed.cpp"],
        )
        self.assertEqual(scope.mode, "full")

    def test_missing_base_ref_falls_back_to_full_analysis(self) -> None:
        scope = self.selector.determine_scope(
            self.project_root,
            "missing-base-ref",
        )
        self.assertEqual(scope.mode, "full")

    def test_no_base_ref_keeps_full_main_analysis(self) -> None:
        scope = self.selector.determine_scope(self.project_root, None)
        self.assertEqual(scope.mode, "full")


if __name__ == "__main__":
    unittest.main()
