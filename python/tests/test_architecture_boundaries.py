"""Check package dependencies, including transitive presentation-model includes."""

import re
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).parents[2]
PUBLIC_ROOT = PROJECT_ROOT / "include/onboard_autonomy"
PACKAGE_DEPENDENCIES = {
    "bootstrap": {"bootstrap", "mission", "hardware", "operator", "diagnostics"},
    "mission": {"mission"},
    "hardware": {"hardware", "mission"},
    "operator": {"operator", "mission"},
    "diagnostics": {"diagnostics", "mission"},
}
# The application currently routes protocol records to mission state machines.
# Keep this exception local; mission headers and controllers cannot use hardware.
PROTOCOL_ROUTER = Path("src/mission/CompanionApplication.cpp")
PROJECT_INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]onboard_autonomy/([^>"\n]+)[>"]', re.MULTILINE)
MODEL_AND_PORT_HEADERS = {
    "mission/AppSnapshot.hpp",
    "mission/Clock.hpp",
    "mission/EnvironmentProfile.hpp",
    "mission/SnapshotSink.hpp",
    "mission/autonomy/AutonomySnapshot.hpp",
    "mission/cv/CameraSnapshot.hpp",
    "mission/cv/VisionSnapshot.hpp",
    "mission/cv/CameraSource.hpp",
    "mission/cv/detection/TargetObservation.hpp",
    "mission/flight/FlightStartupSnapshot.hpp",
    "mission/flight/VehicleSnapshot.hpp",
    "mission/safety/CompanionLinkFailsafeSnapshot.hpp",
    "mission/safety/MotionSafetyStatus.hpp",
}


def project_includes(path: Path) -> list[str]:
    return PROJECT_INCLUDE.findall(path.read_text(encoding="utf-8"))


class ArchitectureBoundaryTests(unittest.TestCase):
    def test_package_include_dependencies(self) -> None:
        violations = []
        for root in (PUBLIC_ROOT, PROJECT_ROOT / "src"):
            for source in sorted(root.rglob("*")):
                if source.suffix not in {".hpp", ".cpp"}:
                    continue
                relative = source.relative_to(root)
                package = relative.parts[0]
                if package not in PACKAGE_DEPENDENCIES:
                    continue
                allowed = PACKAGE_DEPENDENCIES[package]
                for include in project_includes(source):
                    header = PUBLIC_ROOT / include
                    if not header.is_file():
                        violations.append(f"{relative}: missing header {include}")
                        continue
                    dependency = include.split("/", 1)[0]
                    if (
                        package in {"operator", "diagnostics"}
                        and dependency == "mission"
                        and include not in MODEL_AND_PORT_HEADERS
                    ):
                        violations.append(f"{relative}: presentation imports execution {include}")
                    router_exception = (
                        source.relative_to(PROJECT_ROOT) == PROTOCOL_ROUTER
                        and include.startswith("hardware/mavlink/")
                    )
                    if dependency not in allowed and not router_exception:
                        violations.append(f"{relative}: forbidden dependency {include}")
                    if (
                        package == "mission"
                        and relative.parts[1] == "cv"
                        and include.startswith("mission/flight/")
                    ):
                        violations.append(f"{relative}: vision depends on flight {include}")
        self.assertEqual(violations, [], "\n".join(violations))

    def test_snapshot_model_does_not_import_execution(self) -> None:
        visited = set()
        pending = ["mission/AppSnapshot.hpp", "mission/SnapshotSink.hpp"]
        violations = []
        while pending:
            name = pending.pop()
            if name in visited:
                continue
            visited.add(name)
            header = PUBLIC_ROOT / name
            if name not in MODEL_AND_PORT_HEADERS:
                violations.append(name)
            pending.extend(project_includes(header))
        self.assertEqual(violations, [], "Snapshot models import a header outside the model/port allowlist")

    def test_public_header_graph_has_no_cycles(self) -> None:
        graph = {
            header.relative_to(PUBLIC_ROOT).as_posix(): project_includes(header)
            for header in PUBLIC_ROOT.rglob("*.hpp")
        }
        visited = set()
        active = []

        def visit(name: str) -> None:
            self.assertNotIn(name, active, " -> ".join([*active, name]))
            if name in visited:
                return
            active.append(name)
            for dependency in graph[name]:
                visit(dependency)
            active.pop()
            visited.add(name)

        for name in graph:
            visit(name)


class BoundaryRegressionTests(unittest.TestCase):
    def test_presentation_rejects_existing_and_new_execution_headers(self) -> None:
        for delimiter in ('"', '<'):
            closing = '"' if delimiter == '"' else '>'
            for forbidden in (
                "mission/safety/MotionSafetyPolicy.hpp",
                "mission/cv/detection/OpenCvDnnTargetDetector.hpp",
                "mission/NewController.hpp",
            ):
                with self.subTest(delimiter=delimiter, header=forbidden), tempfile.TemporaryDirectory() as temp:
                    root = Path(temp)
                    public = root / "include/onboard_autonomy"
                    header = public / forbidden
                    header.parent.mkdir(parents=True)
                    header.write_text("#pragma once\n", encoding="utf-8")
                    source = root / "src/operator/Screen.cpp"
                    source.parent.mkdir(parents=True)
                    source.write_text(
                        f"#include {delimiter}onboard_autonomy/{forbidden}{closing}\n",
                        encoding="utf-8",
                    )
                    with patch(__name__ + ".PROJECT_ROOT", root), patch(
                        __name__ + ".PUBLIC_ROOT", public
                    ):
                        checks = ArchitectureBoundaryTests()
                        with self.assertRaisesRegex(AssertionError, "presentation imports execution"):
                            checks.test_package_include_dependencies()

    def test_snapshot_rejects_unknown_execution_header_with_both_delimiters(self) -> None:
        for delimiter in ('"', '<'):
            closing = '"' if delimiter == '"' else '>'
            with self.subTest(delimiter=delimiter), tempfile.TemporaryDirectory() as temp:
                public = Path(temp) / "include/onboard_autonomy"
                mission = public / "mission"
                mission.mkdir(parents=True)
                (mission / "AppSnapshot.hpp").write_text(
                    f"#include {delimiter}onboard_autonomy/mission/NewController.hpp{closing}\n",
                    encoding="utf-8",
                )
                (mission / "SnapshotSink.hpp").write_text("#pragma once\n", encoding="utf-8")
                (mission / "NewController.hpp").write_text("#pragma once\n", encoding="utf-8")
                with patch(__name__ + ".PUBLIC_ROOT", public):
                    checks = ArchitectureBoundaryTests()
                    with self.assertRaisesRegex(AssertionError, "outside the model/port allowlist"):
                        checks.test_snapshot_model_does_not_import_execution()

    def test_angle_bracket_include_cycle_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            public = Path(temp) / "include/onboard_autonomy"
            mission = public / "mission"
            mission.mkdir(parents=True)
            for source, destination in (("First", "Second"), ("Second", "First")):
                (mission / f"{source}.hpp").write_text(
                    f"#include <onboard_autonomy/mission/{destination}.hpp>\n",
                    encoding="utf-8",
                )
            with patch(__name__ + ".PUBLIC_ROOT", public):
                checks = ArchitectureBoundaryTests()
                with self.assertRaises(AssertionError):
                    checks.test_public_header_graph_has_no_cycles()


if __name__ == "__main__":
    unittest.main()
