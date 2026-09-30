"""Check package dependencies, including transitive presentation-model includes."""

import re
import unittest
from pathlib import Path

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
PROJECT_INCLUDE = re.compile(r'^\s*#\s*include\s*"onboard_autonomy/([^"\n]+)"', re.MULTILINE)
EXECUTION_HEADERS = {
    "CompanionApplication.hpp", "VehicleState.hpp", "FlightStartupController.hpp",
    "AutonomyRuntime.hpp", "CompanionLinkFailsafe.hpp", "CameraMonitor.hpp",
    "AsyncCameraMonitor.hpp", "VisionMonitor.hpp", "AerialTargetTracker.hpp",
    "AerialYawController.hpp",
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
                    if package in {"operator", "diagnostics"} and header.name in EXECUTION_HEADERS:
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
            if header.name in EXECUTION_HEADERS:
                violations.append(name)
            pending.extend(project_includes(header))
        self.assertEqual(violations, [], "Snapshot models expose execution dependencies")

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


if __name__ == "__main__":
    unittest.main()
